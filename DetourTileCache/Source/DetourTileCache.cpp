#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include <math.h>
#include <string.h>
#include <new>

dtTileCache* dtAllocTileCache()
{
	void* mem = dtAlloc(sizeof(dtTileCache), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtTileCache;
}

void dtFreeTileCache(dtTileCache* tc)
{
	if (!tc) return;
	tc->~dtTileCache();
	dtFree(tc);
}

static bool contains(const dtCompressedTileRef* a, const int n, const dtCompressedTileRef v)
{
	for (int i = 0; i < n; ++i)
		if (a[i] == v)
			return true;
	return false;
}

inline int computeTileHash(int x, int y, const int mask)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & mask);
}


struct BuildContext
{
	inline BuildContext(struct dtTileCacheAlloc* a) : layer(0), lcset(0), lmesh(0), alloc(a) {}
	inline ~BuildContext() { purge(); }
	void purge()
	{
		dtFreeTileCacheLayer(alloc, layer);
		layer = 0;
		dtFreeTileCacheContourSet(alloc, lcset);
		lcset = 0;
		dtFreeTileCachePolyMesh(alloc, lmesh);
		lmesh = 0;
	}
	struct dtTileCacheLayer* layer;
	struct dtTileCacheContourSet* lcset;
	struct dtTileCachePolyMesh* lmesh;
	struct dtTileCacheAlloc* alloc;
};


dtTileCache::dtTileCache() :
	m_nTileLutSize(0),
	m_nTileLutMask(0),
	m_ppPosLookup(0),
	m_pNextFreeTile(0),	
	m_pCompressedTiles(0),	
	m_uSaltBits(0),
	m_uTileBits(0),
	m_talloc(0),
	m_tcomp(0),
	m_tmproc(0),
	m_pObstacles(0),
	m_pNextFreeObstacle(0),
	m_nreqs(0),
	m_nUpdate(0)
{
	memset(&m_Params, 0, sizeof(m_Params));
}
	
dtTileCache::~dtTileCache()
{
	for (int i = 0; i < m_Params.nMaxTiles; ++i)
	{
		if (m_pCompressedTiles[i].uFlags & DT_COMPRESSEDTILE_FREE_DATA)
		{
			dtFree(m_pCompressedTiles[i].pData);
			m_pCompressedTiles[i].pData = 0;
		}
	}
	dtFree(m_pObstacles);
	m_pObstacles = 0;
	dtFree(m_ppPosLookup);
	m_ppPosLookup = 0;
	dtFree(m_pCompressedTiles);
	m_pCompressedTiles = 0;
	m_nreqs = 0;
	m_nUpdate = 0;
}

const dtCompressedTile* dtTileCache::getTileByRef(dtCompressedTileRef Ref) const
{
	if (!Ref)
		return 0;
	unsigned int tileIndex = decodeTileIdTile(Ref);
	unsigned int tileSalt = decodeTileIdSalt(Ref);
	if ((int)tileIndex >= m_Params.nMaxTiles)
		return 0;
	const dtCompressedTile* tile = &m_pCompressedTiles[tileIndex];
	if (tile->uSalt != tileSalt)
		return 0;
	return tile;
}


dtStatus dtTileCache::init(const dtTileCacheParams* params,
						   dtTileCacheAlloc* talloc,
						   dtTileCacheCompressor* tcomp,
						   dtTileCacheMeshProcess* tmproc)
{
	m_talloc = talloc;
	m_tcomp = tcomp;
	m_tmproc = tmproc;
	m_nreqs = 0;
	memcpy(&m_Params, params, sizeof(m_Params));
	
	// Alloc space for obstacles.
	m_pObstacles = (dtTileCacheObstacle*)dtAlloc(sizeof(dtTileCacheObstacle) * m_Params.nMaxObstacles, DT_ALLOC_PERM);
	if (!m_pObstacles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

    memset(m_pObstacles, 0, sizeof(dtTileCacheObstacle) * m_Params.nMaxObstacles);
	m_pNextFreeObstacle = 0;
	for (int i = m_Params.nMaxObstacles - 1; i >= 0; --i)
	{
		m_pObstacles[i].uSalt = 1;
		m_pObstacles[i].pNext = m_pNextFreeObstacle;
		m_pNextFreeObstacle = &m_pObstacles[i];
	}
	
	// Init tiles
	m_nTileLutSize = dtNextPow2(m_Params.nMaxTiles / 4);
	if (!m_nTileLutSize) m_nTileLutSize = 1;
	m_nTileLutMask = m_nTileLutSize - 1;
	
	m_pCompressedTiles = (dtCompressedTile*)dtAlloc(sizeof(dtCompressedTile) * m_Params.nMaxTiles, DT_ALLOC_PERM);
	if (!m_pCompressedTiles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

    m_ppPosLookup = (dtCompressedTile**)dtAlloc(sizeof(dtCompressedTile*) * m_nTileLutSize, DT_ALLOC_PERM);
	if (!m_ppPosLookup)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

    memset(m_pCompressedTiles, 0, sizeof(dtCompressedTile) * m_Params.nMaxTiles);
	memset(m_ppPosLookup, 0, sizeof(dtCompressedTile*) * m_nTileLutSize);
	m_pNextFreeTile = 0;
	for (int i = m_Params.nMaxTiles - 1; i >= 0; --i)
	{
		m_pCompressedTiles[i].uSalt = 1;
		m_pCompressedTiles[i].pNext = m_pNextFreeTile;
		m_pNextFreeTile = &m_pCompressedTiles[i];
	}
	
	// Init ID generator values.
	m_uTileBits = dtIlog2(dtNextPow2((unsigned int)m_Params.nMaxTiles));
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	m_uSaltBits = dtMin((unsigned int)31, 32 - m_uTileBits);
	if (m_uSaltBits < 10)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	return DT_SUCCESS;
}

int dtTileCache::getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const 
{
	int n = 0;
	
	// Find tile based on hash.
	int h = computeTileHash(tx, ty, m_nTileLutMask);
	dtCompressedTile* tile = m_ppPosLookup[h];
	while (tile)
	{
		if (tile->pHeader &&
			tile->pHeader->nTileX == tx &&
			tile->pHeader->nTileY == ty)
		{
			if (n < maxTiles)
				tiles[n++] = getTileRef(tile);
		}
		tile = tile->pNext;
	}
	
	return n;
}

dtCompressedTile* dtTileCache::getTileAt(const int tx, const int ty, const int tlayer)
{
	// Find tile based on hash.
	int h = computeTileHash(tx, ty, m_nTileLutMask);
	dtCompressedTile* tile = m_ppPosLookup[h];
	while (tile)
	{
		if (tile->pHeader &&
			tile->pHeader->nTileX == tx &&
			tile->pHeader->nTileY == ty &&
			tile->pHeader->nTileLayer == tlayer)
		{
			return tile;
		}
		tile = tile->pNext;
	}
	return 0;
}

dtCompressedTileRef dtTileCache::getTileRef(const dtCompressedTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = tile - m_pCompressedTiles;
	return (dtCompressedTileRef)encodeTileId(tile->uSalt, it);
}

dtObstacleRef dtTileCache::getObstacleRef(const dtTileCacheObstacle* ob) const
{
	if (!ob) return 0;
	const unsigned int idx = ob - m_pObstacles;
	return encodeObstacleId(ob->uSalt, idx);
}

const dtTileCacheObstacle* dtTileCache::getObstacleByRef(dtObstacleRef Ref)
{
	if (!Ref)
		return 0;

    unsigned int idx = decodeObstacleIdObstacle(Ref);
	if ((int)idx >= m_Params.nMaxObstacles)
		return 0;

    const dtTileCacheObstacle* ob = &m_pObstacles[idx];
	unsigned int salt = decodeObstacleIdSalt(Ref);
	if (ob->uSalt != salt)
		return 0;

    return ob;
}

dtStatus dtTileCache::addTile(
    unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result)
{
	// Make sure the data is in right format.
	dtTileCacheLayerHeader* header = (dtTileCacheLayerHeader*)data;
	if (header->nMagic != DT_TILECACHE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->nVersion != DT_TILECACHE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	
	// Make sure the location is free.
	if (getTileAt(header->nTileX, header->nTileY, header->nTileLayer))
		return DT_FAILURE;
	
	// Allocate a tile.
	dtCompressedTile* tile = 0;
	if (m_pNextFreeTile)
	{
		tile = m_pNextFreeTile;
		m_pNextFreeTile = tile->pNext;
		tile->pNext = 0;
	}
	
	// Make sure we could allocate a tile.
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// Insert tile into the position lut.
	int h = computeTileHash(header->nTileX, header->nTileY, m_nTileLutMask);
	tile->pNext = m_ppPosLookup[h];
	m_ppPosLookup[h] = tile;
	
	// Init tile.
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	tile->pHeader = (dtTileCacheLayerHeader*)data;
	tile->pData = data;
	tile->nDataSize = dataSize;
	tile->pCompressed = tile->pData + headerSize;
	tile->nCompressedSize = tile->nDataSize - headerSize;
	tile->uFlags = flags;
	
	if (result)
		*result = getTileRef(tile);
	
	return DT_SUCCESS;
}

dtStatus dtTileCache::removeTile(dtCompressedTileRef Ref, unsigned char** data, int* dataSize)
{
	if (!Ref)
		return DT_FAILURE | DT_INVALID_PARAM;

    unsigned int tileIndex = decodeTileIdTile(Ref);
	unsigned int tileSalt = decodeTileIdSalt(Ref);
	if ((int)tileIndex >= m_Params.nMaxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;

    dtCompressedTile* tile = &m_pCompressedTiles[tileIndex];
	if (tile->uSalt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Remove tile from hash lookup.
	const int h = computeTileHash(tile->pHeader->nTileX, tile->pHeader->nTileY, m_nTileLutMask);
	dtCompressedTile* prev = 0;
	dtCompressedTile* cur = m_ppPosLookup[h];
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->pNext = cur->pNext;
			else
				m_ppPosLookup[h] = cur->pNext;
			break;
		}
		prev = cur;
		cur = cur->pNext;
	}
	
	// Reset tile.
	if (tile->uFlags & DT_COMPRESSEDTILE_FREE_DATA)
	{
		// Owns data
		dtFree(tile->pData);
		tile->pData = 0;
		tile->nDataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	else
	{
		if (data) *data = tile->pData;
		if (dataSize) *dataSize = tile->nDataSize;
	}
	
	tile->pHeader = 0;
	tile->pData = 0;
	tile->nDataSize = 0;
	tile->pCompressed = 0;
	tile->nCompressedSize = 0;
	tile->uFlags = 0;
	
	// Update salt, salt should never be zero.
	tile->uSalt = (tile->uSalt + 1) & ((1 << m_uSaltBits) - 1);
	if (tile->uSalt == 0)
		tile->uSalt++;
	
	// Add to free list.
	tile->pNext = m_pNextFreeTile;
	m_pNextFreeTile = tile;
	
	return DT_SUCCESS;
}


dtObstacleRef dtTileCache::addObstacle(
	const float* pos, const float radius, const float height, dtObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	dtTileCacheObstacle* ob = 0;
	if (m_pNextFreeObstacle)
	{
		ob = m_pNextFreeObstacle;
		m_pNextFreeObstacle = ob->pNext;
		ob->pNext = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	unsigned short salt = ob->uSalt;
	memset(ob, 0, sizeof(dtTileCacheObstacle));
	ob->uSalt = salt;
	ob->cState = DT_OBSTACLE_PROCESSING;
	dtVcopy(ob->fPos, pos);
	ob->fRadius = radius;
	ob->fHeight = height;
	
	ObstacleRequest* req = &m_Reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->nAction = REQUEST_ADD;
	req->ref = getObstacleRef(ob);
	
	if (result)
		*result = req->ref;
	
	return DT_SUCCESS;
}

dtObstacleRef dtTileCache::removeObstacle(const dtObstacleRef Ref)
{
	if (!Ref)
		return DT_SUCCESS;
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	ObstacleRequest* req = &m_Reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->nAction = REQUEST_REMOVE;
	req->ref = Ref;
	
	return DT_SUCCESS;
}

//Which tiles require update ?
dtStatus dtTileCache::queryTiles(const float* bmin, const float* bmax,
	dtCompressedTileRef* results, int* resultCount, const int maxResults) const 
{
	const int MAX_TILES = 32;
	dtCompressedTileRef tiles[MAX_TILES];
	
	int n = 0;
	
	const float fTileWidth = m_Params.nWidth * m_Params.fCellSize;
	const float fTileHeight = m_Params.nHeight * m_Params.fCellSize;
	const int tx0 = (int)floorf((bmin[0] - m_Params.fOrigin[0]) / fTileWidth);
	const int tx1 = (int)floorf((bmax[0] - m_Params.fOrigin[0]) / fTileWidth);
	const int ty0 = (int)floorf((bmin[2] - m_Params.fOrigin[2]) / fTileHeight);
	const int ty1 = (int)floorf((bmax[2] - m_Params.fOrigin[2]) / fTileHeight);
	
	for (int ty = ty0; ty <= ty1; ++ty)
	{
		for (int tx = tx0; tx <= tx1; ++tx)
		{
			const int nTiles = getTilesAt(tx, ty, tiles, MAX_TILES);
			
			for (int i = 0; i < nTiles; ++i)
			{
				const dtCompressedTile* tile = &m_pCompressedTiles[decodeTileIdTile(tiles[i])];
				float tbmin[3], tbmax[3];
				calcTightTileBounds(tile->pHeader, tbmin, tbmax);
				
				if (dtOverlapBounds(bmin, bmax, tbmin, tbmax))
				{
					if (n < maxResults)
						results[n++] = tiles[i];
				}
			}
		}
	}
	
	*resultCount = n;
	
	return DT_SUCCESS;
}

dtStatus dtTileCache::update(const float /*dt*/, dtNavMesh* navmesh)
{
	if (m_nUpdate == 0)
	{
		// Process requests.
		for (int i = 0; i < m_nreqs; ++i)
		{
			ObstacleRequest* req = &m_Reqs[i];
			
			unsigned int idx = decodeObstacleIdObstacle(req->ref);
			if ((int)idx >= m_Params.nMaxObstacles)
				continue;

            dtTileCacheObstacle* ob = &m_pObstacles[idx];
			unsigned int salt = decodeObstacleIdSalt(req->ref);
			if (ob->uSalt != salt)
				continue;
			
			if (req->nAction == REQUEST_ADD)
			{
				// Find touched tiles.
				float fBMin[3], fBMax[3];
				getObstacleBounds(ob, fBMin, fBMax);

				int nTouched = 0;
				queryTiles(fBMin, fBMax, ob->Touched, &nTouched, DT_MAX_TOUCHED_TILES);
				ob->cTouched = (unsigned char)nTouched;
				// Add tiles to update list.
				ob->nPending = 0;
				for (int j = 0; j < ob->cTouched; ++j)
				{
					if (m_nUpdate < MAX_UPDATE)
					{
						if (!contains(m_Update, m_nUpdate, ob->Touched[j]))
							m_Update[m_nUpdate++] = ob->Touched[j];
						ob->Pending[ob->nPending++] = ob->Touched[j];
					}
				}
			}
			else if (req->nAction == REQUEST_REMOVE)
			{
				// Prepare to remove obstacle.
				ob->cState = DT_OBSTACLE_REMOVING;
				// Add tiles to update list.
				ob->nPending = 0;
				for (int j = 0; j < ob->cTouched; ++j)
				{
					if (m_nUpdate < MAX_UPDATE)
					{
						if (!contains(m_Update, m_nUpdate, ob->Touched[j]))
							m_Update[m_nUpdate++] = ob->Touched[j];
						ob->Pending[ob->nPending++] = ob->Touched[j];
					}
				}
			}
		}
		
		m_nreqs = 0;
	}
	
	// Process updates
	if (m_nUpdate)
	{
		// Build mesh
		const dtCompressedTileRef Ref = m_Update[0];
		dtStatus status = buildNavMeshTile(Ref, navmesh);
		m_nUpdate--;
		if (m_nUpdate > 0)
			memmove(m_Update, m_Update + 1, m_nUpdate * sizeof(dtCompressedTileRef));

		// Update obstacle states.
		for (int i = 0; i < m_Params.nMaxObstacles; ++i)
		{
			dtTileCacheObstacle* ob = &m_pObstacles[i];
			if (ob->cState == DT_OBSTACLE_PROCESSING || ob->cState == DT_OBSTACLE_REMOVING)
			{
				// Remove handled tile from pending list.
				for (int j = 0; j < (int)ob->nPending; j++)
				{
					if (ob->Pending[j] == Ref)
					{
						ob->Pending[j] = ob->Pending[(int)ob->nPending - 1];
						ob->nPending--;
						break;
					}
				}
				
				// If all pending tiles processed, change state.
				if (ob->nPending == 0)
				{
					if (ob->cState == DT_OBSTACLE_PROCESSING)
					{
						ob->cState = DT_OBSTACLE_PROCESSED;
					}
					else if (ob->cState == DT_OBSTACLE_REMOVING)
					{
						ob->cState = DT_OBSTACLE_EMPTY;
						// Update salt, salt should never be zero.
						ob->uSalt = (ob->uSalt+1) & ((1<<16)-1);
						if (ob->uSalt == 0)
							ob->uSalt++;
						// Return obstacle to free list.
						ob->pNext = m_pNextFreeObstacle;
						m_pNextFreeObstacle = ob;
					}
				}
			}
		}
			
		if (dtStatusFailed(status))
			return status;
	}
	
	return DT_SUCCESS;
}


dtStatus dtTileCache::buildNavMeshTilesAt(const int tx, const int ty, dtNavMesh* navmesh)
{
	const int MAX_TILES = 32;
	dtCompressedTileRef tiles[MAX_TILES];
	const int ntiles = getTilesAt(tx, ty, tiles, MAX_TILES);
	
	for (int i = 0; i < ntiles; ++i)
	{
		dtStatus status = buildNavMeshTile(tiles[i], navmesh);
		if (dtStatusFailed(status))
			return status;
	}
	
	return DT_SUCCESS;
}

dtStatus dtTileCache::buildNavMeshTile(const dtCompressedTileRef Ref, dtNavMesh* navmesh)
{	
	dtAssert(m_talloc);
	dtAssert(m_tcomp);
	
	unsigned int idx = decodeTileIdTile(Ref);
	if (idx > (unsigned int)m_Params.nMaxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	const dtCompressedTile* tile = &m_pCompressedTiles[idx];
	unsigned int salt = decodeTileIdSalt(Ref);
	if (tile->uSalt != salt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_talloc->reset();
	
	BuildContext bc(m_talloc);
	const int nWalkableClimbVx = (int)(m_Params.fWalkableClimb / m_Params.fCellHeight);
	dtStatus status;
	
	// Decompress tile layer data. 
	status = dtDecompressTileCacheLayer(m_talloc, m_tcomp, tile->pData, tile->nDataSize, &bc.layer);
	if (dtStatusFailed(status))
		return status;
	
	// Rasterize obstacles.
	for (int i = 0; i < m_Params.nMaxObstacles; ++i)
	{
		const dtTileCacheObstacle* ob = &m_pObstacles[i];
		if (ob->cState == DT_OBSTACLE_EMPTY || ob->cState == DT_OBSTACLE_REMOVING)
			continue;
		if (contains(ob->Touched, ob->cTouched, Ref))
		{
			dtMarkCylinderArea(*bc.layer, tile->pHeader->fBMin, m_Params.fCellSize, m_Params.fCellHeight,
							   ob->fPos, ob->fRadius, ob->fHeight, 0);
		}
	}
	
	// Build navmesh
	status = dtBuildTileCacheRegions(m_talloc, *bc.layer, nWalkableClimbVx);
	if (dtStatusFailed(status))
		return status;
	
	bc.lcset = dtAllocTileCacheContourSet(m_talloc);
	if (!bc.lcset)
		return status;
	status = dtBuildTileCacheContours(m_talloc, *bc.layer, nWalkableClimbVx,
									  m_Params.fMaxSimplificationError, *bc.lcset);
	if (dtStatusFailed(status))
		return status;
	
	bc.lmesh = dtAllocTileCachePolyMesh(m_talloc);
	if (!bc.lmesh)
		return status;
	status = dtBuildTileCachePolyMesh(m_talloc, *bc.lcset, *bc.lmesh);
	if (dtStatusFailed(status))
		return status;
	
	// Early out if the mesh tile is empty.
	if (!bc.lmesh->npolys)
		return DT_SUCCESS;
	
	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.pVerts = bc.lmesh->pVerts;
	params.nVertCount = bc.lmesh->nVerts;
	params.pPolys = bc.lmesh->pPolys;
	params.pPolyAreas = bc.lmesh->pAreaIDs;
	params.pPolyFlags = bc.lmesh->pFlags;
	params.nPolyCount = bc.lmesh->npolys;
	params.nMaxVertNumPerPoly = DT_VERTS_PER_POLYGON;
	params.fWalkableHeight = m_Params.fWalkableHeight;
	params.fWalkableRadius = m_Params.fWalkableRadius;
	params.fWalkableClimb = m_Params.fWalkableClimb;
	params.nTileX = tile->pHeader->nTileX;
	params.nTileY = tile->pHeader->nTileY;
	params.nTileLayer = tile->pHeader->nTileLayer;
	params.fCellSize = m_Params.fCellSize;
	params.fCellHeight = m_Params.fCellHeight;
	params.bBuildBvTree = false;
	dtVcopy(params.fBMin, tile->pHeader->fBMin);
	dtVcopy(params.fBMax, tile->pHeader->fBMax);
	
	if (m_tmproc)
	{
		m_tmproc->process(&params, bc.lmesh->pAreaIDs, bc.lmesh->pFlags);
	}
	
	unsigned char* navData = 0;
	int navDataSize = 0;
	if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		return DT_FAILURE;

	// Remove existing tile.
	navmesh->removeTile(navmesh->getTileRefAt(tile->pHeader->nTileX, tile->pHeader->nTileY, tile->pHeader->nTileLayer), 0, 0);

	// Add new tile, or leave the location empty.
	if (navData)
	{
		// Let the navmesh own the data.
		status = navmesh->addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, 0);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			return status;
		}
	}
	
	return DT_SUCCESS;
}

void dtTileCache::calcTightTileBounds(const dtTileCacheLayerHeader* header, float* bmin, float* bmax) const
{
	const float cs = m_Params.fCellSize;
	bmin[0] = header->fBMin[0] + header->cMinX * cs;
	bmin[1] = header->fBMin[1];
	bmin[2] = header->fBMin[2] + header->cMinY * cs;
	bmax[0] = header->fBMin[0] + (header->cMaxX + 1) * cs;
	bmax[1] = header->fBMax[1];
	bmax[2] = header->fBMin[2] + (header->cMaxY + 1) * cs;
}

void dtTileCache::getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const
{
	bmin[0] = ob->fPos[0] - ob->fRadius;
	bmin[1] = ob->fPos[1];
	bmin[2] = ob->fPos[2] - ob->fRadius;
	bmax[0] = ob->fPos[0] + ob->fRadius;
	bmax[1] = ob->fPos[1] + ob->fHeight;
	bmax[2] = ob->fPos[2] + ob->fRadius;	
}
