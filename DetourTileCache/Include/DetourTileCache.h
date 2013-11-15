#ifndef DETOURTILECACHE_H
#define DETOURTILECACHE_H

#include "DetourStatus.h"

typedef unsigned int dtObstacleRef;

typedef unsigned int dtCompressedTileRef;

/// Flags for addTile
enum dtCompressedTileFlags
{
	DT_COMPRESSEDTILE_FREE_DATA = 0x01,					///< Navmesh owns the tile memory and should free it.
};

struct dtCompressedTile
{
	unsigned int uSalt;						///< Counter describing modifications to the tile.
	struct dtTileCacheLayerHeader* pHeader;
	unsigned char* pCompressed;
	int nCompressedSize;
	unsigned char* pData;
	int nDataSize;
	unsigned int uFlags;
	dtCompressedTile* pNext;
};

enum ObstacleState
{
	DT_OBSTACLE_EMPTY,
	DT_OBSTACLE_PROCESSING,
	DT_OBSTACLE_PROCESSED,
	DT_OBSTACLE_REMOVING,
};

static const int DT_MAX_TOUCHED_TILES = 8;
struct dtTileCacheObstacle
{
	float fPos[3], fRadius, fHeight;
	dtCompressedTileRef Touched[DT_MAX_TOUCHED_TILES];
	dtCompressedTileRef Pending[DT_MAX_TOUCHED_TILES];
	unsigned short uSalt;
	unsigned char cState;
	unsigned char cTouched;
	unsigned char nPending;
	dtTileCacheObstacle* pNext;
};

struct dtTileCacheParams
{
	float fOrigin[3];
	float fCellSize, fCellHeight;
	int nWidth, nHeight;
	float fWalkableHeight;
	float fWalkableRadius;
	float fWalkableClimb;
	float fMaxSimplificationError;
	int nMaxTiles;
	int nMaxObstacles;
};

struct dtTileCacheMeshProcess
{
	virtual void process(struct dtNavMeshCreateParams* params,
						 unsigned char* polyAreas, unsigned short* polyFlags) = 0;
};


class dtTileCache
{
public:
	dtTileCache();
	~dtTileCache();
	
	struct dtTileCacheAlloc* getAlloc() { return m_talloc; }
	struct dtTileCacheCompressor* getCompressor() { return m_tcomp; }
	const dtTileCacheParams* getParams() const { return &m_Params; }
	
	inline int getTileCount() const { return m_Params.nMaxTiles; }
	inline const dtCompressedTile* getTile(const int i) const { return &m_pCompressedTiles[i]; }
	
	inline int getObstacleCount() const { return m_Params.nMaxObstacles; }
	inline const dtTileCacheObstacle* getObstacle(const int i) const { return &m_pObstacles[i]; }
	
	const dtTileCacheObstacle* getObstacleByRef(dtObstacleRef Ref);
	
	dtObstacleRef getObstacleRef(const dtTileCacheObstacle* obmin) const;
	
	dtStatus init(const dtTileCacheParams* params,
				  struct dtTileCacheAlloc* talloc,
				  struct dtTileCacheCompressor* tcomp,
				  struct dtTileCacheMeshProcess* tmproc);
	
	int getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const ;
	
	dtCompressedTile* getTileAt(const int tx, const int ty, const int tlayer);
	dtCompressedTileRef getTileRef(const dtCompressedTile* tile) const;
	const dtCompressedTile* getTileByRef(dtCompressedTileRef Ref) const;
	
	dtStatus addTile(unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result);
	
	dtStatus removeTile(dtCompressedTileRef Ref, unsigned char** data, int* dataSize);
	
	dtStatus addObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result);
	dtStatus removeObstacle(const dtObstacleRef Ref);
	
	dtStatus queryTiles(const float* bmin, const float* bmax,
						dtCompressedTileRef* results, int* resultCount, const int maxResults) const;
	
	dtStatus update(const float /*dt*/, class dtNavMesh* navmesh);
	
	dtStatus buildNavMeshTilesAt(const int tx, const int ty, class dtNavMesh* navmesh);
	
	dtStatus buildNavMeshTile(const dtCompressedTileRef Ref, class dtNavMesh* navmesh);
	
	void calcTightTileBounds(const struct dtTileCacheLayerHeader* header, float* bmin, float* bmax) const;
	
	void getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const;
	

	/// Encodes a tile id.
	inline dtCompressedTileRef encodeTileId(unsigned int salt, unsigned int it) const
	{
		return ((dtCompressedTileRef)salt << m_uTileBits) | (dtCompressedTileRef)it;
	}
	
	/// Decodes a tile salt.
	inline unsigned int decodeTileIdSalt(dtCompressedTileRef Ref) const
	{
		const dtCompressedTileRef saltMask = ((dtCompressedTileRef)1<<m_uSaltBits)-1;
		return (unsigned int)((Ref >> m_uTileBits) & saltMask);
	}
	
	/// Decodes a tile id.
	inline unsigned int decodeTileIdTile(dtCompressedTileRef Ref) const
	{
		const dtCompressedTileRef tileMask = ((dtCompressedTileRef)1<<m_uTileBits)-1;
		return (unsigned int)(Ref & tileMask);
	}

	/// Encodes an obstacle id.
	inline dtObstacleRef encodeObstacleId(unsigned int salt, unsigned int it) const
	{
		return ((dtObstacleRef)salt << 16) | (dtObstacleRef)it;
	}
	
	/// Decodes an obstacle salt.
	inline unsigned int decodeObstacleIdSalt(dtObstacleRef Ref) const
	{
		const dtObstacleRef saltMask = ((dtObstacleRef)1<<16)-1;
		return (unsigned int)((Ref >> 16) & saltMask);
	}
	
	/// Decodes an obstacle id.
	inline unsigned int decodeObstacleIdObstacle(dtObstacleRef Ref) const
	{
		const dtObstacleRef tileMask = ((dtObstacleRef)1<<16)-1;
		return (unsigned int)(Ref & tileMask);
	}
	
	
private:
	
	enum ObstacleRequestAction
	{
		REQUEST_ADD,
		REQUEST_REMOVE,
	};
	
	struct ObstacleRequest
	{
		int nAction;
		dtObstacleRef ref;
	};
	
	int m_nTileLutSize;						///< Tile hash lookup size (must be pot).
	int m_nTileLutMask;						///< Tile hash lookup mask.
	
	dtCompressedTile** m_ppPosLookup;			///< Tile hash lookup.
	dtCompressedTile* m_pNextFreeTile;		///< Freelist of tiles.
	dtCompressedTile* m_pCompressedTiles;				///< List of tiles.
	
	unsigned int m_uSaltBits;				///< Number of salt bits in the tile ID.
	unsigned int m_uTileBits;				///< Number of tile bits in the tile ID.
	
	dtTileCacheParams m_Params;
	
	dtTileCacheAlloc* m_talloc;
	dtTileCacheCompressor* m_tcomp;
	dtTileCacheMeshProcess* m_tmproc;
	
	dtTileCacheObstacle* m_pObstacles;
	dtTileCacheObstacle* m_pNextFreeObstacle;
	
	static const int MAX_REQUESTS = 64;
	ObstacleRequest m_Reqs[MAX_REQUESTS];
	int m_nreqs;
	
	static const int MAX_UPDATE = 64;
	dtCompressedTileRef m_Update[MAX_UPDATE];
	int m_nUpdate;	
};

dtTileCache* dtAllocTileCache();
void dtFreeTileCache(dtTileCache* tc);

#endif
