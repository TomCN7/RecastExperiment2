//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <new>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_TempObstacles.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourAssert.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "NavMeshTesterTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "fastlz.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
static const int EXPECTED_LAYERS_PER_TILE = 4;

static bool isectSegAABB(const float* sp, const float* sq,
    const float* amin, const float* amax, float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	rcVsub(d, sq, sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
	
	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) rcSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = gridWidth * gridHeight;
	return headerSize + gridSize * 4;
}

struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize * 1.05f);
	}
	
	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
							  unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}
	
	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
								unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	int capacity;
	int top;
	int high;
	
	LinearAllocator(const int cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}
	
	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const int cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}
	
	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}
	
	virtual void* alloc(const int size)
	{
		if (!buffer)
			return 0;
		if (top+size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}
	
	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct MeshProcess : public dtTileCacheMeshProcess
{
	InputGeom* m_pInputGeom;

	inline MeshProcess() : m_pInputGeom(0)
	{
	}

	inline void init(InputGeom* geom)
	{
		m_pInputGeom = geom;
	}
	
	virtual void process(struct dtNavMeshCreateParams* params,
						 unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->nPolyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = SAMPLE_POLYAREA_GROUND;

			if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
				polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
				polyAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		// Pass in off-mesh connections.
		if (m_pInputGeom)
		{
			params->pOffMeshConVerts = m_pInputGeom->getOffMeshConnectionVerts();
			params->pOffMeshConRad = m_pInputGeom->getOffMeshConnectionRads();
			params->pOffMeshConDir = m_pInputGeom->getOffMeshConnectionDirs();
			params->pOffMeshConAreas = m_pInputGeom->getOffMeshConnectionAreas();
			params->pOffMeshConFlags = m_pInputGeom->getOffMeshConnectionFlags();
			params->pOffMeshConUserID = m_pInputGeom->getOffMeshConnectionId();
			params->nOffMeshConCount = m_pInputGeom->getOffMeshConnectionCount();	
		}
	}
};

static const int MAX_LAYERS = 32;

struct TileCacheData
{
	unsigned char* pData;
	int nDataSize;
};

struct RasterizationContext
{
	RasterizationContext() :
		pSolidHeightField(0),
		pTriAreas(0),
		pHeightFieldLayerSet(0),
		pCompactHeightField(0),
		nTiles(0)
	{
		memset(Tiles, 0, sizeof(TileCacheData) * MAX_LAYERS);
	}
	
	~RasterizationContext()
	{
		rcFreeHeightField(pSolidHeightField);
		delete [] pTriAreas;
		rcFreeHeightfieldLayerSet(pHeightFieldLayerSet);
		rcFreeCompactHeightfield(pCompactHeightField);
		for (int i = 0; i < MAX_LAYERS; ++i)
		{
			dtFree(Tiles[i].pData);
			Tiles[i].pData = 0;
		}
	}
	
	rcHeightfield* pSolidHeightField;
	unsigned char* pTriAreas;
	rcHeightfieldLayerSet* pHeightFieldLayerSet;
	rcCompactHeightfield* pCompactHeightField;
	TileCacheData Tiles[MAX_LAYERS];
	int nTiles;
};

static int rasterizeTileLayers(BuildContext* pCtx, InputGeom* pInputGeom,
    const int tx, const int ty, const rcConfig& Cfg,
    TileCacheData* pTiles, const int nMaxTiles)
{
	if (!pInputGeom || !pInputGeom->getMesh() || !pInputGeom->getChunkyMesh())
	{
		pCtx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
		return 0;
	}
	
	FastLZCompressor comp;
	RasterizationContext rc;
	
	const float* pVerts = pInputGeom->getMesh()->getVerts();
	const int nVerts = pInputGeom->getMesh()->getVertCount();
	const rcChunkyTriMesh* pChunkyMesh = pInputGeom->getChunkyMesh();
	
	// Tile bounds.
	const float tcs = Cfg.nTileSize * Cfg.fCellSize;
	
	rcConfig tcfg;
	memcpy(&tcfg, &Cfg, sizeof(tcfg));

	tcfg.fBMin[0] = Cfg.fBMin[0] + tx * tcs;
	tcfg.fBMin[1] = Cfg.fBMin[1];
	tcfg.fBMin[2] = Cfg.fBMin[2] + ty * tcs;
	tcfg.fBMax[0] = Cfg.fBMin[0] + (tx + 1) * tcs;
	tcfg.fBMax[1] = Cfg.fBMax[1];
	tcfg.fBMax[2] = Cfg.fBMin[2] + (ty + 1) * tcs;
	tcfg.fBMin[0] -= tcfg.nBorderSize * tcfg.fCellSize;
	tcfg.fBMin[2] -= tcfg.nBorderSize * tcfg.fCellSize;
	tcfg.fBMax[0] += tcfg.nBorderSize * tcfg.fCellSize;
	tcfg.fBMax[2] += tcfg.nBorderSize * tcfg.fCellSize;
	
	// Allocate voxel heightfield where we rasterize our input data to.
	rc.pSolidHeightField = rcAllocHeightfield();
	if (!rc.pSolidHeightField)
	{
		pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(pCtx, *rc.pSolidHeightField, tcfg.nWidth, tcfg.nHeight, tcfg.fBMin, tcfg.fBMax, tcfg.fCellSize, tcfg.fCellHeight))
	{
		pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}
	
	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// an array which can hold the max number of triangles you need to process.
	rc.pTriAreas = new unsigned char[pChunkyMesh->maxTrisPerChunk];
	if (!rc.pTriAreas)
	{
		pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", pChunkyMesh->maxTrisPerChunk);
		return 0;
	}
	
	float tbmin[2], tbmax[2];
	tbmin[0] = tcfg.fBMin[0];
	tbmin[1] = tcfg.fBMin[2];
	tbmax[0] = tcfg.fBMax[0];
	tbmax[1] = tcfg.fBMax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(pChunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0; // empty
	}
	
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = pChunkyMesh->nodes[cid[i]];
		const int* tris = &pChunkyMesh->tris[node.i * 3];
		const int ntris = node.n;
		
		memset(rc.pTriAreas, 0, ntris*sizeof(unsigned char));
		rcMarkWalkableTriangles(pCtx, tcfg.fWalkableSlopeAngle,
								pVerts, nVerts, tris, ntris, rc.pTriAreas);
		
		rcRasterizeTriangles(pCtx, pVerts, nVerts, tris, rc.pTriAreas, ntris, *rc.pSolidHeightField, tcfg.nWalkableClimb);
	}
	
	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(pCtx, tcfg.nWalkableClimb, *rc.pSolidHeightField);
	rcFilterLedgeSpans(pCtx, tcfg.nWalkableHeight, tcfg.nWalkableClimb, *rc.pSolidHeightField);
	rcFilterWalkableLowHeightSpans(pCtx, tcfg.nWalkableHeight, *rc.pSolidHeightField);
	
	rc.pCompactHeightField = rcAllocCompactHeightfield();
	if (!rc.pCompactHeightField)
	{
		pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(pCtx, tcfg.nWalkableHeight, tcfg.nWalkableClimb, *rc.pSolidHeightField, *rc.pCompactHeightField))
	{
		pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}
	
	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(pCtx, tcfg.nWalkableRadius, *rc.pCompactHeightField))
	{
		pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}
	
	// (Optional) Mark areas.
	const ConvexVolume* vols = pInputGeom->getConvexVolumes();
	for (int i  = 0; i < pInputGeom->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(pCtx, vols[i].verts, vols[i].nverts,
							 vols[i].hmin, vols[i].hmax,
							 (unsigned char)vols[i].area, *rc.pCompactHeightField);
	}
	
	rc.pHeightFieldLayerSet = rcAllocHeightfieldLayerSet();
	if (!rc.pHeightFieldLayerSet)
	{
		pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
		return 0;
	}
	if (!rcBuildHeightfieldLayers(pCtx, *rc.pCompactHeightField, tcfg.nBorderSize, tcfg.nWalkableHeight, *rc.pHeightFieldLayerSet))
	{
		pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
		return 0;
	}
	
	rc.nTiles = 0;
	for (int i = 0; i < rcMin(rc.pHeightFieldLayerSet->nLayers, MAX_LAYERS); ++i)
	{
		TileCacheData* tile = &rc.Tiles[rc.nTiles++];
		const rcHeightfieldLayer* layer = &rc.pHeightFieldLayerSet->pHeightFieldLayers[i];
		
		// Store header
		dtTileCacheLayerHeader header;
		header.nMagic = DT_TILECACHE_MAGIC;
		header.nVersion = DT_TILECACHE_VERSION;
		
		// Tile layer location in the navmesh.
		header.nTileX = tx;
		header.nTileY = ty;
		header.nTileLayer = i;
		dtVcopy(header.fBMin, layer->fBMin);
		dtVcopy(header.fBMax, layer->fBMax);
		
		// Tile info.
		header.cWidth = (unsigned char)layer->nWidth;
		header.cHeight = (unsigned char)layer->nHeight;
		header.cMinX = (unsigned char)layer->nMinX;
		header.cMaxX = (unsigned char)layer->nMaxX;
		header.cMinY = (unsigned char)layer->nMinY;
		header.cMaxY = (unsigned char)layer->nMaxY;
		header.uHeightMin = (unsigned short)layer->nMinHeight;
		header.uHeightMax = (unsigned short)layer->nMaxHeight;

		dtStatus status = dtBuildTileCacheLayer(
            &comp, &header, layer->pHeights, layer->pAreas, layer->pConnections,
            &tile->pData, &tile->nDataSize);
		if (dtStatusFailed(status))
		{
			return 0;
		}
	}

	// Transfer ownership of tile data from build context to the caller.
	int n = 0;
	for (int i = 0; i < rcMin(rc.nTiles, nMaxTiles); ++i)
	{
		pTiles[n++] = rc.Tiles[i];
		rc.Tiles[i].pData = 0;
		rc.Tiles[i].nDataSize = 0;
	}
	
	return n;
}

void drawTiles(duDebugDraw* dd, dtTileCache* tc)
{
	unsigned int fcol[6];
	float bmin[3], bmax[3];

	for (int i = 0; i < tc->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tc->getTile(i);
		if (!tile->pHeader) continue;
		
		tc->calcTightTileBounds(tile->pHeader, bmin, bmax);
		
		const unsigned int col = duIntToCol(i, 64);
		duCalcBoxColors(fcol, col, col);
		duDebugDrawBox(
            dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], fcol);
	}
	
	for (int i = 0; i < tc->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tc->getTile(i);
		if (!tile->pHeader) continue;
		
		tc->calcTightTileBounds(tile->pHeader, bmin, bmax);
		
		const unsigned int col = duIntToCol(i,255);
		const float pad = tc->getParams()->fCellSize * 0.1f;
		duDebugDrawBoxWire(
            dd, bmin[0] - pad, bmin[1] - pad, bmin[2] - pad,
			bmax[0] + pad, bmax[1] + pad, bmax[2] + pad, col, 2.0f);
	}
}

enum DrawDetailType
{
	DRAWDETAIL_AREAS,
	DRAWDETAIL_REGIONS,
	DRAWDETAIL_CONTOURS,
	DRAWDETAIL_MESH,
};

void drawDetail(duDebugDraw* dd, dtTileCache* tc, const int tx, const int ty, int type)
{
	struct TileCacheBuildContext
	{
		inline TileCacheBuildContext(struct dtTileCacheAlloc* a) 
            : layer(0), lcset(0), lmesh(0), alloc(a) {}
		inline ~TileCacheBuildContext() { purge(); }
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

	dtCompressedTileRef Tiles[MAX_LAYERS];
	const int nTiles = tc->getTilesAt(tx, ty, Tiles, MAX_LAYERS);

	dtTileCacheAlloc* talloc = tc->getAlloc();
	dtTileCacheCompressor* tcomp = tc->getCompressor();
	const dtTileCacheParams* params = tc->getParams();

	for (int i = 0; i < nTiles; ++i)
	{
		const dtCompressedTile* pTile = tc->getTileByRef(Tiles[i]);

		talloc->reset();

		TileCacheBuildContext bc(talloc);
		const int walkableClimbVx = (int)(params->fWalkableClimb / params->fCellHeight);
		dtStatus status;
		
		// Decompress tile layer data. 
		status = dtDecompressTileCacheLayer(talloc, tcomp, pTile->pData, pTile->nDataSize, &bc.layer);
		if (dtStatusFailed(status))
			return;
		if (type == DRAWDETAIL_AREAS)
		{
			duDebugDrawTileCacheLayerAreas(dd, *bc.layer, params->fCellSize, params->fCellHeight);
			continue;
		}

		// Build navmesh
		status = dtBuildTileCacheRegions(talloc, *bc.layer, walkableClimbVx);
		if (dtStatusFailed(status))
			return;
		if (type == DRAWDETAIL_REGIONS)
		{
			duDebugDrawTileCacheLayerRegions(dd, *bc.layer, params->fCellSize, params->fCellHeight);
			continue;
		}
		
		bc.lcset = dtAllocTileCacheContourSet(talloc);
		if (!bc.lcset)
			return;
		status = dtBuildTileCacheContours(talloc, *bc.layer, walkableClimbVx,
										  params->fMaxSimplificationError, *bc.lcset);
		if (dtStatusFailed(status))
			return;
		if (type == DRAWDETAIL_CONTOURS)
		{
			duDebugDrawTileCacheContours(dd, *bc.lcset, pTile->pHeader->fBMin, params->fCellSize, params->fCellHeight);
			continue;
		}
		
		bc.lmesh = dtAllocTileCachePolyMesh(talloc);
		if (!bc.lmesh)
			return;
		status = dtBuildTileCachePolyMesh(talloc, *bc.lcset, *bc.lmesh);
		if (dtStatusFailed(status))
			return;

		if (type == DRAWDETAIL_MESH)
		{
			duDebugDrawTileCachePolyMesh(dd, *bc.lmesh, pTile->pHeader->fBMin, params->fCellSize, params->fCellHeight);
			continue;
		}
	}
}

void drawDetailOverlay(const dtTileCache* tc, const int tx, const int ty, double* proj, double* model, int* view)
{
	dtCompressedTileRef Tiles[MAX_LAYERS];
	const int ntiles = tc->getTilesAt(tx,ty,Tiles,MAX_LAYERS);
	if (!ntiles)
		return;
	
	const int rawSize = calcLayerBufferSize(tc->getParams()->nWidth, tc->getParams()->nHeight);
	
	char text[128];

	for (int i = 0; i < ntiles; ++i)
	{
		const dtCompressedTile* tile = tc->getTileByRef(Tiles[i]);
		
		float pos[3];
		pos[0] = (tile->pHeader->fBMin[0] + tile->pHeader->fBMax[0]) / 2.0f;
		pos[1] = tile->pHeader->fBMin[1];
		pos[2] = (tile->pHeader->fBMin[2] + tile->pHeader->fBMax[2]) / 2.0f;
		
		GLdouble x, y, z;
		if (gluProject((GLdouble)pos[0], (GLdouble)pos[1], (GLdouble)pos[2],
					   model, proj, view, &x, &y, &z))
		{
			snprintf(text,128,"(%d,%d)/%d", tile->pHeader->nTileX,tile->pHeader->nTileY,tile->pHeader->nTileLayer);
			imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
			snprintf(text,128,"Compressed: %.1f kB", tile->nDataSize/1024.0f);
			imguiDrawText((int)x, (int)y-45, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,128));
			snprintf(text,128,"Raw:%.1fkB", rawSize/1024.0f);
			imguiDrawText((int)x, (int)y-65, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,128));
		}
	}
}
		
dtObstacleRef hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq)
{
	float tmin = FLT_MAX;
	const dtTileCacheObstacle* obmin = 0;
	for (int i = 0; i < tc->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = tc->getObstacle(i);
		if (ob->cState == DT_OBSTACLE_EMPTY)
			continue;
		
		float bmin[3], bmax[3], t0,t1;
		tc->getObstacleBounds(ob, bmin,bmax);
		
		if (isectSegAABB(sp,sq, bmin,bmax, t0,t1))
		{
			if (t0 < tmin)
			{
				tmin = t0;
				obmin = ob;
			}
		}
	}
	return tc->getObstacleRef(obmin);
}
	
void drawObstacles(duDebugDraw* dd, const dtTileCache* tc)
{
	// Draw obstacles
	for (int i = 0; i < tc->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = tc->getObstacle(i);
		if (ob->cState == DT_OBSTACLE_EMPTY) continue;
		float bmin[3], bmax[3];
		tc->getObstacleBounds(ob, bmin,bmax);

		unsigned int col = 0;
		if (ob->cState == DT_OBSTACLE_PROCESSING)
			col = duRGBA(255,255,0,128);
		else if (ob->cState == DT_OBSTACLE_PROCESSED)
			col = duRGBA(255,192,0,192);
		else if (ob->cState == DT_OBSTACLE_REMOVING)
			col = duRGBA(220,0,0,128);

		duDebugDrawCylinder(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], col);
		duDebugDrawCylinderWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duDarkenCol(col), 2);
	}
}

class TempObstacleHilightTool : public SampleTool
{
	Sample_TempObstacles* m_pSample;
	float m_fHitPos[3];
	bool m_bHitPosSet;
	float m_fAgentRadius;
	int m_eDrawType;
	
public:

	TempObstacleHilightTool() :
		m_pSample(0),
		m_bHitPosSet(false),
		m_fAgentRadius(0),
		m_eDrawType(DRAWDETAIL_AREAS)
	{
		m_fHitPos[0] = m_fHitPos[1] = m_fHitPos[2] = 0;
	}

	virtual ~TempObstacleHilightTool()
	{
	}

	virtual int type() { return TOOL_TILE_HIGHLIGHT; }

	virtual void init(Sample* sample)
	{
		m_pSample = (Sample_TempObstacles*)sample; 
	}
	
	virtual void reset() {}

	virtual void handleMenu()
	{
		imguiLabel("Highlight Tile Cache");
		imguiValue("Click LMB to highlight a tile.");
		imguiSeparator();
		if (imguiCheck("Draw Areas", m_eDrawType == DRAWDETAIL_AREAS))
			m_eDrawType = DRAWDETAIL_AREAS;
		if (imguiCheck("Draw Regions", m_eDrawType == DRAWDETAIL_REGIONS))
			m_eDrawType = DRAWDETAIL_REGIONS;
		if (imguiCheck("Draw Contours", m_eDrawType == DRAWDETAIL_CONTOURS))
			m_eDrawType = DRAWDETAIL_CONTOURS;
		if (imguiCheck("Draw Mesh", m_eDrawType == DRAWDETAIL_MESH))
			m_eDrawType = DRAWDETAIL_MESH;
	}

	virtual void handleClick(const float* /*s*/, const float* p, bool /*shift*/)
	{
		m_bHitPosSet = true;
		rcVcopy(m_fHitPos,p);
	}

	virtual void handleToggle() {}

	virtual void handleStep() {}

	virtual void handleUpdate(const float /*dt*/) {}
	
	virtual void handleRender()
	{
		if (m_bHitPosSet)
		{
			const float s = m_pSample->getAgentRadius();
			glColor4ub(0, 0, 0, 128);
			glLineWidth(2.0f);
			glBegin(GL_LINES);
			glVertex3f(m_fHitPos[0] - s, m_fHitPos[1] + 0.1f, m_fHitPos[2]);
			glVertex3f(m_fHitPos[0] + s, m_fHitPos[1] + 0.1f, m_fHitPos[2]);
			glVertex3f(m_fHitPos[0], m_fHitPos[1] - s + 0.1f, m_fHitPos[2]);
			glVertex3f(m_fHitPos[0], m_fHitPos[1] + s + 0.1f, m_fHitPos[2]);
			glVertex3f(m_fHitPos[0], m_fHitPos[1] + 0.1f, m_fHitPos[2] - s);
			glVertex3f(m_fHitPos[0], m_fHitPos[1] + 0.1f, m_fHitPos[2] + s);
			glEnd();
			glLineWidth(1.0f);
			
			if (m_pSample)
			{
				int tx = 0, ty = 0;
				m_pSample->getTilePos(m_fHitPos, tx, ty);
				m_pSample->renderCachedTile(tx, ty, m_eDrawType);
			}
		}
	}
	
	virtual void handleRenderOverlay(double* proj, double* model, int* view)
	{
		if (m_bHitPosSet)
		{
			if (m_pSample)
			{
				int tx=0, ty=0;
				m_pSample->getTilePos(m_fHitPos, tx, ty);
				m_pSample->renderCachedTileOverlay(tx,ty,proj,model,view);
			}
		}		
	}
};

class TempObstacleCreateTool : public SampleTool
{
	Sample_TempObstacles* m_pSample;
	
public:
	
	TempObstacleCreateTool()
	{
	}
	
	virtual ~TempObstacleCreateTool()
	{
	}
	
	virtual int type() { return TOOL_TEMP_OBSTACLE; }
	
	virtual void init(Sample* sample)
	{
		m_pSample = (Sample_TempObstacles*)sample; 
	}
	
	virtual void reset() {}
	
	virtual void handleMenu()
	{
		imguiLabel("Create Temp Obstacles");
		
		if (imguiButton("Remove All"))
			m_pSample->clearAllTempObstacles();
		
		imguiSeparator();

		imguiValue("Click LMB to create an obstacle.");
		imguiValue("Shift+LMB to remove an obstacle.");
	}
	
	virtual void handleClick(const float* s, const float* p, bool shift)
	{
		if (m_pSample)
		{
			if (shift)
				m_pSample->removeTempObstacle(s,p);
			else
				m_pSample->addTempObstacle(p);
		}
	}
	
	virtual void handleToggle() {}
	virtual void handleStep() {}
	virtual void handleUpdate(const float /*dt*/) {}
	virtual void handleRender() {}
	virtual void handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) { }
};

Sample_TempObstacles::Sample_TempObstacles() :
	m_bKeepInterResults(false),
	m_pTileCache(0),
	m_fCacheBuildTimeMs(0),
	m_nCacheCompressedSize(0),
	m_nCacheRawSize(0),
	m_nCacheLayerCount(0),
	m_nCacheBuildMemUsage(0),
	m_eDrawMode(DRAWMODE_NAVMESH),
	m_nMaxTiles(0),
	m_nMaxPolysPerTile(0),
	m_fTileSize(48)
{
	resetCommonSettings();
	
	m_talloc = new LinearAllocator(32000);
	m_tcomp = new FastLZCompressor;
	m_tmproc = new MeshProcess;
	
	setTool(new TempObstacleCreateTool);
}

Sample_TempObstacles::~Sample_TempObstacles()
{
	dtFreeNavMesh(m_pNavMesh);
	m_pNavMesh = 0;
	dtFreeTileCache(m_pTileCache);
}

void Sample_TempObstacles::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_bKeepInterResults))
		m_bKeepInterResults = !m_bKeepInterResults;

	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_fTileSize, 16.0f, 128.0f, 8.0f);
	
	int gridSize = 1;
	if (m_pInputGeom)
	{
		const float* bmin = m_pInputGeom->getMeshBoundsMin();
		const float* bmax = m_pInputGeom->getMeshBoundsMax();
		char text[64];
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_fCellSize, &gw, &gh);
		const int ts = (int)m_fTileSize;
		const int tw = (gw + ts - 1) / ts;
		const int th = (gh + ts - 1) / ts;
		snprintf(text, 64, "Tiles  %d x %d", tw, th);
		imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;
		m_nMaxTiles = 1 << tileBits;
		m_nMaxPolysPerTile = 1 << polyBits;
		snprintf(text, 64, "Max Tiles  %d", m_nMaxTiles);
		imguiValue(text);
		snprintf(text, 64, "Max Polys  %d", m_nMaxPolysPerTile);
		imguiValue(text);
		gridSize = tw*th;
	}
	else
	{
		m_nMaxTiles = 0;
		m_nMaxPolysPerTile = 0;
	}
	
	imguiSeparator();
	
	imguiLabel("Tile Cache");
	char msg[64];

	const float compressionRatio = (float)m_nCacheCompressedSize / (float)(m_nCacheRawSize + 1);
	
	snprintf(msg, 64, "Layers  %d", m_nCacheLayerCount);
	imguiValue(msg);
	snprintf(msg, 64, "Layers (per tile)  %.1f", (float)m_nCacheLayerCount / (float)gridSize);
	imguiValue(msg);
	
	snprintf(msg, 64, "Memory  %.1f kB / %.1f kB (%.1f%%)", 
        m_nCacheCompressedSize / 1024.0f, m_nCacheRawSize / 1024.0f, compressionRatio * 100.0f);
	imguiValue(msg);
	snprintf(msg, 64, "Navmesh Build Time  %.1f ms", m_fCacheBuildTimeMs);
	imguiValue(msg);
	snprintf(msg, 64, "Build Peak Mem Usage  %.1f kB", m_nCacheBuildMemUsage / 1024.0f);
	imguiValue(msg);
	
	imguiSeparator();
}

void Sample_TempObstacles::handleTools()
{
	int type = !m_pTool ? TOOL_NONE : m_pTool->type();

	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Highlight Tile Cache", type == TOOL_TILE_HIGHLIGHT))
	{
		setTool(new TempObstacleHilightTool);
	}
	if (imguiCheck("Create Temp Obstacles", type == TOOL_TEMP_OBSTACLE))
	{
		setTool(new TempObstacleCreateTool);
	}
	if (imguiCheck("Create Off-Mesh Links", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (imguiCheck("Create Crowds", type == TOOL_CROWD))
	{
		setTool(new CrowdTool);
	}
	
	imguiSeparatorLine();

	imguiIndent();

	if (m_pTool)
		m_pTool->handleMenu();

	imguiUnindent();
}

void Sample_TempObstacles::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;
	
	if (m_pInputGeom)
	{
		valid[DRAWMODE_NAVMESH] = m_pNavMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_pNavMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_pNavMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_pNavQuery != 0;
		valid[DRAWMODE_NAVMESH_PORTALS] = m_pNavMesh != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_pNavMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_CACHE_BOUNDS] = true;
	}
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;
	
	if (unavail == MAX_DRAWMODE)
		return;
	
	imguiLabel("Draw");
	if (imguiCheck("Input Mesh", m_eDrawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_eDrawMode = DRAWMODE_MESH;
	if (imguiCheck("Navmesh", m_eDrawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_eDrawMode = DRAWMODE_NAVMESH;
	if (imguiCheck("Navmesh Invis", m_eDrawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_eDrawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("Navmesh Trans", m_eDrawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_eDrawMode = DRAWMODE_NAVMESH_TRANS;
	if (imguiCheck("Navmesh BVTree", m_eDrawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_eDrawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Navmesh Nodes", m_eDrawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_eDrawMode = DRAWMODE_NAVMESH_NODES;
	if (imguiCheck("Navmesh Portals", m_eDrawMode == DRAWMODE_NAVMESH_PORTALS, valid[DRAWMODE_NAVMESH_PORTALS]))
		m_eDrawMode = DRAWMODE_NAVMESH_PORTALS;
	if (imguiCheck("Cache Bounds", m_eDrawMode == DRAWMODE_CACHE_BOUNDS, valid[DRAWMODE_CACHE_BOUNDS]))
		m_eDrawMode = DRAWMODE_CACHE_BOUNDS;
	
	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("rebuild some tiles to see");
		imguiValue("more debug mode options.");
	}
}

void Sample_TempObstacles::handleRender()
{
	if (!m_pInputGeom || !m_pInputGeom->getMesh())
		return;
	
	DebugDrawGL dd;

	const float texScale = 1.0f / (m_fCellSize * 10.0f);
	
	// Draw mesh
	if (m_eDrawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&dd, m_pInputGeom->getMesh()->getVerts(), m_pInputGeom->getMesh()->getVertCount(),
								m_pInputGeom->getMesh()->getTris(), m_pInputGeom->getMesh()->getNormals(), m_pInputGeom->getMesh()->getTriCount(),
								m_fAgentMaxSlope, texScale);
		m_pInputGeom->drawOffMeshConnections(&dd);
	}
	
	if (m_pTileCache && m_eDrawMode == DRAWMODE_CACHE_BOUNDS)
		drawTiles(&dd, m_pTileCache);
	
	if (m_pTileCache)
		drawObstacles(&dd, m_pTileCache);
	
	
	glDepthMask(GL_FALSE);
	
	// Draw bounds
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();
	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
	
	// Tiling grid.
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_fCellSize, &gw, &gh);
	const int tw = (gw + (int)m_fTileSize-1) / (int)m_fTileSize;
	const int th = (gh + (int)m_fTileSize-1) / (int)m_fTileSize;
	const float s = m_fTileSize*m_fCellSize;
	duDebugDrawGridXZ(&dd, bmin[0],bmin[1],bmin[2], tw,th, s, duRGBA(0,0,0,64), 1.0f);
		
	if (m_pNavMesh && m_pNavQuery &&
		(m_eDrawMode == DRAWMODE_NAVMESH ||
		 m_eDrawMode == DRAWMODE_NAVMESH_TRANS ||
		 m_eDrawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_eDrawMode == DRAWMODE_NAVMESH_NODES ||
		 m_eDrawMode == DRAWMODE_NAVMESH_PORTALS ||
		 m_eDrawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_eDrawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&dd, *m_pNavMesh, *m_pNavQuery, m_cNavMeshDrawFlags/*|DU_DRAWNAVMESH_COLOR_TILES*/);
		if (m_eDrawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&dd, *m_pNavMesh);
		if (m_eDrawMode == DRAWMODE_NAVMESH_PORTALS)
			duDebugDrawNavMeshPortals(&dd, *m_pNavMesh);
		if (m_eDrawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&dd, *m_pNavQuery);
		duDebugDrawNavMeshPolysWithFlags(&dd, *m_pNavMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}
	
	
	glDepthMask(GL_TRUE);
		
	m_pInputGeom->drawConvexVolumes(&dd);
	
	if (m_pTool)
		m_pTool->handleRender();
	renderToolStates();
	
	glDepthMask(GL_TRUE);
}

void Sample_TempObstacles::renderCachedTile(const int tx, const int ty, const int type)
{
	DebugDrawGL dd;
	if (m_pTileCache)
		drawDetail(&dd,m_pTileCache,tx,ty,type);
}

void Sample_TempObstacles::renderCachedTileOverlay(const int tx, const int ty, double* proj, double* model, int* view)
{
	if (m_pTileCache)
		drawDetailOverlay(m_pTileCache, tx, ty, proj, model, view);
}

void Sample_TempObstacles::handleRenderOverlay(double* proj, double* model, int* view)
{	
	if (m_pTool)
		m_pTool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);

	// Stats
/*	imguiDrawRect(280,10,300,100,imguiRGBA(0,0,0,64));
	
	char text[64];
	int y = 110-30;
	
	snprintf(text,64,"Lean Data: %.1fkB", m_tileCache->getRawSize()/1024.0f);
	imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	y -= 20;
	
	snprintf(text,64,"Compressed: %.1fkB (%.1f%%)", m_tileCache->getCompressedSize()/1024.0f,
			 m_tileCache->getRawSize() > 0 ? 100.0f*(float)m_tileCache->getCompressedSize()/(float)m_tileCache->getRawSize() : 0);
	imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	y -= 20;

	if (m_rebuildTileCount > 0 && m_rebuildTime > 0.0f)
	{
		snprintf(text,64,"Changed obstacles, rebuild %d tiles: %.3f ms", m_rebuildTileCount, m_rebuildTime);
		imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,192,0,255));
		y -= 20;
	}
	*/
}

void Sample_TempObstacles::handleMeshChanged(class InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	dtFreeTileCache(m_pTileCache);
	m_pTileCache = 0;
	
	dtFreeNavMesh(m_pNavMesh);
	m_pNavMesh = 0;

	if (m_pTool)
	{
		m_pTool->reset();
		m_pTool->init(this);
		m_tmproc->init(m_pInputGeom);
	}
	resetToolStates();
	initToolStates(this);
}

void Sample_TempObstacles::addTempObstacle(const float* pos)
{
	if (!m_pTileCache)
		return;

    float p[3];
	dtVcopy(p, pos);
	p[1] -= 0.5f;
	m_pTileCache->addObstacle(p, 1.0f, 2.0f, 0);
}

void Sample_TempObstacles::removeTempObstacle(const float* sp, const float* sq)
{
	if (!m_pTileCache)
		return;

    dtObstacleRef Ref = hitTestObstacle(m_pTileCache, sp, sq);
	m_pTileCache->removeObstacle(Ref);
}

void Sample_TempObstacles::clearAllTempObstacles()
{
	if (!m_pTileCache)
		return;

	for (int i = 0; i < m_pTileCache->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = m_pTileCache->getObstacle(i);
		if (ob->cState == DT_OBSTACLE_EMPTY) continue;
		m_pTileCache->removeObstacle(m_pTileCache->getObstacleRef(ob));
	}
}

bool Sample_TempObstacles::handleBuild()
{
	dtStatus status;
	
	if (!m_pInputGeom || !m_pInputGeom->getMesh())
	{
		m_pCtx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	m_tmproc->init(m_pInputGeom);
	
	// Init cache
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_fCellSize, &gw, &gh);
	const int ts = (int)m_fTileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	// Generation params.
	rcConfig Cfg;
	memset(&Cfg, 0, sizeof(Cfg));
	Cfg.fCellSize = m_fCellSize;
	Cfg.fCellHeight = m_fCellHeight;
	Cfg.fWalkableSlopeAngle = m_fAgentMaxSlope;
	Cfg.nWalkableHeight = (int)ceilf(m_fAgentHeight / Cfg.fCellHeight);
	Cfg.nWalkableClimb = (int)floorf(m_fAgentMaxClimb / Cfg.fCellHeight);
	Cfg.nWalkableRadius = (int)ceilf(m_fAgentRadius / Cfg.fCellSize);
	Cfg.nMaxEdgeLen = (int)(m_fEdgeMaxLen / m_fCellSize);
	Cfg.fMaxSimplificationError = m_fEdgeMaxError;
	Cfg.nMinRegionArea = (int)rcSqr(m_fRegionMinSize);		// Note: area = size*size
	Cfg.nMergeRegionArea = (int)rcSqr(m_fRegionMergeSize);	// Note: area = size*size
	Cfg.nMaxVertsPerPoly = (int)m_fVertsPerPoly;
	Cfg.nTileSize = (int)m_fTileSize;
	Cfg.nBorderSize = Cfg.nWalkableRadius + 3; // Reserve enough padding.
	Cfg.nWidth = Cfg.nTileSize + Cfg.nBorderSize * 2;
	Cfg.nHeight = Cfg.nTileSize + Cfg.nBorderSize * 2;
	Cfg.fDetailSampleDist = m_fDetailSampleDist < 0.9f ? 0 : m_fCellSize * m_fDetailSampleDist;
	Cfg.fDetailSampleMaxError = m_fCellHeight * m_fDetailSampleMaxError;
	rcVcopy(Cfg.fBMin, bmin);
	rcVcopy(Cfg.fBMax, bmax);
	
	// Tile cache params.
	dtTileCacheParams tcparams;
	memset(&tcparams, 0, sizeof(tcparams));
	rcVcopy(tcparams.fOrigin, bmin);
	tcparams.fCellSize = m_fCellSize;
	tcparams.fCellHeight = m_fCellHeight;
	tcparams.nWidth = (int)m_fTileSize;
	tcparams.nHeight = (int)m_fTileSize;
	tcparams.fWalkableHeight = m_fAgentHeight;
	tcparams.fWalkableRadius = m_fAgentRadius;
	tcparams.fWalkableClimb = m_fAgentMaxClimb;
	tcparams.fMaxSimplificationError = m_fEdgeMaxError;
	tcparams.nMaxTiles = tw * th * EXPECTED_LAYERS_PER_TILE;
	tcparams.nMaxObstacles = 128;

	dtFreeTileCache(m_pTileCache);
	
	m_pTileCache = dtAllocTileCache();
	if (!m_pTileCache)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
		return false;
	}
	status = m_pTileCache->init(&tcparams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
		return false;
	}
	
	dtFreeNavMesh(m_pNavMesh);
	
	m_pNavMesh = dtAllocNavMesh();
	if (!m_pNavMesh)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams Params;
	memset(&Params, 0, sizeof(Params));
	rcVcopy(Params.fOrigin, m_pInputGeom->getMeshBoundsMin());
	Params.fTileWidth = m_fTileSize * m_fCellSize;
	Params.fTileHeight = m_fTileSize * m_fCellSize;
	Params.nMaxTiles = m_nMaxTiles;
	Params.nMaxPolys = m_nMaxPolysPerTile;
	
	status = m_pNavMesh->init(&Params);
	if (dtStatusFailed(status))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}
	
	status = m_pNavQuery->init(m_pNavMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}
	
	// Preprocess tiles.
	
	m_pCtx->resetTimers();
	
	m_nCacheLayerCount = 0;
	m_nCacheCompressedSize = 0;
	m_nCacheRawSize = 0;
	
	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			TileCacheData Tiles[MAX_LAYERS];
			memset(Tiles, 0, sizeof(Tiles));
			int nTiles = rasterizeTileLayers(m_pCtx, m_pInputGeom, x, y, Cfg, Tiles, MAX_LAYERS);

			for (int i = 0; i < nTiles; ++i)
			{
				TileCacheData* pTile = &Tiles[i];
				status = m_pTileCache->addTile(pTile->pData, pTile->nDataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(pTile->pData);
					pTile->pData = 0;
					continue;
				}
				
				m_nCacheLayerCount++;
				m_nCacheCompressedSize += pTile->nDataSize;
				m_nCacheRawSize += calcLayerBufferSize(tcparams.nWidth, tcparams.nHeight);
			}
		}
	}

	// Build initial meshes
	m_pCtx->startTimer(RC_TIMER_TOTAL);
	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			m_pTileCache->buildNavMeshTilesAt(x, y, m_pNavMesh);
	m_pCtx->stopTimer(RC_TIMER_TOTAL);
	
	m_fCacheBuildTimeMs = m_pCtx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
	m_nCacheBuildMemUsage = m_talloc->high;

	const dtNavMesh* pNav = m_pNavMesh;
	int nNavMeshMemUsage = 0;
	for (int i = 0; i < pNav->getMaxTiles(); ++i)
	{
		const dtMeshTile* pTile = pNav->getTile(i);
		if (pTile->pHeader)
			nNavMeshMemUsage += pTile->nDataSize;
	}
	printf("navmeshMemUsage = %.1f kB", nNavMeshMemUsage / 1024.0f);
		
	if (m_pTool)
		m_pTool->init(this);
	initToolStates(this);

	return true;
}

void Sample_TempObstacles::handleUpdate(const float dt)
{
	Sample::handleUpdate(dt);
	
	if (!m_pNavMesh)
		return;
	if (!m_pTileCache)
		return;
	
	m_pTileCache->update(dt, m_pNavMesh);
}

void Sample_TempObstacles::getTilePos(const float* pos, int& tx, int& ty)
{
	if (!m_pInputGeom) return;
	
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	
	const float ts = m_fTileSize*m_fCellSize;
	tx = (int)((pos[0] - bmin[0]) / ts);
	ty = (int)((pos[2] - bmin[2]) / ts);
}
