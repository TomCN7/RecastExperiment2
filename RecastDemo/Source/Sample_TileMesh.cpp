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
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_TileMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "NavMeshPruneTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"


#ifdef WIN32
#	define snprintf _snprintf
#endif


inline unsigned int nextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int ilog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

class NavMeshTileTool : public SampleTool
{
	Sample_TileMesh* m_sample;
	float m_hitPos[3];
	bool m_hitPosSet;
	float m_agentRadius;
	
public:

	NavMeshTileTool() :
		m_sample(0),
		m_hitPosSet(false),
		m_agentRadius(0)
	{
		m_hitPos[0] = m_hitPos[1] = m_hitPos[2] = 0;
	}

	virtual ~NavMeshTileTool()
	{
	}

	virtual int type() { return TOOL_TILE_EDIT; }

	virtual void init(Sample* sample)
	{
		m_sample = (Sample_TileMesh*)sample; 
	}
	
	virtual void reset() {}

	virtual void handleMenu()
	{
		imguiLabel("Create Tiles");
		if (imguiButton("Create All"))
		{
			if (m_sample)
				m_sample->buildAllTiles();
		}
		if (imguiButton("Remove All"))
		{
			if (m_sample)
				m_sample->removeAllTiles();
		}
	}

	virtual void handleClick(const float* /*s*/, const float* p, bool shift)
	{
		m_hitPosSet = true;
		rcVcopy(m_hitPos,p);
		if (m_sample)
		{
			if (shift)
				m_sample->removeTile(m_hitPos);
			else
				m_sample->buildTile(m_hitPos);
		}
	}

	virtual void handleToggle() {}

	virtual void handleStep() {}

	virtual void handleUpdate(const float /*dt*/) {}
	
	virtual void handleRender()
	{
		if (m_hitPosSet)
		{
			const float s = m_sample->getAgentRadius();
			glColor4ub(0,0,0,128);
			glLineWidth(2.0f);
			glBegin(GL_LINES);
			glVertex3f(m_hitPos[0]-s,m_hitPos[1]+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0]+s,m_hitPos[1]+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]-s+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]+s+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]-s);
			glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]+s);
			glEnd();
			glLineWidth(1.0f);
		}
	}
	
	virtual void handleRenderOverlay(double* proj, double* model, int* view)
	{
		GLdouble x, y, z;
		if (m_hitPosSet && gluProject((GLdouble)m_hitPos[0], (GLdouble)m_hitPos[1], (GLdouble)m_hitPos[2],
									  model, proj, view, &x, &y, &z))
		{
			int tx=0, ty=0;
			m_sample->getTilePos(m_hitPos, tx, ty);
			char text[32];
			snprintf(text,32,"(%d,%d)", tx,ty);
			imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
		}
		
		// Tool help
		const int h = view[3];
		imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Rebuild hit tile.  Shift+LMB: Clear hit tile.", imguiRGBA(255,255,255,192));	
	}
};




Sample_TileMesh::Sample_TileMesh() :
	m_bKeepInterResults(false),
	m_bBuildAll(true),
	m_fTotalBuildTimeMs(0),
	m_pTriAreas(0),
	m_pSolid(0),
	m_pCHF(0),
	m_pContourSet(0),
	m_pMesh(0),
	m_pDetailMesh(0),
	m_eDrawMode(DRAWMODE_NAVMESH),
	m_nMaxTiles(0),
	m_nMaxPolysPerTile(0),
	m_fTileSize(32),
	m_uTileCol(duRGBA(0,0,0,32)),
	m_fTileBuildTime(0),
	m_fTileMemUsage(0),
	m_nTileTriCount(0)
{
	resetCommonSettings();
	memset(m_fTileBmin, 0, sizeof(m_fTileBmin));
	memset(m_fTileBmax, 0, sizeof(m_fTileBmax));
	
	setTool(new NavMeshTileTool);
}

Sample_TileMesh::~Sample_TileMesh()
{
	cleanup();
	dtFreeNavMesh(m_pNavMesh);
	m_pNavMesh = 0;
}

void Sample_TileMesh::cleanup()
{
	delete [] m_pTriAreas;
	m_pTriAreas = 0;
	rcFreeHeightField(m_pSolid);
	m_pSolid = 0;
	rcFreeCompactHeightfield(m_pCHF);
	m_pCHF = 0;
	rcFreeContourSet(m_pContourSet);
	m_pContourSet = 0;
	rcFreePolyMesh(m_pMesh);
	m_pMesh = 0;
	rcFreePolyMeshDetail(m_pDetailMesh);
	m_pDetailMesh = 0;
}


static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

void Sample_TileMesh::saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;
	
	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;
	
	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->pHeader || !tile->nDataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->pHeader || !tile->nDataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->nDataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->pData, tile->nDataSize, 1, fp);
	}

	fclose(fp);
}

dtNavMesh* Sample_TileMesh::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;
	
	// Read header.
	NavMeshSetHeader header;
	fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}
	
	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}
		
	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		fread(data, tileHeader.dataSize, 1, fp);
		
		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}
	
	fclose(fp);
	
	return mesh;
}

void Sample_TileMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_bKeepInterResults))
		m_bKeepInterResults = !m_bKeepInterResults;

	if (imguiCheck("Build All Tiles", m_bBuildAll))
		m_bBuildAll = !m_bBuildAll;
	
	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_fTileSize, 16.0f, 1024.0f, 16.0f);
	
	if (m_pInputGeom)
	{
		const float* bmin = m_pInputGeom->getMeshBoundsMin();
		const float* bmax = m_pInputGeom->getMeshBoundsMax();
		char text[64];
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_fCellSize, &gw, &gh);
		const int ts = (int)m_fTileSize;
		const int tw = (gw + ts-1) / ts;
		const int th = (gh + ts-1) / ts;
		snprintf(text, 64, "Tiles  %d x %d", tw, th);
		imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)ilog2(nextPow2(tw*th)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;
		m_nMaxTiles = 1 << tileBits;
		m_nMaxPolysPerTile = 1 << polyBits;
		snprintf(text, 64, "Max Tiles  %d", m_nMaxTiles);
		imguiValue(text);
		snprintf(text, 64, "Max Polys  %d", m_nMaxPolysPerTile);
		imguiValue(text);
	}
	else
	{
		m_nMaxTiles = 0;
		m_nMaxPolysPerTile = 0;
	}
	
	imguiSeparator();
	
	imguiIndent();
	imguiIndent();
	
	if (imguiButton("Save"))
	{
		saveAll("all_tiles_navmesh.bin", m_pNavMesh);
	}

	if (imguiButton("Load"))
	{
		dtFreeNavMesh(m_pNavMesh);
		m_pNavMesh = loadAll("all_tiles_navmesh.bin");
		m_pNavQuery->init(m_pNavMesh, 2048);
	}

	imguiUnindent();
	imguiUnindent();
	
	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_fTotalBuildTimeMs);
	imguiLabel(msg);
	
	imguiSeparator();
	
	imguiSeparator();
	
}

void Sample_TileMesh::handleTools()
{
	int type = !m_pTool ? TOOL_NONE : m_pTool->type();

	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Prune Navmesh", type == TOOL_NAVMESH_PRUNE))
	{
		setTool(new NavMeshPruneTool);
	}
	if (imguiCheck("Create Tiles", type == TOOL_TILE_EDIT))
	{
		setTool(new NavMeshTileTool);
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

void Sample_TileMesh::handleDebugMode()
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
		valid[DRAWMODE_VOXELS] = m_pSolid != 0;
		valid[DRAWMODE_VOXELS_WALKABLE] = m_pSolid != 0;
		valid[DRAWMODE_COMPACT] = m_pCHF != 0;
		valid[DRAWMODE_COMPACT_DISTANCE] = m_pCHF != 0;
		valid[DRAWMODE_COMPACT_REGIONS] = m_pCHF != 0;
		valid[DRAWMODE_REGION_CONNECTIONS] = m_pContourSet != 0;
		valid[DRAWMODE_RAW_CONTOURS] = m_pContourSet != 0;
		valid[DRAWMODE_BOTH_CONTOURS] = m_pContourSet != 0;
		valid[DRAWMODE_CONTOURS] = m_pContourSet != 0;
		valid[DRAWMODE_POLYMESH] = m_pMesh != 0;
		valid[DRAWMODE_POLYMESH_DETAIL] = m_pDetailMesh != 0;
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
	if (imguiCheck("Voxels", m_eDrawMode == DRAWMODE_VOXELS, valid[DRAWMODE_VOXELS]))
		m_eDrawMode = DRAWMODE_VOXELS;
	if (imguiCheck("Walkable Voxels", m_eDrawMode == DRAWMODE_VOXELS_WALKABLE, valid[DRAWMODE_VOXELS_WALKABLE]))
		m_eDrawMode = DRAWMODE_VOXELS_WALKABLE;
	if (imguiCheck("Compact", m_eDrawMode == DRAWMODE_COMPACT, valid[DRAWMODE_COMPACT]))
		m_eDrawMode = DRAWMODE_COMPACT;
	if (imguiCheck("Compact Distance", m_eDrawMode == DRAWMODE_COMPACT_DISTANCE, valid[DRAWMODE_COMPACT_DISTANCE]))
		m_eDrawMode = DRAWMODE_COMPACT_DISTANCE;
	if (imguiCheck("Compact Regions", m_eDrawMode == DRAWMODE_COMPACT_REGIONS, valid[DRAWMODE_COMPACT_REGIONS]))
		m_eDrawMode = DRAWMODE_COMPACT_REGIONS;
	if (imguiCheck("Region Connections", m_eDrawMode == DRAWMODE_REGION_CONNECTIONS, valid[DRAWMODE_REGION_CONNECTIONS]))
		m_eDrawMode = DRAWMODE_REGION_CONNECTIONS;
	if (imguiCheck("Raw Contours", m_eDrawMode == DRAWMODE_RAW_CONTOURS, valid[DRAWMODE_RAW_CONTOURS]))
		m_eDrawMode = DRAWMODE_RAW_CONTOURS;
	if (imguiCheck("Both Contours", m_eDrawMode == DRAWMODE_BOTH_CONTOURS, valid[DRAWMODE_BOTH_CONTOURS]))
		m_eDrawMode = DRAWMODE_BOTH_CONTOURS;
	if (imguiCheck("Contours", m_eDrawMode == DRAWMODE_CONTOURS, valid[DRAWMODE_CONTOURS]))
		m_eDrawMode = DRAWMODE_CONTOURS;
	if (imguiCheck("Poly Mesh", m_eDrawMode == DRAWMODE_POLYMESH, valid[DRAWMODE_POLYMESH]))
		m_eDrawMode = DRAWMODE_POLYMESH;
	if (imguiCheck("Poly Mesh Detail", m_eDrawMode == DRAWMODE_POLYMESH_DETAIL, valid[DRAWMODE_POLYMESH_DETAIL]))
		m_eDrawMode = DRAWMODE_POLYMESH_DETAIL;
		
	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("rebuild some tiles to see");
		imguiValue("more debug mode options.");
	}
}

void Sample_TileMesh::handleRender()
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
	
	// Draw active tile
	duDebugDrawBoxWire(&dd, m_fTileBmin[0],m_fTileBmin[1],m_fTileBmin[2],
					   m_fTileBmax[0],m_fTileBmax[1],m_fTileBmax[2], m_uTileCol, 1.0f);
		
	if (m_pNavMesh && m_pNavQuery &&
		(m_eDrawMode == DRAWMODE_NAVMESH ||
		 m_eDrawMode == DRAWMODE_NAVMESH_TRANS ||
		 m_eDrawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_eDrawMode == DRAWMODE_NAVMESH_NODES ||
		 m_eDrawMode == DRAWMODE_NAVMESH_PORTALS ||
		 m_eDrawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_eDrawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&dd, *m_pNavMesh, *m_pNavQuery, m_cNavMeshDrawFlags);
		if (m_eDrawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&dd, *m_pNavMesh);
		if (m_eDrawMode == DRAWMODE_NAVMESH_PORTALS)
			duDebugDrawNavMeshPortals(&dd, *m_pNavMesh);
		if (m_eDrawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&dd, *m_pNavQuery);
		duDebugDrawNavMeshPolysWithFlags(&dd, *m_pNavMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}
	
	
	glDepthMask(GL_TRUE);
	
	if (m_pCHF && m_eDrawMode == DRAWMODE_COMPACT)
		duDebugDrawCompactHeightfieldSolid(&dd, *m_pCHF);
	
	if (m_pCHF && m_eDrawMode == DRAWMODE_COMPACT_DISTANCE)
		duDebugDrawCompactHeightfieldDistance(&dd, *m_pCHF);
	if (m_pCHF && m_eDrawMode == DRAWMODE_COMPACT_REGIONS)
		duDebugDrawCompactHeightfieldRegions(&dd, *m_pCHF);
	if (m_pSolid && m_eDrawMode == DRAWMODE_VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&dd, *m_pSolid);
		glDisable(GL_FOG);
	}
	if (m_pSolid && m_eDrawMode == DRAWMODE_VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&dd, *m_pSolid);
		glDisable(GL_FOG);
	}
	
	if (m_pContourSet && m_eDrawMode == DRAWMODE_RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&dd, *m_pContourSet);
		glDepthMask(GL_TRUE);
	}
	
	if (m_pContourSet && m_eDrawMode == DRAWMODE_BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&dd, *m_pContourSet, 0.5f);
		duDebugDrawContours(&dd, *m_pContourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_pContourSet && m_eDrawMode == DRAWMODE_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&dd, *m_pContourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_pCHF && m_pContourSet && m_eDrawMode == DRAWMODE_REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&dd, *m_pCHF);
		
		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&dd, *m_pContourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_pMesh && m_eDrawMode == DRAWMODE_POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&dd, *m_pMesh);
		glDepthMask(GL_TRUE);
	}
	if (m_pDetailMesh && m_eDrawMode == DRAWMODE_POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&dd, *m_pDetailMesh);
		glDepthMask(GL_TRUE);
	}
		
	m_pInputGeom->drawConvexVolumes(&dd);
	
	if (m_pTool)
		m_pTool->handleRender();
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_TileMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_fTileBuildTime > 0.0f && gluProject((GLdouble)(m_fTileBmin[0]+m_fTileBmax[0])/2, (GLdouble)(m_fTileBmin[1]+m_fTileBmax[1])/2, (GLdouble)(m_fTileBmin[2]+m_fTileBmax[2])/2,
											 model, proj, view, &x, &y, &z))
	{
		char text[32];
		snprintf(text,32,"%.3fms / %dTris / %.1fkB", m_fTileBuildTime, m_nTileTriCount, m_fTileMemUsage);
		imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
	}
	
	if (m_pTool)
		m_pTool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);
}

void Sample_TileMesh::handleMeshChanged(class InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	cleanup();

	dtFreeNavMesh(m_pNavMesh);
	m_pNavMesh = 0;

	if (m_pTool)
	{
		m_pTool->reset();
		m_pTool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_TileMesh::handleBuild()
{
	if (!m_pInputGeom || !m_pInputGeom->getMesh())
	{
		m_pCtx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
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
	rcVcopy(Params.fOrigin, m_pInputGeom->getMeshBoundsMin());
	Params.fTileWidth = m_fTileSize*m_fCellSize;
	Params.fTileHeight = m_fTileSize*m_fCellSize;
	Params.nMaxTiles = m_nMaxTiles;
	Params.nMaxPolys = m_nMaxPolysPerTile;
	
	dtStatus status;
	
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
	
	if (m_bBuildAll)
		buildAllTiles();
	
	if (m_pTool)
		m_pTool->init(this);
	initToolStates(this);

	return true;
}

void Sample_TileMesh::buildTile(const float* pos)
{
	if (!m_pInputGeom) return;
	if (!m_pNavMesh) return;
		
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();
	
	const float ts = m_fTileSize*m_fCellSize;
	const int tx = (int)((pos[0] - bmin[0]) / ts);
	const int ty = (int)((pos[2] - bmin[2]) / ts);
	
	m_fTileBmin[0] = bmin[0] + tx*ts;
	m_fTileBmin[1] = bmin[1];
	m_fTileBmin[2] = bmin[2] + ty*ts;
	
	m_fTileBmax[0] = bmin[0] + (tx+1)*ts;
	m_fTileBmax[1] = bmax[1];
	m_fTileBmax[2] = bmin[2] + (ty+1)*ts;
	
	m_uTileCol = duRGBA(255,255,255,64);
	
	m_pCtx->resetLog();
	
	int dataSize = 0;
	unsigned char* data = buildTileMesh(tx, ty, m_fTileBmin, m_fTileBmax, dataSize);

	// Remove any previous data (navmesh owns and deletes the data).
	m_pNavMesh->removeTile(m_pNavMesh->getTileRefAt(tx,ty,0),0,0);

	// Add tile, or leave the location empty.
	if (data)
	{
		// Let the navmesh own the data.
		dtStatus status = m_pNavMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
			dtFree(data);
	}
	
	m_pCtx->dumpLog("Build Tile (%d,%d):", tx,ty);
}

void Sample_TileMesh::getTilePos(const float* pos, int& tx, int& ty)
{
	if (!m_pInputGeom) return;
	
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	
	const float ts = m_fTileSize*m_fCellSize;
	tx = (int)((pos[0] - bmin[0]) / ts);
	ty = (int)((pos[2] - bmin[2]) / ts);
}

void Sample_TileMesh::removeTile(const float* pos)
{
	if (!m_pInputGeom) return;
	if (!m_pNavMesh) return;
	
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();

	const float ts = m_fTileSize*m_fCellSize;
	const int tx = (int)((pos[0] - bmin[0]) / ts);
	const int ty = (int)((pos[2] - bmin[2]) / ts);
	
	m_fTileBmin[0] = bmin[0] + tx*ts;
	m_fTileBmin[1] = bmin[1];
	m_fTileBmin[2] = bmin[2] + ty*ts;
	
	m_fTileBmax[0] = bmin[0] + (tx+1)*ts;
	m_fTileBmax[1] = bmax[1];
	m_fTileBmax[2] = bmin[2] + (ty+1)*ts;
	
	m_uTileCol = duRGBA(128,32,16,64);
	
	m_pNavMesh->removeTile(m_pNavMesh->getTileRefAt(tx,ty,0),0,0);
}

void Sample_TileMesh::buildAllTiles()
{
	if (!m_pInputGeom) return;
	if (!m_pNavMesh) return;
	
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_fCellSize, &gw, &gh);
	const int ts = (int)m_fTileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	const float tcs = m_fTileSize*m_fCellSize;

	
	// Start the build process.
	m_pCtx->startTimer(RC_TIMER_TEMP);

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			m_fTileBmin[0] = bmin[0] + x*tcs;
			m_fTileBmin[1] = bmin[1];
			m_fTileBmin[2] = bmin[2] + y*tcs;
			
			m_fTileBmax[0] = bmin[0] + (x+1)*tcs;
			m_fTileBmax[1] = bmax[1];
			m_fTileBmax[2] = bmin[2] + (y+1)*tcs;
			
			int dataSize = 0;
			unsigned char* data = buildTileMesh(x, y, m_fTileBmin, m_fTileBmax, dataSize);
			if (data)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				m_pNavMesh->removeTile(m_pNavMesh->getTileRefAt(x,y,0),0,0);
				// Let the navmesh own the data.
				dtStatus status = m_pNavMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
				if (dtStatusFailed(status))
					dtFree(data);
			}
		}
	}
	
	// Start the build process.	
	m_pCtx->stopTimer(RC_TIMER_TEMP);

	m_fTotalBuildTimeMs = m_pCtx->getAccumulatedTime(RC_TIMER_TEMP)/1000.0f;
	
}

void Sample_TileMesh::removeAllTiles()
{
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_fCellSize, &gw, &gh);
	const int ts = (int)m_fTileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	
	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			m_pNavMesh->removeTile(m_pNavMesh->getTileRefAt(x,y,0),0,0);
}


unsigned char* Sample_TileMesh::buildTileMesh(const int tx, const int ty, const float* bmin, const float* bmax, int& dataSize)
{
	if (!m_pInputGeom || !m_pInputGeom->getMesh() || !m_pInputGeom->getChunkyMesh())
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return 0;
	}
	
	m_fTileMemUsage = 0;
	m_fTileBuildTime = 0;
	
	cleanup();
	
	const float* verts = m_pInputGeom->getMesh()->getVerts();
	const int nverts = m_pInputGeom->getMesh()->getVertCount();
	const int ntris = m_pInputGeom->getMesh()->getTriCount();
	const rcChunkyTriMesh* chunkyMesh = m_pInputGeom->getChunkyMesh();
		
	// Init build configuration from GUI
	memset(&m_Cfg, 0, sizeof(m_Cfg));
	m_Cfg.fCellSize = m_fCellSize;
	m_Cfg.fCellHeight = m_fCellHeight;
	m_Cfg.fWalkableSlopeAngle = m_fAgentMaxSlope;
	m_Cfg.nWalkableHeight = (int)ceilf(m_fAgentHeight / m_Cfg.fCellHeight);
	m_Cfg.nWalkableClimb = (int)floorf(m_fAgentMaxClimb / m_Cfg.fCellHeight);
	m_Cfg.nWalkableRadius = (int)ceilf(m_fAgentRadius / m_Cfg.fCellSize);
	m_Cfg.nMaxEdgeLen = (int)(m_fEdgeMaxLen / m_fCellSize);
	m_Cfg.fMaxSimplificationError = m_fEdgeMaxError;
	m_Cfg.nMinRegionArea = (int)rcSqr(m_fRegionMinSize);		// Note: area = size*size
	m_Cfg.nMergeRegionArea = (int)rcSqr(m_fRegionMergeSize);	// Note: area = size*size
	m_Cfg.nMaxVertsPerPoly = (int)m_fVertsPerPoly;
	m_Cfg.nTileSize = (int)m_fTileSize;
	m_Cfg.nBorderSize = m_Cfg.nWalkableRadius + 3; // Reserve enough padding.
	m_Cfg.nWidth = m_Cfg.nTileSize + m_Cfg.nBorderSize*2;
	m_Cfg.nHeight = m_Cfg.nTileSize + m_Cfg.nBorderSize*2;
	m_Cfg.fDetailSampleDist = m_fDetailSampleDist < 0.9f ? 0 : m_fCellSize * m_fDetailSampleDist;
	m_Cfg.fDetailSampleMaxError = m_fCellHeight * m_fDetailSampleMaxError;
	
	rcVcopy(m_Cfg.fBMin, bmin);
	rcVcopy(m_Cfg.fBMax, bmax);
	m_Cfg.fBMin[0] -= m_Cfg.nBorderSize*m_Cfg.fCellSize;
	m_Cfg.fBMin[2] -= m_Cfg.nBorderSize*m_Cfg.fCellSize;
	m_Cfg.fBMax[0] += m_Cfg.nBorderSize*m_Cfg.fCellSize;
	m_Cfg.fBMax[2] += m_Cfg.nBorderSize*m_Cfg.fCellSize;
	
	// Reset build times gathering.
	m_pCtx->resetTimers();
	
	// Start the build process.
	m_pCtx->startTimer(RC_TIMER_TOTAL);
	
	m_pCtx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_pCtx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_Cfg.nWidth, m_Cfg.nHeight);
	m_pCtx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts/1000.0f, ntris/1000.0f);
	
	// Allocate voxel heightfield where we rasterize our input data to.
	m_pSolid = rcAllocHeightfield();
	if (!m_pSolid)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(m_pCtx, *m_pSolid, m_Cfg.nWidth, m_Cfg.nHeight, m_Cfg.fBMin, m_Cfg.fBMax, m_Cfg.fCellSize, m_Cfg.fCellHeight))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}
	
	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_pTriAreas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!m_pTriAreas)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}
	
	float tbmin[2], tbmax[2];
	tbmin[0] = m_Cfg.fBMin[0];
	tbmin[1] = m_Cfg.fBMin[2];
	tbmax[0] = m_Cfg.fBMax[0];
	tbmax[1] = m_Cfg.fBMax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
		return 0;
	
	m_nTileTriCount = 0;
	
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* ctris = &chunkyMesh->tris[node.i*3];
		const int nctris = node.n;
		
		m_nTileTriCount += nctris;
		
		memset(m_pTriAreas, 0, nctris*sizeof(unsigned char));
		rcMarkWalkableTriangles(m_pCtx, m_Cfg.fWalkableSlopeAngle,
								verts, nverts, ctris, nctris, m_pTriAreas);
		
		rcRasterizeTriangles(m_pCtx, verts, nverts, ctris, m_pTriAreas, nctris, *m_pSolid, m_Cfg.nWalkableClimb);
	}
	
	if (!m_bKeepInterResults)
	{
		delete [] m_pTriAreas;
		m_pTriAreas = 0;
	}
	
	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(m_pCtx, m_Cfg.nWalkableClimb, *m_pSolid);
	rcFilterLedgeSpans(m_pCtx, m_Cfg.nWalkableHeight, m_Cfg.nWalkableClimb, *m_pSolid);
	rcFilterWalkableLowHeightSpans(m_pCtx, m_Cfg.nWalkableHeight, *m_pSolid);
	
	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_pCHF = rcAllocCompactHeightfield();
	if (!m_pCHF)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_pCtx, m_Cfg.nWalkableHeight, m_Cfg.nWalkableClimb, *m_pSolid, *m_pCHF))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}
	
	if (!m_bKeepInterResults)
	{
		rcFreeHeightField(m_pSolid);
		m_pSolid = 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_pCtx, m_Cfg.nWalkableRadius, *m_pCHF))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_pInputGeom->getConvexVolumes();
	for (int i  = 0; i < m_pInputGeom->getConvexVolumeCount(); ++i)
		rcMarkConvexPolyArea(m_pCtx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_pCHF);
	
	if (m_bMonotonePartitioning)
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegionsMonotone(m_pCtx, *m_pCHF, m_Cfg.nBorderSize, m_Cfg.nMinRegionArea, m_Cfg.nMergeRegionArea))
		{
			m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
			return 0;
		}
	}
	else
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_pCtx, *m_pCHF))
		{
			m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return 0;
		}
		
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_pCtx, *m_pCHF, m_Cfg.nBorderSize, m_Cfg.nMinRegionArea, m_Cfg.nMergeRegionArea))
		{
			m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
			return 0;
		}
	}
 	
	// Create contours.
	m_pContourSet = rcAllocContourSet();
	if (!m_pContourSet)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return 0;
	}
	if (!rcBuildContours(m_pCtx, *m_pCHF, m_Cfg.fMaxSimplificationError, m_Cfg.nMaxEdgeLen, *m_pContourSet))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return 0;
	}
	
	if (m_pContourSet->nContours == 0)
	{
		return 0;
	}
	
	// Build polygon navmesh from the contours.
	m_pMesh = rcAllocPolyMesh();
	if (!m_pMesh)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return 0;
	}
	if (!rcBuildPolyMesh(m_pCtx, *m_pContourSet, m_Cfg.nMaxVertsPerPoly, *m_pMesh))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return 0;
	}
	
	// Build detail mesh.
	m_pDetailMesh = rcAllocPolyMeshDetail();
	if (!m_pDetailMesh)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'.");
		return 0;
	}
	
	if (!rcBuildPolyMeshDetail(m_pCtx, *m_pMesh, *m_pCHF,
							   m_Cfg.fDetailSampleDist, m_Cfg.fDetailSampleMaxError,
							   *m_pDetailMesh))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail.");
		return 0;
	}
	
	if (!m_bKeepInterResults)
	{
		rcFreeCompactHeightfield(m_pCHF);
		m_pCHF = 0;
		rcFreeContourSet(m_pContourSet);
		m_pContourSet = 0;
	}
	
	unsigned char* navData = 0;
	int navDataSize = 0;
	if (m_Cfg.nMaxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (m_pMesh->nVerts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			m_pCtx->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", m_pMesh->nVerts, 0xffff);
			return 0;
		}
		
		// Update poly flags from areas.
		for (int i = 0; i < m_pMesh->nPolys; ++i)
		{
			if (m_pMesh->pAreas[i] == RC_WALKABLE_AREA)
				m_pMesh->pAreas[i] = SAMPLE_POLYAREA_GROUND;
			
			if (m_pMesh->pAreas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pMesh->pAreas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pMesh->pAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pMesh->pFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_pMesh->pAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pMesh->pFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_pMesh->pAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_pMesh->pFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}
		
		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.pVerts = m_pMesh->pVerts;
		params.nVertCount = m_pMesh->nVerts;
		params.pPolys = m_pMesh->pPolys;
		params.pPolyAreas = m_pMesh->pAreas;
		params.pPolyFlags = m_pMesh->pFlags;
		params.nPolyCount = m_pMesh->nPolys;
		params.nMaxVertNumPerPoly = m_pMesh->nVertexNumPerPoly;
		params.pDetailMeshes = m_pDetailMesh->pMeshes;
		params.pDetailVerts = m_pDetailMesh->fVerts;
		params.nDetailVertsCount = m_pDetailMesh->nVerts;
		params.pDetailTris = m_pDetailMesh->pTris;
		params.nDetailTriCount = m_pDetailMesh->nTris;
		params.pOffMeshConVerts = m_pInputGeom->getOffMeshConnectionVerts();
		params.pOffMeshConRad = m_pInputGeom->getOffMeshConnectionRads();
		params.pOffMeshConDir = m_pInputGeom->getOffMeshConnectionDirs();
		params.pOffMeshConAreas = m_pInputGeom->getOffMeshConnectionAreas();
		params.pOffMeshConFlags = m_pInputGeom->getOffMeshConnectionFlags();
		params.pOffMeshConUserID = m_pInputGeom->getOffMeshConnectionId();
		params.nOffMeshConCount = m_pInputGeom->getOffMeshConnectionCount();
		params.fWalkableHeight = m_fAgentHeight;
		params.fWalkableRadius = m_fAgentRadius;
		params.fWalkableClimb = m_fAgentMaxClimb;
		params.nTileX = tx;
		params.nTileY = ty;
		params.nTileLayer = 0;
		rcVcopy(params.fBMin, m_pMesh->fBMin);
		rcVcopy(params.fBMax, m_pMesh->fBMax);
		params.fCellSize = m_Cfg.fCellSize;
		params.fCellHeight = m_Cfg.fCellHeight;
		params.bBuildBvTree = true;
		
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_pCtx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return 0;
		}		
	}
	m_fTileMemUsage = navDataSize/1024.0f;
	
	m_pCtx->stopTimer(RC_TIMER_TOTAL);
	
	// Show performance stats.
	duLogBuildTimes(*m_pCtx, m_pCtx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_pCtx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pMesh->nVerts, m_pMesh->nPolys);
	
	m_fTileBuildTime = m_pCtx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;

	dataSize = navDataSize;
	return navData;
}
