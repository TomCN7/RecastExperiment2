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
#include "Sample_SoloMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "RecastDump.h"
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


Sample_SoloMesh::Sample_SoloMesh() :
	m_bKeepInterResults(true),
	m_fTotalBuildTimeMs(0),
	m_pTriAreas(0),
	m_pSolid(0),
	m_pCHF(0),
	m_pContourSet(0),
	m_pMesh(0),
	m_pDetailMesh(0),
	m_eDrawMode(DRAWMODE_NAVMESH)
{
	setTool(new NavMeshTesterTool);
}
		
Sample_SoloMesh::~Sample_SoloMesh()
{
	cleanup();
}
	
void Sample_SoloMesh::cleanup()
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
	dtFreeNavMesh(m_pNavMesh);
	m_pNavMesh = 0;
}

void Sample_SoloMesh::handleSettings()
{
	Sample::handleCommonSettings();
	
	if (imguiCheck("Keep Itermediate Results", m_bKeepInterResults))
		m_bKeepInterResults = !m_bKeepInterResults;

	imguiSeparator();
	
	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_fTotalBuildTimeMs);
	imguiLabel(msg);
	
	imguiSeparator();
}

void Sample_SoloMesh::handleTools()
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
	if (imguiCheck("Create Off-Mesh Connections", type == TOOL_OFFMESH_CONNECTION))
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

void Sample_SoloMesh::handleDebugMode()
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
		imguiValue("to see more debug mode options.");
	}
}

void Sample_SoloMesh::handleRender()
{
	if (!m_pInputGeom || !m_pInputGeom->getMesh())
		return;
	
	DebugDrawGL dd;
	
	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);

	const float texScale = 1.0f / (m_fCellSize * 10.0f);
	
	if (m_eDrawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&dd, m_pInputGeom->getMesh()->getVerts(), m_pInputGeom->getMesh()->getVertCount(),
								m_pInputGeom->getMesh()->getTris(), m_pInputGeom->getMesh()->getNormals(), m_pInputGeom->getMesh()->getTriCount(),
								m_fAgentMaxSlope, texScale);
		m_pInputGeom->drawOffMeshConnections(&dd);
	}
	
	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();
	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
	dd.begin(DU_DRAW_POINTS, 5.0f);
	dd.vertex(bmin[0],bmin[1],bmin[2],duRGBA(255,255,255,128));
	dd.end();
	
	if (m_pNavMesh && m_pNavQuery &&
		(m_eDrawMode == DRAWMODE_NAVMESH ||
		m_eDrawMode == DRAWMODE_NAVMESH_TRANS ||
		m_eDrawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_eDrawMode == DRAWMODE_NAVMESH_NODES ||
		m_eDrawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_eDrawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&dd, *m_pNavMesh, *m_pNavQuery, m_cNavMeshDrawFlags);
		if (m_eDrawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&dd, *m_pNavMesh);
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

void Sample_SoloMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	if (m_pTool)
		m_pTool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);
}

void Sample_SoloMesh::handleMeshChanged(class InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

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


bool Sample_SoloMesh::handleBuild()
{
	if (!m_pInputGeom || !m_pInputGeom->getMesh())
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}
	
	cleanup();
	
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();
	const float* verts = m_pInputGeom->getMesh()->getVerts();
	const int nverts = m_pInputGeom->getMesh()->getVertCount();
	const int* tris = m_pInputGeom->getMesh()->getTris();
	const int ntris = m_pInputGeom->getMesh()->getTriCount();
	
	//
	// Step 1. Initialize build config.
	//
	
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
	m_Cfg.fDetailSampleDist = m_fDetailSampleDist < 0.9f ? 0 : m_fCellSize * m_fDetailSampleDist;
	m_Cfg.fDetailSampleMaxError = m_fCellHeight * m_fDetailSampleMaxError;
	
	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	rcVcopy(m_Cfg.fBMin, bmin);
	rcVcopy(m_Cfg.fBMax, bmax);
	rcCalcGridSize(m_Cfg.fBMin, m_Cfg.fBMax, m_Cfg.fCellSize, &m_Cfg.nWidth, &m_Cfg.nHeight);

	// Reset build times gathering.
	m_pCtx->resetTimers();

	// Start the build process.	
	m_pCtx->startTimer(RC_TIMER_TOTAL);
	
	m_pCtx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_pCtx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_Cfg.nWidth, m_Cfg.nHeight);
	m_pCtx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts/1000.0f, ntris/1000.0f);
	
	//
	// Step 2. Rasterize input polygon soup.
	//
	
	// Allocate voxel heightfield where we rasterize our input data to.
	m_pSolid = rcAllocHeightfield();
	if (!m_pSolid)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	if (!rcCreateHeightfield(m_pCtx, *m_pSolid, m_Cfg.nWidth, m_Cfg.nHeight, m_Cfg.fBMin, m_Cfg.fBMax, m_Cfg.fCellSize, m_Cfg.fCellHeight))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}
	
	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_pTriAreas = new unsigned char[ntris];
	if (!m_pTriAreas)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return false;
	}
	
	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(m_pTriAreas, 0, ntris*sizeof(unsigned char));
	rcMarkWalkableTriangles(m_pCtx, m_Cfg.fWalkableSlopeAngle, verts, nverts, tris, ntris, m_pTriAreas);
	rcRasterizeTriangles(m_pCtx, verts, nverts, tris, m_pTriAreas, ntris, *m_pSolid, m_Cfg.nWalkableClimb);

	if (!m_bKeepInterResults)
	{
		delete [] m_pTriAreas;
		m_pTriAreas = 0;
	}
	
	//
	// Step 3. Filter walkables surfaces.
	//
	
	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(m_pCtx, m_Cfg.nWalkableClimb, *m_pSolid);
	rcFilterLedgeSpans(m_pCtx, m_Cfg.nWalkableHeight, m_Cfg.nWalkableClimb, *m_pSolid);
	rcFilterWalkableLowHeightSpans(m_pCtx, m_Cfg.nWalkableHeight, *m_pSolid);


	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_pCHF = rcAllocCompactHeightfield();
	if (!m_pCHF)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(m_pCtx, m_Cfg.nWalkableHeight, m_Cfg.nWalkableClimb, *m_pSolid, *m_pCHF))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
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
		return false;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_pInputGeom->getConvexVolumes();
	for (int i  = 0; i < m_pInputGeom->getConvexVolumeCount(); ++i)
		rcMarkConvexPolyArea(m_pCtx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_pCHF);
	
	if (m_bMonotonePartitioning)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_pCtx, *m_pCHF, 0, m_Cfg.nMinRegionArea, m_Cfg.nMergeRegionArea))
		{
			m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
			return false;
		}
	}
	else
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_pCtx, *m_pCHF))
		{
			m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_pCtx, *m_pCHF, 0, m_Cfg.nMinRegionArea, m_Cfg.nMergeRegionArea))
		{
			m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
			return false;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//
	
	// Create contours.
	m_pContourSet = rcAllocContourSet();
	if (!m_pContourSet)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(m_pCtx, *m_pCHF, m_Cfg.fMaxSimplificationError, m_Cfg.nMaxEdgeLen, *m_pContourSet))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}
	
	//
	// Step 6. Build polygons mesh from contours.
	//
	
	// Build polygon navmesh from the contours.
	m_pMesh = rcAllocPolyMesh();
	if (!m_pMesh)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(m_pCtx, *m_pContourSet, m_Cfg.nMaxVertsPerPoly, *m_pMesh))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}
	
	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//
	
	m_pDetailMesh = rcAllocPolyMeshDetail();
	if (!m_pDetailMesh)
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(m_pCtx, *m_pMesh, *m_pCHF, m_Cfg.fDetailSampleDist, m_Cfg.fDetailSampleMaxError, *m_pDetailMesh))
	{
		m_pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	if (!m_bKeepInterResults)
	{
		rcFreeCompactHeightfield(m_pCHF);
		m_pCHF = 0;
		rcFreeContourSet(m_pContourSet);
		m_pContourSet = 0;
	}

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.
	
	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//
	
	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	if (m_Cfg.nMaxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

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
		rcVcopy(params.fBMin, m_pMesh->fBMin);
		rcVcopy(params.fBMax, m_pMesh->fBMax);
		params.fCellSize = m_Cfg.fCellSize;
		params.fCellHeight = m_Cfg.fCellHeight;
		params.bBuildBvTree = true;
		
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_pCtx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}
		
		m_pNavMesh = dtAllocNavMesh();
		if (!m_pNavMesh)
		{
			dtFree(navData);
			m_pCtx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}
		
		dtStatus status;
		
		status = m_pNavMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			m_pCtx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}
		
		status = m_pNavQuery->init(m_pNavMesh, 2048);
		if (dtStatusFailed(status))
		{
			m_pCtx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
			return false;
		}
	}
	
	m_pCtx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_pCtx, m_pCtx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_pCtx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pMesh->nVerts, m_pMesh->nPolys);
	
	m_fTotalBuildTimeMs = m_pCtx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
	
	if (m_pTool)
		m_pTool->init(this);
	initToolStates(this);

	return true;
}
