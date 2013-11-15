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
#include "Sample.h"
#include "InputGeom.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

Sample::Sample() :
	m_pInputGeom(0),
	m_pNavMesh(0),
	m_pNavQuery(0),
	m_pCrowd(0),
	m_cNavMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS|DU_DRAWNAVMESH_CLOSEDLIST),
	m_tool(0),
	m_ctx(0)
{
	resetCommonSettings();
	m_pNavQuery = dtAllocNavMeshQuery();
	m_pCrowd = dtAllocCrowd();

	for (int i = 0; i < MAX_TOOLS; i++)
		m_toolStates[i] = 0;
}

Sample::~Sample()
{
	dtFreeNavMeshQuery(m_pNavQuery);
	dtFreeNavMesh(m_pNavMesh);
	dtFreeCrowd(m_pCrowd);
	delete m_tool;
	for (int i = 0; i < MAX_TOOLS; i++)
		delete m_toolStates[i];
}

void Sample::setTool(SampleTool* tool)
{
	delete m_tool;
	m_tool = tool;
	if (tool)
		m_tool->init(this);
}

void Sample::handleSettings()
{
}

void Sample::handleTools()
{
}

void Sample::handleDebugMode()
{
}

void Sample::handleRender()
{
	if (!m_pInputGeom)
		return;
	
	DebugDrawGL dd;
		
	// Draw mesh
	duDebugDrawTriMesh(&dd, m_pInputGeom->getMesh()->getVerts(), m_pInputGeom->getMesh()->getVertCount(),
					   m_pInputGeom->getMesh()->getTris(), m_pInputGeom->getMesh()->getNormals(), m_pInputGeom->getMesh()->getTriCount(), 0, 1.0f);
	// Draw bounds
	const float* bmin = m_pInputGeom->getMeshBoundsMin();
	const float* bmax = m_pInputGeom->getMeshBoundsMax();
	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample::handleMeshChanged(InputGeom* geom)
{
	m_pInputGeom = geom;
}

const float* Sample::getBoundsMin()
{
	if (!m_pInputGeom) return 0;
	return m_pInputGeom->getMeshBoundsMin();
}

const float* Sample::getBoundsMax()
{
	if (!m_pInputGeom) return 0;
	return m_pInputGeom->getMeshBoundsMax();
}

void Sample::resetCommonSettings()
{
	m_fCellSize = 0.3f;
	m_fCellHeight = 0.2f;
	m_fAgentHeight = 2.0f;
	m_fAgentRadius = 0.6f;
	m_fAgentMaxClimb = 0.9f;
	m_fAgentMaxSlope = 45.0f;
	m_fRegionMinSize = 8;
	m_fRegionMergeSize = 20;
	m_bMonotonePartitioning = false;
	m_fEdgeMaxLen = 12.0f;
	m_fEdgeMaxError = 1.3f;
	m_fVertsPerPoly = 6.0f;
	m_fDetailSampleDist = 6.0f;
	m_fDetailSampleMaxError = 1.0f;
}

void Sample::handleCommonSettings()
{
	imguiLabel("Rasterization");
	imguiSlider("Cell Size", &m_fCellSize, 0.1f, 1.0f, 0.01f);
	imguiSlider("Cell Height", &m_fCellHeight, 0.1f, 1.0f, 0.01f);
	
	if (m_pInputGeom)
	{
		const float* bmin = m_pInputGeom->getMeshBoundsMin();
		const float* bmax = m_pInputGeom->getMeshBoundsMax();
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_fCellSize, &gw, &gh);
		char text[64];
		snprintf(text, 64, "Voxels  %d x %d", gw, gh);
		imguiValue(text);
	}
	
	imguiSeparator();
	imguiLabel("Agent");
	imguiSlider("Height", &m_fAgentHeight, 0.1f, 5.0f, 0.1f);
	imguiSlider("Radius", &m_fAgentRadius, 0.0f, 5.0f, 0.1f);
	imguiSlider("Max Climb", &m_fAgentMaxClimb, 0.1f, 5.0f, 0.1f);
	imguiSlider("Max Slope", &m_fAgentMaxSlope, 0.0f, 90.0f, 1.0f);
	
	imguiSeparator();
	imguiLabel("Region");
	imguiSlider("Min Region Size", &m_fRegionMinSize, 0.0f, 150.0f, 1.0f);
	imguiSlider("Merged Region Size", &m_fRegionMergeSize, 0.0f, 150.0f, 1.0f);
	if (imguiCheck("Monotore Partitioning", m_bMonotonePartitioning))
		m_bMonotonePartitioning = !m_bMonotonePartitioning;
	
	imguiSeparator();
	imguiLabel("Polygonization");
	imguiSlider("Max Edge Length", &m_fEdgeMaxLen, 0.0f, 50.0f, 1.0f);
	imguiSlider("Max Edge Error", &m_fEdgeMaxError, 0.1f, 3.0f, 0.1f);
	imguiSlider("Verts Per Poly", &m_fVertsPerPoly, 3.0f, 12.0f, 1.0f);		

	imguiSeparator();
	imguiLabel("Detail Mesh");
	imguiSlider("Sample Distance", &m_fDetailSampleDist, 0.0f, 16.0f, 1.0f);
	imguiSlider("Max Sample Error", &m_fDetailSampleMaxError, 0.0f, 16.0f, 1.0f);
	
	imguiSeparator();
}

void Sample::handleClick(const float* s, const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

void Sample::handleStep()
{
	if (m_tool)
		m_tool->handleStep();
}

bool Sample::handleBuild()
{
	return true;
}

void Sample::handleUpdate(const float dt)
{
	if (m_tool)
		m_tool->handleUpdate(dt);
	updateToolStates(dt);
}


void Sample::updateToolStates(const float dt)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleUpdate(dt);
	}
}

void Sample::initToolStates(Sample* sample)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->init(sample);
	}
}

void Sample::resetToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->reset();
	}
}

void Sample::renderToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRender();
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRenderOverlay(proj, model, view);
	}
}

