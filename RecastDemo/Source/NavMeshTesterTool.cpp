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
#include <stdlib.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "NavMeshTesterTool.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

// Uncomment this to dump all the requests in stdout.
#define DUMP_REQS

// Returns a random number [0..1)
static float frand()
{
//	return ((float)(rand() & 0xffff)/(float)0xffff);
	return (float)rand()/(float)RAND_MAX;
}

inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
}


static int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
						 const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = npath-1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}

	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;
	
	// Concatenate paths.	

	// Adjust beginning of the buffer to include the visited.
	const int req = nvisited - furthestVisited;
	const int orig = rcMin(furthestPath+1, npath);
	int size = rcMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));
	
	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited-1)-i];				
	
	return req+size;
}

static bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
						   const float minTargetDist,
						   const dtPolyRef* path, const int pathSize,
						   float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
						   float* outPoints = 0, int* outPointCount = 0)							 
{
	// Find steer target.
	static const int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS*3];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = 0;
	navQuery->findStraightPath(startPos, endPos, path, pathSize,
							   steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
	if (!nsteerPath)
		return false;
		
	if (outPoints && outPointCount)
	{
		*outPointCount = nsteerPath;
		for (int i = 0; i < nsteerPath; ++i)
			dtVcopy(&outPoints[i*3], &steerPath[i*3]);
	}

	
	// Find vertex far enough to steer to.
	int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRange(&steerPath[ns*3], startPos, minTargetDist, 1000.0f))
			break;
		ns++;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath)
		return false;
	
	dtVcopy(steerPos, &steerPath[ns*3]);
	steerPos[1] = startPos[1];
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];
	
	return true;
}


NavMeshTesterTool::NavMeshTesterTool() :
	m_pSample(0),
	m_pNavMesh(0),
	m_pNavQuery(0),
	m_nPathFindStatus(DT_FAILURE),
	m_eToolMode(TOOLMODE_PATHFIND_FOLLOW),
	m_nStraightPathOptions(0),
	m_StartRef(0),
	m_EndRef(0),
	m_nPolys(0),
	m_nStraightPath(0),
	m_nSmoothPath(0),
	m_nRandPoints(0),
	m_bRandPointsInCircle(false),
	m_bHitResult(false),
	m_fDistanceToWall(0),
	m_bStartPosSet(false),
	m_bEndPosSet(false),
	m_nPathIterNum(0),
	m_nSteerPointCount(0)
{
	m_Filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_Filter.setExcludeFlags(0);

	m_fPolyPickExt[0] = 2;
	m_fPolyPickExt[1] = 4;
	m_fPolyPickExt[2] = 2;
	
	m_fNeighbourhoodRadius = 2.5f;
	m_fRandomRadius = 5.0f;
}

NavMeshTesterTool::~NavMeshTesterTool()
{
}

void NavMeshTesterTool::init(Sample* sample)
{
	m_pSample = sample;
	m_pNavMesh = sample->getNavMesh();
	m_pNavQuery = sample->getNavMeshQuery();
	recalc();

	if (m_pNavQuery)
	{
		// Change costs.
		m_Filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
		m_Filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
		m_Filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
		m_Filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
		m_Filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
		m_Filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);
	}
	
	m_fNeighbourhoodRadius = sample->getAgentRadius() * 20.0f;
	m_fRandomRadius = sample->getAgentRadius() * 30.0f;
}

void NavMeshTesterTool::handleMenu()
{
	if (imguiCheck("Pathfind Follow", m_eToolMode == TOOLMODE_PATHFIND_FOLLOW))
	{
		m_eToolMode = TOOLMODE_PATHFIND_FOLLOW;
		recalc();
	}
	if (imguiCheck("Pathfind Straight", m_eToolMode == TOOLMODE_PATHFIND_STRAIGHT))
	{
		m_eToolMode = TOOLMODE_PATHFIND_STRAIGHT;
		recalc();
	}
	if (m_eToolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		imguiIndent();
		imguiLabel("Vertices at crossings");
		if (imguiCheck("None", m_nStraightPathOptions == 0))
		{
			m_nStraightPathOptions = 0;
			recalc();
		}
		if (imguiCheck("Area", m_nStraightPathOptions == DT_STRAIGHTPATH_AREA_CROSSINGS))
		{
			m_nStraightPathOptions = DT_STRAIGHTPATH_AREA_CROSSINGS;
			recalc();
		}
		if (imguiCheck("All", m_nStraightPathOptions == DT_STRAIGHTPATH_ALL_CROSSINGS))
		{
			m_nStraightPathOptions = DT_STRAIGHTPATH_ALL_CROSSINGS;
			recalc();
		}

		imguiUnindent();
	}
	if (imguiCheck("Pathfind Sliced", m_eToolMode == TOOLMODE_PATHFIND_SLICED))
	{
		m_eToolMode = TOOLMODE_PATHFIND_SLICED;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Distance to Wall", m_eToolMode == TOOLMODE_DISTANCE_TO_WALL))
	{
		m_eToolMode = TOOLMODE_DISTANCE_TO_WALL;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Raycast", m_eToolMode == TOOLMODE_RAYCAST))
	{
		m_eToolMode = TOOLMODE_RAYCAST;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Find Polys in Circle", m_eToolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE))
	{
		m_eToolMode = TOOLMODE_FIND_POLYS_IN_CIRCLE;
		recalc();
	}
	if (imguiCheck("Find Polys in Shape", m_eToolMode == TOOLMODE_FIND_POLYS_IN_SHAPE))
	{
		m_eToolMode = TOOLMODE_FIND_POLYS_IN_SHAPE;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Find Local Neighbourhood", m_eToolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD))
	{
		m_eToolMode = TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD;
		recalc();
	}

	imguiSeparator();
	
	if (imguiButton("Set Random Start"))
	{
		dtStatus status = m_pNavQuery->findRandomPoint(&m_Filter, frand, &m_StartRef, m_fStartPos);
		if (dtStatusSucceed(status))
		{
			m_bStartPosSet = true;
			recalc();
		}
	}
	if (imguiButton("Set Random End", m_bStartPosSet))
	{
		if (m_bStartPosSet)
		{
			dtStatus status = m_pNavQuery->findRandomPointAroundCircle(m_StartRef, m_fStartPos, m_fRandomRadius, &m_Filter, frand, &m_EndRef, m_fEndPos);
			if (dtStatusSucceed(status))
			{
				m_bEndPosSet = true;
				recalc();
			}
		}
	}

	imguiSeparator();

	if (imguiButton("Make Random Points"))
	{
		m_bRandPointsInCircle = false;
		m_nRandPoints = 0;
		for (int i = 0; i < MAX_RAND_POINTS; i++)
		{
			float pt[3];
			dtPolyRef Ref;
			dtStatus status = m_pNavQuery->findRandomPoint(&m_Filter, frand, &Ref, pt);
			if (dtStatusSucceed(status))
			{
				dtVcopy(&m_fRandPoints[m_nRandPoints*3], pt);
				m_nRandPoints++;
			}
		}
	}
	if (imguiButton("Make Random Points Around", m_bStartPosSet))
	{
		if (m_bStartPosSet)
		{
			m_nRandPoints = 0;
			m_bRandPointsInCircle = true;
			for (int i = 0; i < MAX_RAND_POINTS; i++)
			{
				float pt[3];
				dtPolyRef Ref;
				dtStatus status = m_pNavQuery->findRandomPointAroundCircle(m_StartRef, m_fStartPos, m_fRandomRadius, &m_Filter, frand, &Ref, pt);
				if (dtStatusSucceed(status))
				{
					dtVcopy(&m_fRandPoints[m_nRandPoints*3], pt);
					m_nRandPoints++;
				}
			}
		}
	}

	
	imguiSeparator();

	imguiLabel("Include Flags");

	imguiIndent();
	if (imguiCheck("Walk", (m_Filter.getIncludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_Filter.setIncludeFlags(m_Filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	if (imguiCheck("Swim", (m_Filter.getIncludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0))
	{
		m_Filter.setIncludeFlags(m_Filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	if (imguiCheck("Door", (m_Filter.getIncludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_Filter.setIncludeFlags(m_Filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	if (imguiCheck("Jump", (m_Filter.getIncludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0))
	{
		m_Filter.setIncludeFlags(m_Filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	imguiUnindent();

	imguiSeparator();
	imguiLabel("Exclude Flags");
	
	imguiIndent();
	if (imguiCheck("Walk", (m_Filter.getExcludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_Filter.setExcludeFlags(m_Filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	if (imguiCheck("Swim", (m_Filter.getExcludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0))
	{
		m_Filter.setExcludeFlags(m_Filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	if (imguiCheck("Door", (m_Filter.getExcludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_Filter.setExcludeFlags(m_Filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	if (imguiCheck("Jump", (m_Filter.getExcludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0))
	{
		m_Filter.setExcludeFlags(m_Filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	imguiUnindent();

	imguiSeparator();	
}

void NavMeshTesterTool::handleClick(const float* /*s*/, const float* p, bool shift)
{
	if (shift)
	{
		m_bStartPosSet = true;
		dtVcopy(m_fStartPos, p);
	}
	else
	{
		m_bEndPosSet = true;
		dtVcopy(m_fEndPos, p);
	}
	recalc();
}

void NavMeshTesterTool::handleStep()
{
}

void NavMeshTesterTool::handleToggle()
{
	// TODO: merge separate to a path iterator. Use same code in recalc() too.
	if (m_eToolMode != TOOLMODE_PATHFIND_FOLLOW)
		return;
		
	if (!m_bStartPosSet || !m_bEndPosSet || !m_StartRef || !m_EndRef)
		return;
		
	static const float STEP_SIZE = 0.5f;
	static const float SLOP = 0.01f;

	if (m_nPathIterNum == 0)
	{
		m_pNavQuery->findPath(m_StartRef, m_EndRef, m_fStartPos, m_fEndPos, &m_Filter, m_Polys, &m_nPolys, MAX_POLYS);
		m_nSmoothPath = 0;

		m_nPathIterPolyCount = m_nPolys;
		if (m_nPathIterPolyCount)
			memcpy(m_PathIterPolys, m_Polys, sizeof(dtPolyRef)*m_nPathIterPolyCount); 
		
		if (m_nPathIterPolyCount)
		{
			// Iterate over the path to find smooth path on the detail mesh surface.
			m_pNavQuery->closestPointOnPoly(m_StartRef, m_fStartPos, m_fIterPos);
			m_pNavQuery->closestPointOnPoly(m_PathIterPolys[m_nPathIterPolyCount-1], m_fEndPos, m_fTargetPos);
			
			m_nSmoothPath = 0;
			
			dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], m_fIterPos);
			m_nSmoothPath++;
		}
	}
	
	dtVcopy(m_fPrevIterPos, m_fIterPos);

	m_nPathIterNum++;

	if (!m_nPathIterPolyCount)
		return;

	if (m_nSmoothPath >= MAX_SMOOTH)
		return;

	// Move towards target a small advancement at a time until target reached or
	// when ran out of memory to store the path.

	// Find location to steer towards.
	float steerPos[3];
	unsigned char steerPosFlag;
	dtPolyRef steerPosRef;
		
	if (!getSteerTarget(m_pNavQuery, m_fIterPos, m_fTargetPos, SLOP,
						m_PathIterPolys, m_nPathIterPolyCount, steerPos, steerPosFlag, steerPosRef,
						m_fSteerPoints, &m_nSteerPointCount))
		return;
		
	dtVcopy(m_fSteerPos, steerPos);
	
	bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
	bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
		
	// Find movement delta.
	float delta[3], len;
	dtVsub(delta, steerPos, m_fIterPos);
	len = sqrtf(dtVdot(delta,delta));
	// If the steer target is end of path or off-mesh link, do not move past the location.
	if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
		len = 1;
	else
		len = STEP_SIZE / len;
	float moveTgt[3];
	dtVmad(moveTgt, m_fIterPos, delta, len);
		
	// Move
	float result[3];
	dtPolyRef visited[16];
	int nvisited = 0;
	m_pNavQuery->moveAlongSurface(m_PathIterPolys[0], m_fIterPos, moveTgt, &m_Filter,
								 result, visited, &nvisited, 16);
	m_nPathIterPolyCount = fixupCorridor(m_PathIterPolys, m_nPathIterPolyCount, MAX_POLYS, visited, nvisited);
	float h = 0;
	m_pNavQuery->getPolyHeight(m_PathIterPolys[0], result, &h);
	result[1] = h;
	dtVcopy(m_fIterPos, result);
	
	// Handle end of path and off-mesh links when close enough.
	if (endOfPath && inRange(m_fIterPos, steerPos, SLOP, 1.0f))
	{
		// Reached end of path.
		dtVcopy(m_fIterPos, m_fTargetPos);
		if (m_nSmoothPath < MAX_SMOOTH)
		{
			dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], m_fIterPos);
			m_nSmoothPath++;
		}
		return;
	}
	else if (offMeshConnection && inRange(m_fIterPos, steerPos, SLOP, 1.0f))
	{
		// Reached off-mesh connection.
		float startPos[3], endPos[3];
		
		// Advance the path up to and over the off-mesh connection.
		dtPolyRef prevRef = 0, polyRef = m_PathIterPolys[0];
		int npos = 0;
		while (npos < m_nPathIterPolyCount && polyRef != steerPosRef)
		{
			prevRef = polyRef;
			polyRef = m_PathIterPolys[npos];
			npos++;
		}
		for (int i = npos; i < m_nPathIterPolyCount; ++i)
			m_PathIterPolys[i-npos] = m_PathIterPolys[i];
		m_nPathIterPolyCount -= npos;
				
		// Handle the connection.
		dtStatus status = m_pNavMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
		if (dtStatusSucceed(status))
		{
			if (m_nSmoothPath < MAX_SMOOTH)
			{
				dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], startPos);
				m_nSmoothPath++;
				// Hack to make the dotted path not visible during off-mesh connection.
				if (m_nSmoothPath & 1)
				{
					dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], startPos);
					m_nSmoothPath++;
				}
			}
			// Move position at the other side of the off-mesh link.
			dtVcopy(m_fIterPos, endPos);
			float eh = 0.0f;
			m_pNavQuery->getPolyHeight(m_PathIterPolys[0], m_fIterPos, &eh);
			m_fIterPos[1] = eh;
		}
	}
	
	// Store results.
	if (m_nSmoothPath < MAX_SMOOTH)
	{
		dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], m_fIterPos);
		m_nSmoothPath++;
	}
}

void NavMeshTesterTool::handleUpdate(const float /*dt*/)
{
	if (m_eToolMode == TOOLMODE_PATHFIND_SLICED)
	{
		if (dtStatusInProgress(m_nPathFindStatus))
		{
			m_nPathFindStatus = m_pNavQuery->updateSlicedFindPath(1,0);
		}
		if (dtStatusSucceed(m_nPathFindStatus))
		{
			m_pNavQuery->finalizeSlicedFindPath(m_Polys, &m_nPolys, MAX_POLYS);
			m_nStraightPath = 0;
			if (m_nPolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				float epos[3];
				dtVcopy(epos, m_fEndPos);
				if (m_Polys[m_nPolys-1] != m_EndRef)
				m_pNavQuery->closestPointOnPoly(m_Polys[m_nPolys-1], m_fEndPos, epos);

				m_pNavQuery->findStraightPath(m_fStartPos, epos, m_Polys, m_nPolys,
											 m_fStraightPath, m_cStraightPathFlags,
											 m_StraightPathPolys, &m_nStraightPath, MAX_POLYS);
			}
			 
			m_nPathFindStatus = DT_FAILURE;
		}
	}
}

void NavMeshTesterTool::reset()
{
	m_StartRef = 0;
	m_EndRef = 0;
	m_nPolys = 0;
	m_nStraightPath = 0;
	m_nSmoothPath = 0;
	memset(m_fHitPos, 0, sizeof(m_fHitPos));
	memset(m_fHitNormal, 0, sizeof(m_fHitNormal));
	m_fDistanceToWall = 0;
}

void NavMeshTesterTool::recalc()
{
	if (!m_pNavMesh)
		return;
	
	if (m_bStartPosSet)
		m_pNavQuery->findNearestPoly(m_fStartPos, m_fPolyPickExt, &m_Filter, &m_StartRef, 0);
	else
		m_StartRef = 0;
	
	if (m_bEndPosSet)
		m_pNavQuery->findNearestPoly(m_fEndPos, m_fPolyPickExt, &m_Filter, &m_EndRef, 0);
	else
		m_EndRef = 0;
	
	m_nPathFindStatus = DT_FAILURE;
	
	if (m_eToolMode == TOOLMODE_PATHFIND_FOLLOW)
	{
		m_nPathIterNum = 0;
		if (m_bStartPosSet && m_bEndPosSet && m_StartRef && m_EndRef)
		{
#ifdef DUMP_REQS
			printf("pi  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_fStartPos[0],m_fStartPos[1],m_fStartPos[2], m_fEndPos[0],m_fEndPos[1],m_fEndPos[2],
				   m_Filter.getIncludeFlags(), m_Filter.getExcludeFlags()); 
#endif

			m_pNavQuery->findPath(m_StartRef, m_EndRef, m_fStartPos, m_fEndPos, &m_Filter, m_Polys, &m_nPolys, MAX_POLYS);

			m_nSmoothPath = 0;

			if (m_nPolys)
			{
				// Iterate over the path to find smooth path on the detail mesh surface.
				dtPolyRef polys[MAX_POLYS];
				memcpy(polys, m_Polys, sizeof(dtPolyRef)*m_nPolys); 
				int npolys = m_nPolys;
				
				float iterPos[3], targetPos[3];
				m_pNavQuery->closestPointOnPoly(m_StartRef, m_fStartPos, iterPos);
				m_pNavQuery->closestPointOnPoly(polys[npolys-1], m_fEndPos, targetPos);
				
				static const float STEP_SIZE = 0.5f;
				static const float SLOP = 0.01f;
				
				m_nSmoothPath = 0;
				
				dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], iterPos);
				m_nSmoothPath++;
				
				// Move towards target a small advancement at a time until target reached or
				// when ran out of memory to store the path.
				while (npolys && m_nSmoothPath < MAX_SMOOTH)
				{
					// Find location to steer towards.
					float steerPos[3];
					unsigned char steerPosFlag;
					dtPolyRef steerPosRef;
					
					if (!getSteerTarget(m_pNavQuery, iterPos, targetPos, SLOP,
										polys, npolys, steerPos, steerPosFlag, steerPosRef))
						break;
					
					bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
					bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
					
					// Find movement delta.
					float delta[3], len;
					dtVsub(delta, steerPos, iterPos);
					len = dtSqrt(dtVdot(delta,delta));
					// If the steer target is end of path or off-mesh link, do not move past the location.
					if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
						len = 1;
					else
						len = STEP_SIZE / len;
					float moveTgt[3];
					dtVmad(moveTgt, iterPos, delta, len);
					
					// Move
					float result[3];
					dtPolyRef visited[16];
					int nvisited = 0;
					m_pNavQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &m_Filter,
												 result, visited, &nvisited, 16);
															   
					npolys = fixupCorridor(polys, npolys, MAX_POLYS, visited, nvisited);
					float h = 0;
					m_pNavQuery->getPolyHeight(polys[0], result, &h);
					result[1] = h;
					dtVcopy(iterPos, result);

					// Handle end of path and off-mesh links when close enough.
					if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached end of path.
						dtVcopy(iterPos, targetPos);
						if (m_nSmoothPath < MAX_SMOOTH)
						{
							dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], iterPos);
							m_nSmoothPath++;
						}
						break;
					}
					else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached off-mesh connection.
						float startPos[3], endPos[3];
						
						// Advance the path up to and over the off-mesh connection.
						dtPolyRef prevRef = 0, polyRef = polys[0];
						int npos = 0;
						while (npos < npolys && polyRef != steerPosRef)
						{
							prevRef = polyRef;
							polyRef = polys[npos];
							npos++;
						}
						for (int i = npos; i < npolys; ++i)
							polys[i-npos] = polys[i];
						npolys -= npos;
						
						// Handle the connection.
						dtStatus status = m_pNavMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
						if (dtStatusSucceed(status))
						{
							if (m_nSmoothPath < MAX_SMOOTH)
							{
								dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], startPos);
								m_nSmoothPath++;
								// Hack to make the dotted path not visible during off-mesh connection.
								if (m_nSmoothPath & 1)
								{
									dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], startPos);
									m_nSmoothPath++;
								}
							}
							// Move position at the other side of the off-mesh link.
							dtVcopy(iterPos, endPos);
							float eh = 0.0f;
							m_pNavQuery->getPolyHeight(polys[0], iterPos, &eh);
							iterPos[1] = eh;
						}
					}
					
					// Store results.
					if (m_nSmoothPath < MAX_SMOOTH)
					{
						dtVcopy(&m_fSmoothPath[m_nSmoothPath*3], iterPos);
						m_nSmoothPath++;
					}
				}
			}

		}
		else
		{
			m_nPolys = 0;
			m_nSmoothPath = 0;
		}
	}
	else if (m_eToolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		if (m_bStartPosSet && m_bEndPosSet && m_StartRef && m_EndRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_fStartPos[0],m_fStartPos[1],m_fStartPos[2], m_fEndPos[0],m_fEndPos[1],m_fEndPos[2],
				   m_Filter.getIncludeFlags(), m_Filter.getExcludeFlags()); 
#endif
			m_pNavQuery->findPath(m_StartRef, m_EndRef, m_fStartPos, m_fEndPos, &m_Filter, m_Polys, &m_nPolys, MAX_POLYS);
			m_nStraightPath = 0;
			if (m_nPolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				float epos[3];
				dtVcopy(epos, m_fEndPos);
				if (m_Polys[m_nPolys-1] != m_EndRef)
					m_pNavQuery->closestPointOnPoly(m_Polys[m_nPolys-1], m_fEndPos, epos);
				
				m_pNavQuery->findStraightPath(m_fStartPos, epos, m_Polys, m_nPolys,
											 m_fStraightPath, m_cStraightPathFlags,
											 m_StraightPathPolys, &m_nStraightPath, MAX_POLYS, m_nStraightPathOptions);
			}
		}
		else
		{
			m_nPolys = 0;
			m_nStraightPath = 0;
		}
	}
	else if (m_eToolMode == TOOLMODE_PATHFIND_SLICED)
	{
		if (m_bStartPosSet && m_bEndPosSet && m_StartRef && m_EndRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_fStartPos[0],m_fStartPos[1],m_fStartPos[2], m_fEndPos[0],m_fEndPos[1],m_fEndPos[2],
				   m_Filter.getIncludeFlags(), m_Filter.getExcludeFlags()); 
#endif
			m_nPolys = 0;
			m_nStraightPath = 0;
			
			m_nPathFindStatus = m_pNavQuery->initSlicedFindPath(m_StartRef, m_EndRef, m_fStartPos, m_fEndPos, &m_Filter);
		}
		else
		{
			m_nPolys = 0;
			m_nStraightPath = 0;
		}
	}
	else if (m_eToolMode == TOOLMODE_RAYCAST)
	{
		m_nStraightPath = 0;
		if (m_bStartPosSet && m_bEndPosSet && m_StartRef)
		{
#ifdef DUMP_REQS
			printf("rc  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_fStartPos[0],m_fStartPos[1],m_fStartPos[2], m_fEndPos[0],m_fEndPos[1],m_fEndPos[2],
				   m_Filter.getIncludeFlags(), m_Filter.getExcludeFlags()); 
#endif
			float t = 0;
			m_nPolys = 0;
			m_nStraightPath = 2;
			m_fStraightPath[0] = m_fStartPos[0];
			m_fStraightPath[1] = m_fStartPos[1];
			m_fStraightPath[2] = m_fStartPos[2];
			m_pNavQuery->raycast(m_StartRef, m_fStartPos, m_fEndPos, &m_Filter, &t, m_fHitNormal, m_Polys, &m_nPolys, MAX_POLYS);
			if (t > 1)
			{
				// No hit
				dtVcopy(m_fHitPos, m_fEndPos);
				m_bHitResult = false;
			}
			else
			{
				// Hit
				m_fHitPos[0] = m_fStartPos[0] + (m_fEndPos[0] - m_fStartPos[0]) * t;
				m_fHitPos[1] = m_fStartPos[1] + (m_fEndPos[1] - m_fStartPos[1]) * t;
				m_fHitPos[2] = m_fStartPos[2] + (m_fEndPos[2] - m_fStartPos[2]) * t;
				if (m_nPolys)
				{
					float h = 0;
					m_pNavQuery->getPolyHeight(m_Polys[m_nPolys-1], m_fHitPos, &h);
					m_fHitPos[1] = h;
				}
				m_bHitResult = true;
			}
			dtVcopy(&m_fStraightPath[3], m_fHitPos);
		}
	}
	else if (m_eToolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		m_fDistanceToWall = 0;
		if (m_bStartPosSet && m_StartRef)
		{
#ifdef DUMP_REQS
			printf("dw  %f %f %f  %f  0x%x 0x%x\n",
				   m_fStartPos[0],m_fStartPos[1],m_fStartPos[2], 100.0f,
				   m_Filter.getIncludeFlags(), m_Filter.getExcludeFlags()); 
#endif
			m_fDistanceToWall = 0.0f;
			m_pNavQuery->findDistanceToWall(m_StartRef, m_fStartPos, 100.0f, &m_Filter, &m_fDistanceToWall, m_fHitPos, m_fHitNormal);
		}
	}
	else if (m_eToolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE)
	{
		if (m_bStartPosSet && m_StartRef && m_bEndPosSet)
		{
			const float dx = m_fEndPos[0] - m_fStartPos[0];
			const float dz = m_fEndPos[2] - m_fStartPos[2];
			float dist = sqrtf(dx*dx + dz*dz);
#ifdef DUMP_REQS
			printf("fpc  %f %f %f  %f  0x%x 0x%x\n",
				   m_fStartPos[0],m_fStartPos[1],m_fStartPos[2], dist,
				   m_Filter.getIncludeFlags(), m_Filter.getExcludeFlags());
#endif
			m_pNavQuery->findPolysAroundCircle(m_StartRef, m_fStartPos, dist, &m_Filter,
											  m_Polys, m_Parent, 0, &m_nPolys, MAX_POLYS);

		}
	}
	else if (m_eToolMode == TOOLMODE_FIND_POLYS_IN_SHAPE)
	{
		if (m_bStartPosSet && m_StartRef && m_bEndPosSet)
		{
			const float nx = (m_fEndPos[2] - m_fStartPos[2])*0.25f;
			const float nz = -(m_fEndPos[0] - m_fStartPos[0])*0.25f;
			const float agentHeight = m_pSample ? m_pSample->getAgentHeight() : 0;

			m_fQueryPoly[0] = m_fStartPos[0] + nx*1.2f;
			m_fQueryPoly[1] = m_fStartPos[1] + agentHeight/2;
			m_fQueryPoly[2] = m_fStartPos[2] + nz*1.2f;

			m_fQueryPoly[3] = m_fStartPos[0] - nx*1.3f;
			m_fQueryPoly[4] = m_fStartPos[1] + agentHeight/2;
			m_fQueryPoly[5] = m_fStartPos[2] - nz*1.3f;

			m_fQueryPoly[6] = m_fEndPos[0] - nx*0.8f;
			m_fQueryPoly[7] = m_fEndPos[1] + agentHeight/2;
			m_fQueryPoly[8] = m_fEndPos[2] - nz*0.8f;

			m_fQueryPoly[9] = m_fEndPos[0] + nx;
			m_fQueryPoly[10] = m_fEndPos[1] + agentHeight/2;
			m_fQueryPoly[11] = m_fEndPos[2] + nz;
			
#ifdef DUMP_REQS
			printf("fpp  %f %f %f  %f %f %f  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_fQueryPoly[0],m_fQueryPoly[1],m_fQueryPoly[2],
				   m_fQueryPoly[3],m_fQueryPoly[4],m_fQueryPoly[5],
				   m_fQueryPoly[6],m_fQueryPoly[7],m_fQueryPoly[8],
				   m_fQueryPoly[9],m_fQueryPoly[10],m_fQueryPoly[11],
				   m_Filter.getIncludeFlags(), m_Filter.getExcludeFlags());
#endif
			m_pNavQuery->findPolysAroundShape(m_StartRef, m_fQueryPoly, 4, &m_Filter,
											 m_Polys, m_Parent, 0, &m_nPolys, MAX_POLYS);
		}
	}
	else if (m_eToolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD)
	{
		if (m_bStartPosSet && m_StartRef)
		{
#ifdef DUMP_REQS
			printf("fln  %f %f %f  %f  0x%x 0x%x\n",
				   m_fStartPos[0],m_fStartPos[1],m_fStartPos[2], m_fNeighbourhoodRadius,
				   m_Filter.getIncludeFlags(), m_Filter.getExcludeFlags());
#endif
			m_pNavQuery->findLocalNeighbourhood(m_StartRef, m_fStartPos, m_fNeighbourhoodRadius, &m_Filter,
											   m_Polys, m_Parent, &m_nPolys, MAX_POLYS);
		}
	}
}

static void getPolyCenter(dtNavMesh* navMesh, dtPolyRef Ref, float* center)
{
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	dtStatus status = navMesh->getTileAndPolyByRef(Ref, &tile, &poly);
	if (dtStatusFailed(status))
		return;
		
	for (int i = 0; i < (int)poly->cVertCount; ++i)
	{
		const float* v = &tile->fVerts[poly->Verts[i]*3];
		center[0] += v[0];
		center[1] += v[1];
		center[2] += v[2];
	}
	const float s = 1.0f / poly->cVertCount;
	center[0] *= s;
	center[1] *= s;
	center[2] *= s;
}



void NavMeshTesterTool::handleRender()
{
	DebugDrawGL dd;
	
	static const unsigned int startCol = duRGBA(128,25,0,192);
	static const unsigned int endCol = duRGBA(51,102,0,129);
	static const unsigned int pathCol = duRGBA(0,0,0,64);
	
	const float agentRadius = m_pSample->getAgentRadius();
	const float agentHeight = m_pSample->getAgentHeight();
	const float agentClimb = m_pSample->getAgentClimb();
	
	dd.depthMask(false);
	if (m_bStartPosSet)
		drawAgent(m_fStartPos, agentRadius, agentHeight, agentClimb, startCol);
	if (m_bEndPosSet)
		drawAgent(m_fEndPos, agentRadius, agentHeight, agentClimb, endCol);
	dd.depthMask(true);
	
	if (!m_pNavMesh)
	{
		return;
	}

	if (m_eToolMode == TOOLMODE_PATHFIND_FOLLOW)
	{
		duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_StartRef, startCol);
		duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_EndRef, endCol);
		
		if (m_nPolys)
		{
			for (int i = 0; i < m_nPolys; ++i)
			{
				if (m_Polys[i] == m_StartRef || m_Polys[i] == m_EndRef)
					continue;
				duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_Polys[i], pathCol);
			}
		}
				
		if (m_nSmoothPath)
		{
			dd.depthMask(false);
			const unsigned int spathCol = duRGBA(0,0,0,220);
			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int i = 0; i < m_nSmoothPath; ++i)
				dd.vertex(m_fSmoothPath[i*3], m_fSmoothPath[i*3+1]+0.1f, m_fSmoothPath[i*3+2], spathCol);
			dd.end();
			dd.depthMask(true);
		}
		
		if (m_nPathIterNum)
		{
			duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_PathIterPolys[0], duRGBA(255,255,255,128));

			dd.depthMask(false);
			dd.begin(DU_DRAW_LINES, 1.0f);
			
			const unsigned int prevCol = duRGBA(255,192,0,220);
			const unsigned int curCol = duRGBA(255,255,255,220);
			const unsigned int steerCol = duRGBA(0,192,255,220);

			dd.vertex(m_fPrevIterPos[0],m_fPrevIterPos[1]-0.3f,m_fPrevIterPos[2], prevCol);
			dd.vertex(m_fPrevIterPos[0],m_fPrevIterPos[1]+0.3f,m_fPrevIterPos[2], prevCol);

			dd.vertex(m_fIterPos[0],m_fIterPos[1]-0.3f,m_fIterPos[2], curCol);
			dd.vertex(m_fIterPos[0],m_fIterPos[1]+0.3f,m_fIterPos[2], curCol);

			dd.vertex(m_fPrevIterPos[0],m_fPrevIterPos[1]+0.3f,m_fPrevIterPos[2], prevCol);
			dd.vertex(m_fIterPos[0],m_fIterPos[1]+0.3f,m_fIterPos[2], prevCol);

			dd.vertex(m_fPrevIterPos[0],m_fPrevIterPos[1]+0.3f,m_fPrevIterPos[2], steerCol);
			dd.vertex(m_fSteerPos[0],m_fSteerPos[1]+0.3f,m_fSteerPos[2], steerCol);
			
			for (int i = 0; i < m_nSteerPointCount-1; ++i)
			{
				dd.vertex(m_fSteerPoints[i*3+0],m_fSteerPoints[i*3+1]+0.2f,m_fSteerPoints[i*3+2], duDarkenCol(steerCol));
				dd.vertex(m_fSteerPoints[(i+1)*3+0],m_fSteerPoints[(i+1)*3+1]+0.2f,m_fSteerPoints[(i+1)*3+2], duDarkenCol(steerCol));
			}
			
			dd.end();
			dd.depthMask(true);
		}
	}
	else if (m_eToolMode == TOOLMODE_PATHFIND_STRAIGHT ||
			 m_eToolMode == TOOLMODE_PATHFIND_SLICED)
	{
		duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_StartRef, startCol);
		duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_EndRef, endCol);
		
		if (m_nPolys)
		{
			for (int i = 0; i < m_nPolys; ++i)
			{
				if (m_Polys[i] == m_StartRef || m_Polys[i] == m_EndRef)
					continue;
				duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_Polys[i], pathCol);
			}
		}
		
		if (m_nStraightPath)
		{
			dd.depthMask(false);
			const unsigned int spathCol = duRGBA(64,16,0,220);
			const unsigned int offMeshCol = duRGBA(128,96,0,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < m_nStraightPath-1; ++i)
			{
				unsigned int col = 0;
				if (m_cStraightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
					col = offMeshCol;
				else
					col = spathCol;
				
				dd.vertex(m_fStraightPath[i*3], m_fStraightPath[i*3+1]+0.4f, m_fStraightPath[i*3+2], col);
				dd.vertex(m_fStraightPath[(i+1)*3], m_fStraightPath[(i+1)*3+1]+0.4f, m_fStraightPath[(i+1)*3+2], col);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 6.0f);
			for (int i = 0; i < m_nStraightPath; ++i)
			{
				unsigned int col = 0;
				if (m_cStraightPathFlags[i] & DT_STRAIGHTPATH_START)
					col = startCol;
				else if (m_cStraightPathFlags[i] & DT_STRAIGHTPATH_START)
					col = endCol;
				else if (m_cStraightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
					col = offMeshCol;
				else
					col = spathCol;
				dd.vertex(m_fStraightPath[i*3], m_fStraightPath[i*3+1]+0.4f, m_fStraightPath[i*3+2], spathCol);
			}
			dd.end();
			dd.depthMask(true);
		}
	}
	else if (m_eToolMode == TOOLMODE_RAYCAST)
	{
		duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_StartRef, startCol);
		
		if (m_nStraightPath)
		{
			for (int i = 1; i < m_nPolys; ++i)
				duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_Polys[i], pathCol);
			
			dd.depthMask(false);
			const unsigned int spathCol = m_bHitResult ? duRGBA(64,16,0,220) : duRGBA(240,240,240,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < m_nStraightPath-1; ++i)
			{
				dd.vertex(m_fStraightPath[i*3], m_fStraightPath[i*3+1]+0.4f, m_fStraightPath[i*3+2], spathCol);
				dd.vertex(m_fStraightPath[(i+1)*3], m_fStraightPath[(i+1)*3+1]+0.4f, m_fStraightPath[(i+1)*3+2], spathCol);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 4.0f);
			for (int i = 0; i < m_nStraightPath; ++i)
				dd.vertex(m_fStraightPath[i*3], m_fStraightPath[i*3+1]+0.4f, m_fStraightPath[i*3+2], spathCol);
			dd.end();

			if (m_bHitResult)
			{
				const unsigned int hitCol = duRGBA(0,0,0,128);
				dd.begin(DU_DRAW_LINES, 2.0f);
				dd.vertex(m_fHitPos[0], m_fHitPos[1] + 0.4f, m_fHitPos[2], hitCol);
				dd.vertex(m_fHitPos[0] + m_fHitNormal[0]*agentRadius,
						  m_fHitPos[1] + 0.4f + m_fHitNormal[1]*agentRadius,
						  m_fHitPos[2] + m_fHitNormal[2]*agentRadius, hitCol);
				dd.end();
			}
			dd.depthMask(true);
		}
	}
	else if (m_eToolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_StartRef, startCol);
		dd.depthMask(false);
		duDebugDrawCircle(&dd, m_fStartPos[0], m_fStartPos[1]+agentHeight/2, m_fStartPos[2], m_fDistanceToWall, duRGBA(64,16,0,220), 2.0f);
		dd.begin(DU_DRAW_LINES, 3.0f);
		dd.vertex(m_fHitPos[0], m_fHitPos[1] + 0.02f, m_fHitPos[2], duRGBA(0,0,0,192));
		dd.vertex(m_fHitPos[0], m_fHitPos[1] + agentHeight, m_fHitPos[2], duRGBA(0,0,0,192));
		dd.end();
		dd.depthMask(true);
	}
	else if (m_eToolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE)
	{
		for (int i = 0; i < m_nPolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_Polys[i], pathCol);
			dd.depthMask(false);
			if (m_Parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_pNavMesh, m_Parent[i], p0);
				getPolyCenter(m_pNavMesh, m_Polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}
		
		if (m_bStartPosSet && m_bEndPosSet)
		{
			dd.depthMask(false);
			const float dx = m_fEndPos[0] - m_fStartPos[0];
			const float dz = m_fEndPos[2] - m_fStartPos[2];
			const float dist = sqrtf(dx*dx + dz*dz);
			duDebugDrawCircle(&dd, m_fStartPos[0], m_fStartPos[1]+agentHeight/2, m_fStartPos[2], dist, duRGBA(64,16,0,220), 2.0f);
			dd.depthMask(true);
		}
	}	
	else if (m_eToolMode == TOOLMODE_FIND_POLYS_IN_SHAPE)
	{
		for (int i = 0; i < m_nPolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_Polys[i], pathCol);
			dd.depthMask(false);
			if (m_Parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_pNavMesh, m_Parent[i], p0);
				getPolyCenter(m_pNavMesh, m_Polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}
		
		if (m_bStartPosSet && m_bEndPosSet)
		{
			dd.depthMask(false);
			const unsigned int col = duRGBA(64,16,0,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0, j = 3; i < 4; j=i++)
			{
				const float* p0 = &m_fQueryPoly[j*3];
				const float* p1 = &m_fQueryPoly[i*3];
				dd.vertex(p0, col);
				dd.vertex(p1, col);
			}
			dd.end();
			dd.depthMask(true);
		}
	}
	else if (m_eToolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD)
	{
		for (int i = 0; i < m_nPolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_pNavMesh, m_Polys[i], pathCol);
			dd.depthMask(false);
			if (m_Parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_pNavMesh, m_Parent[i], p0);
				getPolyCenter(m_pNavMesh, m_Polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}

			static const int MAX_SEGS = DT_VERTS_PER_POLYGON*4;
			float segs[MAX_SEGS*6];
			dtPolyRef refs[MAX_SEGS];
			memset(refs, 0, sizeof(dtPolyRef)*MAX_SEGS); 
			int nsegs = 0;
			m_pNavQuery->getPolyWallSegments(m_Polys[i], &m_Filter, segs, refs, &nsegs, MAX_SEGS);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int j = 0; j < nsegs; ++j)
			{
				const float* s = &segs[j*6];
				
				// Skip too distant segments.
				float tseg;
				float distSqr = dtDistancePtSegSqr2D(m_fStartPos, s, s+3, tseg);
				if (distSqr > dtSqr(m_fNeighbourhoodRadius))
					continue;
				
				float delta[3], norm[3], p0[3], p1[3];
				dtVsub(delta, s+3,s);
				dtVmad(p0, s, delta, 0.5f);
				norm[0] = delta[2];
				norm[1] = 0;
				norm[2] = -delta[0];
				dtVnormalize(norm);
				dtVmad(p1, p0, norm, agentRadius*0.5f);

				// Skip backfacing segments.
				if (refs[j])
				{
					unsigned int col = duRGBA(255,255,255,32);
					dd.vertex(s[0],s[1]+agentClimb,s[2],col);
					dd.vertex(s[3],s[4]+agentClimb,s[5],col);
				}
				else
				{
					unsigned int col = duRGBA(192,32,16,192);
					if (dtTriArea2D(m_fStartPos, s, s+3) < 0.0f)
						col = duRGBA(96,32,16,192);
					
					dd.vertex(p0[0],p0[1]+agentClimb,p0[2],col);
					dd.vertex(p1[0],p1[1]+agentClimb,p1[2],col);

					dd.vertex(s[0],s[1]+agentClimb,s[2],col);
					dd.vertex(s[3],s[4]+agentClimb,s[5],col);
				}
			}
			dd.end();
			
			dd.depthMask(true);
		}
		
		if (m_bStartPosSet)
		{
			dd.depthMask(false);
			duDebugDrawCircle(&dd, m_fStartPos[0], m_fStartPos[1]+agentHeight/2, m_fStartPos[2], m_fNeighbourhoodRadius, duRGBA(64,16,0,220), 2.0f);
			dd.depthMask(true);
		}
	}
	
	if (m_nRandPoints > 0)
	{
		dd.begin(DU_DRAW_POINTS, 6.0f);
		for (int i = 0; i < m_nRandPoints; i++)
		{
			const float* p = &m_fRandPoints[i*3];
			dd.vertex(p[0],p[1]+0.1,p[2], duRGBA(220,32,16,192));
		} 
		dd.end();
		
		if (m_bRandPointsInCircle && m_bStartPosSet)
		{
			duDebugDrawCircle(&dd, m_fStartPos[0], m_fStartPos[1]+agentHeight/2, m_fStartPos[2], m_fRandomRadius, duRGBA(64,16,0,220), 2.0f);
		}
	}
}

void NavMeshTesterTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_bStartPosSet && gluProject((GLdouble)m_fStartPos[0], (GLdouble)m_fStartPos[1], (GLdouble)m_fStartPos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0,0,0,220));
	}
	if (m_bEndPosSet && gluProject((GLdouble)m_fEndPos[0], (GLdouble)m_fEndPos[1], (GLdouble)m_fEndPos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "End", imguiRGBA(0,0,0,220));
	}
	
	// Tool help
	const int h = view[3];
	imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB+SHIFT: Set start location  LMB: Set end location", imguiRGBA(255,255,255,192));	
}

void NavMeshTesterTool::drawAgent(const float* pos, float r, float h, float c, const unsigned int col)
{
	DebugDrawGL dd;
	
	dd.depthMask(false);
	
	// Agent dimensions.	
	duDebugDrawCylinderWire(&dd, pos[0]-r, pos[1]+0.02f, pos[2]-r, pos[0]+r, pos[1]+h, pos[2]+r, col, 2.0f);

	duDebugDrawCircle(&dd, pos[0],pos[1]+c,pos[2],r,duRGBA(0,0,0,64),1.0f);

	unsigned int colb = duRGBA(0,0,0,196);
	dd.begin(DU_DRAW_LINES);
	dd.vertex(pos[0], pos[1]-c, pos[2], colb);
	dd.vertex(pos[0], pos[1]+c, pos[2], colb);
	dd.vertex(pos[0]-r/2, pos[1]+0.02f, pos[2], colb);
	dd.vertex(pos[0]+r/2, pos[1]+0.02f, pos[2], colb);
	dd.vertex(pos[0], pos[1]+0.02f, pos[2]-r/2, colb);
	dd.vertex(pos[0], pos[1]+0.02f, pos[2]+r/2, colb);
	dd.end();
	
	dd.depthMask(true);
}
