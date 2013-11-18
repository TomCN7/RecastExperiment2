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

#ifndef NAVMESHTESTERTOOL_H
#define NAVMESHTESTERTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

class NavMeshTesterTool : public SampleTool
{
	Sample* m_pSample;
	
	dtNavMesh* m_pNavMesh;
	dtNavMeshQuery* m_pNavQuery;

	dtQueryFilter m_Filter;

	dtStatus m_nPathFindStatus;

	enum ToolMode
	{
		TOOLMODE_PATHFIND_FOLLOW,
		TOOLMODE_PATHFIND_STRAIGHT,
		TOOLMODE_PATHFIND_SLICED,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_IN_CIRCLE,
		TOOLMODE_FIND_POLYS_IN_SHAPE,
		TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD,
	};
	
	ToolMode m_eToolMode;

	int m_nStraightPathOptions;
	
	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;
	
	dtPolyRef m_StartRef;
	dtPolyRef m_EndRef;
	dtPolyRef m_Polys[MAX_POLYS];
	dtPolyRef m_Parent[MAX_POLYS];
	int m_nPolys;
	float m_fStraightPath[MAX_POLYS*3];
	unsigned char m_cStraightPathFlags[MAX_POLYS];
	dtPolyRef m_StraightPathPolys[MAX_POLYS];
	int m_nStraightPath;
	float m_fPolyPickExt[3];
	float m_fSmoothPath[MAX_SMOOTH*3];
	int m_nSmoothPath;
	float m_fQueryPoly[4*3];

	static const int MAX_RAND_POINTS = 64;
	float m_fRandPoints[MAX_RAND_POINTS*3];
	int m_nRandPoints;
	bool m_bRandPointsInCircle;
	
	float m_fStartPos[3];
	float m_fEndPos[3];
	float m_fHitPos[3];
	float m_fHitNormal[3];
	bool m_bHitResult;
	float m_fDistanceToWall;
	float m_fNeighbourhoodRadius;
	float m_fRandomRadius;
	bool m_bStartPosSet;
	bool m_bEndPosSet;

	int m_nPathIterNum;
	dtPolyRef m_PathIterPolys[MAX_POLYS]; 
	int m_nPathIterPolyCount;
	float m_fPrevIterPos[3], m_fIterPos[3], m_fSteerPos[3], m_fTargetPos[3];
	
	static const int MAX_STEER_POINTS = 10;
	float m_fSteerPoints[MAX_STEER_POINTS*3];
	int m_nSteerPointCount;
	
public:
	NavMeshTesterTool();
	~NavMeshTesterTool();

	virtual int type() { return TOOL_NAVMESH_TESTER; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);

	void recalc();
	void drawAgent(const float* pos, float r, float h, float c, const unsigned int col);
};

#endif // NAVMESHTESTERTOOL_H