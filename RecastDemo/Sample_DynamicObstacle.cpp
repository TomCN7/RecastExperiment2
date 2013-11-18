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
#include "Sample_DynamicObstacles.h"
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

void Sample_DynamicObstacles::handleSettings()
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

