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
#include <stdarg.h>
#include <string.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastDump.h"


duFileIO::~duFileIO()
{
	// Empty
}
	
static void ioprintf(duFileIO* io, const char* format, ...)
{
	char line[256];
	va_list ap;
	va_start(ap, format);
	const int n = vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	if (n > 0)
		io->write(line, sizeof(char)*n);
}

bool duDumpPolyMeshToObj(rcPolyMesh& pmesh, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpPolyMeshToObj: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpPolyMeshToObj: input IO not writing.\n"); 
		return false;
	}
	
	const int nvp = pmesh.nVertexNumPerPoly;
	const float cs = pmesh.fCellSize;
	const float ch = pmesh.fCellHeight;
	const float* orig = pmesh.fBMin;
	
	ioprintf(io, "# Recast Navmesh\n");
	ioprintf(io, "o NavMesh\n");

	ioprintf(io, "\n");
	
	for (int i = 0; i < pmesh.nVerts; ++i)
	{
		const unsigned short* v = &pmesh.pVerts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		ioprintf(io, "v %f %f %f\n", x,y,z);
	}

	ioprintf(io, "\n");

	for (int i = 0; i < pmesh.nPolys; ++i)
	{
		const unsigned short* p = &pmesh.pPolys[i*nvp*2];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			ioprintf(io, "f %d %d %d\n", p[0]+1, p[j-1]+1, p[j]+1); 
		}
	}
	
	return true;
}

bool duDumpPolyMeshDetailToObj(rcPolyMeshDetail& dmesh, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpPolyMeshDetailToObj: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpPolyMeshDetailToObj: input IO not writing.\n"); 
		return false;
	}
	
	ioprintf(io, "# Recast Navmesh\n");
	ioprintf(io, "o NavMesh\n");
	
	ioprintf(io, "\n");

	for (int i = 0; i < dmesh.nVerts; ++i)
	{
		const float* v = &dmesh.fVerts[i*3];
		ioprintf(io, "v %f %f %f\n", v[0],v[1],v[2]);
	}
	
	ioprintf(io, "\n");
	
	for (int i = 0; i < dmesh.nMeshes; ++i)
	{
		const unsigned int* m = &dmesh.pMeshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const unsigned int ntris = m[3];
		const unsigned char* tris = &dmesh.pTris[btris*4];
		for (unsigned int j = 0; j < ntris; ++j)
		{
			ioprintf(io, "f %d %d %d\n",
					(int)(bverts+tris[j*4+0])+1,
					(int)(bverts+tris[j*4+1])+1,
					(int)(bverts+tris[j*4+2])+1);
		}
	}
	
	return true;
}

static const int CSET_MAGIC = ('c' << 24) | ('s' << 16) | ('e' << 8) | 't';
static const int CSET_VERSION = 2;

bool duDumpContourSet(struct rcContourSet& cset, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpContourSet: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpContourSet: input IO not writing.\n"); 
		return false;
	}
	
	io->write(&CSET_MAGIC, sizeof(CSET_MAGIC));
	io->write(&CSET_VERSION, sizeof(CSET_VERSION));

	io->write(&cset.nContours, sizeof(cset.nContours));
	
	io->write(cset.fBMin, sizeof(cset.fBMin));
	io->write(cset.fBMax, sizeof(cset.fBMax));
	
	io->write(&cset.fCellSize, sizeof(cset.fCellSize));
	io->write(&cset.fCellHeight, sizeof(cset.fCellHeight));

	io->write(&cset.nWidth, sizeof(cset.nWidth));
	io->write(&cset.nHeight, sizeof(cset.nHeight));
	io->write(&cset.nBorderSize, sizeof(cset.nBorderSize));

	for (int i = 0; i < cset.nContours; ++i)
	{
		const rcContour& cont = cset.pContours[i];
		io->write(&cont.nVerts, sizeof(cont.nVerts));
		io->write(&cont.nRawVerts, sizeof(cont.nRawVerts));
		io->write(&cont.uRegionID, sizeof(cont.uRegionID));
		io->write(&cont.uAreaID, sizeof(cont.uAreaID));
		io->write(cont.pVerts, sizeof(int)*4*cont.nVerts);
		io->write(cont.pRawVerts, sizeof(int)*4*cont.nRawVerts);
	}

	return true;
}

bool duReadContourSet(struct rcContourSet& cset, duFileIO* io)
{
	if (!io)
	{
		printf("duReadContourSet: input IO is null.\n"); 
		return false;
	}
	if (!io->isReading())
	{
		printf("duReadContourSet: input IO not reading.\n"); 
		return false;
	}
	
	int magic = 0;
	int version = 0;
	
	io->read(&magic, sizeof(magic));
	io->read(&version, sizeof(version));
	
	if (magic != CSET_MAGIC)
	{
		printf("duReadContourSet: Bad voodoo.\n");
		return false;
	}
	if (version != CSET_VERSION)
	{
		printf("duReadContourSet: Bad version.\n");
		return false;
	}
	
	io->read(&cset.nContours, sizeof(cset.nContours));

	cset.pContours = (rcContour*)rcAlloc(sizeof(rcContour)*cset.nContours, RC_ALLOC_PERM);
	if (!cset.pContours)
	{
		printf("duReadContourSet: Could not alloc contours (%d)\n", cset.nContours);
		return false;
	}
	memset(cset.pContours, 0, sizeof(rcContour)*cset.nContours);
	
	io->read(cset.fBMin, sizeof(cset.fBMin));
	io->read(cset.fBMax, sizeof(cset.fBMax));
	
	io->read(&cset.fCellSize, sizeof(cset.fCellSize));
	io->read(&cset.fCellHeight, sizeof(cset.fCellHeight));
	
	io->read(&cset.nWidth, sizeof(cset.nWidth));
	io->read(&cset.nHeight, sizeof(cset.nHeight));
	io->read(&cset.nBorderSize, sizeof(cset.nBorderSize));
	
	for (int i = 0; i < cset.nContours; ++i)
	{
		rcContour& cont = cset.pContours[i];
		io->read(&cont.nVerts, sizeof(cont.nVerts));
		io->read(&cont.nRawVerts, sizeof(cont.nRawVerts));
		io->read(&cont.uRegionID, sizeof(cont.uRegionID));
		io->read(&cont.uAreaID, sizeof(cont.uAreaID));

		cont.pVerts = (int*)rcAlloc(sizeof(int)*4*cont.nVerts, RC_ALLOC_PERM);
		if (!cont.pVerts)
		{
			printf("duReadContourSet: Could not alloc contour verts (%d)\n", cont.nVerts);
			return false;
		}
		cont.pRawVerts = (int*)rcAlloc(sizeof(int)*4*cont.nRawVerts, RC_ALLOC_PERM);
		if (!cont.pRawVerts)
		{
			printf("duReadContourSet: Could not alloc contour rverts (%d)\n", cont.nRawVerts);
			return false;
		}
		
		io->read(cont.pVerts, sizeof(int)*4*cont.nVerts);
		io->read(cont.pRawVerts, sizeof(int)*4*cont.nRawVerts);
	}
	
	return true;
}
	

static const int CHF_MAGIC = ('r' << 24) | ('c' << 16) | ('h' << 8) | 'f';
static const int CHF_VERSION = 3;

bool duDumpCompactHeightfield(struct rcCompactHeightfield& chf, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpCompactHeightfield: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpCompactHeightfield: input IO not writing.\n"); 
		return false;
	}
	
	io->write(&CHF_MAGIC, sizeof(CHF_MAGIC));
	io->write(&CHF_VERSION, sizeof(CHF_VERSION));
	
	io->write(&chf.nWidth, sizeof(chf.nWidth));
	io->write(&chf.nHeight, sizeof(chf.nHeight));
	io->write(&chf.nSpanCount, sizeof(chf.nSpanCount));

	io->write(&chf.nWalkableHeight, sizeof(chf.nWalkableHeight));
	io->write(&chf.nWalkableClimb, sizeof(chf.nWalkableClimb));
	io->write(&chf.nBorderSize, sizeof(chf.nBorderSize));

	io->write(&chf.uMaxDistance, sizeof(chf.uMaxDistance));
	io->write(&chf.uMaxRegions, sizeof(chf.uMaxRegions));

	io->write(chf.fBMin, sizeof(chf.fBMin));
	io->write(chf.fBMax, sizeof(chf.fBMax));

	io->write(&chf.fCellSize, sizeof(chf.fCellSize));
	io->write(&chf.fCellHeight, sizeof(chf.fCellHeight));

	int tmp = 0;
	if (chf.pCompactCells) tmp |= 1;
	if (chf.pCompactSpans) tmp |= 2;
	if (chf.pBorderDist) tmp |= 4;
	if (chf.pAreas) tmp |= 8;

	io->write(&tmp, sizeof(tmp));

	if (chf.pCompactCells)
		io->write(chf.pCompactCells, sizeof(rcCompactCell)*chf.nWidth*chf.nHeight);
	if (chf.pCompactSpans)
		io->write(chf.pCompactSpans, sizeof(rcCompactSpan)*chf.nSpanCount);
	if (chf.pBorderDist)
		io->write(chf.pBorderDist, sizeof(unsigned short)*chf.nSpanCount);
	if (chf.pAreas)
		io->write(chf.pAreas, sizeof(unsigned char)*chf.nSpanCount);

	return true;
}

bool duReadCompactHeightfield(struct rcCompactHeightfield& chf, duFileIO* io)
{
	if (!io)
	{
		printf("duReadCompactHeightfield: input IO is null.\n"); 
		return false;
	}
	if (!io->isReading())
	{
		printf("duReadCompactHeightfield: input IO not reading.\n"); 
		return false;
	}

	int magic = 0;
	int version = 0;
	
	io->read(&magic, sizeof(magic));
	io->read(&version, sizeof(version));
	
	if (magic != CHF_MAGIC)
	{
		printf("duReadCompactHeightfield: Bad voodoo.\n");
		return false;
	}
	if (version != CHF_VERSION)
	{
		printf("duReadCompactHeightfield: Bad version.\n");
		return false;
	}
	
	io->read(&chf.nWidth, sizeof(chf.nWidth));
	io->read(&chf.nHeight, sizeof(chf.nHeight));
	io->read(&chf.nSpanCount, sizeof(chf.nSpanCount));
	
	io->read(&chf.nWalkableHeight, sizeof(chf.nWalkableHeight));
	io->read(&chf.nWalkableClimb, sizeof(chf.nWalkableClimb));
	io->write(&chf.nBorderSize, sizeof(chf.nBorderSize));

	io->read(&chf.uMaxDistance, sizeof(chf.uMaxDistance));
	io->read(&chf.uMaxRegions, sizeof(chf.uMaxRegions));
	
	io->read(chf.fBMin, sizeof(chf.fBMin));
	io->read(chf.fBMax, sizeof(chf.fBMax));
	
	io->read(&chf.fCellSize, sizeof(chf.fCellSize));
	io->read(&chf.fCellHeight, sizeof(chf.fCellHeight));
	
	int tmp = 0;
	io->read(&tmp, sizeof(tmp));
	
	if (tmp & 1)
	{
		chf.pCompactCells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell)*chf.nWidth*chf.nHeight, RC_ALLOC_PERM);
		if (!chf.pCompactCells)
		{
			printf("duReadCompactHeightfield: Could not alloc cells (%d)\n", chf.nWidth*chf.nHeight);
			return false;
		}
		io->read(chf.pCompactCells, sizeof(rcCompactCell)*chf.nWidth*chf.nHeight);
	}
	if (tmp & 2)
	{
		chf.pCompactSpans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan)*chf.nSpanCount, RC_ALLOC_PERM);
		if (!chf.pCompactSpans)
		{
			printf("duReadCompactHeightfield: Could not alloc spans (%d)\n", chf.nSpanCount);
			return false;
		}
		io->read(chf.pCompactSpans, sizeof(rcCompactSpan)*chf.nSpanCount);
	}
	if (tmp & 4)
	{
		chf.pBorderDist = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.nSpanCount, RC_ALLOC_PERM);
		if (!chf.pBorderDist)
		{
			printf("duReadCompactHeightfield: Could not alloc dist (%d)\n", chf.nSpanCount);
			return false;
		}
		io->read(chf.pBorderDist, sizeof(unsigned short)*chf.nSpanCount);
	}
	if (tmp & 8)
	{
		chf.pAreas = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.nSpanCount, RC_ALLOC_PERM);
		if (!chf.pAreas)
		{
			printf("duReadCompactHeightfield: Could not alloc areas (%d)\n", chf.nSpanCount);
			return false;
		}
		io->read(chf.pAreas, sizeof(unsigned char)*chf.nSpanCount);
	}
	
	return true;
}


static void logLine(rcContext& ctx, rcTimerLabel label, const char* name, const float pc)
{
	const int t = ctx.getAccumulatedTime(label);
	if (t < 0) return;
	ctx.log(RC_LOG_PROGRESS, "%s:\t%.2fms\t(%.1f%%)", name, t/1000.0f, t*pc);
}

void duLogBuildTimes(rcContext& ctx, const int totalTimeUsec)
{
	const float pc = 100.0f / totalTimeUsec;
 
	ctx.log(RC_LOG_PROGRESS, "Build Times");
	logLine(ctx, RC_TIMER_RASTERIZE_TRIANGLES,		"- Rasterize", pc);
	logLine(ctx, RC_TIMER_BUILD_COMPACTHEIGHTFIELD,	"- Build Compact", pc);
	logLine(ctx, RC_TIMER_FILTER_BORDER,				"- Filter Border", pc);
	logLine(ctx, RC_TIMER_FILTER_WALKABLE,			"- Filter Walkable", pc);
	logLine(ctx, RC_TIMER_ERODE_AREA,				"- Erode Area", pc);
	logLine(ctx, RC_TIMER_MEDIAN_AREA,				"- Median Area", pc);
	logLine(ctx, RC_TIMER_MARK_BOX_AREA,				"- Mark Box Area", pc);
	logLine(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA,		"- Mark Convex Area", pc);
	logLine(ctx, RC_TIMER_MARK_CYLINDER_AREA,		"- Mark Cylinder Area", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD,		"- Build Distance Field", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST,	"    - Distance", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR,	"    - Blur", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS,				"- Build Regions", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_WATERSHED,	"    - Watershed", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_EXPAND,		"      - Expand", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FLOOD,		"      - Find Basins", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FILTER,		"    - Filter", pc);
	logLine(ctx, RC_TIMER_BUILD_LAYERS,				"- Build Layers", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS,			"- Build Contours", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_TRACE,		"    - Trace", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_SIMPLIFY,	"    - Simplify", pc);
	logLine(ctx, RC_TIMER_BUILD_POLYMESH,			"- Build Polymesh", pc);
	logLine(ctx, RC_TIMER_BUILD_POLYMESHDETAIL,		"- Build Polymesh Detail", pc);
	logLine(ctx, RC_TIMER_MERGE_POLYMESH,			"- Merge Polymeshes", pc);
	logLine(ctx, RC_TIMER_MERGE_POLYMESHDETAIL,		"- Merge Polymesh Details", pc);
	ctx.log(RC_LOG_PROGRESS, "=== TOTAL:\t%.2fms", totalTimeUsec/1000.0f);
}

