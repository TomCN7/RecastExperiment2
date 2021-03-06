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

#include <float.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include <new>


static void calculateDistanceField(rcCompactHeightfield& chf, unsigned short* src, unsigned short& maxDist)
{
	const int w = chf.nWidth;
	const int h = chf.nHeight;
	
	// Init distance and points.
	for (int i = 0; i < chf.nSpanCount; ++i)
		src[i] = 0xffff;
	
	// Mark boundary cells.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.pCompactCells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.pCompactSpans[i];
				const unsigned char area = chf.pAreas[i];
				
				int nc = 0;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, dir);
						if (area == chf.pAreas[ai])
							nc++;
					}
				}
				if (nc != 4) // at least one neighbor is not accessible  
					src[i] = 0;
			}
		}
	}
	
			
	// Pass 1
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{ // to right, to top
			const rcCompactCell& c = chf.pCompactCells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.pCompactSpans[i];
				
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0) Left
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, 0);
					const rcCompactSpan& as = chf.pCompactSpans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (-1,-1) left,down
					if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(3);
						const int aay = ay + rcGetDirOffsetY(3);
						const int aai = (int)chf.pCompactCells[aax+aay*w].index + rcGetCon(as, 3);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1) down
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, 3);
					const rcCompactSpan& as = chf.pCompactSpans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (1,-1) right, down
					if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(2);
						const int aay = ay + rcGetDirOffsetY(2);
						const int aai = (int)chf.pCompactCells[aax+aay*w].index + rcGetCon(as, 2);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}
	
	// Pass 2
	for (int y = h-1; y >= 0; --y)
	{
		for (int x = w-1; x >= 0; --x)
		{ // down, left
			const rcCompactCell& c = chf.pCompactCells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.pCompactSpans[i];
				
				if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
				{
					// (1,0) right
					const int ax = x + rcGetDirOffsetX(2);
					const int ay = y + rcGetDirOffsetY(2);
					const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, 2);
					const rcCompactSpan& as = chf.pCompactSpans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (1,1) right, up
					if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(1);
						const int aay = ay + rcGetDirOffsetY(1);
						const int aai = (int)chf.pCompactCells[aax+aay*w].index + rcGetCon(as, 1);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
				if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
				{
					// (0,1) up
					const int ax = x + rcGetDirOffsetX(1);
					const int ay = y + rcGetDirOffsetY(1);
					const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, 1);
					const rcCompactSpan& as = chf.pCompactSpans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (-1,1) left, up
					if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(0);
						const int aay = ay + rcGetDirOffsetY(0);
						const int aai = (int)chf.pCompactCells[aax+aay*w].index + rcGetCon(as, 0);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}	
	
	maxDist = 0;
	for (int i = 0; i < chf.nSpanCount; ++i)
		maxDist = rcMax(src[i], maxDist);
	
}

static unsigned short* boxBlur(rcCompactHeightfield& chf, int thr,
							   unsigned short* src, unsigned short* dst)
{
	const int w = chf.nWidth;
	const int h = chf.nHeight;
	
	thr *= 2;
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.pCompactCells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.pCompactSpans[i];
				const unsigned short cd = src[i];
				if (cd <= thr)
				{
					dst[i] = cd;
					continue;
				}

				int d = (int)cd;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, dir);
						d += (int)src[ai];
						
						const rcCompactSpan& as = chf.pCompactSpans[ai];
						const int dir2 = (dir+1) & 0x3;
						if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
						{
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.pCompactCells[ax2+ay2*w].index + rcGetCon(as, dir2);
							d += (int)src[ai2];
						}
						else
						{
							d += cd;
						}
					}
					else
					{
						d += cd*2;
					}
				}
				dst[i] = (unsigned short)((d+5)/9);
			}
		}
	}
	return dst;
}


static bool floodRegion(int x, int y, int i,
						unsigned short level, unsigned short r,
						rcCompactHeightfield& chf,
						unsigned short* srcReg, unsigned short* srcDist,
						rcIntArray& stack)
{
	const int w = chf.nWidth;
	
	const unsigned char area = chf.pAreas[i];
	
	// Flood fill mark region.
	stack.resize(0);
	stack.push((int)x);
	stack.push((int)y);
	stack.push((int)i);
	srcReg[i] = r;
	srcDist[i] = 0;
	
	unsigned short lev = level >= 2 ? level-2 : 0;
	int count = 0;
	
	while (stack.size() > 0)
	{
		int ci = stack.pop();
		int cy = stack.pop();
		int cx = stack.pop();
		
		const rcCompactSpan& cs = chf.pCompactSpans[ci];
		
		// Check if any of the neighbours already have a valid region set.
		unsigned short ar = 0;
		for (int dir = 0; dir < 4; ++dir)
		{
			// 8 connected
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(cs, dir);
				if (chf.pAreas[ai] != area)
					continue;
				unsigned short nr = srcReg[ai];
				if (nr & RC_BORDER_REG) // Do not take borders into account.
					continue;
				if (nr != 0 && nr != r)
					ar = nr;
				
				const rcCompactSpan& as = chf.pCompactSpans[ai];
				
				const int dir2 = (dir+1) & 0x3;
				if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
				{
					const int ax2 = ax + rcGetDirOffsetX(dir2);
					const int ay2 = ay + rcGetDirOffsetY(dir2);
					const int ai2 = (int)chf.pCompactCells[ax2+ay2*w].index + rcGetCon(as, dir2);
					if (chf.pAreas[ai2] != area)
						continue;
					unsigned short nr2 = srcReg[ai2];
					if (nr2 != 0 && nr2 != r)
						ar = nr2;
				}				
			}
		}
		if (ar != 0)
		{
			srcReg[ci] = 0;
			continue;
		}
		count++;
		
		// Expand neighbours.
		for (int dir = 0; dir < 4; ++dir)
		{
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(cs, dir);
				if (chf.pAreas[ai] != area)
					continue;
				if (chf.pBorderDist[ai] >= lev && srcReg[ai] == 0)
				{
					srcReg[ai] = r;
					srcDist[ai] = 0;
					stack.push(ax);
					stack.push(ay);
					stack.push(ai);
				}
			}
		}
	}
	
	return count > 0;
}

static unsigned short* expandRegions(int maxIter, unsigned short level,
									 rcCompactHeightfield& chf,
									 unsigned short* srcReg, unsigned short* srcDist,
									 unsigned short* dstReg, unsigned short* dstDist, 
									 rcIntArray& stack)
{
	const int w = chf.nWidth;
	const int h = chf.nHeight;

	// Find cells revealed by the raised level.
	stack.resize(0);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.pCompactCells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.pBorderDist[i] >= level && srcReg[i] == 0 && chf.pAreas[i] != RC_NULL_AREA)
				{
					stack.push(x);
					stack.push(y);
					stack.push(i);
				}
			}
		}
	}
	
	int iter = 0;
	while (stack.size() > 0)
	{
		int failed = 0;
		
		memcpy(dstReg, srcReg, sizeof(unsigned short)*chf.nSpanCount);
		memcpy(dstDist, srcDist, sizeof(unsigned short)*chf.nSpanCount);
		
		for (int j = 0; j < stack.size(); j += 3)
		{
			int x = stack[j+0];
			int y = stack[j+1];
			int i = stack[j+2];
			if (i < 0)
			{
				failed++;
				continue;
			}
			
			unsigned short r = srcReg[i];
			unsigned short d2 = 0xffff;
			const unsigned char area = chf.pAreas[i];
			const rcCompactSpan& s = chf.pCompactSpans[i];
			for (int dir = 0; dir < 4; ++dir)
			{
				if (rcGetCon(s, dir) == RC_NOT_CONNECTED) continue;
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, dir);
				if (chf.pAreas[ai] != area) continue;
				if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0)
				{
					if ((int)srcDist[ai]+2 < (int)d2)
					{
						r = srcReg[ai];
						d2 = srcDist[ai]+2;
					}
				}
			}
			if (r)
			{
				stack[j+2] = -1; // mark as used
				dstReg[i] = r;
				dstDist[i] = d2;
			}
			else
			{
				failed++;
			}
		}
		
		// rcSwap source and dest.
		rcSwap(srcReg, dstReg);
		rcSwap(srcDist, dstDist);
		
		if (failed*3 == stack.size())
			break;
		
		if (level > 0)
		{
			++iter;
			if (iter >= maxIter)
				break;
		}
	}
	
	return srcReg;
}


struct rcRegion
{
	inline rcRegion(unsigned short i) :
		spanCount(0),
		id(i),
		areaType(0),
		remap(false),
		visited(false)
	{}
	
	int spanCount;					// Number of spans belonging to this region
	unsigned short id;				// ID of the region
	unsigned char areaType;			// Are type.
	bool remap;
	bool visited;
	rcIntArray connections;
	rcIntArray floors;
};

static void removeAdjacentNeighbours(rcRegion& reg)
{
	// Remove adjacent duplicates.
	for (int i = 0; i < reg.connections.size() && reg.connections.size() > 1; )
	{
		int ni = (i+1) % reg.connections.size();
		if (reg.connections[i] == reg.connections[ni])
		{
			// Remove duplicate
			for (int j = i; j < reg.connections.size()-1; ++j)
				reg.connections[j] = reg.connections[j+1];
			reg.connections.pop();
		}
		else
			++i;
	}
}

static void replaceNeighbour(rcRegion& reg, unsigned short oldId, unsigned short newId)
{
	bool neiChanged = false;
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == oldId)
		{
			reg.connections[i] = newId;
			neiChanged = true;
		}
	}
	for (int i = 0; i < reg.floors.size(); ++i)
	{
		if (reg.floors[i] == oldId)
			reg.floors[i] = newId;
	}
	if (neiChanged)
		removeAdjacentNeighbours(reg);
}

static bool canMergeWithRegion(const rcRegion& rega, const rcRegion& regb)
{
	if (rega.areaType != regb.areaType)
		return false;
	int n = 0;
	for (int i = 0; i < rega.connections.size(); ++i)
	{
		if (rega.connections[i] == regb.id)
			n++;
	}
	if (n > 1)
		return false;
	for (int i = 0; i < rega.floors.size(); ++i)
	{
		if (rega.floors[i] == regb.id)
			return false;
	}
	return true;
}

static void addUniqueFloorRegion(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.floors.size(); ++i)
		if (reg.floors[i] == n)
			return;
	reg.floors.push(n);
}

static bool mergeRegions(rcRegion& rega, rcRegion& regb)
{
	unsigned short aid = rega.id;
	unsigned short bid = regb.id;
	
	// Duplicate current neighbourhood.
	rcIntArray acon;
	acon.resize(rega.connections.size());
	for (int i = 0; i < rega.connections.size(); ++i)
		acon[i] = rega.connections[i];
	rcIntArray& bcon = regb.connections;
	
	// Find insertion point on A.
	int insa = -1;
	for (int i = 0; i < acon.size(); ++i)
	{
		if (acon[i] == bid)
		{
			insa = i;
			break;
		}
	}
	if (insa == -1)
		return false;
	
	// Find insertion point on B.
	int insb = -1;
	for (int i = 0; i < bcon.size(); ++i)
	{
		if (bcon[i] == aid)
		{
			insb = i;
			break;
		}
	}
	if (insb == -1)
		return false;
	
	// Merge neighbours.
	rega.connections.resize(0);
	for (int i = 0, ni = acon.size(); i < ni-1; ++i)
		rega.connections.push(acon[(insa+1+i) % ni]);
		
	for (int i = 0, ni = bcon.size(); i < ni-1; ++i)
		rega.connections.push(bcon[(insb+1+i) % ni]);
	
	removeAdjacentNeighbours(rega);
	
	for (int j = 0; j < regb.floors.size(); ++j)
		addUniqueFloorRegion(rega, regb.floors[j]);
	rega.spanCount += regb.spanCount;
	regb.spanCount = 0;
	regb.connections.resize(0);

	return true;
}

static bool isRegionConnectedToBorder(const rcRegion& reg)
{
	// Region is connected to border if
	// one of the neighbours is null id.
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == 0)
			return true;
	}
	return false;
}

static bool isSolidEdge(rcCompactHeightfield& chf, unsigned short* srcReg,
						int x, int y, int i, int dir)
{
	const rcCompactSpan& s = chf.pCompactSpans[i];
	unsigned short r = 0;
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.pCompactCells[ax+ay*chf.nWidth].index + rcGetCon(s, dir);
		r = srcReg[ai];
	}
	if (r == srcReg[i])
		return false;
	return true;
}

static void walkContour(int x, int y, int i, int dir,
						rcCompactHeightfield& chf,
						unsigned short* srcReg,
						rcIntArray& cont)
{
	int startDir = dir;
	int starti = i;

	const rcCompactSpan& ss = chf.pCompactSpans[i];
	unsigned short curReg = 0;
	if (rcGetCon(ss, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.pCompactCells[ax+ay*chf.nWidth].index + rcGetCon(ss, dir);
		curReg = srcReg[ai];
	}
	cont.push(curReg);
			
	int iter = 0;
	while (++iter < 40000)
	{
		const rcCompactSpan& s = chf.pCompactSpans[i];
		
		if (isSolidEdge(chf, srcReg, x, y, i, dir))
		{
			// Choose the edge corner
			unsigned short r = 0;
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.pCompactCells[ax+ay*chf.nWidth].index + rcGetCon(s, dir);
				r = srcReg[ai];
			}
			if (r != curReg)
			{
				curReg = r;
				cont.push(curReg);
			}
			
			dir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = chf.pCompactCells[nx+ny*chf.nWidth];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (starti == i && startDir == dir)
		{
			break;
		}
	}

	// Remove adjacent duplicates.
	if (cont.size() > 1)
	{
		for (int j = 0; j < cont.size(); )
		{
			int nj = (j+1) % cont.size();
			if (cont[j] == cont[nj])
			{
				for (int k = j; k < cont.size()-1; ++k)
					cont[k] = cont[k+1];
				cont.pop();
			}
			else
				++j;
		}
	}
}

static bool FilterSmallRegions(rcContext* pCtx, int nMinRegionArea, int nMergeRegionSize,
    unsigned short& uMaxRegionID, rcCompactHeightfield& rCHF, unsigned short* pSrcReg)
{
	const int w = rCHF.nWidth;
	const int h = rCHF.nHeight;
	
	const int nReg = uMaxRegionID + 1;
	rcRegion* pRegions = (rcRegion*)rcAlloc(sizeof(rcRegion)*nReg, RC_ALLOC_TEMP);
	if (!pRegions)
	{
		pCtx->log(RC_LOG_ERROR, "filterSmallRegions: Out of memory 'regions' (%d).", nReg);
		return false;
	}

	// Construct regions
	for (int i = 0; i < nReg; ++i)
		new(&pRegions[i]) rcRegion((unsigned short)i);
	
	// Find edge of a region and find connections around the contour.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& Cell = rCHF.pCompactCells[x + y * w];
			for (int i = (int)Cell.index, ni = (int)(Cell.index + Cell.count); i < ni; ++i)
			{
				unsigned short uRegion = pSrcReg[i];
				if (uRegion == 0 || uRegion >= nReg)
					continue;
				
				rcRegion& rRegion = pRegions[uRegion];
				rRegion.spanCount++;
				
				
				// Update floors.
				for (int j = (int)Cell.index; j < ni; ++j)
				{
					if (i == j) continue;
					unsigned short uFloorID = pSrcReg[j];
					if (uFloorID == 0 || uFloorID >= nReg)
						continue;
					addUniqueFloorRegion(rRegion, uFloorID);
				}
				
				// Have found contour
				if (rRegion.connections.size() > 0)
					continue;
				
				rRegion.areaType = rCHF.pAreas[i];
				
				// Check if this cell is next to a border.
				int ndir = -1;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (isSolidEdge(rCHF, pSrcReg, x, y, i, dir))
					{
						ndir = dir;
						break;
					}
				}
				
				if (ndir != -1)
				{
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					walkContour(x, y, i, ndir, rCHF, pSrcReg, rRegion.connections);
				}
			}
		}
	}

	// Remove too small regions.
	rcIntArray Stack(32);
	rcIntArray Trace(32);
	for (int i = 0; i < nReg; ++i)
	{
		rcRegion& rRegion = pRegions[i];
		if (rRegion.id == 0 || (rRegion.id & RC_BORDER_REG))
			continue;                       
		if (rRegion.spanCount == 0)
			continue;
		if (rRegion.visited)
			continue;
		
		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		bool bConnectsToBorder = false;
		int nSpanCount = 0;
		Stack.resize(0);
		Trace.resize(0);

		rRegion.visited = true;
		Stack.push(i);
		
		while (Stack.size())
		{
			// Pop
			int ri = Stack.pop();
			
			rcRegion& creg = pRegions[ri];

			nSpanCount += creg.spanCount;
			Trace.push(ri);

			for (int j = 0; j < creg.connections.size(); ++j)
			{
				if (creg.connections[j] & RC_BORDER_REG)
				{
					bConnectsToBorder = true;
					continue;
				}
				rcRegion& neireg = pRegions[creg.connections[j]];
				if (neireg.visited)
					continue;
				if (neireg.id == 0 || (neireg.id & RC_BORDER_REG))
					continue;
				// Visit
				Stack.push(neireg.id);
				neireg.visited = true;
			}
		}
		
		// If the accumulated regions size is too small, remove it.
		// Do not remove areas which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areas.
		if (nSpanCount < nMinRegionArea && !bConnectsToBorder)
		{
			// Kill all visited regions.
			for (int j = 0; j < Trace.size(); ++j)
			{
				pRegions[Trace[j]].spanCount = 0;
				pRegions[Trace[j]].id = 0;
			}
		}
	}
		
	// Merge too small regions to neighbour regions.
	int mergeCount = 0 ;
	do
	{
		mergeCount = 0;
		for (int i = 0; i < nReg; ++i)
		{
			rcRegion& reg = pRegions[i];
			if (reg.id == 0 || (reg.id & RC_BORDER_REG))
				continue;                       
			if (reg.spanCount == 0)
				continue;
			
			// Check to see if the region should be merged.
			if (reg.spanCount > nMergeRegionSize && isRegionConnectedToBorder(reg))
				continue;
			
			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			int smallest = 0xfffffff;
			unsigned short mergeId = reg.id;
			for (int j = 0; j < reg.connections.size(); ++j)
			{
				if (reg.connections[j] & RC_BORDER_REG) continue;
				rcRegion& mreg = pRegions[reg.connections[j]];
				if (mreg.id == 0 || (mreg.id & RC_BORDER_REG)) continue;
				if (mreg.spanCount < smallest &&
					canMergeWithRegion(reg, mreg) &&
					canMergeWithRegion(mreg, reg))
				{
					smallest = mreg.spanCount;
					mergeId = mreg.id;
				}
			}
			// Found new id.
			if (mergeId != reg.id)
			{
				unsigned short oldId = reg.id;
				rcRegion& target = pRegions[mergeId];
				
				// Merge neighbours.
				if (mergeRegions(target, reg))
				{
					// Fixup regions pointing to current region.
					for (int j = 0; j < nReg; ++j)
					{
						if (pRegions[j].id == 0 || (pRegions[j].id & RC_BORDER_REG)) continue;
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if (pRegions[j].id == oldId)
							pRegions[j].id = mergeId;
						// Replace the current region with the new one if the
						// current regions is neighbour.
						replaceNeighbour(pRegions[j], oldId, mergeId);
					}
					mergeCount++;
				}
			}
		}
	}
	while (mergeCount > 0);
	
	// Compress region Ids.
	for (int i = 0; i < nReg; ++i)
	{
		pRegions[i].remap = false;
		if (pRegions[i].id == 0) continue;       // Skip nil regions.
		if (pRegions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
		pRegions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nReg; ++i)
	{
		if (!pRegions[i].remap)
			continue;
		unsigned short oldId = pRegions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nReg; ++j)
		{
			if (pRegions[j].id == oldId)
			{
				pRegions[j].id = newId;
				pRegions[j].remap = false;
			}
		}
	}
	uMaxRegionID = regIdGen;
	
	// Remap regions.
	for (int i = 0; i < rCHF.nSpanCount; ++i)
	{
		if ((pSrcReg[i] & RC_BORDER_REG) == 0)
			pSrcReg[i] = pRegions[pSrcReg[i]].id;
	}
	
	for (int i = 0; i < nReg; ++i)
		pRegions[i].~rcRegion();
	rcFree(pRegions);
	
	return true;
}

/// @par
/// 
/// This is usually the second to the last step in creating a fully built
/// compact height field.  This step is required before regions are built
/// using #rcBuildRegions or #rcBuildRegionsMonotone.
/// 
/// After this step, the distance data is available via the rcCompactHeightfield::maxDistance
/// and rcCompactHeightfield::dist fields.
///
/// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_BUILD_DISTANCEFIELD);
	
	if (chf.pBorderDist)
	{
		rcFree(chf.pBorderDist);
		chf.pBorderDist = 0;
	}
	
	unsigned short* src = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.nSpanCount, RC_ALLOC_TEMP);
	if (!src)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'src' (%d).", chf.nSpanCount);
		return false;
	}
	unsigned short* dst = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.nSpanCount, RC_ALLOC_TEMP);
	if (!dst)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'dst' (%d).", chf.nSpanCount);
		rcFree(src);
		return false;
	}
	
	unsigned short maxDist = 0;

	ctx->startTimer(RC_TIMER_BUILD_DISTANCEFIELD_DIST);
	
	calculateDistanceField(chf, src, maxDist);
	chf.uMaxDistance = maxDist;
	
	ctx->stopTimer(RC_TIMER_BUILD_DISTANCEFIELD_DIST);
	
	ctx->startTimer(RC_TIMER_BUILD_DISTANCEFIELD_BLUR);
	
	// Blur
	if (boxBlur(chf, 1, src, dst) != src)
		rcSwap(src, dst);
	
	// Store distance.
	chf.pBorderDist = src;
	
	ctx->stopTimer(RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

	ctx->stopTimer(RC_TIMER_BUILD_DISTANCEFIELD);
	
	rcFree(dst);
	
	return true;
}

static void paintRectRegion(int minx, int maxx, int miny, int maxy, unsigned short regId,
							rcCompactHeightfield& chf, unsigned short* srcReg)
{
	const int w = chf.nWidth;	
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			const rcCompactCell& c = chf.pCompactCells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.pAreas[i] != RC_NULL_AREA)
					srcReg[i] = regId;
			}
		}
	}
}


static const unsigned short RC_NULL_NEI = 0xffff;

struct rcSweepSpan
{
	unsigned short rid;	// row id
	unsigned short id;	// region id
	unsigned short ns;	// number samples
	unsigned short nei;	// neighbour id
};

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps 
/// reduce unecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
    const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.nWidth;
	const int h = chf.nHeight;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.nSpanCount, RC_ALLOC_TEMP);
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'src' (%d).", chf.nSpanCount);
		return false;
	}
	memset(srcReg ,0, sizeof(unsigned short) * chf.nSpanCount);

	const int nsweeps = rcMax(chf.nWidth, chf.nHeight);
	rcScopedDelete<rcSweepSpan> sweeps = (rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan) * nsweeps, RC_ALLOC_TEMP);
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
		
		chf.nBorderSize = borderSize;
	}
	
	rcIntArray prev(256);

	// Sweep one line at a time.
	for (int y = borderSize; y < h - borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id + 1);
		memset(&prev[0], 0, sizeof(int) * id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.pCompactCells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.pCompactSpans[i];
				if (chf.pAreas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.pAreas[i] == chf.pAreas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}

				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.pCompactCells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.pAreas[i] == chf.pAreas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}

				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.pCompactCells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}

	ctx->startTimer(RC_TIMER_BUILD_REGIONS_FILTER);

	// Filter out small regions.
	chf.uMaxRegions = id;
	if (!FilterSmallRegions(ctx, minRegionArea, mergeRegionArea, chf.uMaxRegions, chf, srcReg))
		return false;

	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_FILTER);
	
	// Store the result out.
	for (int i = 0; i < chf.nSpanCount; ++i)
		chf.pCompactSpans[i].reg = srcReg[i];
	
	ctx->stopTimer(RC_TIMER_BUILD_REGIONS);

	return true;
}

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors. 
/// @p mergeRegionArea helps reduce unecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
					const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.nWidth;
	const int h = chf.nHeight;
	
	rcScopedDelete<unsigned short> buf = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.nSpanCount*4, RC_ALLOC_TEMP);
	if (!buf)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegions: Out of memory 'tmp' (%d).", chf.nSpanCount*4);
		return false;
	}
	
	ctx->startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);
	
	rcIntArray stack(1024);
	rcIntArray visited(1024);
	
	unsigned short* srcReg = buf;
	unsigned short* srcDist = buf+chf.nSpanCount;
	unsigned short* dstReg = buf+chf.nSpanCount * 2;
	unsigned short* dstDist = buf+chf.nSpanCount * 3;
	
	memset(srcReg, 0, sizeof(unsigned short)*chf.nSpanCount);
	memset(srcDist, 0, sizeof(unsigned short)*chf.nSpanCount);
	
	unsigned short regionId = 1;
	unsigned short level = (chf.uMaxDistance+1) & ~1;

	// TODO: Figure better formula, expandIters defines how much the 
	// watershed "overflows" and simplifies the regions. Tying it to
	// agent radius was usually good indication how greedy it could be.
//	const int expandIters = 4 + walkableRadius * 2;
	const int expandIters = 8;

	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(w-bw, w, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, 0, bh, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, h-bh, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;

		chf.nBorderSize = borderSize;
	}
	
	while (level > 0)
	{
		level = level >= 2 ? level-2 : 0;
		
		ctx->startTimer(RC_TIMER_BUILD_REGIONS_EXPAND);
		
		// Expand current regions until no empty connected cells found.
		if (expandRegions(expandIters, level, chf, srcReg, srcDist, dstReg, dstDist, stack) != srcReg)
		{
			rcSwap(srcReg, dstReg);
			rcSwap(srcDist, dstDist);
		}
		
		ctx->stopTimer(RC_TIMER_BUILD_REGIONS_EXPAND);
		
		ctx->startTimer(RC_TIMER_BUILD_REGIONS_FLOOD);
		
		// Mark new regions with IDs.
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.pCompactCells[x+y*w];
				for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
				{
					if (chf.pBorderDist[i] < level || srcReg[i] != 0 || chf.pAreas[i] == RC_NULL_AREA)
						continue;
					if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack))
						regionId++;
				}
			}
		}
		
		ctx->stopTimer(RC_TIMER_BUILD_REGIONS_FLOOD);
	}
	
	// Expand current regions until no empty connected cells found.
	if (expandRegions(expandIters*8, 0, chf, srcReg, srcDist, dstReg, dstDist, stack) != srcReg)
	{
		rcSwap(srcReg, dstReg);
		rcSwap(srcDist, dstDist);
	}
	
	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);
	
	ctx->startTimer(RC_TIMER_BUILD_REGIONS_FILTER);
	
	// Filter out small regions.
	chf.uMaxRegions = regionId;
	if (!FilterSmallRegions(ctx, minRegionArea, mergeRegionArea, chf.uMaxRegions, chf, srcReg))
		return false;
	
	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_FILTER);
		
	// Write the result out.
	for (int i = 0; i < chf.nSpanCount; ++i)
		chf.pCompactSpans[i].reg = srcReg[i];
	
	ctx->stopTimer(RC_TIMER_BUILD_REGIONS);
	
	return true;
}


