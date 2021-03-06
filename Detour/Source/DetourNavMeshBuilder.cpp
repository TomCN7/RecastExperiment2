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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourNavMeshBuilder.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"

static unsigned short MESH_NULL_IDX = 0xffff;


struct BVItem
{
	unsigned short bmin[3];
	unsigned short bmax[3];
	int i;
};

static int compareItemX(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[0] < b->bmin[0])
		return -1;
	if (a->bmin[0] > b->bmin[0])
		return 1;
	return 0;
}

static int compareItemY(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[1] < b->bmin[1])
		return -1;
	if (a->bmin[1] > b->bmin[1])
		return 1;
	return 0;
}

static int compareItemZ(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[2] < b->bmin[2])
		return -1;
	if (a->bmin[2] > b->bmin[2])
		return 1;
	return 0;
}

static void calcExtends(BVItem* items, const int /*nitems*/, const int imin, const int imax,
						unsigned short* bmin, unsigned short* bmax)
{
	bmin[0] = items[imin].bmin[0];
	bmin[1] = items[imin].bmin[1];
	bmin[2] = items[imin].bmin[2];
	
	bmax[0] = items[imin].bmax[0];
	bmax[1] = items[imin].bmax[1];
	bmax[2] = items[imin].bmax[2];
	
	for (int i = imin+1; i < imax; ++i)
	{
		const BVItem& it = items[i];
		if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
		if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];
		if (it.bmin[2] < bmin[2]) bmin[2] = it.bmin[2];
		
		if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
		if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
		if (it.bmax[2] > bmax[2]) bmax[2] = it.bmax[2];
	}
}

inline int longestAxis(unsigned short x, unsigned short y, unsigned short z)
{
	int	axis = 0;
	unsigned short maxVal = x;
	if (y > maxVal)
	{
		axis = 1;
		maxVal = y;
	}
	if (z > maxVal)
	{
		axis = 2;
		maxVal = z;
	}
	return axis;
}

static void subdivide(BVItem* items, int nitems, int imin, int imax, int& curNode, dtBVNode* nodes)
{
	int inum = imax - imin;
	int icur = curNode;
	
	dtBVNode& node = nodes[curNode++];
	
	if (inum == 1)
	{
		// Leaf
		node.bmin[0] = items[imin].bmin[0];
		node.bmin[1] = items[imin].bmin[1];
		node.bmin[2] = items[imin].bmin[2];
		
		node.bmax[0] = items[imin].bmax[0];
		node.bmax[1] = items[imin].bmax[1];
		node.bmax[2] = items[imin].bmax[2];
		
		node.i = items[imin].i;
	}
	else
	{
		// Split
		calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);
		
		int	axis = longestAxis(node.bmax[0] - node.bmin[0],
							   node.bmax[1] - node.bmin[1],
							   node.bmax[2] - node.bmin[2]);
		
		if (axis == 0)
		{
			// Sort along x-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemX);
		}
		else if (axis == 1)
		{
			// Sort along y-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemY);
		}
		else
		{
			// Sort along z-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemZ);
		}
		
		int isplit = imin+inum/2;
		
		// Left
		subdivide(items, nitems, imin, isplit, curNode, nodes);
		// Right
		subdivide(items, nitems, isplit, imax, curNode, nodes);
		
		int iescape = curNode - icur;
		// Negative index means escape.
		node.i = -iescape;
	}
}

static int createBVTree(const unsigned short* verts, const int /*nverts*/,
						const unsigned short* polys, const int npolys, const int nvp,
						const float cs, const float ch,
						const int /*nnodes*/, dtBVNode* nodes)
{
	// Build tree
	BVItem* items = (BVItem*)dtAlloc(sizeof(BVItem)*npolys, DT_ALLOC_TEMP);
	for (int i = 0; i < npolys; i++)
	{
		BVItem& it = items[i];
		it.i = i;
		// Calc polygon bounds.
		const unsigned short* p = &polys[i*nvp*2];
		it.bmin[0] = it.bmax[0] = verts[p[0]*3+0];
		it.bmin[1] = it.bmax[1] = verts[p[0]*3+1];
		it.bmin[2] = it.bmax[2] = verts[p[0]*3+2];
		
		for (int j = 1; j < nvp; ++j)
		{
			if (p[j] == MESH_NULL_IDX) break;
			unsigned short x = verts[p[j]*3+0];
			unsigned short y = verts[p[j]*3+1];
			unsigned short z = verts[p[j]*3+2];
			
			if (x < it.bmin[0]) it.bmin[0] = x;
			if (y < it.bmin[1]) it.bmin[1] = y;
			if (z < it.bmin[2]) it.bmin[2] = z;
			
			if (x > it.bmax[0]) it.bmax[0] = x;
			if (y > it.bmax[1]) it.bmax[1] = y;
			if (z > it.bmax[2]) it.bmax[2] = z;
		}
		// Remap y
		it.bmin[1] = (unsigned short)floorf((float)it.bmin[1]*ch/cs);
		it.bmax[1] = (unsigned short)ceilf((float)it.bmax[1]*ch/cs);
	}
	
	int curNode = 0;
	subdivide(items, npolys, 0, npolys, curNode, nodes);
	
	dtFree(items);
	
	return curNode;
}

static unsigned char classifyOffMeshPoint(const float* pt, const float* bmin, const float* bmax)
{
	static const unsigned char XP = 1<<0;
	static const unsigned char ZP = 1<<1;
	static const unsigned char XM = 1<<2;
	static const unsigned char ZM = 1<<3;	

	unsigned char outcode = 0; 
	outcode |= (pt[0] >= bmax[0]) ? XP : 0;
	outcode |= (pt[2] >= bmax[2]) ? ZP : 0;
	outcode |= (pt[0] < bmin[0])  ? XM : 0;
	outcode |= (pt[2] < bmin[2])  ? ZM : 0;

	switch (outcode)
	{
	case XP: return 0;
	case XP|ZP: return 1;
	case ZP: return 2;
	case XM|ZP: return 3;
	case XM: return 4;
	case XM|ZM: return 5;
	case ZM: return 6;
	case XP|ZM: return 7;
	};

	return 0xff;	
}

// TODO: Better error handling.

/// @par
/// 
/// The output data array is allocated using the detour allocator (dtAlloc()).  The method
/// used to free the memory will be determined by how the tile is added to the navigation
/// mesh.
///
/// @see dtNavMesh, dtNavMesh::addTile()
bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize)
{
	if (params->nMaxVertNumPerPoly > DT_VERTS_PER_POLYGON)
		return false;
	if (params->nVertCount >= 0xffff)
		return false;
	if (!params->nVertCount || !params->pVerts)
		return false;
	if (!params->nPolyCount || !params->pPolys)
		return false;

	const int nvp = params->nMaxVertNumPerPoly;
	
	// Classify off-mesh connection points. We store only the connections
	// whose start point is inside the tile.
	unsigned char* offMeshConClass = 0;
	int storedOffMeshConCount = 0;
	int offMeshConLinkCount = 0;
	
	if (params->nOffMeshConCount > 0)
	{
		offMeshConClass = (unsigned char*)dtAlloc(sizeof(unsigned char)*params->nOffMeshConCount*2, DT_ALLOC_TEMP);
		if (!offMeshConClass)
			return false;

		// Find tight heigh bounds, used for culling out off-mesh start locations.
		float hmin = FLT_MAX;
		float hmax = -FLT_MAX;
		
		if (params->pDetailVerts && params->nDetailVertsCount)
		{
			for (int i = 0; i < params->nDetailVertsCount; ++i)
			{
				const float h = params->pDetailVerts[i*3+1];
				hmin = dtMin(hmin,h);
				hmax = dtMax(hmax,h);
			}
		}
		else
		{
			for (int i = 0; i < params->nVertCount; ++i)
			{
				const unsigned short* iv = &params->pVerts[i*3];
				const float h = params->fBMin[1] + iv[1] * params->fCellHeight;
				hmin = dtMin(hmin,h);
				hmax = dtMax(hmax,h);
			}
		}
		hmin -= params->fWalkableClimb;
		hmax += params->fWalkableClimb;
		float bmin[3], bmax[3];
		dtVcopy(bmin, params->fBMin);
		dtVcopy(bmax, params->fBMax);
		bmin[1] = hmin;
		bmax[1] = hmax;

		for (int i = 0; i < params->nOffMeshConCount; ++i)
		{
			const float* p0 = &params->pOffMeshConVerts[(i*2+0)*3];
			const float* p1 = &params->pOffMeshConVerts[(i*2+1)*3];
			offMeshConClass[i*2+0] = classifyOffMeshPoint(p0, bmin, bmax);
			offMeshConClass[i*2+1] = classifyOffMeshPoint(p1, bmin, bmax);

			// Zero out off-mesh start positions which are not even potentially touching the mesh.
			if (offMeshConClass[i*2+0] == 0xff)
			{
				if (p0[1] < bmin[1] || p0[1] > bmax[1])
					offMeshConClass[i*2+0] = 0;
			}

			// Cound how many links should be allocated for off-mesh connections.
			if (offMeshConClass[i*2+0] == 0xff)
				offMeshConLinkCount++;
			if (offMeshConClass[i*2+1] == 0xff)
				offMeshConLinkCount++;

			if (offMeshConClass[i*2+0] == 0xff)
				storedOffMeshConCount++;
		}
	}
	
	// Off-mesh connectionss are stored as polygons, adjust values.
	const int totPolyCount = params->nPolyCount + storedOffMeshConCount;
	const int totVertCount = params->nVertCount + storedOffMeshConCount*2;
	
	// Find portal edges which are at tile borders.
	int edgeCount = 0;
	int portalCount = 0;
	for (int i = 0; i < params->nPolyCount; ++i)
	{
		const unsigned short* p = &params->pPolys[i*2*nvp];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == MESH_NULL_IDX) break;
			edgeCount++;
			
			if (p[nvp+j] & 0x8000)
			{
				unsigned short dir = p[nvp+j] & 0xf;
				if (dir != 0xf)
					portalCount++;
			}
		}
	}

	const int maxLinkCount = edgeCount + portalCount*2 + offMeshConLinkCount*2;
	
	// Find unique detail vertices.
	int uniqueDetailVertCount = 0;
	int detailTriCount = 0;
	if (params->pDetailMeshes)
	{
		// Has detail mesh, count unique detail vertex count and use input detail tri count.
		detailTriCount = params->nDetailTriCount;
		for (int i = 0; i < params->nPolyCount; ++i)
		{
			const unsigned short* p = &params->pPolys[i*nvp*2];
			int ndv = params->pDetailMeshes[i*4+1];
			int nv = 0;
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;
				nv++;
			}
			ndv -= nv;
			uniqueDetailVertCount += ndv;
		}
	}
	else
	{
		// No input detail mesh, build detail mesh from nav polys.
		uniqueDetailVertCount = 0; // No extra detail verts.
		detailTriCount = 0;
		for (int i = 0; i < params->nPolyCount; ++i)
		{
			const unsigned short* p = &params->pPolys[i*nvp*2];
			int nv = 0;
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;
				nv++;
			}
			detailTriCount += nv-2;
		}
	}
	
	// Calculate data size
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*totVertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*totPolyCount);
	const int linksSize = dtAlign4(sizeof(dtLink)*maxLinkCount);
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*params->nPolyCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*uniqueDetailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*detailTriCount);
	const int bvTreeSize = params->bBuildBvTree ? dtAlign4(sizeof(dtBVNode)*params->nPolyCount*2) : 0;
	const int offMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection)*storedOffMeshConCount);
	
	const int dataSize = headerSize + vertsSize + polysSize + linksSize +
						 detailMeshesSize + detailVertsSize + detailTrisSize +
						 bvTreeSize + offMeshConsSize;
						 
	unsigned char* data = (unsigned char*)dtAlloc(sizeof(unsigned char)*dataSize, DT_ALLOC_PERM);
	if (!data)
	{
		dtFree(offMeshConClass);
		return false;
	}
	memset(data, 0, dataSize);
	
	unsigned char* d = data;
	dtMeshHeader* header = (dtMeshHeader*)d; d += headerSize;
	float* navVerts = (float*)d; d += vertsSize;
	dtPoly* navPolys = (dtPoly*)d; d += polysSize;
	d += linksSize;
	dtPolyDetail* navDMeshes = (dtPolyDetail*)d; d += detailMeshesSize;
	float* navDVerts = (float*)d; d += detailVertsSize;
	unsigned char* navDTris = (unsigned char*)d; d += detailTrisSize;
	dtBVNode* navBvtree = (dtBVNode*)d; d += bvTreeSize;
	dtOffMeshConnection* offMeshCons = (dtOffMeshConnection*)d; d += offMeshConsSize;
	
	
	// Store header
	header->nMagic = DT_NAVMESH_MAGIC;
	header->nVersion = DT_NAVMESH_VERSION;
	header->nTileX = params->nTileX;
	header->nTileY = params->nTileY;
	header->nLayer = params->nTileLayer;
	header->userId = params->nUserID;
	header->nPolyCount = totPolyCount;
	header->nVertCount = totVertCount;
	header->nMaxLinkCount = maxLinkCount;
	dtVcopy(header->fBMin, params->fBMin);
	dtVcopy(header->fBMax, params->fBMax);
	header->nDetailMeshCount = params->nPolyCount;
	header->nDetailVertCount = uniqueDetailVertCount;
	header->nDetailTriCount = detailTriCount;
	header->fBoundingVolumeQuantFactor = 1.0f / params->fCellSize;
	header->nOffMeshBase = params->nPolyCount;
	header->fWalkableHeight = params->fWalkableHeight;
	header->fWalkableRadius = params->fWalkableRadius;
	header->fWalkableClimb = params->fWalkableClimb;
	header->nOffMeshConCount = storedOffMeshConCount;
	header->nBoundingVolumeNodeCount = params->bBuildBvTree ? params->nPolyCount*2 : 0;
	
	const int offMeshVertsBase = params->nVertCount;
	const int offMeshPolyBase = params->nPolyCount;
	
	// Store vertices
	// Mesh vertices
	for (int i = 0; i < params->nVertCount; ++i)
	{
		const unsigned short* iv = &params->pVerts[i*3];
		float* v = &navVerts[i*3];
		v[0] = params->fBMin[0] + iv[0] * params->fCellSize;
		v[1] = params->fBMin[1] + iv[1] * params->fCellHeight;
		v[2] = params->fBMin[2] + iv[2] * params->fCellSize;
	}
	// Off-mesh link vertices.
	int n = 0;
	for (int i = 0; i < params->nOffMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i*2+0] == 0xff)
		{
			const float* linkv = &params->pOffMeshConVerts[i*2*3];
			float* v = &navVerts[(offMeshVertsBase + n*2)*3];
			dtVcopy(&v[0], &linkv[0]);
			dtVcopy(&v[3], &linkv[3]);
			n++;
		}
	}
	
	// Store polygons
	// Mesh polys
	const unsigned short* src = params->pPolys;
	for (int i = 0; i < params->nPolyCount; ++i)
	{
		dtPoly* p = &navPolys[i];
		p->cVertCount = 0;
		p->uFlags = params->pPolyFlags[i];
		p->setArea(params->pPolyAreas[i]);
		p->setType(DT_POLYTYPE_GROUND);
		for (int j = 0; j < nvp; ++j)
		{
			if (src[j] == MESH_NULL_IDX) break;
			p->Verts[j] = src[j];
			if (src[nvp+j] & 0x8000)
			{
				// Border or portal edge.
				unsigned short dir = src[nvp+j] & 0xf;
				if (dir == 0xf) // Border
					p->Neibours[j] = 0;
				else if (dir == 0) // Portal x-
					p->Neibours[j] = DT_EXT_LINK | 4;
				else if (dir == 1) // Portal z+
					p->Neibours[j] = DT_EXT_LINK | 2;
				else if (dir == 2) // Portal x+
					p->Neibours[j] = DT_EXT_LINK | 0;
				else if (dir == 3) // Portal z-
					p->Neibours[j] = DT_EXT_LINK | 6;
			}
			else
			{
				// Normal connection
				p->Neibours[j] = src[nvp+j]+1;
			}
			
			p->cVertCount++;
		}
		src += nvp*2;
	}
	// Off-mesh connection vertices.
	n = 0;
	for (int i = 0; i < params->nOffMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i*2+0] == 0xff)
		{
			dtPoly* p = &navPolys[offMeshPolyBase+n];
			p->cVertCount = 2;
			p->Verts[0] = (unsigned short)(offMeshVertsBase + n*2+0);
			p->Verts[1] = (unsigned short)(offMeshVertsBase + n*2+1);
			p->uFlags = params->pOffMeshConFlags[i];
			p->setArea(params->pOffMeshConAreas[i]);
			p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
			n++;
		}
	}

	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// We compress the mesh data by skipping them and using the navmesh coordinates.
	if (params->pDetailMeshes)
	{
		unsigned short vbase = 0;
		for (int i = 0; i < params->nPolyCount; ++i)
		{
			dtPolyDetail& dtl = navDMeshes[i];
			const int vb = (int)params->pDetailMeshes[i*4+0];
			const int ndv = (int)params->pDetailMeshes[i*4+1];
			const int nv = navPolys[i].cVertCount;
			dtl.uVertBase = (unsigned int)vbase;
			dtl.cVertCount = (unsigned char)(ndv-nv);
			dtl.uTriBase = (unsigned int)params->pDetailMeshes[i*4+2];
			dtl.cTriCount = (unsigned char)params->pDetailMeshes[i*4+3];
			// Copy vertices except the first 'nv' verts which are equal to nav poly verts.
			if (ndv-nv)
			{
				memcpy(&navDVerts[vbase*3], &params->pDetailVerts[(vb+nv)*3], sizeof(float)*3*(ndv-nv));
				vbase += (unsigned short)(ndv-nv);
			}
		}
		// Store triangles.
		memcpy(navDTris, params->pDetailTris, sizeof(unsigned char)*4*params->nDetailTriCount);
	}
	else
	{
		// Create dummy detail mesh by triangulating polys.
		int tbase = 0;
		for (int i = 0; i < params->nPolyCount; ++i)
		{
			dtPolyDetail& dtl = navDMeshes[i];
			const int nv = navPolys[i].cVertCount;
			dtl.uVertBase = 0;
			dtl.cVertCount = 0;
			dtl.uTriBase = (unsigned int)tbase;
			dtl.cTriCount = (unsigned char)(nv-2);
			// Triangulate polygon (local indices).
			for (int j = 2; j < nv; ++j)
			{
				unsigned char* t = &navDTris[tbase*4];
				t[0] = 0;
				t[1] = (unsigned char)(j-1);
				t[2] = (unsigned char)j;
				// Bit for each edge that belongs to poly boundary.
				t[3] = (1<<2);
				if (j == 2) t[3] |= (1<<0);
				if (j == nv-1) t[3] |= (1<<4);
				tbase++;
			}
		}
	}

	// Store and create BVtree.
	// TODO: take detail mesh into account! use byte per bbox extent?
	if (params->bBuildBvTree)
	{
		createBVTree(params->pVerts, params->nVertCount, params->pPolys, params->nPolyCount,
					 nvp, params->fCellSize, params->fCellHeight, params->nPolyCount*2, navBvtree);
	}
	
	// Store Off-Mesh connections.
	n = 0;
	for (int i = 0; i < params->nOffMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i*2+0] == 0xff)
		{
			dtOffMeshConnection* con = &offMeshCons[n];
			con->uPolyRef = (unsigned short)(offMeshPolyBase + n);
			// Copy connection end-points.
			const float* endPts = &params->pOffMeshConVerts[i*2*3];
			dtVcopy(&con->fPosition[0], &endPts[0]);
			dtVcopy(&con->fPosition[3], &endPts[3]);
			con->fRadius = params->pOffMeshConRad[i];
			con->uFlags = params->pOffMeshConDir[i] ? DT_OFFMESH_CON_BIDIR : 0;
			con->uSide = offMeshConClass[i*2+1];
			if (params->pOffMeshConUserID)
				con->uUserID = params->pOffMeshConUserID[i];
			n++;
		}
	}
		
	dtFree(offMeshConClass);
	
	*outData = data;
	*outDataSize = dataSize;
	
	return true;
}

bool dtNavMeshHeaderSwapEndian(unsigned char* data, const int /*dataSize*/)
{
	dtMeshHeader* header = (dtMeshHeader*)data;
	
	int swappedMagic = DT_NAVMESH_MAGIC;
	int swappedVersion = DT_NAVMESH_VERSION;
	dtSwapEndian(&swappedMagic);
	dtSwapEndian(&swappedVersion);
	
	if ((header->nMagic != DT_NAVMESH_MAGIC || header->nVersion != DT_NAVMESH_VERSION) &&
		(header->nMagic != swappedMagic || header->nVersion != swappedVersion))
	{
		return false;
	}
		
	dtSwapEndian(&header->nMagic);
	dtSwapEndian(&header->nVersion);
	dtSwapEndian(&header->nTileX);
	dtSwapEndian(&header->nTileY);
	dtSwapEndian(&header->nLayer);
	dtSwapEndian(&header->userId);
	dtSwapEndian(&header->nPolyCount);
	dtSwapEndian(&header->nVertCount);
	dtSwapEndian(&header->nMaxLinkCount);
	dtSwapEndian(&header->nDetailMeshCount);
	dtSwapEndian(&header->nDetailVertCount);
	dtSwapEndian(&header->nDetailTriCount);
	dtSwapEndian(&header->nBoundingVolumeNodeCount);
	dtSwapEndian(&header->nOffMeshConCount);
	dtSwapEndian(&header->nOffMeshBase);
	dtSwapEndian(&header->fWalkableHeight);
	dtSwapEndian(&header->fWalkableRadius);
	dtSwapEndian(&header->fWalkableClimb);
	dtSwapEndian(&header->fBMin[0]);
	dtSwapEndian(&header->fBMin[1]);
	dtSwapEndian(&header->fBMin[2]);
	dtSwapEndian(&header->fBMax[0]);
	dtSwapEndian(&header->fBMax[1]);
	dtSwapEndian(&header->fBMax[2]);
	dtSwapEndian(&header->fBoundingVolumeQuantFactor);

	// Freelist index and pointers are updated when tile is added, no need to swap.

	return true;
}

/// @par
///
/// @warning This function assumes that the header is in the correct endianess already. 
/// Call #dtNavMeshHeaderSwapEndian() first on the data if the data is expected to be in wrong endianess 
/// to start with. Call #dtNavMeshHeaderSwapEndian() after the data has been swapped if converting from 
/// native to foreign endianess.
bool dtNavMeshDataSwapEndian(unsigned char* data, const int /*dataSize*/)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->nMagic != DT_NAVMESH_MAGIC)
		return false;
	if (header->nVersion != DT_NAVMESH_VERSION)
		return false;
	
	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*header->nVertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*header->nPolyCount);
	const int linksSize = dtAlign4(sizeof(dtLink)*(header->nMaxLinkCount));
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*header->nDetailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*header->nDetailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*header->nDetailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(dtBVNode)*header->nBoundingVolumeNodeCount);
	const int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection)*header->nOffMeshConCount);
	
	unsigned char* d = data + headerSize;
	float* verts = (float*)d; d += vertsSize;
	dtPoly* polys = (dtPoly*)d; d += polysSize;
	/*dtLink* links = (dtLink*)d;*/ d += linksSize;
	dtPolyDetail* detailMeshes = (dtPolyDetail*)d; d += detailMeshesSize;
	float* detailVerts = (float*)d; d += detailVertsSize;
	/*unsigned char* detailTris = (unsigned char*)d;*/ d += detailTrisSize;
	dtBVNode* bvTree = (dtBVNode*)d; d += bvtreeSize;
	dtOffMeshConnection* offMeshCons = (dtOffMeshConnection*)d; d += offMeshLinksSize;
	
	// Vertices
	for (int i = 0; i < header->nVertCount*3; ++i)
	{
		dtSwapEndian(&verts[i]);
	}

	// Polys
	for (int i = 0; i < header->nPolyCount; ++i)
	{
		dtPoly* p = &polys[i];
		// poly->firstLink is update when tile is added, no need to swap.
		for (int j = 0; j < DT_VERTS_PER_POLYGON; ++j)
		{
			dtSwapEndian(&p->Verts[j]);
			dtSwapEndian(&p->Neibours[j]);
		}
		dtSwapEndian(&p->uFlags);
	}

	// Links are rebuild when tile is added, no need to swap.

	// Detail meshes
	for (int i = 0; i < header->nDetailMeshCount; ++i)
	{
		dtPolyDetail* pd = &detailMeshes[i];
		dtSwapEndian(&pd->uVertBase);
		dtSwapEndian(&pd->uTriBase);
	}
	
	// Detail verts
	for (int i = 0; i < header->nDetailVertCount*3; ++i)
	{
		dtSwapEndian(&detailVerts[i]);
	}

	// BV-tree
	for (int i = 0; i < header->nBoundingVolumeNodeCount; ++i)
	{
		dtBVNode* node = &bvTree[i];
		for (int j = 0; j < 3; ++j)
		{
			dtSwapEndian(&node->bmin[j]);
			dtSwapEndian(&node->bmax[j]);
		}
		dtSwapEndian(&node->i);
	}

	// Off-mesh Connections.
	for (int i = 0; i < header->nOffMeshConCount; ++i)
	{
		dtOffMeshConnection* con = &offMeshCons[i];
		for (int j = 0; j < 6; ++j)
			dtSwapEndian(&con->fPosition[j]);
		dtSwapEndian(&con->fRadius);
		dtSwapEndian(&con->uPolyRef);
	}
	
	return true;
}
