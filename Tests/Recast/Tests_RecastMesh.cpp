#include "Recast.h"
#include "RecastAlloc.h"

#include "catch2/catch_all.hpp"

#include <cstring>

namespace
{

// Cell coordinates of an axis aligned rectangle. Vertices are emitted in the winding
// rcBuildContours produces: (minX, minZ), (minX, maxZ), (maxX, maxZ), (maxX, minZ). Edge j goes
// from vertex j to vertex j+1, so edge 0 is the x- edge, edge 1 the z+ edge, edge 2 the x+ edge
// and edge 3 the z- edge.
struct RectContour
{
	int minX;
	int minZ;
	int maxX;
	int maxZ;

	void getVerts(int (&xz)[4][2]) const
	{
		xz[0][0] = minX; xz[0][1] = minZ;
		xz[1][0] = minX; xz[1][1] = maxZ;
		xz[2][0] = maxX; xz[2][1] = maxZ;
		xz[3][0] = maxX; xz[3][1] = minZ;
	}
};

void addContour(const RectContour& rect, int y, unsigned short reg, unsigned char area, rcContour& cont)
{
	cont.nverts = 4;
	cont.verts = (int*)rcAlloc(sizeof(int) * 4 * cont.nverts, RC_ALLOC_PERM);
	REQUIRE(cont.verts != nullptr);
	int xz[4][2];
	rect.getVerts(xz);
	for (int i = 0; i < 4; ++i)
	{
		cont.verts[i * 4 + 0] = xz[i][0];
		cont.verts[i * 4 + 1] = y;
		cont.verts[i * 4 + 2] = xz[i][1];
		cont.verts[i * 4 + 3] = 0;
	}
	cont.rverts = nullptr;
	cont.nrverts = 0;
	cont.reg = reg;
	cont.area = area;
}

constexpr int walkableHeight = 10;
constexpr int walkableClimb = 4;
// Vertical extent of the source heightfield in cells. rcBuildCompactHeightfield grows the upper
// bound of the contour set by walkableHeight on top of this.
constexpr int slabHeight = 100;

// Builds a contour set covering [0, width] x [0, height] cells with a single rectangular contour
// at the given height in cells.
void makeContourSet(const RectContour& rect, int y, int width, int height, int borderSize, rcContourSet& cset)
{
	cset.conts = (rcContour*)rcAlloc(sizeof(rcContour), RC_ALLOC_PERM);
	REQUIRE(cset.conts != nullptr);
	std::memset(cset.conts, 0, sizeof(rcContour));
	cset.nconts = 1;
	addContour(rect, y, 1, RC_WALKABLE_AREA, cset.conts[0]);
	cset.cs = 0.1f;
	cset.ch = 0.1f;
	cset.bmin[0] = 0;
	cset.bmin[1] = 0;
	cset.bmin[2] = 0;
	cset.bmax[0] = width * cset.cs;
	cset.bmax[1] = (slabHeight + walkableHeight) * cset.ch;
	cset.bmax[2] = height * cset.cs;
	cset.width = width;
	cset.height = height;
	cset.borderSize = borderSize;
	cset.maxError = 1.3f;
}

// Allocates a poly mesh holding a single rectangle with the given per edge adjacency entries.
void makePolyMesh(const RectContour& rect, const unsigned short (&neis)[4], float originX, float originZ,
	float cellSize, int width, int height, rcPolyMesh& mesh)
{
	mesh.nvp = 4;
	mesh.cs = cellSize;
	mesh.ch = cellSize;
	mesh.bmin[0] = originX;
	mesh.bmin[1] = 0;
	mesh.bmin[2] = originZ;
	mesh.bmax[0] = originX + width * cellSize;
	mesh.bmax[1] = 1;
	mesh.bmax[2] = originZ + height * cellSize;
	mesh.borderSize = 0;
	mesh.maxEdgeError = 1.3f;

	mesh.nverts = 4;
	mesh.verts = (unsigned short*)rcAlloc(sizeof(unsigned short) * 3 * mesh.nverts, RC_ALLOC_PERM);
	REQUIRE(mesh.verts != nullptr);
	int xz[4][2];
	rect.getVerts(xz);
	for (int i = 0; i < 4; ++i)
	{
		mesh.verts[i * 3 + 0] = (unsigned short)xz[i][0];
		mesh.verts[i * 3 + 1] = 0;
		mesh.verts[i * 3 + 2] = (unsigned short)xz[i][1];
	}

	mesh.npolys = 1;
	mesh.polys = (unsigned short*)rcAlloc(sizeof(unsigned short) * 2 * mesh.nvp, RC_ALLOC_PERM);
	REQUIRE(mesh.polys != nullptr);
	for (int i = 0; i < 4; ++i)
	{
		mesh.polys[i] = (unsigned short)i;
		mesh.polys[mesh.nvp + i] = neis[i];
	}

	mesh.regs = (unsigned short*)rcAlloc(sizeof(unsigned short), RC_ALLOC_PERM);
	REQUIRE(mesh.regs != nullptr);
	mesh.regs[0] = 1;
	mesh.areas = (unsigned char*)rcAlloc(sizeof(unsigned char), RC_ALLOC_PERM);
	REQUIRE(mesh.areas != nullptr);
	mesh.areas[0] = RC_WALKABLE_AREA;
	mesh.flags = (unsigned short*)rcAlloc(sizeof(unsigned short), RC_ALLOC_PERM);
	REQUIRE(mesh.flags != nullptr);
	mesh.flags[0] = 1;
}

// rcBuildPolyMesh rotates the vertices of a polygon while merging triangles, so edges have to be
// looked up by position rather than by index.
unsigned short getEdgeNei(const rcPolyMesh& mesh, int polyIndex, int ax, int az, int bx, int bz)
{
	const unsigned short* const poly = &mesh.polys[polyIndex * 2 * mesh.nvp];
	int nv = 0;
	while (nv < mesh.nvp && poly[nv] != RC_MESH_NULL_IDX)
		++nv;
	for (int j = 0; j < nv; ++j)
	{
		const unsigned short* const va = &mesh.verts[poly[j] * 3];
		const unsigned short* const vb = &mesh.verts[poly[(j + 1) % nv] * 3];
		if (va[0] == ax && va[2] == az && vb[0] == bx && vb[2] == bz)
			return poly[mesh.nvp + j];
	}
	FAIL("no edge from (" << ax << ", " << az << ") to (" << bx << ", " << bz << ")");
	return RC_MESH_NULL_IDX;
}

constexpr unsigned short portalXMin = 0x8000 | 0;
constexpr unsigned short portalZMax = 0x8000 | 1;
constexpr unsigned short portalXMax = 0x8000 | 2;
constexpr unsigned short portalZMin = 0x8000 | 3;

}

TEST_CASE("rcBuildContours grows the upper vertical bound by the walkable height", "[recast]")
{
	rcContext context;

	// One flat quad, large enough for a region to survive the pipeline.
	const float verts[] = {
		0, 1, 0,
		0, 1, 4,
		4, 1, 4,
		4, 1, 0,
	};
	const int tris[] = {0, 1, 2, 0, 2, 3};
	const float bmin[3] = {0, 0, 0};
	const float bmax[3] = {4, 3, 4};
	constexpr float cs = 0.1f;
	constexpr float ch = 0.1f;

	rcHeightfield solid;
	REQUIRE(rcCreateHeightfield(&context, solid, 40, 40, bmin, bmax, cs, ch));

	unsigned char areas[2] = {RC_WALKABLE_AREA, RC_WALKABLE_AREA};
	REQUIRE(rcRasterizeTriangles(&context, verts, 4, tris, areas, 2, solid, walkableClimb));

	rcCompactHeightfield compact;
	REQUIRE(rcBuildCompactHeightfield(&context, walkableHeight, walkableClimb, solid, compact));
	REQUIRE(rcBuildDistanceField(&context, compact));
	REQUIRE(rcBuildRegions(&context, compact, 0, 8, 20));

	rcContourSet cset;
	REQUIRE(rcBuildContours(&context, compact, 1.3f, 12, cset));
	REQUIRE(cset.nconts > 0);

	// rcBuildCompactHeightfield raises the upper bound by walkableHeight and rcBuildContours
	// passes both bounds through, so the vertical extent of the source heightfield is recoverable
	// from the contour set.
	CHECK(cset.bmin[1] == Catch::Approx(bmin[1]));
	CHECK(cset.bmax[1] == Catch::Approx(bmax[1] + walkableHeight * ch));
}

TEST_CASE("rcBuildPolyMesh marks edges lying on a tile boundary with the matching direction", "[recast]")
{
	rcContext context;
	rcContourSet cset;
	// The contour fills the whole tile, so each of its edges lies on one tile boundary.
	makeContourSet(RectContour{0, 0, 20, 20}, slabHeight / 2, 20, 20, 4, cset);

	rcPolyMesh mesh;
	REQUIRE(rcBuildPolyMesh(&context, cset, 4, mesh));
	REQUIRE(mesh.npolys == 1);
	REQUIRE(mesh.nverts == 4);

	CHECK(getEdgeNei(mesh, 0, 0, 0, 0, 20) == (0x8000 | 0));
	CHECK(getEdgeNei(mesh, 0, 0, 20, 20, 20) == (0x8000 | 1));
	CHECK(getEdgeNei(mesh, 0, 20, 20, 20, 0) == (0x8000 | 2));
	CHECK(getEdgeNei(mesh, 0, 20, 0, 0, 0) == (0x8000 | 3));
}

TEST_CASE("rcBuildPolyMesh leaves interior boundary edges unmarked", "[recast]")
{
	rcContext context;
	rcContourSet cset;
	// The contour lies strictly inside the tile, so none of its edges is on a tile boundary and
	// none of them gets a portal direction: they are all solid walls.
	makeContourSet(RectContour{5, 5, 15, 15}, slabHeight / 2, 20, 20, 4, cset);

	rcPolyMesh mesh;
	REQUIRE(rcBuildPolyMesh(&context, cset, 4, mesh));
	REQUIRE(mesh.npolys == 1);
	REQUIRE(mesh.nverts == 4);

	for (int j = 0; j < 4; ++j)
	{
		INFO("edge " << j);
		CHECK(mesh.polys[mesh.nvp + j] == RC_MESH_NULL_IDX);
	}
}

TEST_CASE("rcBuildPolyMesh marks no portal edges at all without a border", "[recast]")
{
	// Without a border there is no neighbouring tile to connect to, so even a contour filling the
	// whole tile gets no portal directions.
	rcContext context;
	rcContourSet cset;
	makeContourSet(RectContour{0, 0, 20, 20}, slabHeight / 2, 20, 20, 0, cset);

	rcPolyMesh mesh;
	REQUIRE(rcBuildPolyMesh(&context, cset, 4, mesh));
	REQUIRE(mesh.npolys == 1);

	for (int j = 0; j < 4; ++j)
	{
		INFO("edge " << j);
		CHECK(mesh.polys[mesh.nvp + j] == RC_MESH_NULL_IDX);
	}
}

TEST_CASE("rcMergePolyMeshes keeps a tile boundary portal only on the matching merged boundary", "[recast]")
{
	rcContext context;

	// A two by two block of tiles, each holding one polygon whose four edges are all marked as
	// tile boundary portals. Merging resolves the edges which end up inside the block and keeps
	// the ones still on its boundary.
	const unsigned short neis[4] = {portalXMin, portalZMax, portalXMax, portalZMin};

	rcPolyMesh meshes[4];
	makePolyMesh(RectContour{5, 5, 15, 15}, neis, 0.0f, 0.0f, 0.1f, 20, 20, meshes[0]);
	makePolyMesh(RectContour{5, 5, 15, 15}, neis, 2.0f, 0.0f, 0.1f, 20, 20, meshes[1]);
	makePolyMesh(RectContour{5, 5, 15, 15}, neis, 0.0f, 2.0f, 0.1f, 20, 20, meshes[2]);
	makePolyMesh(RectContour{5, 5, 15, 15}, neis, 2.0f, 2.0f, 0.1f, 20, 20, meshes[3]);

	rcPolyMesh* pointers[4] = {&meshes[0], &meshes[1], &meshes[2], &meshes[3]};
	rcPolyMesh merged;
	REQUIRE(rcMergePolyMeshes(&context, pointers, 4, merged));
	REQUIRE(merged.npolys == 4);

	// Which of the four marks each tile keeps, following its position in the block.
	const unsigned short expected[4][4] = {
		{portalXMin, RC_MESH_NULL_IDX, RC_MESH_NULL_IDX, portalZMin}, // min x, min z
		{RC_MESH_NULL_IDX, RC_MESH_NULL_IDX, portalXMax, portalZMin}, // max x, min z
		{portalXMin, portalZMax, RC_MESH_NULL_IDX, RC_MESH_NULL_IDX}, // min x, max z
		{RC_MESH_NULL_IDX, portalZMax, portalXMax, RC_MESH_NULL_IDX}, // max x, max z
	};

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			INFO("poly " << i << " edge " << j);
			CHECK(merged.polys[i * 2 * merged.nvp + merged.nvp + j] == expected[i][j]);
		}
	}
}

