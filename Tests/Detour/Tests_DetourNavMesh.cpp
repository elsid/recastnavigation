#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"

#include "catch2/catch_all.hpp"

#include <array>
#include <cstring>
#include <vector>

namespace
{

constexpr float cellSize = 0.01f;
constexpr float tileSize = 10;
constexpr float minTileY = -2;
constexpr float maxTileY = 2;
constexpr float walkableHeight = 2;
constexpr float walkableRadius = 0.6f;
constexpr float walkableClimb = 0.9f;
constexpr int nvp = 4;
constexpr unsigned short walkableFlag = 1;
constexpr unsigned char walkableArea = 63;
constexpr unsigned short meshNullIdx = 0xffff;

// rcPolyMesh edge directions, as understood by dtCreateNavMeshData.
constexpr unsigned short portalXMin = 0x8000 | 0;
constexpr unsigned short portalZMax = 0x8000 | 1;
constexpr unsigned short portalXMax = 0x8000 | 2;
constexpr unsigned short portalZMin = 0x8000 | 3;
constexpr unsigned short heightPortal = 0x8000 | 4;

// An axis aligned rectangle, with the vertex winding rcBuildPolyMesh produces:
// (minX, minZ), (minX, maxZ), (maxX, maxZ), (maxX, minZ). Edge j goes from vertex j to vertex j+1,
// so edge 0 is the x- edge, edge 1 the z+ edge, edge 2 the x+ edge and edge 3 the z- edge.
// The rectangle sits at height y along z == minZ and rises by slopeZ towards z == maxZ.
struct RectPoly
{
	float minX;
	float minZ;
	float maxX;
	float maxZ;
	float y;
	unsigned short neis[4];
	float slopeZ = 0;
};

unsigned short quantize(float value, float min)
{
	return (unsigned short)((value - min) / cellSize + 0.5f);
}

// A polygon of any vertex count, given in world space and in the same winding as RectPoly.
// Edge j goes from vertex j to vertex j+1, wrapping around at the last one.
struct Poly
{
	std::vector<std::array<float, 3>> verts;
	std::vector<unsigned short> neis;
};

Poly toPoly(const RectPoly& rect)
{
	const float yMaxZ = rect.y + rect.slopeZ;
	return Poly{
		{
			{{rect.minX, rect.y, rect.minZ}},
			{{rect.minX, yMaxZ, rect.maxZ}},
			{{rect.maxX, yMaxZ, rect.maxZ}},
			{{rect.maxX, rect.y, rect.minZ}},
		},
		{rect.neis[0], rect.neis[1], rect.neis[2], rect.neis[3]},
	};
}

void addPolyTile(const std::vector<Poly>& polys, int tileNvp, int tileX, int tileY, int layer, dtNavMesh& navMesh)
{
	REQUIRE(tileNvp <= DT_VERTS_PER_POLYGON);

	const float bmin[3] = {tileX * tileSize, minTileY, tileY * tileSize};
	const float bmax[3] = {(tileX + 1) * tileSize, maxTileY, (tileY + 1) * tileSize};

	std::vector<unsigned short> verts;
	std::vector<unsigned short> meshPolys;
	std::vector<unsigned char> areas;
	std::vector<unsigned short> flags;

	for (const Poly& poly : polys)
	{
		REQUIRE(poly.verts.size() == poly.neis.size());
		REQUIRE((int)poly.verts.size() <= tileNvp);
		const int nv = (int)poly.verts.size();
		const unsigned short base = (unsigned short)(verts.size() / 3);
		for (const std::array<float, 3>& v : poly.verts)
			for (int i = 0; i < 3; ++i)
				verts.push_back(quantize(v[i], bmin[i]));
		// Shorter polygons are padded out to the tile's vertices per polygon.
		for (int i = 0; i < tileNvp; ++i)
			meshPolys.push_back(i < nv ? (unsigned short)(base + i) : meshNullIdx);
		for (int i = 0; i < tileNvp; ++i)
			meshPolys.push_back(i < nv ? poly.neis[i] : meshNullIdx);
		areas.push_back(walkableArea);
		flags.push_back(walkableFlag);
	}

	dtNavMeshCreateParams params;
	std::memset(&params, 0, sizeof(params));
	params.verts = verts.data();
	params.vertCount = (int)(verts.size() / 3);
	params.polys = meshPolys.data();
	params.polyAreas = areas.data();
	params.polyFlags = flags.data();
	params.polyCount = (int)polys.size();
	params.nvp = tileNvp;
	params.walkableHeight = walkableHeight;
	params.walkableRadius = walkableRadius;
	params.walkableClimb = walkableClimb;
	dtVcopy(params.bmin, bmin);
	dtVcopy(params.bmax, bmax);
	params.cs = cellSize;
	params.ch = cellSize;
	params.buildBvTree = true;
	params.tileX = tileX;
	params.tileY = tileY;
	params.tileLayer = layer;

	unsigned char* data = nullptr;
	int dataSize = 0;
	REQUIRE(dtCreateNavMeshData(&params, &data, &dataSize));
	REQUIRE(dtStatusSucceed(navMesh.addTile(data, dataSize, DT_TILE_FREE_DATA, 0, nullptr)));
}

void addTile(const std::vector<RectPoly>& polys, int tileX, int tileY, int layer, dtNavMesh& navMesh)
{
	std::vector<Poly> converted;
	for (const RectPoly& rect : polys)
		converted.push_back(toPoly(rect));
	addPolyTile(converted, nvp, tileX, tileY, layer, navMesh);
}

void initNavMesh(dtNavMesh& navMesh)
{
	dtNavMeshParams params;
	std::memset(&params, 0, sizeof(params));
	params.tileWidth = tileSize;
	params.tileHeight = tileSize;
	params.maxTiles = 32;
	params.maxPolys = 64;
	REQUIRE(dtStatusSucceed(navMesh.init(&params)));
}

struct LinkInfo
{
	dtPolyRef ref;
	unsigned char edge;
	unsigned char side;
	unsigned char bmin;
	unsigned char bmax;
};

std::vector<LinkInfo> getLinks(const dtMeshTile* tile, int polyIndex)
{
	std::vector<LinkInfo> result;
	const dtPoly& poly = tile->polys[polyIndex];
	for (unsigned int i = poly.firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
	{
		const dtLink& link = tile->links[i];
		result.push_back(LinkInfo{link.ref, link.edge, link.side, link.bmin, link.bmax});
	}
	return result;
}

const LinkInfo* findLink(const std::vector<LinkInfo>& links, dtPolyRef ref)
{
	for (const LinkInfo& link : links)
		if (link.ref == ref)
			return &link;
	return nullptr;
}

dtQueryFilter makeFilter()
{
	dtQueryFilter filter;
	filter.setIncludeFlags(walkableFlag);
	filter.setAreaCost(walkableArea, 1);
	return filter;
}

}

TEST_CASE("dtCreateNavMeshData serializes a height portal edge apart from the tile boundary ones", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	addTile(
		{
			RectPoly{2, 2, 4, 8, 0.0f, {portalXMin, portalZMax, portalXMax, portalZMin}},
			RectPoly{5, 2, 8, 8, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}},
		},
		0, 0, 0, navMesh);

	const dtMeshTile* const tile = navMesh.getTileAt(0, 0, 0);
	REQUIRE(tile != nullptr);

	// The four tile boundary directions keep their existing encoding.
	const dtPoly& boundary = tile->polys[0];
	CHECK(boundary.neis[0] == (DT_EXT_LINK | 4));
	CHECK(boundary.neis[1] == (DT_EXT_LINK | 2));
	CHECK(boundary.neis[2] == (DT_EXT_LINK | 0));
	CHECK(boundary.neis[3] == (DT_EXT_LINK | 6));

	// A height portal gets a side of its own, outside the 0..7 tile boundary range, and a hard
	// border still becomes no neighbour at all.
	const dtPoly& height = tile->polys[1];
	CHECK(height.neis[0] == (DT_EXT_LINK | 0xfe));
	CHECK(height.neis[1] == 0);
	CHECK(height.neis[2] == 0);
	CHECK(height.neis[3] == 0);
}

TEST_CASE("dtCreateNavMeshData reserves a link budget for every height portal edge", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// Same polygon shape either way, so only the edge marks differ.
	addTile({RectPoly{2, 2, 8, 8, 0.0f, {portalXMax, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	addTile({RectPoly{12, 2, 18, 8, 0.0f, {portalXMax, heightPortal, heightPortal, meshNullIdx}}}, 1, 0, 0, navMesh);

	const dtMeshTile* const withoutHeightPortals = navMesh.getTileAt(0, 0, 0);
	REQUIRE(withoutHeightPortals != nullptr);
	const dtMeshTile* const withHeightPortals = navMesh.getTileAt(1, 0, 0);
	REQUIRE(withHeightPortals != nullptr);

	// 4 edges plus 2 links for the one tile boundary portal.
	CHECK(withoutHeightPortals->header->maxLinkCount == 6);
	// A height portal edge is linked to one polygon per same position tile instead of to a bounded
	// number of neighbours, so it is budgeted separately and more generously.
	CHECK(withHeightPortals->header->maxLinkCount == 6 + 2*DT_HEIGHT_PORTAL_LINKS_PER_EDGE);
}

TEST_CASE("Neighbour tiles are linked across their boundary in either order they are added", "[detour]")
{
	// addTile creates the neighbour tile links before the same position layer links, so this has
	// to hold no matter which tile arrives first.
	const bool neighbourFirst = GENERATE(false, true);
	CAPTURE(neighbourFirst);

	dtNavMesh navMesh;
	initNavMesh(navMesh);

	const RectPoly left{2, 2, 10, 8, 0.0f, {meshNullIdx, meshNullIdx, portalXMax, meshNullIdx}};
	const RectPoly right{10, 2, 18, 8, 0.0f, {portalXMin, meshNullIdx, meshNullIdx, meshNullIdx}};

	if (neighbourFirst)
	{
		addTile({right}, 1, 0, 0, navMesh);
		addTile({left}, 0, 0, 0, navMesh);
	}
	else
	{
		addTile({left}, 0, 0, 0, navMesh);
		addTile({right}, 1, 0, 0, navMesh);
	}

	const dtMeshTile* const leftTile = navMesh.getTileAt(0, 0, 0);
	REQUIRE(leftTile != nullptr);
	const dtMeshTile* const rightTile = navMesh.getTileAt(1, 0, 0);
	REQUIRE(rightTile != nullptr);

	const std::vector<LinkInfo> fromLeft = getLinks(leftTile, 0);
	REQUIRE(fromLeft.size() == 1);
	CHECK(fromLeft[0].ref == (navMesh.getPolyRefBase(rightTile) | 0));
	CHECK(fromLeft[0].edge == 2);
	CHECK((int)fromLeft[0].side == 0);

	const std::vector<LinkInfo> fromRight = getLinks(rightTile, 0);
	REQUIRE(fromRight.size() == 1);
	CHECK(fromRight[0].ref == (navMesh.getPolyRefBase(leftTile) | 0));
	CHECK(fromRight[0].edge == 0);
	CHECK((int)fromRight[0].side == 4);
}

TEST_CASE("Tile boundary portal edges are not linked between tiles at the same position", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// Two layers whose polygons fill the whole tile, so every edge of both is a tile boundary
	// portal, and they are close enough in height to be linked if anything tried to. Only a height
	// portal edge connects to a tile at the same position: a tile boundary portal has to wait for
	// the tile on the other side of that boundary.
	const RectPoly poly{0, 0, 10, 10, 0.0f, {portalXMin, portalZMax, portalXMax, portalZMin}};
	addTile({poly}, 0, 0, 0, navMesh);
	addTile({RectPoly{0, 0, 10, 10, 0.1f, {portalXMin, portalZMax, portalXMax, portalZMin}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);
	const dtMeshTile* const upper = navMesh.getTileAt(0, 0, 1);
	REQUIRE(upper != nullptr);

	CHECK(getLinks(lower, 0).empty());
	CHECK(getLinks(upper, 0).empty());
}

TEST_CASE("Height portal edge is not linked past the walkable climb", "[detour]")
{
	const float y = GENERATE(-1.0f, -0.8f, 0.8f, 1.0f);
	const bool climbable = y > -walkableClimb && y < walkableClimb;
	CAPTURE(y);

	dtNavMesh navMesh;
	initNavMesh(navMesh);

	addTile({RectPoly{2, 1, 8, 9, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	// Backs the whole edge in xz, so only the height difference decides.
	addTile({RectPoly{1, 1, 2.5f, 9, y, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);

	CHECK(getLinks(lower, 0).size() == (climbable ? 1u : 0u));
}

TEST_CASE("Height portal edge is not linked to a sloped polygon out of climbing reach", "[detour]")
{
	// The bounding box the candidate polygons are collected with is the edge grown by
	// walkableClimb, which for a flat polygon already decides the question. A sloped one can reach
	// into that box and still be out of reach where the edge is actually crossed, so the height
	// has to be measured there.
	const float slopeZ = GENERATE(1.6f, 2.0f);
	// The target rises from y == -1 at z == 1, so at the middle of the edge at z == 5 it sits at
	// -1 + slopeZ/2, against an edge at y == -1.
	const bool climbable = slopeZ / 2 < walkableClimb;
	CAPTURE(slopeZ);

	dtNavMesh navMesh;
	initNavMesh(navMesh);

	addTile({RectPoly{2, 1, 8, 9, -1.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	addTile({RectPoly{1, 1, 2.5f, 9, -1.0f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}, slopeZ}}, 0, 0, 1,
		navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);

	CHECK(getLinks(lower, 0).size() == (climbable ? 1u : 0u));
}

TEST_CASE("Height portal edge is not linked to a polygon that only the broad phase returns", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	addTile({RectPoly{2, 1, 8, 9, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	// One centimetre clear of the edge at x == 2, and overhanging it in z at both ends. The
	// quantized bounding box the candidates are collected with rounds outwards, so this polygon
	// still comes back as a candidate, and only actually crossing the edge rules it out.
	addTile({RectPoly{2.01f, 0, 5, 10, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);

	CHECK(getLinks(lower, 0).empty());
}

TEST_CASE("Height portal edge is not linked over an overlap too short to express", "[detour]")
{
	// dtLink stores the ends of a portal in 1/255 of the edge. The edge here runs from z == 1 to
	// z == 9, so one step of that is 8/255, and a thinner overlap has to be dropped rather than
	// rounded up into a link claiming a part of the edge that is not backed.
	const float maxZ = GENERATE(1.02f, 1.4f);
	const bool expressible = maxZ - 1 > 8.0f / 255;
	CAPTURE(maxZ);

	dtNavMesh navMesh;
	initNavMesh(navMesh);

	addTile({RectPoly{2, 1, 8, 9, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	addTile({RectPoly{1, 1, 2.5f, maxZ, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);

	const std::vector<LinkInfo> links = getLinks(lower, 0);
	REQUIRE(links.size() == (expressible ? 1u : 0u));

	if (expressible)
	{
		CHECK((int)links[0].bmin == 0);
		CHECK((int)links[0].bmax == 13);
	}
}

TEST_CASE("Raycast crosses a height portal backed only at the far end of the edge", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	addTile({RectPoly{2, 1, 8, 9, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	// Backs z in [5, 9] of an edge running from z == 1 to z == 9, which is t in [0.5, 1], so the
	// interval starts away from the beginning of the edge.
	addTile({RectPoly{1, 5, 2.5f, 9, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);
	const dtMeshTile* const upper = navMesh.getTileAt(0, 0, 1);
	REQUIRE(upper != nullptr);

	const std::vector<LinkInfo> links = getLinks(lower, 0);
	REQUIRE(links.size() == 1);
	CHECK((int)links[0].bmin == 128);
	CHECK((int)links[0].bmax == 255);

	dtNavMeshQuery query;
	REQUIRE(dtStatusSucceed(query.init(&navMesh, 128)));
	const dtQueryFilter filter = makeFilter();

	const float z = GENERATE(3.0f, 7.0f);
	const bool backed = z > 5;
	CAPTURE(z);

	const float startPos[3] = {5, 0, z};
	const float endPos[3] = {0, 0, z};

	dtRaycastHit hit;
	dtPolyRef path[8];
	hit.path = path;
	hit.maxPath = 8;
	const dtPolyRef startRef = navMesh.getPolyRefBase(lower) | 0;
	REQUIRE(dtStatusSucceed(query.raycast(startRef, startPos, endPos, &filter, 0, &hit)));

	if (backed)
	{
		REQUIRE(hit.pathCount == 2);
		CHECK(path[1] == (navMesh.getPolyRefBase(upper) | 0));
	}
	else
	{
		CHECK(hit.pathCount == 1);
		CHECK(hit.t == Catch::Approx(0.6f).margin(1e-3));
	}
}

TEST_CASE("Height portal link covers only the part of the edge backed by the target polygon", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// Layer 0 holds one polygon whose x- edge spans z in [1, 9] and is marked as a height portal.
	addTile({RectPoly{2, 1, 8, 9, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	// Layer 1 backs only the z in [1, 6] part of that edge.
	addTile({RectPoly{1, 1, 2.5f, 6, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);
	const dtMeshTile* const upper = navMesh.getTileAt(0, 0, 1);
	REQUIRE(upper != nullptr);

	const std::vector<LinkInfo> links = getLinks(lower, 0);
	REQUIRE(links.size() == 1);
	CHECK(links[0].ref == (navMesh.getPolyRefBase(upper) | 0));
	CHECK(links[0].edge == 0);

	SECTION("the link does not claim the whole edge")
	{
		// The edge runs from z == 1 to z == 9 and the target polygon backs z in [1, 6] of it,
		// which is t in [0, 0.625]. bmin == 0 && bmax == 255 instead makes dtNavMeshQuery treat
		// the whole edge as crossable.
		CHECK((int)links[0].bmin == 0);
		CHECK((int)links[0].bmax == 159);
	}

	SECTION("a straight path may only leave the polygon where the edge is backed")
	{
		dtNavMeshQuery query;
		REQUIRE(dtStatusSucceed(query.init(&navMesh, 128)));
		const dtQueryFilter filter = makeFilter();

		// The straight line between these two points crosses the edge at z == 6.26, which is
		// beyond the part of the edge the layer 1 polygon backs.
		const float startPos[3] = {7.9f, 0, 8.9f};
		const float endPos[3] = {1.2f, 0.1f, 5.9f};
		const dtPolyRef startRef = navMesh.getPolyRefBase(lower) | 0;
		const dtPolyRef endRef = navMesh.getPolyRefBase(upper) | 0;

		dtPolyRef path[8];
		int pathCount = 0;
		REQUIRE(dtStatusSucceed(query.findPath(startRef, endRef, startPos, endPos, &filter, path, &pathCount, 8)));
		REQUIRE(pathCount == 2);

		float straightPath[8 * 3];
		int straightPathCount = 0;
		REQUIRE(dtStatusSucceed(query.findStraightPath(
			startPos, endPos, path, pathCount, straightPath, nullptr, nullptr, &straightPathCount, 8)));

		// The path has to bend around the end of the portal at (2, 6) instead of going straight.
		// dtLink stores the portal ends in 1/255 of the edge, so the corner lands slightly
		// inside the backed part rather than exactly on z == 6.
		REQUIRE(straightPathCount == 3);
		CHECK(straightPath[3] == Catch::Approx(2).margin(1e-2));
		CHECK(straightPath[5] == Catch::Approx(6).margin(8.0f / 255));
	}

	SECTION("a raycast crosses the edge exactly where the target polygon backs it")
	{
		dtNavMeshQuery query;
		REQUIRE(dtStatusSucceed(query.init(&navMesh, 128)));
		const dtQueryFilter filter = makeFilter();
		const dtPolyRef startRef = navMesh.getPolyRefBase(lower) | 0;
		const dtPolyRef targetRef = navMesh.getPolyRefBase(upper) | 0;

		const float z = GENERATE(3.0f, 8.0f);
		const bool backed = z < 6;
		CAPTURE(z);

		const float startPos[3] = {5, 0, z};
		const float endPos[3] = {0, 0, z};

		dtRaycastHit hit;
		dtPolyRef path[8];
		hit.path = path;
		hit.maxPath = 8;
		REQUIRE(dtStatusSucceed(query.raycast(startRef, startPos, endPos, &filter, 0, &hit)));

		if (backed)
		{
			REQUIRE(hit.pathCount == 2);
			CHECK(path[1] == targetRef);
		}
		else
		{
			// z == 8 is outside of the layer 1 polygon, so the ray must stop at the wall.
			CHECK(hit.pathCount == 1);
			CHECK(hit.t == Catch::Approx(0.6f).margin(1e-3));
		}
	}
}

TEST_CASE("Height portal connects layers when the target has a gap at the edge midpoint", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// Layer 0 holds one polygon whose x- edge spans z in [1, 9] and is marked as a height portal.
	addTile({RectPoly{2, 1, 8, 9, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	// Layer 1 backs the edge everywhere except around its midpoint at z == 5.
	addTile(
		{
			RectPoly{1, 1, 2.5f, 4, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}},
			RectPoly{1, 6, 2.5f, 9, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}},
		},
		0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);

	// Both backed parts of the edge are walkable into layer 1, only the midpoint sample is not.
	CHECK(getLinks(lower, 0).size() == 2);
}

TEST_CASE("Height portal edge links to every layer polygon backing a part of it", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	addTile({RectPoly{2, 1, 8, 9, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	// Two layer 1 polygons at different heights, each backing one half of the edge.
	addTile(
		{
			RectPoly{1, 1, 2.5f, 5, 0.8f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}},
			RectPoly{1, 5, 2.5f, 9, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}},
		},
		0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);
	const dtMeshTile* const upper = navMesh.getTileAt(0, 0, 1);
	REQUIRE(upper != nullptr);

	const std::vector<LinkInfo> links = getLinks(lower, 0);
	REQUIRE(links.size() == 2);

	// The edge runs from z == 1 to z == 9, so the first polygon backs t in [0, 0.5] of it and
	// the second one t in [0.5, 1].
	const LinkInfo* const first = findLink(links, navMesh.getPolyRefBase(upper) | 0);
	REQUIRE(first != nullptr);
	CHECK(first->edge == 0);
	CHECK((int)first->bmin == 0);
	CHECK((int)first->bmax == 128);

	const LinkInfo* const second = findLink(links, navMesh.getPolyRefBase(upper) | 1);
	REQUIRE(second != nullptr);
	CHECK(second->edge == 0);
	CHECK((int)second->bmin == 128);
	CHECK((int)second->bmax == 255);
}

TEST_CASE("Layer links do not consume the link budget of neighbour tile links", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// The neighbour tile the x+ edge of the layer 0 polygon has to connect to.
	addTile({RectPoly{10, 2, 18, 8, 0.0f, {portalXMin, meshNullIdx, meshNullIdx, meshNullIdx}}}, 1, 0, 0, navMesh);

	// Four more layers at the same tile position, each within walkableClimb of layer 0 and each
	// covering all three interior edges of the layer 0 polygon.
	for (int layer = 1; layer < 5; ++layer)
		addTile({RectPoly{1, 1, 10, 9, 0.1f * layer, {heightPortal, heightPortal, portalXMax, heightPortal}}}, 0, 0,
			layer, navMesh);

	// Added last, so that its layer links are created before its neighbour tile links.
	addTile({RectPoly{2, 2, 10, 8, 0.0f, {heightPortal, heightPortal, portalXMax, heightPortal}}}, 0, 0, 0, navMesh);

	const dtMeshTile* const tile = navMesh.getTileAt(0, 0, 0);
	REQUIRE(tile != nullptr);
	const dtMeshTile* const neighbour = navMesh.getTileAt(1, 0, 0);
	REQUIRE(neighbour != nullptr);
	const dtPolyRef neighbourRef = navMesh.getPolyRefBase(neighbour) | 0;

	int layerLinks = 0;
	int neighbourLinks = 0;
	for (const LinkInfo& link : getLinks(tile, 0))
	{
		if (link.ref == neighbourRef)
			++neighbourLinks;
		else
			++layerLinks;
	}

	// 3 height portal edges times 4 layers, all of which fit in the budget alongside the
	// neighbour tile link.
	CHECK(layerLinks == 12);
	CHECK(neighbourLinks == 1);
	CHECK(layerLinks + neighbourLinks < tile->header->maxLinkCount);
}

TEST_CASE("Neighbour tile links survive a link pool exhausted by layer links", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// The neighbour tile the x+ edge of the layer 0 polygon has to connect to.
	addTile({RectPoly{10, 2, 18, 8, 0.0f, {portalXMin, meshNullIdx, meshNullIdx, meshNullIdx}}}, 1, 0, 0, navMesh);

	// Six more layers at the same tile position. The layer 0 polygon has 3 height portal edges,
	// so they ask for 18 links, which together with the neighbour tile link is one more than the
	// budget: something has to be dropped, and it must not be the neighbour tile link.
	for (int layer = 1; layer < 7; ++layer)
		addTile({RectPoly{1, 1, 10, 9, 0.1f * layer, {heightPortal, heightPortal, portalXMax, heightPortal}}}, 0, 0,
			layer, navMesh);

	// Added last, so that all the layers are already there when its own links are created.
	addTile({RectPoly{2, 2, 10, 8, 0.0f, {heightPortal, heightPortal, portalXMax, heightPortal}}}, 0, 0, 0, navMesh);

	const dtMeshTile* const tile = navMesh.getTileAt(0, 0, 0);
	REQUIRE(tile != nullptr);
	const dtMeshTile* const neighbour = navMesh.getTileAt(1, 0, 0);
	REQUIRE(neighbour != nullptr);
	const dtPolyRef neighbourRef = navMesh.getPolyRefBase(neighbour) | 0;

	int layerLinks = 0;
	int neighbourLinks = 0;
	for (const LinkInfo& link : getLinks(tile, 0))
	{
		if (link.ref == neighbourRef)
			++neighbourLinks;
		else
			++layerLinks;
	}

	// The pool really is full, otherwise this would not be testing anything.
	REQUIRE(layerLinks + neighbourLinks == tile->header->maxLinkCount);
	CHECK(neighbourLinks == 1);
}

TEST_CASE("Path over a height portal crosses the edge the path actually goes through", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// All three interior edges of the layer 0 polygon are height portals into the same layer 1
	// polygon, so the target polygon alone does not say which edge the path leaves through.
	addTile({RectPoly{2, 2, 8, 8, 0.0f, {heightPortal, heightPortal, meshNullIdx, heightPortal}}}, 0, 0, 0, navMesh);
	addTile({RectPoly{1, 1, 9, 9, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);
	const dtMeshTile* const upper = navMesh.getTileAt(0, 0, 1);
	REQUIRE(upper != nullptr);
	REQUIRE(getLinks(lower, 0).size() == 3);

	dtNavMeshQuery query;
	REQUIRE(dtStatusSucceed(query.init(&navMesh, 128)));
	const dtQueryFilter filter = makeFilter();

	// Leaving straight through each of the three edges in turn, from the middle of the polygon.
	struct Crossing
	{
		const char* edge;
		float endX;
		float endZ;
	};
	static const Crossing crossings[] = {
		{"x- at x == 2", 1.5f, 5.0f},
		{"z+ at z == 8", 5.0f, 8.5f},
		{"z- at z == 2", 5.0f, 1.5f},
	};
	const Crossing& crossing = crossings[GENERATE(range(0, 3))];
	INFO("crossing " << crossing.edge);

	const float startPos[3] = {5, 0, 5};
	const float endPos[3] = {crossing.endX, 0.1f, crossing.endZ};
	const dtPolyRef startRef = navMesh.getPolyRefBase(lower) | 0;
	const dtPolyRef endRef = navMesh.getPolyRefBase(upper) | 0;

	dtPolyRef path[8];
	int pathCount = 0;
	REQUIRE(dtStatusSucceed(query.findPath(startRef, endRef, startPos, endPos, &filter, path, &pathCount, 8)));
	REQUIRE(pathCount == 2);

	float straightPath[8 * 3];
	int straightPathCount = 0;
	REQUIRE(dtStatusSucceed(query.findStraightPath(
		startPos, endPos, path, pathCount, straightPath, nullptr, nullptr, &straightPathCount, 8)));

	// The straight line already goes through the crossed edge, so the funnel has nothing to bend
	// around: any corner here means it was handed one of the other two edges as the portal.
	REQUIRE(straightPathCount == 2);
	CHECK(straightPath[3] == Catch::Approx(crossing.endX).margin(1e-3));
	CHECK(straightPath[5] == Catch::Approx(crossing.endZ).margin(1e-3));
}


TEST_CASE("Polygon wall segments split a height portal edge where it stops being backed", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// The x- edge of the layer 0 polygon runs from z == 1 to z == 9 and layer 1 backs z in [1, 6].
	addTile({RectPoly{2, 1, 8, 9, 0.0f, {heightPortal, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	addTile({RectPoly{1, 1, 2.5f, 6, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);
	const dtMeshTile* const upper = navMesh.getTileAt(0, 0, 1);
	REQUIRE(upper != nullptr);

	dtNavMeshQuery query;
	REQUIRE(dtStatusSucceed(query.init(&navMesh, 128)));
	const dtQueryFilter filter = makeFilter();

	float verts[6 * 6];
	dtPolyRef refs[6];
	int count = 0;
	REQUIRE(dtStatusSucceed(
		query.getPolyWallSegments(navMesh.getPolyRefBase(lower) | 0, &filter, verts, refs, &count, 6)));

	// Four edges, but the height portal one is reported as two segments: the backed part leads
	// into layer 1 and the rest of it is wall. dtLink stores the split in 1/255 of the edge.
	REQUIRE(count == 5);
	const float split = 1 + 8.0f * 159 / 255;

	CHECK(refs[1] == (navMesh.getPolyRefBase(upper) | 0));
	CHECK(verts[1 * 6 + 0] == Catch::Approx(2).margin(1e-3));
	CHECK(verts[1 * 6 + 2] == Catch::Approx(1).margin(1e-3));
	CHECK(verts[1 * 6 + 3] == Catch::Approx(2).margin(1e-3));
	CHECK(verts[1 * 6 + 5] == Catch::Approx(split).margin(1e-3));

	CHECK(refs[2] == 0);
	CHECK(verts[2 * 6 + 2] == Catch::Approx(split).margin(1e-3));
	CHECK(verts[2 * 6 + 5] == Catch::Approx(9).margin(1e-3));

	// The three hard borders stay whole walls.
	CHECK(refs[0] == 0);
	CHECK(refs[3] == 0);
	CHECK(refs[4] == 0);
}

TEST_CASE("Straight path over a height portal on the last edge of a six vertex polygon", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// A hexagon whose height portal is its last edge, so finding the far end of that edge has to
	// wrap around to the first vertex. Six is DT_VERTS_PER_POLYGON: reading one past the last
	// vertex leaves dtPoly::verts altogether and lands in neis, whereas with fewer vertices the
	// slot is a zero that happens to be the right answer anyway. The first edge is a real z+ tile
	// boundary portal, so that neis[0] is not zero either.
	const Poly hexagon{
		{
			{{2, 0, 10}},
			{{7, 0, 10}},
			{{8, 0, 8}},
			{{8, 0, 2}},
			{{7, 0, 1}},
			{{2, 0, 1}},
		},
		{portalZMax, meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx, heightPortal},
	};
	addPolyTile({hexagon}, 6, 0, 0, 0, navMesh);
	addTile({RectPoly{1, 1, 2.5f, 10, 0.1f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 1, navMesh);

	const dtMeshTile* const lower = navMesh.getTileAt(0, 0, 0);
	REQUIRE(lower != nullptr);
	const dtMeshTile* const upper = navMesh.getTileAt(0, 0, 1);
	REQUIRE(upper != nullptr);
	REQUIRE(lower->polys[0].vertCount == 6);
	REQUIRE(lower->polys[0].neis[0] != 0);

	const std::vector<LinkInfo> links = getLinks(lower, 0);
	REQUIRE(links.size() == 1);
	CHECK(links[0].edge == 5);
	CHECK((int)links[0].bmin == 0);
	CHECK((int)links[0].bmax == 255);

	dtNavMeshQuery query;
	REQUIRE(dtStatusSucceed(query.init(&navMesh, 128)));
	const dtQueryFilter filter = makeFilter();

	// Straight out through the last edge at x == 2.
	const float startPos[3] = {5, 0, 5};
	const float endPos[3] = {1.5f, 0.1f, 5};
	const dtPolyRef startRef = navMesh.getPolyRefBase(lower) | 0;
	const dtPolyRef endRef = navMesh.getPolyRefBase(upper) | 0;

	dtPolyRef path[8];
	int pathCount = 0;
	REQUIRE(dtStatusSucceed(query.findPath(startRef, endRef, startPos, endPos, &filter, path, &pathCount, 8)));
	REQUIRE(pathCount == 2);

	float straightPath[8 * 3];
	int straightPathCount = 0;
	REQUIRE(dtStatusSucceed(query.findStraightPath(
		startPos, endPos, path, pathCount, straightPath, nullptr, nullptr, &straightPathCount, 8)));

	// The portal spans the whole edge and the line already crosses it, so nothing bends the path.
	REQUIRE(straightPathCount == 2);
	CHECK(straightPath[3] == Catch::Approx(endPos[0]).margin(1e-3));
	CHECK(straightPath[5] == Catch::Approx(endPos[2]).margin(1e-3));
}
