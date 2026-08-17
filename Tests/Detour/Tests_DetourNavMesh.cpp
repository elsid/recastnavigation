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

// An axis aligned rectangle, with the vertex winding rcBuildPolyMesh produces:
// (minX, minZ), (minX, maxZ), (maxX, maxZ), (maxX, minZ). Edge j goes from vertex j to vertex j+1,
// so edge 0 is the x- edge, edge 1 the z+ edge, edge 2 the x+ edge and edge 3 the z- edge.
struct RectPoly
{
	float minX;
	float minZ;
	float maxX;
	float maxZ;
	float y;
	unsigned short neis[4];
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
	return Poly{
		{
			{{rect.minX, rect.y, rect.minZ}},
			{{rect.minX, rect.y, rect.maxZ}},
			{{rect.maxX, rect.y, rect.maxZ}},
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

dtQueryFilter makeFilter()
{
	dtQueryFilter filter;
	filter.setIncludeFlags(walkableFlag);
	filter.setAreaCost(walkableArea, 1);
	return filter;
}

}

TEST_CASE("dtCreateNavMeshData serializes edge marks into polygon neighbours", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	addTile(
		{
			RectPoly{2, 2, 4, 8, 0.0f, {portalXMin, portalZMax, portalXMax, portalZMin}},
			RectPoly{5, 2, 8, 8, 0.0f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}},
		},
		0, 0, 0, navMesh);

	const dtMeshTile* const tile = navMesh.getTileAt(0, 0, 0);
	REQUIRE(tile != nullptr);

	// Each tile boundary direction becomes an external link on the matching side.
	const dtPoly& boundary = tile->polys[0];
	CHECK(boundary.neis[0] == (DT_EXT_LINK | 4));
	CHECK(boundary.neis[1] == (DT_EXT_LINK | 2));
	CHECK(boundary.neis[2] == (DT_EXT_LINK | 0));
	CHECK(boundary.neis[3] == (DT_EXT_LINK | 6));

	// A hard border becomes no neighbour at all.
	const dtPoly& border = tile->polys[1];
	for (int j = 0; j < 4; ++j)
	{
		INFO("edge " << j);
		CHECK(border.neis[j] == 0);
	}
}

TEST_CASE("dtCreateNavMeshData reserves two links for every tile boundary portal edge", "[detour]")
{
	dtNavMesh navMesh;
	initNavMesh(navMesh);

	// Same polygon shape either way, so only the edge marks differ.
	addTile({RectPoly{2, 2, 8, 8, 0.0f, {meshNullIdx, meshNullIdx, meshNullIdx, meshNullIdx}}}, 0, 0, 0, navMesh);
	addTile({RectPoly{12, 2, 18, 8, 0.0f, {portalXMin, meshNullIdx, portalXMax, meshNullIdx}}}, 1, 0, 0, navMesh);

	const dtMeshTile* const withoutPortals = navMesh.getTileAt(0, 0, 0);
	REQUIRE(withoutPortals != nullptr);
	const dtMeshTile* const withPortals = navMesh.getTileAt(1, 0, 0);
	REQUIRE(withPortals != nullptr);

	// One link per edge, plus two more for each edge that can reach a neighbouring tile.
	CHECK(withoutPortals->header->maxLinkCount == 4);
	CHECK(withPortals->header->maxLinkCount == 4 + 2*2);
}

TEST_CASE("Neighbour tiles are linked across their boundary in either order they are added", "[detour]")
{
	// Whichever tile arrives second links itself to the one already there, in both directions.
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
	// portal, and they are close enough in height to be linked if anything tried to. A tile
	// boundary portal only connects across that boundary, so it has to wait for the tile on the
	// other side of it rather than pair up with a tile at the same position.
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

