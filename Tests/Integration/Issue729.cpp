#include "catch2/catch_all.hpp"

#include "MeshLoaderObj.h"
#include "Recast.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#include <filesystem>
#include <map>
#include <utility>
#include <array>

const std::filesystem::path meshesPath = std::filesystem::path{ RECASTNAVIGATION_TESTS_INTEGRATION_MESHES } / "Issue729";

std::string toString(const std::filesystem::path& path)
{
	const auto str = path.u8string();
	return std::string(str.begin(), str.end());
}

struct TileMesh
{
	std::pair<int, int> tile;
	std::unique_ptr<rcMeshLoaderObj> mesh {new rcMeshLoaderObj};

	explicit TileMesh(int x, int y) : tile(x, y) {}
};

TEST_CASE("Issue 729")
{
	SECTION("Build navmesh and find path")
	{
		std::vector<TileMesh> meshes;
		REQUIRE(meshes.emplace(meshes.end(), -1, -1)->mesh->load(toString(meshesPath / "-1.-1.recastmesh.obj")));
		REQUIRE(meshes.emplace(meshes.end(), 0, -1)->mesh->load(toString(meshesPath / "0.-1.recastmesh.obj")));
		REQUIRE(meshes.emplace(meshes.end(), 0, 1)->mesh->load(toString(meshesPath / "0.1.recastmesh.obj")));
		REQUIRE(meshes.emplace(meshes.end(), -1, 0)->mesh->load(toString(meshesPath / "-1.0.recastmesh.obj")));
		REQUIRE(meshes.emplace(meshes.end(), 0, 0)->mesh->load(toString(meshesPath / "0.0.recastmesh.obj")));
		REQUIRE(meshes.emplace(meshes.end(), -1, 1)->mesh->load(toString(meshesPath / "-1.1.recastmesh.obj")));

		const std::map<std::pair<int, int>, std::array<float, 3>> minBounds = {
			{{-1, -1}, {-28.8f, -4.88335f, -28.8f}},
			{{-1, 0}, {-28.8f, -4.88335f, -3.2f}},
			{{-1, 1}, {-28.8f, -4.09722f, 22.4f}},
			{{0, -1}, {-3.2f, -4.88335f, -28.8f}},
			{{0, 0}, {-3.2f, -4.88335f, -3.2f}},
			{{0, 1}, {-3.2f, -4.09722f, 22.4f}},
		};
		std::map<std::pair<int, int>, std::array<float, 3>> maxBounds = {
			{{-1, -1}, {3.2f, 11.153f, 3.2f}},
			{{-1, 0}, {3.2f, 10.5932f, 28.8f}},
			{{-1, 1}, {3.2f, 11.0453f, 54.4f}},
			{{0, -1}, {28.8f, 11.153f, 3.2f}},
			{{0, 0}, {28.8f, 10.6229f, 28.8f}},
			{{0, 1}, {28.8f, 11.0453f, 54.4f}},
		};
		std::map<std::pair<int, int>, std::vector<float>> offMeshConVerts = {
			{{-1, -1}, {-2.26471, 3.14706, 0.441176, -1.88235, 3.14706, -5.29412, -1.88235, 3.14706, -5.29412, -2.26471,
						3.14706, 0.441176, -1.88235, 3.14706, -5.29412, 0.5, 3.14706, -12.5294, 0.5, 3.14706, -12.5294,
						-1.88235, 3.14706, -5.29412}},
			{{-1, 0}, {-2.82353, 3.14706, 6.5, -2.32353, 3.14706, 18.1176, -2.82353, 3.14706, 6.5, -2.26471, 3.14706,
					   0.441176, -2.82353, 3.14706, 6.5, 5.64706, 3.14706, 6.14706, -2.32353, 3.14706, 18.1176,
					   -2.82353, 3.14706, 6.5, -2.32353, 3.14706, 18.1176, -0.588235, 3.14706, 25.5, -2.32353, 3.14706,
					   18.1176, 2.91176, 3.14706, 17.3235, -2.26471, 3.14706, 0.441176, -2.82353, 3.14706, 6.5,
					   -2.26471, 3.14706, 0.441176, -1.88235, 3.14706, -5.29412, -1.88235, 3.14706, -5.29412, -2.26471,
					   3.14706, 0.441176, -0.588235, 3.14706, 25.5, -2.32353, 3.14706, 18.1176, -0.588235, 3.14706,
					   25.5, 0.441176, 3.14706, 32.4118, -0.588235, 3.14706, 25.5, 2.91176, 3.14706, 17.3235, 0.441176,
					   3.14706, 32.4118, -0.588235, 3.14706, 25.5, 2.91176, 3.14706, 17.3235, -2.32353, 3.14706,
					   18.1176, 2.91176, 3.14706, 17.3235, -0.588235, 3.14706, 25.5, 5.64706, 3.14706, 6.14706,
					   -2.82353, 3.14706, 6.5}},
			{{-1, 1}, {-3.47059, 3.14706, 40.9706, -3.29412, 3.14706, 32.6471, -3.47059, 3.14706, 40.9706, 0.205882,
					   3.14706, 43.2353, -3.29412, 3.14706, 32.6471, -3.47059, 3.14706, 40.9706, -3.29412, 3.14706,
					   32.6471, 0.441176, 3.14706, 32.4118, 0.205882, 3.14706, 43.2353, -3.47059, 3.14706, 40.9706,
					   0.441176, 3.14706, 32.4118, -3.29412, 3.14706, 32.6471}},
			{{0, -1}, {-1.88235, 3.14706, -5.29412, 0.5, 3.14706, -12.5294, 0.5, 3.14706, -12.5294, -1.88235, 3.14706,
					   -5.29412, 2.61765, -3.14706, -3.08824, 2.79412, -3.14706, 0.794118, 2.79412, -3.14706, 0.794118,
					   2.61765, -3.14706, -3.08824, 5.64706, 3.14706, 6.14706, 6.32353, 3.14706, -1.23529, 6.32353,
					   3.14706, -1.23529, 5.64706, 3.14706, 6.14706}},
			{{0, 0}, {-2.82353, 3.14706, 6.5, 5.64706, 3.14706, 6.14706, -2.32353, 3.14706, 18.1176, 2.91176, 3.14706,
					  17.3235, -0.588235, 3.14706, 25.5, 2.91176, 3.14706, 17.3235, 0.441176, 3.14706, 32.4118,
					  2.91176, 3.14706, 17.3235, 0.5, -3.14706, 29.8529, 2.32353, -3.14706, 21.2941, 2.32353, -3.14706,
					  21.2941, 0.5, -3.14706, 29.8529, 2.32353, -3.14706, 21.2941, 2.47059, -3.14706, 8.52941, 2.47059,
					  -3.14706, 8.52941, 2.32353, -3.14706, 21.2941, 2.47059, -3.14706, 8.52941, 2.79412, -3.14706,
					  0.794118, 2.61765, -3.14706, -3.08824, 2.79412, -3.14706, 0.794118, 2.79412, -3.14706, 0.794118,
					  2.47059, -3.14706, 8.52941, 2.79412, -3.14706, 0.794118, 2.61765, -3.14706, -3.08824, 2.91176,
					  3.14706, 17.3235, -2.32353, 3.14706, 18.1176, 2.91176, 3.14706, 17.3235, -0.588235, 3.14706,
					  25.5, 2.91176, 3.14706, 17.3235, 0.441176, 3.14706, 32.4118, 2.91176, 3.14706, 17.3235, 5.64706,
					  3.14706, 6.14706, 5.64706, 3.14706, 6.14706, -2.82353, 3.14706, 6.5, 5.64706, 3.14706, 6.14706,
					  2.91176, 3.14706, 17.3235, 5.64706, 3.14706, 6.14706, 6.32353, 3.14706, -1.23529, 6.32353,
					  3.14706, -1.23529, 5.64706, 3.14706, 6.14706}},
			{{0, 1}, {-3.47059, 3.14706, 40.9706, 0.205882, 3.14706, 43.2353, -3.29412, 3.14706, 32.6471, 0.441176,
					  3.14706, 32.4118, -0.588235, 3.14706, 25.5, 0.441176, 3.14706, 32.4118, 0.205882, 3.14706,
					  43.2353, -3.47059, 3.14706, 40.9706, 0.205882, 3.14706, 43.2353, 0.205882, 3.14706, 43.2353,
					  0.205882, 3.14706, 43.2353, 0.264706, 1.05882, 37.8235, 0.264706, 1.05882, 37.8235, 0.205882,
					  3.14706, 43.2353, 0.264706, 1.05882, 37.8235, 0.5, -3.14706, 29.8529, 0.441176, 3.14706, 32.4118,
					  -3.29412, 3.14706, 32.6471, 0.441176, 3.14706, 32.4118, -0.588235, 3.14706, 25.5, 0.441176,
					  3.14706, 32.4118, 0.441176, 3.14706, 32.4118, 0.441176, 3.14706, 32.4118, 2.91176, 3.14706,
					  17.3235, 0.5, -3.14706, 29.8529, 0.264706, 1.05882, 37.8235, 0.5, -3.14706, 29.8529, 2.32353,
					  -3.14706, 21.2941, 2.32353, -3.14706, 21.2941, 0.5, -3.14706, 29.8529, 2.91176, 3.14706, 17.3235,
					  0.441176, 3.14706, 32.4118}},
		};

		const int width = 160;
		const int height = 160;
		const float cellSize = 0.2;
		const float cellHeight = 0.2;
		const int walkableClimb = 5;
		const int walkableHeight = 20;
		const int walkableRadius = 5;
		const int borderSize = 16;
		const int minRegionArea = 64;
		const int mergeRegionArea = 400;
		const float maxError = 1.3f;
		const int maxEdgeLen = 12;
		const int buildFlags = RC_CONTOUR_TESS_WALL_EDGES;
		const int maxVertsPerPoly = 6;
		const float sampleDist = 1.2f;
		const float sampleMaxError = 0.2f;
		const int walkableFlag = 1;
		const float walkableRadiusFloat = 0.861176f;

		rcContext context;

		dtNavMesh navMesh;

		dtNavMeshParams navMeshParams;
		std::memset(navMeshParams.orig, 0, sizeof(navMeshParams.orig));
		navMeshParams.tileWidth = 25.6f;
		navMeshParams.tileHeight = 25.6f;
		navMeshParams.maxTiles = 1024;
		navMeshParams.maxPolys = 4096;

		REQUIRE(navMesh.init(&navMeshParams) == DT_SUCCESS);

		for (const TileMesh& tileMesh: meshes)
		{
			const std::pair<int, int>& tile = tileMesh.tile;
			const rcMeshLoaderObj& mesh = *tileMesh.mesh;

			rcHeightfield solid;
			REQUIRE(rcCreateHeightfield(&context, solid, width, height, minBounds.at(tile).data(), maxBounds.at(tile).data(), cellSize, cellHeight));

			std::vector<unsigned char> areas(static_cast<std::size_t>(mesh.getTriCount()), RC_WALKABLE_AREA);
			REQUIRE(rcRasterizeTriangles(&context, mesh.getVerts(), mesh.getVertCount(), mesh.getTris(), areas.data(), mesh.getTriCount(), solid, walkableClimb));

			rcFilterLowHangingWalkableObstacles(&context, walkableClimb, solid);
			rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, solid);
			rcFilterWalkableLowHeightSpans(&context, walkableHeight, solid);

			rcCompactHeightfield compact;
			REQUIRE(rcBuildCompactHeightfield(&context, walkableHeight, walkableClimb, solid, compact));
			REQUIRE(rcErodeWalkableArea(&context, walkableRadius, compact));
			REQUIRE(rcBuildDistanceField(&context, compact));
			REQUIRE(rcBuildRegions(&context, compact, borderSize, minRegionArea, mergeRegionArea));

			rcContourSet contourSet;
			REQUIRE(rcBuildContours(&context, compact, maxError, maxEdgeLen, contourSet, buildFlags));

			REQUIRE(contourSet.nconts != 0);

			rcPolyMesh polyMesh;
			REQUIRE(rcBuildPolyMesh(&context, contourSet, maxVertsPerPoly, polyMesh));

			rcPolyMeshDetail polyMeshDetail;
			REQUIRE(rcBuildPolyMeshDetail(&context, polyMesh, compact, sampleDist, sampleMaxError, polyMeshDetail));

			for (int i = 0; i < polyMesh.npolys; ++i)
				polyMesh.flags[i] = walkableFlag;

			const std::vector<float>& tileOffMeshConVerts = offMeshConVerts.at(tile);
			const int offMeshConCount = static_cast<int>(tileOffMeshConVerts.size() / 6);
			const std::vector<float> offMeshConRad(offMeshConCount, walkableRadiusFloat);
			const std::vector<unsigned char> offMeshConDir(offMeshConCount, 0);
			const std::vector<unsigned char> offMeshConAreas(offMeshConCount, RC_WALKABLE_AREA);
			const std::vector<unsigned short> offMeshConFlags(offMeshConCount, walkableFlag);

			dtNavMeshCreateParams params;
			params.verts = polyMesh.verts;
			params.vertCount = polyMesh.nverts;
			params.polys = polyMesh.polys;
			params.polyAreas = polyMesh.areas;
			params.polyFlags = polyMesh.flags;
			params.polyCount = polyMesh.npolys;
			params.nvp = polyMesh.nvp;
			params.detailMeshes = polyMeshDetail.meshes;
			params.detailVerts = polyMeshDetail.verts;
			params.detailVertsCount = polyMeshDetail.nverts;
			params.detailTris = polyMeshDetail.tris;
			params.detailTriCount = polyMeshDetail.ntris;
			params.offMeshConVerts = tileOffMeshConVerts.data();
			params.offMeshConRad = offMeshConRad.data();
			params.offMeshConDir = offMeshConDir.data();
			params.offMeshConAreas = offMeshConAreas.data();
			params.offMeshConFlags = offMeshConFlags.data();
			params.offMeshConUserID = nullptr;
			params.offMeshConCount = offMeshConCount;
			params.walkableHeight = 3.91176f;
			params.walkableRadius = walkableRadiusFloat;
			params.walkableClimb = 1;
			rcVcopy(params.bmin, polyMesh.bmin);
			rcVcopy(params.bmax, polyMesh.bmax);
			params.cs = cellSize;
			params.ch = cellHeight;
			params.buildBvTree = true;
			params.userId = 0;
			params.tileX = tile.first;
			params.tileY = tile.second;
			params.tileLayer = 0;

			unsigned char* navMeshData;
			int navMeshDataSize;
			REQUIRE(dtCreateNavMeshData(&params, &navMeshData, &navMeshDataSize));

			const int flags = DT_TILE_FREE_DATA;
			const dtTileRef lastRef = 0;
			dtTileRef* const result = nullptr;
			REQUIRE(navMesh.addTile(navMeshData, navMeshDataSize, flags, lastRef, result) == DT_SUCCESS);
		}

		dtQueryFilter queryFilter;
		queryFilter.setIncludeFlags(walkableFlag);
		queryFilter.setAreaCost(RC_WALKABLE_AREA, 1);

		const float startPos[3] = {1.00857f, 3.21409f, 23.7162f};
		const float endPos[3] = {2.64706f, -2.58824f, -2.64706f};
		const float polyHalfExtents[3] = {3.44471f, 7.82353f, 3.35059f};

		dtNavMeshQuery navMeshQuery;
		navMeshQuery.init(&navMesh, 2048);

		float startNavMeshPos[3];
		dtPolyRef startRef = 0;
		REQUIRE(navMeshQuery.findNearestPoly(startPos, polyHalfExtents, &queryFilter, &startRef, startNavMeshPos) == DT_SUCCESS);
		REQUIRE(startRef != 0);
		REQUIRE(startNavMeshPos[0] == 1.00857f);
		// REQUIRE(startNavMeshPos[1] == 3.31665f);
		REQUIRE(startNavMeshPos[2] == 23.7162f);

		float endNavMeshPos[3];
		dtPolyRef endRef;
		REQUIRE(navMeshQuery.findNearestPoly(endPos, polyHalfExtents, &queryFilter, &endRef, endNavMeshPos) == DT_SUCCESS);
		REQUIRE(endRef != 0);
		REQUIRE(endNavMeshPos[0] == 2.64706f);
		// REQUIRE(endNavMeshPos[1] == -3.08335f);
		REQUIRE(endNavMeshPos[2] == -2.64706f);

		int pathLen = 0;
		dtPolyRef pathBuffer[2048];
		REQUIRE(navMeshQuery.findPath(startRef, endRef, startPos, endPos, &queryFilter, pathBuffer, &pathLen, std::size(pathBuffer)) == DT_SUCCESS);
		REQUIRE(pathLen == 32);
	}
}
