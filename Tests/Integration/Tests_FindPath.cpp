#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"
#include "RecastAlloc.h"

#include "catch2/catch_amalgamated.hpp"

#include <memory>

namespace
{

constexpr int tileSize = 128;
constexpr float cellSize = 0.2f;
constexpr float cellHeight = cellSize;
constexpr float tileSizeFloat = tileSize * cellSize;
constexpr int borderSize = 16;
constexpr int width = tileSize + 2 * borderSize;
constexpr int height = tileSize + 2 * borderSize;
constexpr float walkableSlopeAngle = 49;
constexpr int walkableClimb = 3;
constexpr int walkableHeight = 12;
constexpr int walkableRadius = 4;
constexpr int minRegionArea = 64;
constexpr int mergeRegionArea = 400;
constexpr float maxError = 1.3f;
constexpr int maxEdgeLen = 60;
constexpr int buildFlags = RC_CONTOUR_TESS_WALL_EDGES;
constexpr int maxVertsPerPoly = 6;
constexpr float sampleDist = 1.2f;
constexpr float sampleMaxError = 0.2f;
constexpr int walkableFlag = 1;
constexpr float walkableHeightFloat = 3.91176f;
constexpr float walkableRadiusFloat = 0.723745f;
constexpr float walkableClimbFloat = 0.6f;
constexpr int maxNodes = 128;

struct FreePolyMeshDetail
{
    void operator()(rcPolyMeshDetail* ptr) const
    {
        rcFree(ptr->meshes);
        ptr->meshes = nullptr;
        rcFree(ptr->verts);
        ptr->verts = nullptr;
        rcFree(ptr->tris);
        ptr->tris = nullptr;
    }
};

float minXyBounds(int value)
{
    return static_cast<float>(value * tileSize - borderSize) * cellSize;
}

float maxXyBounds(int value)
{
    return static_cast<float>((value + 1) * tileSize + borderSize) * cellSize;
}

float minLayerBounds(int value)
{
    return static_cast<float>(value * tileSize - borderSize) * cellHeight;
}

float maxLayerBounds(int value)
{
    return static_cast<float>((value + 1) * tileSize + borderSize) * cellHeight;
}

unsigned short getFlag(unsigned char area)
{
    if (area == RC_WALKABLE_AREA)
        return walkableFlag;
    return 0;
}

void addTile(rcContext& context, int x, int y, int layer, bool limitLayer, const float* verts, const int vertCount, const int* tris, const int triCount, dtNavMesh& navMesh)
{
    REQUIRE(vertCount > 0);
    REQUIRE(triCount > 0);

    float minLayer;
    float maxLayer;
    if (limitLayer)
    {
        minLayer = minLayerBounds(layer);
        maxLayer = maxLayerBounds(layer);
    }
    else
    {
        float minBounds[3];
        float maxBounds[3];
        rcCalcBounds(verts, vertCount, minBounds, maxBounds);
        minLayer = minBounds[1];
        maxLayer = maxBounds[1];
    }

    const float minBounds[3] = {
        minXyBounds(x),
        minLayer,
        minXyBounds(y),
    };

    const float maxBounds[3] = {
        maxXyBounds(x),
        maxLayer,
        maxXyBounds(y),
    };

    rcHeightfield solid;
    REQUIRE(rcCreateHeightfield(&context, solid, width, height, minBounds, maxBounds, cellSize, cellHeight));

    rcTempVector<unsigned char> triAreaIDs(static_cast<std::size_t>(triCount), RC_WALKABLE_AREA);

    rcClearUnwalkableTriangles(&context, walkableSlopeAngle, verts, vertCount, tris, triCount, triAreaIDs.data());

    REQUIRE(rcRasterizeTriangles(&context, verts, vertCount, tris, triAreaIDs.data(), triCount, solid, walkableClimb));

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

    REQUIRE(contourSet.nconts > 0);

    rcPolyMesh polyMesh;
    REQUIRE(rcBuildPolyMesh(&context, contourSet, maxVertsPerPoly, polyMesh));

    REQUIRE(polyMesh.nverts > 0);
    REQUIRE(polyMesh.npolys > 0);

    rcPolyMeshDetail polyMeshDetail;
    const std::unique_ptr<rcPolyMeshDetail, FreePolyMeshDetail> polyMeshDetailPtr(&polyMeshDetail);
    REQUIRE(rcBuildPolyMeshDetail(&context, polyMesh, compact, sampleDist, sampleMaxError, polyMeshDetail));

    for (int i = 0; i < polyMesh.npolys; ++i)
        polyMesh.flags[i] = getFlag(polyMesh.areas[i]);

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
    params.offMeshConVerts = nullptr;
    params.offMeshConRad = nullptr;
    params.offMeshConDir = nullptr;
    params.offMeshConAreas = nullptr;
    params.offMeshConFlags = nullptr;
    params.offMeshConUserID = nullptr;
    params.offMeshConCount = 0;
    params.walkableHeight = walkableHeightFloat;
    params.walkableRadius = walkableRadiusFloat;
    params.walkableClimb = walkableClimbFloat;
    rcVcopy(params.bmin, polyMesh.bmin);
    rcVcopy(params.bmax, polyMesh.bmax);
    params.cs = cellSize;
    params.ch = cellHeight;
    params.buildBvTree = true;
    params.userId = 0;
    params.tileX = x;
    params.tileY = y;
    params.tileLayer = layer;

    unsigned char* navMeshData;
    int navMeshDataSize;
    REQUIRE(dtCreateNavMeshData(&params, &navMeshData, &navMeshDataSize));

    const int flags = DT_TILE_FREE_DATA;
    const dtTileRef lastRef = 0;
    dtTileRef* const result = nullptr;
    REQUIRE(navMesh.addTile(navMeshData, navMeshDataSize, flags, lastRef, result) == DT_SUCCESS);
}

}

TEST_CASE("FindPathOverSlope")
{
    using Catch::Matchers::WithinAbs;

    dtNavMeshParams navMeshParams;
    std::memset(navMeshParams.orig, 0, sizeof(navMeshParams.orig));
    navMeshParams.tileWidth = tileSizeFloat;
    navMeshParams.tileHeight = tileSizeFloat;
    navMeshParams.maxTiles = 2048;
    navMeshParams.maxPolys = 2048;

    dtNavMesh navMesh;

    REQUIRE(navMesh.init(&navMeshParams) == DT_SUCCESS);

    dtNavMeshQuery navMeshQuery;

    REQUIRE(navMeshQuery.init(&navMesh, maxNodes) == DT_SUCCESS);

    dtQueryFilter queryFilter;
    queryFilter.setIncludeFlags(walkableFlag);
    queryFilter.setAreaCost(RC_WALKABLE_AREA, 1);

    rcContext context;

    constexpr float verts[] = {
        0, -8, 1, // 0
        6, -4, 1, // 1
        12, 0, 1, // 2
        18, 4, 1, // 3
        24, 8, 1, // 4

        0, -8, 25, // 5
        6, -4, 25, // 6
        12, 0, 25, // 7
        18, 4, 25, // 8
        24, 8, 25, // 9
    };
    constexpr int tris[] = {
        0, 6, 1,
        0, 5, 6,
        1, 7, 2,
        1, 6, 7,
        2, 8, 3,
        2, 7, 8,
        3, 9, 4,
        3, 8, 9,
    };

    constexpr int vertCount = std::size(verts) / 3;
    constexpr int triCount = std::size(tris) / 3;

    constexpr float startPos[3] = {1, -8, 13};
    constexpr float endPos[3] = {24, 8, 13};
    constexpr float polyHalfExtents[3] = {1, 1, 1};

    SECTION("Should build single tile navmesh and find path")
    {
        addTile(context, 0, 0, 0, false, verts, vertCount, tris, triCount, navMesh);

        float startNavMeshPos[3] {};
        dtPolyRef startRef = 0;
        CHECK(navMeshQuery.findNearestPoly(startPos, polyHalfExtents, &queryFilter, &startRef, startNavMeshPos) == DT_SUCCESS);
        CHECK(startRef != 0);
        CHECK_THAT(startNavMeshPos[0], WithinAbs(1.0f, 1e-3));
        CHECK_THAT(startNavMeshPos[1], WithinAbs(-7.2f, 1e-3));
        CHECK_THAT(startNavMeshPos[2], WithinAbs(13.0f, 1e-3));

        float endNavMeshPos[3] {};
        dtPolyRef endRef;
        CHECK(navMeshQuery.findNearestPoly(endPos, polyHalfExtents, &queryFilter, &endRef, endNavMeshPos) == DT_SUCCESS);
        CHECK(endRef != 0);
        CHECK_THAT(endNavMeshPos[0], WithinAbs(23.0f, 1e-3));
        CHECK_THAT(endNavMeshPos[1], WithinAbs(7.6f, 1e-3));
        CHECK_THAT(endNavMeshPos[2], WithinAbs(13.0f, 1e-3));

        CHECK(startRef == endRef);

        int pathLen = 0;
        dtPolyRef pathBuffer[16] {};
        CHECK(navMeshQuery.findPath(startRef, endRef, startPos, endPos, &queryFilter, pathBuffer, &pathLen, std::size(pathBuffer)) == DT_SUCCESS);
        CHECK(pathLen == 1);
        CHECK(pathBuffer[0] == startRef);
    }

    SECTION("Should build 2 layered tiles navmesh and find path")
    {
        addTile(context, 0, 0, -1, true, verts, vertCount, tris, triCount, navMesh);
        addTile(context, 0, 0, 0, true, verts, vertCount, tris, triCount, navMesh);

        float startNavMeshPos[3] {};
        dtPolyRef startRef = 0;
        CHECK(navMeshQuery.findNearestPoly(startPos, polyHalfExtents, &queryFilter, &startRef, startNavMeshPos) == DT_SUCCESS);
        CHECK(startRef != 0);
        CHECK_THAT(startNavMeshPos[0], WithinAbs(1.0f, 1e-3));
        CHECK_THAT(startNavMeshPos[1], WithinAbs(-7.2f, 1e-3));
        CHECK_THAT(startNavMeshPos[2], WithinAbs(13.0f, 1e-3));

        float endNavMeshPos[3] {};
        dtPolyRef endRef;
        CHECK(navMeshQuery.findNearestPoly(endPos, polyHalfExtents, &queryFilter, &endRef, endNavMeshPos) == DT_SUCCESS);
        CHECK(endRef != 0);
        CHECK_THAT(endNavMeshPos[0], WithinAbs(23.0f, 1e-3));
        CHECK_THAT(endNavMeshPos[1], WithinAbs(7.6f, 1e-3));
        CHECK_THAT(endNavMeshPos[2], WithinAbs(13.0f, 1e-3));

        int pathLen = 0;
        dtPolyRef pathBuffer[16] {};
        CHECK(navMeshQuery.findPath(startRef, endRef, startPos, endPos, &queryFilter, pathBuffer, &pathLen, std::size(pathBuffer)) == DT_SUCCESS);
        CHECK(pathLen >= 2);
        CHECK(pathBuffer[0] == startRef);
        CHECK(pathBuffer[pathLen - 1] == endRef);
    }
}
