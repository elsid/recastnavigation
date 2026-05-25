#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"
#include "RecastAlloc.h"

#include "catch2/catch_all.hpp"

#include <cmath>
#include <memory>

namespace
{

constexpr int tileSize = 128;
constexpr float cellSize = 0.3f;
constexpr float cellHeight = 0.2f;
constexpr float tileSizeFloat = tileSize * cellSize;
constexpr float walkableHeightFloat = 2;
constexpr float walkableRadiusFloat = 0.6f;
constexpr float walkableClimbFloat = 0.9f;
constexpr float walkableSlopeAngle = 45;
const int walkableHeight = static_cast<int>(std::ceil(walkableHeightFloat / cellHeight));
const int walkableRadius = static_cast<int>(std::ceil(walkableRadiusFloat / cellSize));
const int walkableClimb = static_cast<int>(std::floor(walkableClimbFloat / cellHeight));
const int borderSize = walkableRadius + 3;
const int width = tileSize + 2 * borderSize;
const int height = tileSize + 2 * borderSize;
constexpr int minRegionArea = 64;
constexpr int mergeRegionArea = 400;
constexpr float maxError = 1.3f;
constexpr int maxEdgeLen = 60;
constexpr int buildFlags = RC_CONTOUR_TESS_WALL_EDGES;
constexpr int maxVertsPerPoly = 6;
constexpr float sampleDist = 1.2f;
constexpr float sampleMaxError = 0.2f;
constexpr int walkableFlag = 1;
constexpr int maxNodes = 1024;
constexpr float offsetX = tileSize / 2 * cellSize * 0.9f;
constexpr float offsetY = 0;
constexpr float offsetZ = offsetX;
constexpr float radius = offsetX * 0.9f;
const float stepHeight = walkableClimb * cellHeight * 0.9f;
constexpr float stepAngle = 0.19634954084936207f;
constexpr int steps = 1024;

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

template <typename T, std::size_t n>
constexpr std::size_t size(const T (&)[n]) noexcept
{
    return n;
}

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

void addTile(rcContext& context, int x, int y, int layer, const float* verts, const int vertCount, const int* tris, const int triCount, const float* minBounds, const float* maxBounds, dtNavMesh& navMesh)
{
    REQUIRE(vertCount > 0);
    REQUIRE(triCount > 0);

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

    unsigned char* navMeshData = 0;
    int navMeshDataSize = 0;
    REQUIRE(dtCreateNavMeshData(&params, &navMeshData, &navMeshDataSize));

    REQUIRE(navMeshData != nullptr);
    REQUIRE(navMeshDataSize > 0);

    const int flags = DT_TILE_FREE_DATA;
    const dtTileRef lastRef = 0;
    dtTileRef* const result = nullptr;
    REQUIRE(navMesh.addTile(navMeshData, navMeshDataSize, flags, lastRef, result) == DT_SUCCESS);
}

void addTile2d(rcContext& context, int x, int y, const float* verts, const int vertCount, const int* tris, const int triCount, dtNavMesh& navMesh)
{
    float minLayer;
    float maxLayer;

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

    addTile(context, x, y, 0, verts, vertCount, tris, triCount, minBounds, maxBounds, navMesh);
}

void addTile3d(rcContext& context, int x, int y, int layer, const float* verts, const int vertCount, const int* tris, const int triCount, dtNavMesh& navMesh)
{
    const float minBounds[3] = {
        minXyBounds(x),
        minLayerBounds(layer),
        minXyBounds(y),
    };

    const float maxBounds[3] = {
        maxXyBounds(x),
        maxLayerBounds(layer),
        maxXyBounds(y),
    };

    addTile(context, x, y, layer, verts, vertCount, tris, triCount, minBounds, maxBounds, navMesh);
}

struct Mesh
{
    rcPermVector<float> verts;
    rcPermVector<int> tris;
};

void addVertex(const float* v, Mesh& mesh)
{
    for (int i = 0; i < 3; ++i)
        mesh.verts.push_back(v[i]);
}

void addTriangle(int a, int b, int c, Mesh& mesh)
{
    mesh.tris.push_back(a);
    mesh.tris.push_back(b);
    mesh.tris.push_back(c);
}

Mesh generateSpiralStairs(float offsetX, float offsetY, float offsetZ, float radius, float stepHeight, float stepAngle, int steps)
{
    Mesh result;
    int baseIndex = 0;

    for (int step = 0; step < steps; ++step)
    {
        const float height = offsetY + step * stepHeight;
        const float angle = step * stepAngle;
        const float nextAngle = (step + 1) * stepAngle;

        const float inner[3] = {offsetX, height, offsetZ};
        addVertex(inner, result);

        const float outerA[3] = {offsetX + radius * std::cos(angle), height, offsetZ + radius * std::sin(angle)};
        addVertex(outerA, result);

        const float outerB[3] = {offsetX + radius * std::cos(nextAngle), height, offsetZ + radius * std::sin(nextAngle)};
        addVertex(outerB, result);

        if (step > 0)
        {
            addTriangle(baseIndex, baseIndex - 1, baseIndex - 3, result);
            addTriangle(baseIndex, baseIndex + 1, baseIndex - 1, result);
        }

        addTriangle(baseIndex, baseIndex + 2, baseIndex + 1, result);

        baseIndex += 3;
    }

    return result;
}

}

TEST_CASE("FindPathOverSpiralStairsWithNavMesh")
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

    rcContext context;

    const Mesh mesh = generateSpiralStairs(offsetX, offsetY, offsetZ, radius, stepHeight, stepAngle, steps);

    const int vertCount = mesh.verts.size() / 3;
    const int triCount = mesh.tris.size() / 3;

    const float* startPos = &mesh.verts[1 * 3];
    const float* endPos = &mesh.verts[mesh.verts.size() - 3];
    constexpr float polyHalfExtents[3] = {2, 2, 2};

    SECTION("Should build single tile navmesh and find path")
    {
        addTile2d(context, 0, 0, mesh.verts.data(), vertCount, mesh.tris.data(), triCount, navMesh);

        dtNavMeshQuery navMeshQuery;

        REQUIRE(navMeshQuery.init(&navMesh, maxNodes) == DT_SUCCESS);

        dtQueryFilter queryFilter;
        queryFilter.setIncludeFlags(walkableFlag);
        queryFilter.setAreaCost(RC_WALKABLE_AREA, 1);

        float startNavMeshPos[3];
        dtPolyRef startRef = 0;
        REQUIRE(navMeshQuery.findNearestPoly(startPos, polyHalfExtents, &queryFilter, &startRef, startNavMeshPos) == DT_SUCCESS);
        REQUIRE(startRef != 0);
        CHECK_THAT(startNavMeshPos[0], WithinAbs(32.1f, 1e-3));
        CHECK_THAT(startNavMeshPos[1], WithinAbs(0.2f, 1e-3));
        CHECK_THAT(startNavMeshPos[2], WithinAbs(18.0f, 1e-3));

        float endNavMeshPos[3];
        dtPolyRef endRef = 0;
        REQUIRE(navMeshQuery.findNearestPoly(endPos, polyHalfExtents, &queryFilter, &endRef, endNavMeshPos) == DT_SUCCESS);
        REQUIRE(endRef != 0);
        CHECK_THAT(endNavMeshPos[0], WithinAbs(32.1f, 1e-3));
        CHECK_THAT(endNavMeshPos[1], WithinAbs(736.6f, 1e-3));
        CHECK_THAT(endNavMeshPos[2], WithinAbs(16.5f, 1e-3));

        int pathLen = 0;
        dtPolyRef pathBuffer[1024];
        CHECK(navMeshQuery.findPath(startRef, endRef, startNavMeshPos, endNavMeshPos, &queryFilter, pathBuffer, &pathLen, size(pathBuffer)) == DT_SUCCESS);
        REQUIRE(pathLen > 0);
        CHECK(pathBuffer[0] == startRef);
        CHECK(pathBuffer[pathLen - 1] == endRef);
    }

    SECTION("Should build 2 layered tiles navmesh and find path")
    {
        for (int i = 0; i < 29; ++i)
            addTile3d(context, 0, 0, i, mesh.verts.data(), vertCount, mesh.tris.data(), triCount, navMesh);

        dtNavMeshQuery navMeshQuery;

        REQUIRE(navMeshQuery.init(&navMesh, maxNodes) == DT_SUCCESS);

        dtQueryFilter queryFilter;
        queryFilter.setIncludeFlags(walkableFlag);
        queryFilter.setAreaCost(RC_WALKABLE_AREA, 1);

        float startNavMeshPos[3];
        dtPolyRef startRef = 0;
        REQUIRE(navMeshQuery.findNearestPoly(startPos, polyHalfExtents, &queryFilter, &startRef, startNavMeshPos) == DT_SUCCESS);
        REQUIRE(startRef != 0);
        CHECK_THAT(startNavMeshPos[0], WithinAbs(32.1f, 1e-3));
        CHECK_THAT(startNavMeshPos[1], WithinAbs(0.2f, 1e-3));
        CHECK_THAT(startNavMeshPos[2], WithinAbs(18.0f, 1e-3));

        float endNavMeshPos[3];
        dtPolyRef endRef = 0;
        REQUIRE(navMeshQuery.findNearestPoly(endPos, polyHalfExtents, &queryFilter, &endRef, endNavMeshPos) == DT_SUCCESS);
        REQUIRE(endRef != 0);
        CHECK_THAT(endNavMeshPos[0], WithinAbs(32.1f, 1e-3));
        CHECK_THAT(endNavMeshPos[1], WithinAbs(736.6f, 1e-3));
        CHECK_THAT(endNavMeshPos[2], WithinAbs(16.5f, 1e-3));

        int pathLen = 0;
        dtPolyRef pathBuffer[1024];
        CHECK(navMeshQuery.findPath(startRef, endRef, startNavMeshPos, endNavMeshPos, &queryFilter, pathBuffer, &pathLen, size(pathBuffer)) == DT_SUCCESS);
        REQUIRE(pathLen > 0);
        CHECK(pathBuffer[0] == startRef);
        CHECK(pathBuffer[pathLen - 1] == endRef);
    }
}
