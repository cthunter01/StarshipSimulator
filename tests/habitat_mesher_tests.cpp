#include "StarshipSimulator/core/procgen/habitat_mesher.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <utility>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/hull_mesh.h"
#include "StarshipSimulator/core/procgen/mesh.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

/// Island Three meshed coarsely, so the tests run fast.
const HabitatMeshes& coarseIslandThree()
{
    static const HabitatGeometry kGeometry{OneillCylinderSpec{}};
    static const HabitatMeshes   kMeshes = buildHabitatMeshes(
        kGeometry,
        MeshingSettings{
            .cellSizeM = 200.0, .chunkSizeM = 2000.0, .glassCellSizeM = 1000.0, .threads = 0});
    return kMeshes;
}

const HabitatGeometry& islandThree()
{
    static const HabitatGeometry kGeometry{OneillCylinderSpec{}};
    return kGeometry;
}

TEST(HabitatMesher, AngleGridPutsWindowEdgesOnGridLines)
{
    const HabitatGeometry& geometry = islandThree();
    const AngleGrid        grid     = buildAngleGrid(geometry, 25.0);
    ASSERT_EQ(grid.angles.size(), grid.windowSegment.size() + 1);
    EXPECT_NEAR(grid.angles.back() - grid.angles.front(), 2.0 * kPi, 1e-12);
    for (int i = 0; i < geometry.stripCount(); ++i)
    {
        for (const double edge : {geometry.windowCenter(i) - geometry.windowHalfAngle(),
                                  geometry.windowCenter(i) + geometry.windowHalfAngle()})
        {
            bool found = false;
            for (const double angle : grid.angles)
            {
                found = found || std::abs(std::remainder(angle - edge, 2.0 * kPi)) < 1e-9;
            }
            EXPECT_TRUE(found) << "window edge " << edge;
        }
    }
    // About 25 m cells at the hull.
    EXPECT_NEAR(2.0 * kPi * 4000.0 / static_cast<double>(grid.windowSegment.size()), 25.0, 0.5);
}

TEST(HabitatMesher, ProducesTerrainAndGlass)
{
    const HabitatMeshes& meshes  = coarseIslandThree();
    std::size_t          terrain = 0;
    std::size_t          glass   = 0;
    for (const MeshChunk& chunk : meshes.chunks)
    {
        (chunk.kind == ChunkKind::Glass ? glass : terrain) += 1;
        EXPECT_FALSE(chunk.mesh.indices.empty());
    }
    EXPECT_GT(terrain, 10U);
    EXPECT_GT(glass, 3U);
    EXPECT_GT(meshes.triangleCount, 10000U);
}

TEST(HabitatMesher, NormalsPointIntoTheHabitatAndTrianglesFaceThem)
{
    const HabitatGeometry& geometry = islandThree();
    for (const MeshChunk& chunk : coarseIslandThree().chunks)
    {
        const auto& vertices = chunk.mesh.vertices;
        for (const Vertex& vertex : vertices)
        {
            ASSERT_FALSE(std::isnan(vertex.position.x) || std::isnan(vertex.normal.x));
            EXPECT_NEAR(glm::length(vertex.normal), 1.0F, 1e-4F);
            const Vec3d world = chunk.origin + Vec3d(vertex.position);
            if (vertex.material == material::kValley || vertex.material == material::kGlass)
            {
                // Valley floors and windows face the axis.
                EXPECT_GT(glm::dot(Vec3d(vertex.normal), HabitatGeometry::localUp(world)), 0.5);
            }
            EXPECT_TRUE(glm::all(glm::greaterThanEqual(vertex.position, chunk.boundsMin)));
            EXPECT_TRUE(glm::all(glm::lessThanEqual(vertex.position, chunk.boundsMax)));
        }
        for (std::size_t i = 0; i + 2 < chunk.mesh.indices.size(); i += 3)
        {
            const Vertex& a    = vertices[chunk.mesh.indices[i]];
            const Vertex& b    = vertices[chunk.mesh.indices[i + 1]];
            const Vertex& c    = vertices[chunk.mesh.indices[i + 2]];
            const Vec3f   face = glm::cross(b.position - a.position, c.position - a.position);
            if (glm::length(face) > 1e-2F)  // skip slivers at the dome's pole
            {
                EXPECT_GT(glm::dot(face, a.normal + b.normal + c.normal), 0.0F);
            }
        }
    }
    (void)geometry;
}

TEST(HabitatMesher, NeighbouringChunksShareTheirEdgesExactly)
{
    // Terrain vertices are identified by their grid coordinates (uv); every chunk that has a given
    // grid vertex must put it in the same place, or cracks would appear.
    std::map<std::pair<std::int64_t, std::int64_t>, Vec3d> seen;
    std::size_t                                            shared = 0;
    for (const MeshChunk& chunk : coarseIslandThree().chunks)
    {
        if (chunk.kind != ChunkKind::Terrain ||
            chunk.mesh.vertices.front().material == material::kMetal)
        {
            continue;
        }
        for (const Vertex& vertex : chunk.mesh.vertices)
        {
            const auto  key   = std::make_pair(std::llround(vertex.uv.x * 10.0F),
                                               std::llround(vertex.uv.y * 10.0F));
            const Vec3d world = chunk.origin + Vec3d(vertex.position);
            if (const auto found = seen.find(key); found != seen.end())
            {
                EXPECT_LT(glm::distance(found->second, world), 0.01);
                ++shared;
            }
            else
            {
                seen.emplace(key, world);
            }
        }
    }
    EXPECT_GT(shared, 100U);
}

TEST(HabitatMesher, OutputDoesNotDependOnTheThreadCount)
{
    OneillCylinderSpec spec;
    spec.radiusM           = 1000.0;
    spec.lengthM           = 8000.0;
    spec.antisunwardEndcap = makeEndcap(EndcapShape::Hemisphere);
    const HabitatGeometry geometry(spec);
    const auto            build = [&](unsigned threads) {
        return buildHabitatMeshes(geometry, MeshingSettings{.cellSizeM      = 100.0,
                                                            .chunkSizeM     = 1000.0,
                                                            .glassCellSizeM = 500.0,
                                                            .threads        = threads});
    };
    const HabitatMeshes one  = build(1);
    const HabitatMeshes many = build(4);
    ASSERT_EQ(one.chunks.size(), many.chunks.size());
    for (std::size_t i = 0; i < one.chunks.size(); ++i)
    {
        const auto& a = one.chunks[i].mesh.vertices;
        const auto& b = many.chunks[i].mesh.vertices;
        ASSERT_EQ(a.size(), b.size());
        EXPECT_EQ(std::memcmp(a.data(), b.data(), a.size() * sizeof(Vertex)), 0);
    }
}

// ---- The hull, seen from outside --------------------------------------------------------------

TEST(HullMesh, ClosedOutwardFacingShellWithWindowStrips)
{
    const HabitatGeometry& geometry = islandThree();
    const CpuMesh          hull     = buildHullMesh(geometry);
    ASSERT_FALSE(hull.indices.empty());
    std::size_t glass = 0;
    for (std::size_t i = 0; i < hull.indices.size(); i += 3)
    {
        const Vertex& a = hull.vertices[hull.indices[i]];
        const Vertex& b = hull.vertices[hull.indices[i + 1]];
        const Vertex& c = hull.vertices[hull.indices[i + 2]];
        const Vec3f   n = glm::cross(b.position - a.position, c.position - a.position);
        if (glm::length(n) < 1e-3F)
        {
            continue;  // degenerate at the axis
        }
        // Counter-clockwise seen from outside: the face normal agrees with the vertex normal.
        EXPECT_GT(glm::dot(n, a.normal), 0.0F);
        // Every vertex lies on the hull: radius R on the wall, inside the dome elsewhere.
        const float r = std::hypot(a.position.x, a.position.y);
        EXPECT_LE(r, static_cast<float>(geometry.radius()) + 0.01F);
        if (a.material == material::kGlass)
        {
            ++glass;
            EXPECT_NEAR(r, geometry.radius(), 0.01);
            EXPECT_GE(a.position.z, geometry.floorZMin() - 0.01);
            EXPECT_LE(a.position.z, geometry.floorZMax() + 0.01);
            const Vec3d centre = Vec3d(a.position + b.position + c.position) / 3.0;
            EXPECT_EQ(geometry.regionAt(centre.z, HabitatGeometry::angleOf(centre)).kind,
                      RegionKind::Window);
        }
    }
    EXPECT_GT(glass, 0U);
}

TEST(HullMesh, PartnerCounterRotatesAlongside)
{
    const double separation = 80000.0;
    const Mat4d  atRest     = partnerTransform(separation, 0.0);
    const Vec4d  centre     = atRest * Vec4d(0.0, 0.0, 0.0, 1.0);
    EXPECT_NEAR(centre.x, separation, 1e-6);
    EXPECT_NEAR(centre.y, 0.0, 1e-6);

    // A quarter turn later the partner has swung a quarter turn backwards around us, and its own
    // +X (turned the other way) has turned half a turn relative to ours.
    const Mat4d later = partnerTransform(separation, kPi / 2.0);
    const Vec4d moved = later * Vec4d(0.0, 0.0, 0.0, 1.0);
    EXPECT_NEAR(moved.x, 0.0, 1e-6);
    EXPECT_NEAR(moved.y, -separation, 1e-6);
    const Vec4d x = later * Vec4d(1.0, 0.0, 0.0, 0.0);
    EXPECT_NEAR(x.x, -1.0, 1e-12);
    const Vec4d z = later * Vec4d(0.0, 0.0, 1.0, 0.0);
    EXPECT_NEAR(z.z, 1.0, 1e-12);  // axes stay parallel: both point at the Sun
}

}  // namespace
