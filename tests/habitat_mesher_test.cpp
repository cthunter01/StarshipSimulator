#include "StarshipSimulator/core/procgen/habitat_mesher.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <utility>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

/// Island Three meshed coarsely, so the tests run fast.
const HabitatMeshes& coarseIslandThree()
{
    static const HabitatGeometry geometry{OneillCylinderSpec{}};
    static const HabitatMeshes   meshes = buildHabitatMeshes(
        geometry,
        MeshingSettings{
            .cellSizeM = 200.0, .chunkSizeM = 2000.0, .glassCellSizeM = 1000.0, .threads = 0});
    return meshes;
}

const HabitatGeometry& islandThree()
{
    static const HabitatGeometry geometry{OneillCylinderSpec{}};
    return geometry;
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

}  // namespace
