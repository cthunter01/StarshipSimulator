#include "StarshipSimulator/core/procgen/buildings.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/gpu_abi/ground_atlas.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

/// A small habitat with a river (quick to plan), and what it builds.
struct Village
{
    Village()
    {
        spec.radiusM                    = 1200.0;
        spec.lengthM                    = 9000.0;
        spec.partner.enabled            = false;
        spec.terrain.riverWidthM        = 25.0;
        spec.terrain.lakesPerValley     = 0;
        spec.settlements.townsPerValley = 1;
        spec.settlements.townRadiusM    = 150.0;
        spec.settlements.farmsPerValley = 2;
    }
    OneillCylinderSpec spec;
    HabitatGeometry    geometry{spec};
    TerrainGrid        grid        = sampleTerrain(geometry, 4.0);
    Settlements        settlements = planSettlements(geometry, grid);
};

const Village& village()
{
    static const Village v;
    return v;
}

/// Every triangle's winding agrees with its vertices' normals (counter-clockwise from the front).
void expectConsistentWinding(const CpuMesh& mesh)
{
    std::size_t bad = 0;
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const Vertex& a = mesh.vertices[mesh.indices[i]];
        const Vertex& b = mesh.vertices[mesh.indices[i + 1]];
        const Vertex& c = mesh.vertices[mesh.indices[i + 2]];
        const Vec3f   n = glm::cross(b.position - a.position, c.position - a.position);
        if (glm::length(n) > 1e-6F && glm::dot(n, a.normal + b.normal + c.normal) <= 0.0F)
        {
            ++bad;
        }
    }
    EXPECT_EQ(bad, 0U) << "of " << mesh.indices.size() / 3 << " triangles";
}

TEST(Buildings, FacadesPackAndUnpack)
{
    const Facade facade{.surface = buildingMaterial::kWall,
                        .colour  = 7,
                        .windows = 42,
                        .door    = true,
                        .shop    = false,
                        .style   = FacadeStyle::Hall,
                        .shutter = 3,
                        .seed    = 0x5A5};
    const Facade back = unpackFacade(packFacade(facade));
    EXPECT_EQ(back.surface, facade.surface);
    EXPECT_EQ(back.colour, 7U);
    EXPECT_EQ(back.windows, 42U);
    EXPECT_TRUE(back.door);
    EXPECT_FALSE(back.shop);
    EXPECT_EQ(back.style, FacadeStyle::Hall);
    EXPECT_EQ(back.shutter, 3U);
    EXPECT_EQ(back.seed, 0x5A5U);
    EXPECT_DOUBLE_EQ(storeyHeight(FacadeStyle::House), 3.0);
}

TEST(Buildings, TheVillageHasBuildingsBridgesAndFurniture)
{
    const Settlements& s = village().settlements;
    ASSERT_GE(s.townCount(), 1U);
    EXPECT_GT(s.buildings.size(), 30U);
    EXPECT_FALSE(s.bridges.empty());
    EXPECT_TRUE(std::ranges::any_of(
        s.furniture, [](const Furniture& f) { return f.kind == FurnitureKind::Fountain; }));
    EXPECT_TRUE(std::ranges::any_of(s.buildings,
                                    [](const Building& b) { return b.use == BuildingUse::Tower; }));
}

TEST(Buildings, MeshesFaceOutAndStayInTheirBounds)
{
    const std::vector<SettlementMesh> meshes = buildSettlementMeshes(village().settlements);
    ASSERT_FALSE(meshes.empty());
    for (const SettlementMesh& m : meshes)
    {
        expectConsistentWinding(m.mesh);
        for (const Vertex& v : m.mesh.vertices)
        {
            EXPECT_TRUE(glm::all(glm::greaterThanEqual(v.position, m.boundsMin)));
            EXPECT_TRUE(glm::all(glm::lessThanEqual(v.position, m.boundsMax)));
            ASSERT_NEAR(glm::length(v.normal), 1.0F, 1e-3F);
        }
        // Everything stands near its settlement (a few hundred metres at most).
        EXPECT_LT(glm::length(Vec3d(m.boundsMax - m.boundsMin)), 1500.0);
    }
}

TEST(Buildings, WallsFaceAwayFromTheirBuilding)
{
    // One building alone, on the floor at theta = pi (where up is +x): the bottoms of its walls
    // (below the ground floor; chimneys start higher) face away from its middle.
    Settlements one;
    Settlement  farm;
    farm.kind  = SettlementKind::Farm;
    farm.plane = FloorPlane{.z0 = 0.0, .theta0 = kPi, .radius = 4000.0};
    one.places.push_back(farm);
    one.buildings.push_back({.halfSize = Vec2d(6.0, 4.0), .storeys = 2, .roof = RoofKind::Gable});
    const std::vector<SettlementMesh> meshes = buildSettlementMeshes(one);
    ASSERT_EQ(meshes.size(), 1U);
    const Vec3d base(-4000.0, 0.0, 0.0);
    const Vec3d middle = base + Vec3d(3.0, 0.0, 0.0);
    int         walls  = 0;
    for (const Vertex& v : meshes[0].mesh.vertices)
    {
        const Vec3d p = Vec3d(v.position) + meshes[0].origin;
        if ((v.material & 0xFU) == buildingMaterial::kWall && p.x < base.x)
        {
            EXPECT_GT(glm::dot(Vec3d(v.normal), p - middle), 0.0);
            ++walls;
        }
    }
    EXPECT_EQ(walls, 8);  // two bottom corners of each of the four walls
    expectConsistentWinding(meshes[0].mesh);
}

TEST(Buildings, EveryBuildingHasABodyAndPitchedRoofsAHull)
{
    const Settlements&    s         = village().settlements;
    const StaticColliders colliders = settlementColliders(s);
    const auto            pitched   = static_cast<std::size_t>(std::ranges::count_if(
        s.buildings, [](const Building& b) { return b.roof != RoofKind::Flat; }));
    EXPECT_GE(colliders.boxes.size(), s.buildings.size());
    EXPECT_GE(colliders.hulls.size(), pitched);
    for (const StaticBox& box : colliders.boxes)
    {
        EXPECT_GT(box.halfExtents.x, 0.0);
        EXPECT_GT(box.halfExtents.y, 0.0);
        EXPECT_GT(box.halfExtents.z, 0.0);
        EXPECT_NEAR(glm::length(box.orientation), 1.0, 1e-9);
    }
    for (const StaticHull& hull : colliders.hulls)
    {
        EXPECT_GE(hull.points.size(), 4U);
    }
}

TEST(Buildings, GroundAtlasHoldsEveryTownMap)
{
    const Settlements&     s     = village().settlements;
    const gpu::GroundAtlas atlas = gpu::packGroundAtlas(s);
    const std::size_t      towns = s.townCount();
    ASSERT_EQ(atlas.records.size(), 1 + (3 * towns));
    EXPECT_EQ(atlas.records[0].x, static_cast<float>(towns));
    EXPECT_EQ(atlas.pixels.size(), static_cast<std::size_t>(atlas.width) * atlas.height * 4);
    // The first texel of the tallest map sits at its slot, copied exactly.
    const Settlement* town = nullptr;
    for (const Settlement& place : s.places)
    {
        if (place.kind == SettlementKind::Town &&
            (town == nullptr || place.ground.height > town->ground.height))
        {
            town = &place;
        }
    }
    ASSERT_NE(town, nullptr);
    const Vec4f where = atlas.records[3];
    const auto x = static_cast<std::size_t>(std::lround(where.x * static_cast<float>(atlas.width)));
    const auto y =
        static_cast<std::size_t>(std::lround(where.y * static_cast<float>(atlas.height)));
    for (std::size_t c = 0; c < 4; ++c)
    {
        EXPECT_EQ(atlas.pixels[(((y * atlas.width) + x) * 4) + c], town->ground.texels[c]);
    }
    // Unpainted texels are far from any paving.
    EXPECT_EQ(atlas.pixels[0], 255);
    // No towns: a 1x1 atlas.
    EXPECT_EQ(gpu::packGroundAtlas(Settlements{}).pixels.size(), 4U);
}

TEST(Props, EveryKindHasAMeshAndAShape)
{
    for (std::size_t k = 0; k < kPropKindCount; ++k)
    {
        const auto      kind = static_cast<PropKind>(k);
        const PropInfo& info = propInfo(kind);
        EXPECT_FALSE(info.parts.empty()) << propKindName(kind);
        EXPECT_GT(info.massKg, 0.0F);
        const CpuMesh mesh = makePropMesh(kind);
        EXPECT_FALSE(mesh.indices.empty());
        expectConsistentWinding(mesh);
        // Standing on its origin: nothing below the ground.
        for (const Vertex& v : mesh.vertices)
        {
            EXPECT_GE(v.position.y, -1e-4F) << propKindName(kind);
        }
    }
}

}  // namespace
