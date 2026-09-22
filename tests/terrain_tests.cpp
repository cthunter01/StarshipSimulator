#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/TerrainLod.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/trees.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

const HabitatGeometry& islandThree()
{
    static const HabitatGeometry kGeometry{OneillCylinderSpec{}};
    return kGeometry;
}

OneillCylinderSpec playgroundSpec()
{
    OneillCylinderSpec spec;
    spec.radiusM                      = 250.0;
    spec.lengthM                      = 800.0;
    spec.windowFraction               = 0.4;
    spec.partner.enabled              = false;
    spec.terrain.hillHeightM          = 3.0;
    spec.terrain.mountainHeightM      = 6.0;
    spec.terrain.featureSizeM         = 60.0;
    spec.terrain.riverWidthM          = 0.0;
    spec.terrain.lakesPerValley       = 1;
    spec.terrain.lakeRadiusM          = 20.0;
    spec.sunwardEndcap                = makeEndcap(EndcapShape::Hemisphere);
    spec.antisunwardEndcap.hubRadiusM = 10.0;
    return spec;
}

// ---- Water and woods --------------------------------------------------------------------------

TEST(Landscape, RiversRunDownTheMiddleOfEachValley)
{
    const HabitatGeometry& geometry  = islandThree();
    const Landscape&       landscape = geometry.landscape();
    ASSERT_TRUE(landscape.hasRivers());
    const double landHalfWidth = geometry.landHalfAngle() * geometry.radius();
    for (int valley = 0; valley < geometry.stripCount(); ++valley)
    {
        const auto steps =
            static_cast<int>((geometry.floorZMax() - geometry.floorZMin() - 2000.0) / 250.0);
        for (int step = 0; step < steps; ++step)
        {
            const double z     = geometry.floorZMin() + 1000.0 + (250.0 * step);
            const double theta = landscape.riverAngle(valley, z);
            // Well clear of the windows, with room for the floodplain.
            const double offCentre =
                HabitatGeometry::angularDistance(theta, geometry.landCenter(valley)) *
                geometry.radius();
            EXPECT_LT(offCentre, landHalfWidth - 800.0) << z;
            // In the water, at wading depth, and the land rises away from it.
            EXPECT_LT(landscape.shoreDistance(z, theta), 0.0);
            EXPECT_NEAR(geometry.waterDepth(z, theta), kWaterDepthM, 0.05) << z;
            const bool   inLake = std::ranges::any_of(landscape.lakes(), [&](const Lake& lake) {
                return lake.valley == valley && std::abs(z - lake.z) < lake.halfLengthM + 100.0;
            });
            const double aside  = theta + (60.0 / geometry.radius());
            EXPECT_TRUE(inLake || geometry.terrainHeight(z, aside) > kWaterLevelM) << z;
            EXPECT_EQ(geometry.regionAt(z, theta).kind, RegionKind::Land);
        }
    }
}

TEST(Landscape, LakesFitInTheirValleys)
{
    const HabitatGeometry& geometry = islandThree();
    const auto&            lakes    = geometry.landscape().lakes();
    EXPECT_EQ(lakes.size(), 6U);  // two per valley
    for (const Lake& lake : lakes)
    {
        EXPECT_GT(lake.halfWidthM, 100.0);
        EXPECT_GT(lake.halfLengthM, lake.halfWidthM);
        EXPECT_GT(lake.z - lake.halfLengthM, geometry.floorZMin());
        EXPECT_LT(lake.z + lake.halfLengthM, geometry.floorZMax());
        EXPECT_GT(geometry.waterDepth(lake.z, lake.theta), 1.0);
        // The shore is the waterline, rising to dry land.
        const double across = (lake.halfWidthM * 1.5) / geometry.radius();
        EXPECT_EQ(geometry.waterDepth(lake.z, lake.theta + across), 0.0);
    }
}

TEST(Landscape, SmallHabitatsGetAPondAndNoRiver)
{
    const HabitatGeometry geometry(playgroundSpec());
    EXPECT_FALSE(geometry.landscape().hasRivers());
    EXPECT_EQ(geometry.landscape().lakes().size(), 3U);
}

TEST(Landscape, WoodsCoverAboutTheRequestedShareOfTheFloor)
{
    const HabitatGeometry& geometry = islandThree();
    SplitMix64             random(42);
    int                    wooded = 0;
    int                    land   = 0;
    for (int i = 0; i < 4000; ++i)
    {
        const double z     = random.uniform(geometry.floorZMin(), geometry.floorZMax());
        const double theta = random.uniform(0.0, 2.0 * kPi);
        if (geometry.regionAt(z, theta).kind != RegionKind::Land)
        {
            continue;
        }
        ++land;
        wooded += geometry.forestDensity(z, theta) > 0.5 ? 1 : 0;
        // Never in the water or on the walkways by the windows.
        if (geometry.waterDepth(z, theta) > 0.0)
        {
            EXPECT_EQ(geometry.forestDensity(z, theta), 0.0);
        }
    }
    // Somewhat less than the woodland share: water, meadows and walkways take some of it.
    const double share = static_cast<double>(wooded) / land;
    EXPECT_GT(share, 0.2);
    EXPECT_LT(share, 0.4);
    EXPECT_EQ(geometry.forestDensity(0.0, geometry.windowCenter(0)), 0.0);
}

TEST(Landscape, ShapeNearWaterIsContinuous)
{
    double previous = Landscape::shapeNearWater(10.0, -50.0);
    for (int step = 0; step < 1400; ++step)
    {
        const double shore = -50.0 + (0.25 * step);
        const double h     = Landscape::shapeNearWater(10.0, shore);
        EXPECT_LT(std::abs(h - previous), 0.2) << shore;
        previous = h;
    }
    EXPECT_DOUBLE_EQ(Landscape::shapeNearWater(10.0, 500.0), 10.0);
    EXPECT_NEAR(Landscape::shapeNearWater(10.0, 0.0), kWaterLevelM, 1e-12);
}

// ---- The grid -----------------------------------------------------------------------------------

TEST(TerrainGrid, LayoutTilesWithWholeQuadtreeRoots)
{
    const TerrainGridLayout layout = makeTerrainLayout(islandThree(), 4.0);
    EXPECT_EQ(layout.columns % layout.rootCells(), 0U);
    EXPECT_EQ(layout.cells % layout.rootCells(), 0U);
    EXPECT_GE(layout.columns / layout.rootCells(), 3U);
    EXPECT_NEAR(layout.cellArcM, 4.0, 1.0);
    EXPECT_NEAR(layout.cellU, 4.0, 1.0);
    EXPECT_NEAR(layout.profileLength(), islandThree().profile().length(), 1e-6);
    EXPECT_NEAR(layout.theta(layout.columns), 2.0 * kPi, 1e-12);
}

TEST(TerrainGrid, HeightsMatchTheTerrainModel)
{
    const HabitatGeometry geometry(playgroundSpec());
    const TerrainGrid     grid = sampleTerrain(geometry, 1.0);
    ASSERT_EQ(grid.heights.size(),
              static_cast<std::size_t>(grid.layout.rows()) * grid.layout.columns);
    ASSERT_EQ(grid.profile.size(), grid.layout.rows());
    SplitMix64 random(7);
    for (int i = 0; i < 200; ++i)
    {
        const auto   column = static_cast<std::uint32_t>(random.next() % grid.layout.columns);
        const auto   row    = static_cast<std::uint32_t>(random.next() % grid.layout.rows());
        const auto   z      = static_cast<double>(grid.profile[row].x);
        const double h      = geometry.terrainHeight(z, grid.layout.theta(column));
        // Hills are sampled at every other grid point and interpolated in between.
        const bool exact = column % 2 == 0 && row % 2 == 0;
        EXPECT_NEAR(grid.height(column, row), h, exact ? 0.01 : 0.2);
    }
    // The profile table follows the meridian profile.
    const Vec2d start = geometry.profile().pointAt(0.0);
    EXPECT_NEAR(grid.profile.front().x, start.x, 1e-3);
    EXPECT_NEAR(grid.profile.front().y, start.y, 1e-3);
    EXPECT_NEAR(grid.arcByZ.front(), 0.0F, 1e-3F);
    EXPECT_NEAR(grid.arcByZ.back(), static_cast<float>(geometry.profile().length()), 0.1F);
}

TEST(TerrainGrid, SamplingDoesNotDependOnTheThreadCount)
{
    const HabitatGeometry geometry(playgroundSpec());
    const TerrainGrid     one  = sampleTerrain(geometry, 2.0, 1);
    const TerrainGrid     many = sampleTerrain(geometry, 2.0, 5);
    EXPECT_EQ(one.heights, many.heights);
    EXPECT_EQ(one.cover, many.cover);
}

// ---- Level of detail ----------------------------------------------------------------------------

const TerrainGrid& coarseIslandThreeGrid()
{
    static const TerrainGrid kGrid = sampleTerrain(islandThree(), 16.0);  // coarse, for speed
    return kGrid;
}

struct Selection
{
    std::vector<TerrainPatch> patches;
    TerrainGridLayout         layout;
    Vec3d                     eye{0.0};
};

Selection selectFromValley(double pitchDeg)
{
    const TerrainGrid&      grid = coarseIslandThreeGrid();
    static const TerrainLod kLod(grid, islandThree(), 200.0);
    const HabitatGeometry&  geometry = islandThree();
    Camera                  camera;
    camera.position =
        geometry.surfacePoint(-2000.0, geometry.landCenter(1)) +
        (HabitatGeometry::localUp(geometry.surfacePoint(-2000.0, geometry.landCenter(1))) * 1.7);
    LookRig look(HabitatGeometry::localUp(camera.position), Vec3d(0.0, 0.0, 1.0));
    look.setAngles(0.0, degreesToRadians(pitchDeg));
    camera.orientation = look.orientation();
    const Frustum frustum(cameraRelativeViewProjection(camera, 16.0 / 9.0));
    return {.patches = kLod.select(camera.position, frustum),
            .layout  = grid.layout,
            .eye     = camera.position};
}

TEST(TerrainLod, PatchesNeverOverlap)
{
    for (const double pitch : {0.0, 60.0})
    {
        const Selection selection = selectFromValley(pitch);
        ASSERT_FALSE(selection.patches.empty());
        const auto& patches = selection.patches;
        for (std::size_t i = 0; i < patches.size(); ++i)
        {
            const std::uint32_t sizeI = selection.layout.nodeCells(patches[i].level);
            for (std::size_t j = i + 1; j < patches.size(); ++j)
            {
                const std::uint32_t sizeJ  = selection.layout.nodeCells(patches[j].level);
                const bool          apartX = patches[i].column + sizeI <= patches[j].column ||
                                             patches[j].column + sizeJ <= patches[i].column;
                const bool          apartY = patches[i].row + sizeI <= patches[j].row ||
                                             patches[j].row + sizeJ <= patches[i].row;
                EXPECT_TRUE(apartX || apartY) << i << " overlaps " << j;
            }
        }
    }
}

TEST(TerrainLod, FinestNearTheCameraCoarserFarAway)
{
    const Selection selection = selectFromValley(-10.0);
    const auto&     layout    = selection.layout;
    const double    eyeTheta  = HabitatGeometry::angleOf(selection.eye);
    const double    eyeRow    = islandThree().profile().arcAt(selection.eye.z) / layout.cellU;
    const double    eyeColumn = eyeTheta / (2.0 * kPi) * layout.columns;
    std::uint32_t   coarsest  = 0;
    bool            underEye  = false;
    for (const TerrainPatch& patch : selection.patches)
    {
        const double size = layout.nodeCells(patch.level);
        if (eyeColumn >= patch.column && eyeColumn < patch.column + size && eyeRow >= patch.row &&
            eyeRow < patch.row + size)
        {
            underEye = true;
            EXPECT_EQ(patch.level, 0U);
        }
        coarsest = std::max(coarsest, patch.level);
    }
    EXPECT_TRUE(underEye);
    EXPECT_GE(coarsest, 3U);
    EXPECT_LT(selection.patches.size(), 3000U);
}

TEST(TerrainLod, MorphBandsGrowWithDistance)
{
    const TerrainGrid& grid = coarseIslandThreeGrid();
    const TerrainLod   lod(grid, islandThree(), 200.0);
    const auto&        morphs = lod.morphs();
    ASSERT_EQ(morphs.size(), grid.layout.rootLevel + 1);
    for (std::size_t level = 1; level + 1 < morphs.size(); ++level)
    {
        EXPECT_LT(morphs[level].start, morphs[level].end);
        EXPECT_NEAR(morphs[level].end, 2.0 * morphs[level - 1].end, 1e-9);
        EXPECT_GT(morphs[level].start, morphs[level - 1].end);
    }
}

// ---- Trees
// ----------------------------------------------------------------------------------------

TEST(Trees, MeshesFaceOutwardAndStandOneUnitTall)
{
    for (const TreeSpecies species :
         {TreeSpecies::Broadleaf, TreeSpecies::Conifer, TreeSpecies::Poplar})
    {
        std::size_t triangles = 0;
        for (const bool detailed : {false, true})
        {
            const CpuMesh mesh = makeTreeMesh(species, detailed);
            ASSERT_FALSE(mesh.indices.empty());
            float top = 0.0F;
            for (const Vertex& v : mesh.vertices)
            {
                top = std::max(top, v.position.y);
                EXPECT_GE(v.position.y, 0.0F);
            }
            EXPECT_GT(top, 0.8F);
            EXPECT_LE(top, 1.1F);
            for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
            {
                const Vertex& a = mesh.vertices[mesh.indices[i]];
                const Vertex& b = mesh.vertices[mesh.indices[i + 1]];
                const Vertex& c = mesh.vertices[mesh.indices[i + 2]];
                const Vec3f   n = glm::cross(b.position - a.position, c.position - a.position);
                if (glm::length(n) > 1e-9F)
                {
                    EXPECT_GT(glm::dot(n, a.normal + b.normal + c.normal), 0.0F);
                }
            }
            // The distant version is much cheaper.
            if (!detailed)
            {
                triangles = mesh.indices.size() / 3;
                EXPECT_LT(triangles, 40U);
            }
            else
            {
                EXPECT_GT(mesh.indices.size() / 3, triangles);
            }
        }
    }
}

TEST(Trees, PackingKeepsHeightAndSpecies)
{
    const std::uint32_t packed = packTree(18.3, 0.5, TreeSpecies::Poplar, 1.0);
    EXPECT_EQ(packed & 0xFFFU, 183U);
    EXPECT_EQ((packed >> 20U) & 0xFU, static_cast<std::uint32_t>(TreeSpecies::Poplar));
    EXPECT_EQ(packed >> 24U, 255U);
}

TEST(Trees, WoodsRiverbanksAndLoneTrees)
{
    const HabitatGeometry& geometry = islandThree();
    const TerrainGrid&     grid     = coarseIslandThreeGrid();
    const TreeLayer        layer =
        plantTrees(geometry, grid,
                   TreeSettings{.spacingM = 20.0, .threads = 0, .clearings = {}, .keepOff = {}});
    ASSERT_FALSE(layer.tiles.empty());
    std::array<std::size_t, kTreeSpeciesCount> bySpecies{};
    for (const TreeTile& tile : layer.tiles)
    {
        std::uint32_t index = tile.first;
        for (std::size_t s = 0; s < kTreeSpeciesCount; ++s)
        {
            bySpecies.at(s) += tile.counts.at(s);
            for (std::uint32_t k = 0; k < tile.counts.at(s); ++k, ++index)
            {
                const TreeInstance& tree  = layer.instances[index];
                const Vec3d         world = tile.origin + Vec3d(tree.position);
                const double        theta = HabitatGeometry::angleOf(world);
                // On dry land, never on the window glass, inside the tile's bounds.
                EXPECT_NE(geometry.regionAt(world.z, theta).kind, RegionKind::Window);
                EXPECT_LT(geometry.waterDepth(world.z, theta), 0.3);
                EXPECT_TRUE(glm::all(glm::greaterThanEqual(tree.position, tile.boundsMin)));
                EXPECT_TRUE(glm::all(glm::lessThanEqual(tree.position, tile.boundsMax)));
                EXPECT_EQ((tree.packed >> 20U) & 0xFU, s);
            }
        }
    }
    // Hundreds of thousands at full density; here a sparse sample of every kind.
    EXPECT_GT(bySpecies[0], 1000U);
    EXPECT_GT(bySpecies[1], 1000U);
    EXPECT_GT(bySpecies[2], 20U);
}

TEST(Trees, PlantingDoesNotDependOnTheThreadCount)
{
    const HabitatGeometry geometry(playgroundSpec());
    const TerrainGrid     grid = sampleTerrain(geometry, 2.0);
    const TreeLayer       one =
        plantTrees(geometry, grid,
                   TreeSettings{.spacingM = 4.0, .threads = 1, .clearings = {}, .keepOff = {}});
    const TreeLayer many =
        plantTrees(geometry, grid,
                   TreeSettings{.spacingM = 4.0, .threads = 6, .clearings = {}, .keepOff = {}});
    ASSERT_EQ(one.instances.size(), many.instances.size());
    for (std::size_t i = 0; i < one.instances.size(); ++i)
    {
        EXPECT_EQ(one.instances[i].packed, many.instances[i].packed);
        EXPECT_EQ(one.instances[i].position, many.instances[i].position);
    }
}

}  // namespace
