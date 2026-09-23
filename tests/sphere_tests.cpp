#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/habitat/Enclosure.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/habitat/weather.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/core/units.h"
#include "StarshipSimulator/core/utf8_path.h"

// Island One, a Bernal sphere: a band of land round the equator of a sphere 500 m across, the
// ground climbing toward the poles on either hand, and glass windows round the poles.
namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

constexpr double kRadius = 250.0;

HabitatSpec islandOne()
{
    const auto scenario =
        loadScenario(pathFromUtf8(STARSHIPSIMULATOR_DATA_DIR) / "presets" / "island_one.toml");
    EXPECT_TRUE(scenario.has_value()) << scenario.error().describe();
    return scenario ? scenario->habitat : HabitatSpec{};
}

/// The preset's world, generated once for all the tests.
struct World
{
    HabitatGeometry       geometry{islandOne()};
    TerrainGrid           grid = sampleTerrain(geometry, 1.0);
    std::vector<TramLine> lines;
    Settlements           settlements;

    World()
    {
        lines = planTramLines(geometry, grid);
        gradeForTrack(grid, lines);
        settlements = planSettlements(geometry, grid);
        addTramStops(lines, settlements);
    }

    /// The ground's distance from the axis at a point, as graded and drawn.
    [[nodiscard]] double groundRadius(const Vec3d& p) const
    {
        const double theta = HabitatGeometry::angleOf(p);
        return geometry.floorRadiusAt(p.z) - grid.groundHeight(p.z, theta);
    }
};

const World& world()
{
    static const World kWorld;
    return kWorld;
}

/// The same sphere with bare ground: no hills, no mountains, no water.
const HabitatGeometry& bareSphere()
{
    static const HabitatGeometry kGeometry = [] {
        HabitatSpec spec             = islandOne();
        spec.terrain.hillHeightM     = 0.0;
        spec.terrain.mountainHeightM = 0.0;
        spec.terrain.riverWidthM     = 0.0;
        spec.terrain.lakesPerValley  = 0;
        return HabitatGeometry(spec);
    }();
    return kGeometry;
}

double latitudeZ(double degrees)
{
    return kRadius * std::sin(degreesToRadians(degrees));
}

TEST(Sphere, TheProfileEndsAtTheWindowRim)
{
    const HabitatGeometry& geometry = world().geometry;
    const MeridianProfile& profile  = geometry.profile();
    EXPECT_EQ(geometry.kind(), HabitatKind::BERNAL_SPHERE);
    EXPECT_NEAR(profile.radiusAt(0.0).value_or(0.0), kRadius, 1e-9);
    EXPECT_NEAR(profile.zMax(), latitudeZ(55.0), 1e-6);
    EXPECT_NEAR(profile.zMin(), -latitudeZ(55.0), 1e-6);
    EXPECT_NEAR(profile.radiusAt(profile.zMax()).value_or(0.0),
                kRadius * std::cos(degreesToRadians(55.0)), 1e-6);
    for (const double z : {10.0, 80.0, 150.0, 200.0})
    {
        EXPECT_NEAR(profile.arcAt(z) - profile.arcAt(0.0), profile.arcAt(0.0) - profile.arcAt(-z),
                    1e-9);
    }
    // The equator to a rim: 55 degrees of arc.
    EXPECT_NEAR(profile.length(), 2.0 * kRadius * degreesToRadians(55.0), 0.05);
    EXPECT_NEAR(geometry.walkableZMax(), profile.zMax() - 1.0, 1e-9);
    EXPECT_NEAR(geometry.walkableZMin(), profile.zMin() + 1.0, 1e-9);
    // The land reaches 35 degrees either side of the equator.
    EXPECT_NEAR(geometry.floorZMax(), latitudeZ(35.0), 1e-9);
    EXPECT_NEAR(geometry.floorZMin(), -latitudeZ(35.0), 1e-9);
}

TEST(Sphere, RegionsGoByLatitude)
{
    const HabitatGeometry& geometry = world().geometry;
    for (const double theta : {0.0, 2.0, 4.0})
    {
        for (const double z : {0.0, 140.0, -140.0})
        {
            EXPECT_EQ(geometry.regionAt(z, theta).kind, RegionKind::LAND) << z;
            EXPECT_TRUE(geometry.onLand(z, theta));
        }
        for (const double z : {150.0, -150.0, 200.0, -200.0})
        {
            EXPECT_EQ(geometry.regionAt(z, theta).kind, RegionKind::ENDCAP) << z;
            EXPECT_FALSE(geometry.onLand(z, theta));
        }
        for (const double z : {210.0, -210.0})
        {
            EXPECT_EQ(geometry.regionAt(z, theta).kind, RegionKind::OUTSIDE) << z;
        }
    }
    // One band of land, all the way round, 35 degrees either side of the equator.
    ASSERT_EQ(geometry.bandCount(), 1);
    const LandBand& band = geometry.band(0);
    EXPECT_EQ(band.axis, BandAxis::AROUND);
    EXPECT_TRUE(band.wraps);
    EXPECT_NEAR(band.radiusM, kRadius, 1e-9);
    EXPECT_NEAR(band.halfWidthM, kRadius * degreesToRadians(35.0), 0.05);
}

TEST(Sphere, TheGroundSlopesAtTheLatitude)
{
    const HabitatGeometry& geometry = bareSphere();
    for (const double degrees : {10.0, 25.0, 40.0})
    {
        for (const double theta : {0.3, 3.3})
        {
            const double       z      = latitudeZ(degrees);
            const GroundSample sample = geometry.ground(geometry.surfacePoint(z, theta));
            EXPECT_NEAR(radiansToDegrees(sample.slopeRadians), degrees, 1.0) << z;
        }
    }
    // Spin gravity falls with the distance from the axis: g cos(latitude) up the slope.
    const double edge = geometry.floorRadiusAt(latitudeZ(35.0));
    EXPECT_NEAR(geometry.gravityAt(edge) / units::kStandardGravity,
                std::cos(degreesToRadians(35.0)), 1e-3);
}

TEST(Sphere, TheEnclosureIncludesTheCaps)
{
    const Enclosure& air = world().geometry.enclosure();
    EXPECT_TRUE(air.contains(Vec3d(0.0, 0.0, 230.0)));
    EXPECT_TRUE(air.contains(Vec3d(0.0, 0.0, -230.0)));
    EXPECT_FALSE(air.contains(Vec3d(0.0, 0.0, 260.0)));
    EXPECT_TRUE(air.contains(Vec3d(249.0, 0.0, 0.0)));
    EXPECT_FALSE(air.contains(Vec3d(251.0, 0.0, 0.0)));
    EXPECT_FALSE(air.contains(Vec3d(180.0, 0.0, 180.0)));  // 254.6 m out
}

TEST(Sphere, ThereIsGlassAtBothPoles)
{
    const HabitatGeometry& geometry = world().geometry;
    const double           rimZ     = geometry.profile().zMax();
    const HabitatMeshes    meshes =
        buildHabitatMeshes(geometry, MeshingSettings{.cellSizeM      = 5.0,
                                                     .chunkSizeM     = 200.0,
                                                     .glassCellSizeM = 100.0,
                                                     .threads        = 0,
                                                     .terrain        = false});
    std::size_t glass   = 0;
    double      lowest  = 0.0;
    double      highest = 0.0;
    for (const MeshChunk& chunk : meshes.chunks)
    {
        if (chunk.kind != ChunkKind::GLASS)
        {
            continue;
        }
        ++glass;
        for (const Vertex& vertex : chunk.mesh.vertices)
        {
            const Vec3d p = chunk.origin + Vec3d(vertex.position);
            EXPECT_NEAR(glm::length(p), kRadius, 0.05);
            EXPECT_GE(std::abs(p.z), rimZ - 0.05);
            EXPECT_EQ(vertex.material, material::kGlass);
            // Glass faces into the sphere.
            EXPECT_GT(glm::dot(Vec3d(vertex.normal), -p / kRadius), 0.999);
            lowest  = std::min(lowest, p.z);
            highest = std::max(highest, p.z);
        }
        const auto& vertices = chunk.mesh.vertices;
        for (std::size_t i = 0; i + 2 < chunk.mesh.indices.size(); i += 3)
        {
            const Vertex& a = vertices[chunk.mesh.indices[i]];
            const Vertex& b = vertices[chunk.mesh.indices[i + 1]];
            const Vertex& c = vertices[chunk.mesh.indices[i + 2]];
            const Vec3d   n =
                glm::cross(Vec3d(b.position - a.position), Vec3d(c.position - a.position));
            if (glm::length(n) > 1e-2)  // skip slivers at the pole
            {
                EXPECT_GT(glm::dot(n, Vec3d(a.normal)), 0.0) << "triangles face into the sphere";
            }
        }
    }
    EXPECT_EQ(glass, 2U);
    EXPECT_NEAR(lowest, -kRadius, 0.05);
    EXPECT_NEAR(highest, kRadius, 0.05);
}

TEST(Sphere, TheSphereMeasuresRight)
{
    const HabitatMetrics m = computeMetrics(world().geometry.spec());
    EXPECT_NEAR(m.periodS, 31.7, 0.1);
    EXPECT_NEAR(m.rimSpeed, 49.5, 0.1);
    EXPECT_NEAR(m.coriolisWalkingRatio, 0.0566, 5e-4);
    EXPECT_NEAR(m.landAreaM2 / 1e6, 0.4505, 1e-3);
    EXPECT_NEAR(m.windowAreaM2 / 1e4, 14.20, 0.01);  // two caps of 7.1 hectares
    EXPECT_NEAR(m.population, 10000.0, 100.0);
}

// ---- Water ------------------------------------------------------------------------------------

TEST(Sphere, WaterLiesAtOneRadius)
{
    const HabitatGeometry& geometry = world().geometry;
    const double           reach    = geometry.waterReachAcrossM().value_or(0.0);
    EXPECT_NEAR(reach, 35.4, 0.1);
    for (int i = 0; i <= 40; ++i)
    {
        const double z = -35.0 + (1.75 * i);
        EXPECT_NEAR(geometry.waterLevelAt(z) - geometry.waterLevelAt(0.0),
                    geometry.floorRadiusAt(z) - kRadius, 1e-9);
    }
    EXPECT_DOUBLE_EQ(geometry.waterLevelAt(0.0), kWaterLevelM);

    // On the grid too: the water's surface is the same distance from the axis on every row.
    const TerrainGrid& grid = world().grid;
    ASSERT_GT(grid.waterDatumRadius, 0.0);
    for (std::uint32_t row = 0; row < grid.layout.rows(); row += 97)
    {
        EXPECT_NEAR(static_cast<double>(grid.profile[row].y) - grid.waterLevelAtRow(row),
                    kRadius - kWaterLevelM, 1e-3);
    }
    // Nothing is cut deeper than the grid can hold (a bed clamped flat would be a wedge).
    double lowest = 1e9;
    for (std::uint32_t row = 0; row < grid.layout.rows(); ++row)
    {
        for (std::uint32_t column = 0; column < grid.layout.columns; column += 7)
        {
            lowest = std::min(lowest, grid.height(column, row));
        }
    }
    EXPECT_GT(lowest, static_cast<double>(grid.heightMin) + 0.05);
}

TEST(Sphere, TheWaterStaysInTheBottomOfTheBand)
{
    const HabitatGeometry& geometry  = world().geometry;
    const Landscape&       landscape = geometry.landscape();
    const LandBand&        band      = geometry.band(0);
    const double           reach     = geometry.waterReachAcrossM().value_or(0.0);
    const double           halfRiver = 0.5 * geometry.spec().terrain.riverWidthM;
    ASSERT_TRUE(landscape.hasRivers());
    EXPECT_FALSE(landscape.lakes().empty());
    for (const Lake& lake : landscape.lakes())
    {
        EXPECT_LE(std::abs(lake.plan.x) + lake.halfWidthM, reach + 1e-9);
    }
    const TerrainGrid& grid        = world().grid;
    const double       waterRadius = kRadius - kWaterLevelM;
    for (int step = 0; step * 10.0 < band.alongLengthM(); ++step)
    {
        const double along = band.alongMinM + (step * 10.0);
        const double river = landscape.riverAcross(along).x;
        EXPECT_LE(std::abs(river) + halfRiver, reach + 1e-9);
        // Under water in the middle of the river, and dry ground a little way from either bank:
        // the water does not spill down the slope, nor climb it.
        const SurfaceSpot middle = band.toSurface(Vec2d(river, along));
        EXPECT_GT(geometry.floorRadiusAt(middle.z) - grid.groundHeight(middle.z, middle.theta),
                  waterRadius)
            << "at " << along;
        for (const double side : {-1.0, 1.0})
        {
            const SurfaceSpot bank =
                band.toSurface(Vec2d(river + (side * (halfRiver + 15.0)), along));
            if (landscape.shoreDistance(bank.z, bank.theta) > 5.0)  // not by a lake
            {
                EXPECT_LT(geometry.floorRadiusAt(bank.z) - grid.groundHeight(bank.z, bank.theta),
                          waterRadius)
                    << "at " << along;
            }
        }
    }
}

TEST(Sphere, TheGpuDrawsTheWaterAtOneRadius)
{
    const gpu::LandscapeUniforms sphere = gpu::makeLandscapeUniforms(world().grid, {});
    EXPECT_EQ(sphere.extent.z, 1.0F);
    EXPECT_EQ(sphere.extent.w, 250.0F);
    EXPECT_EQ(sphere.heights.z, static_cast<float>(kWaterLevelM));

    HabitatSpec cylinder;
    cylinder.radiusM                    = 400.0;
    cylinder.lengthM                    = 3000.0;
    const HabitatGeometry        level  = HabitatGeometry(cylinder);
    const TerrainGrid            flat   = sampleTerrain(level, 6.0);
    const gpu::LandscapeUniforms oneill = gpu::makeLandscapeUniforms(flat, {});
    EXPECT_EQ(oneill.extent.z, 0.0F);
    EXPECT_EQ(oneill.extent.w, 0.0F);
    EXPECT_EQ(flat.waterLevelAtRow(flat.layout.rows() / 2), kWaterLevelM);
}

// ---- Daylight ---------------------------------------------------------------------------------

TEST(Sphere, TheApertureMatchesABruteForceMarch)
{
    const HabitatGeometry& geometry = world().geometry;
    const double           rimZ     = geometry.profile().zMax();
    SplitMix64             random(2024);
    int                    lit  = 0;
    int                    dark = 0;
    for (const double angle : {45.0, 70.0, 88.0})
    {
        for (const SunBeam& beam : sunBeams(geometry, degreesToRadians(angle)))
        {
            for (int i = 0; i < 300; ++i)
            {
                const double z      = random.uniform(-rimZ, rimZ);
                const double theta  = random.uniform(0.0, 2.0 * kPi);
                const double inward = random.uniform(1.0, 60.0);
                const double r      = geometry.floorRadiusAt(z) - inward;
                const Vec3d  p(r * std::cos(theta), r * std::sin(theta), z);
                // March the ray until it leaves the sphere, and see where.
                const Vec3d d = towardSunFrom(beam, p);
                Vec3d       q = p;
                while (glm::length(q) < kRadius)
                {
                    q += d * 0.1;
                }
                const double toward = beam.image.value_or(Vec3d(0.0)).z > 0.0 ? q.z : -q.z;
                const double reach  = beamReach(geometry, p, beam);
                if (toward > rimZ + 2.5)
                {
                    EXPECT_GT(reach, 0.99) << p.x << ", " << p.y << ", " << p.z;
                    ++lit;
                }
                else if (toward < rimZ - 2.5)
                {
                    EXPECT_LT(reach, 0.01) << p.x << ", " << p.y << ", " << p.z;
                    ++dark;
                }
            }
        }
    }
    EXPECT_GT(lit, 200);
    EXPECT_GT(dark, 40);  // most places see both windows
}

TEST(Sphere, TheGpuSeesTheSphereAndItsWindows)
{
    const gpu::HabitatUniforms habitat =
        gpu::makeHabitatUniforms(world().geometry, degreesToRadians(45.0), gpu::LightingSettings{},
                                 Weather{}, gpu::CloudSettings{});
    EXPECT_EQ(habitat.light.x, 1.0F);  // points
    EXPECT_EQ(habitat.light.y, 2.0F);
    EXPECT_NEAR(habitat.light.z, kRadius * std::cos(degreesToRadians(55.0)), 1e-3);
    EXPECT_EQ(habitat.light.w, 250.0F);
    EXPECT_EQ(habitat.strips.y, 0.0F);  // no window strips along the hull
    EXPECT_NEAR(habitat.strips.w, latitudeZ(55.0), 1e-3);
    EXPECT_EQ(habitat.band.x, 1.0F);  // the land runs round
}

// ---- Villages and transit ---------------------------------------------------------------------

TEST(Sphere, VillagesStandOnTheSlopeWithTheirFeetOnTheGround)
{
    const World&       w = world();
    const Settlements& s = w.settlements;
    EXPECT_GE(s.townCount(), 2U);
    EXPECT_GE(s.places.size() - s.townCount(), 2U) << "farmsteads";
    ASSERT_FALSE(s.buildings.empty());
    for (const Building& b : s.buildings)
    {
        const Settlement& place = s.places[b.settlement];
        EXPECT_EQ(place.plane.axis, BandAxis::AROUND);
        const SurfaceSpot spot = place.plane.surface(b.centre);
        EXPECT_TRUE(w.geometry.onLand(spot.z, spot.theta)) << "none on the polar slopes";
        EXPECT_GT(w.geometry.landscape().shoreDistance(spot.z, spot.theta), 4.0);
        // Every wall reaches down into the ground, even on the downhill side of the slope.
        const Vec3d base  = place.plane.point(b.centre, b.floorHeight);
        const Quatd frame = floorOrientation(base, -b.angle, place.plane.alongDirection(b.centre));
        for (const double sx : {-1.0, 1.0})
        {
            for (const double sz : {-1.0, 1.0})
            {
                const Vec3d foot =
                    base + (frame * Vec3d(sx * b.halfSize.x, -b.foundation, sz * b.halfSize.y));
                EXPECT_GE(std::hypot(foot.x, foot.y), w.groundRadius(foot) - 0.5)
                    << "a wall ends in the air at z " << foot.z;
                // And the ground floor is not buried either.
                const Vec3d floor =
                    base + (frame * Vec3d(sx * b.halfSize.x, 0.0, sz * b.halfSize.y));
                EXPECT_LE(std::hypot(floor.x, floor.y), w.groundRadius(floor) + 0.5)
                    << "a floor under the ground at z " << floor.z;
            }
        }
    }
}

TEST(Sphere, TheFunicularClimbsToTheWindow)
{
    const World&           w         = world();
    const HabitatGeometry& geometry  = w.geometry;
    std::size_t            loops     = 0;
    const TramLine*        funicular = nullptr;
    for (const TramLine& line : w.lines)
    {
        loops += line.kind == LineKind::LOOP ? 1U : 0U;
        if (line.kind == LineKind::ENDCAP)
        {
            EXPECT_EQ(funicular, nullptr) << "one funicular";
            funicular = &line;
        }
    }
    EXPECT_EQ(loops, 1U);
    ASSERT_NE(funicular, nullptr);
    const TrackPoint& foot = funicular->track.front();
    const TrackPoint& top  = funicular->track.back();
    EXPECT_NEAR(foot.position.z, geometry.floorZMin() + 15.0, 1.0) << "from the edge of the land";
    EXPECT_LT(top.position.z - geometry.profile().zMin(), 5.0) << "up to the window's rim";
    EXPECT_GT(std::hypot(top.position.x, top.position.y), 70.0);
    ASSERT_GE(funicular->stops.size(), 2U);
    EXPECT_EQ(funicular->stops.front().name, "the fields");
    EXPECT_EQ(funicular->stops.back().name, "the window");
    for (std::size_t i = 1; i < funicular->stops.size(); ++i)
    {
        EXPECT_LT(funicular->stops[i - 1].alongM, funicular->stops[i].alongM);
    }
    // Clear of the villages: it runs up between two of them.
    for (const TrackPoint& point : funicular->track)
    {
        EXPECT_EQ(w.settlements.townAt(point.position.z, HabitatGeometry::angleOf(point.position)),
                  nullptr);
    }
}

TEST(Sphere, TheLoopsShelfIsLevelAcross)
{
    const World&           w        = world();
    const HabitatGeometry& geometry = w.geometry;
    const TramLine*        loop     = nullptr;
    for (const TramLine& line : w.lines)
    {
        loop = line.kind == LineKind::LOOP ? &line : loop;
    }
    ASSERT_NE(loop, nullptr);
    EXPECT_NEAR(loop->lengthM, 2.0 * kPi * loop->radiusM, 0.02 * loop->lengthM);
    int checked = 0;
    for (std::size_t i = 0; i < loop->track.size(); i += loop->track.size() / 20)
    {
        const TrackPoint& point = loop->track[i];
        if (point.carried)
        {
            continue;
        }
        const double theta = HabitatGeometry::angleOf(point.position);
        const double u     = geometry.profile().arcAt(point.position.z);
        const double below = geometry.profile().pointAt(u - 2.0).x;
        const double above = geometry.profile().pointAt(u + 2.0).x;
        const double lower = geometry.floorRadiusAt(below) - w.grid.groundHeight(below, theta);
        const double upper = geometry.floorRadiusAt(above) - w.grid.groundHeight(above, theta);
        EXPECT_NEAR(lower, upper, 0.1) << "at " << point.alongM << " m round";
        // The rails lie on it.
        EXPECT_NEAR(point.railHeightM - 0.28, w.grid.groundHeight(point.position.z, theta), 0.35);
        ++checked;
    }
    EXPECT_GE(checked, 15);
}

}  // namespace
