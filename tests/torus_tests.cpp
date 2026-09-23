#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/habitat/Enclosure.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/habitat/weather.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/PlayerController.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/enclosure_mesh.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/core/units.h"
#include "StarshipSimulator/core/utf8_path.h"
#include "StarshipSimulator/physics/PhysicsWorld.h"

// The Stanford torus: a tube 130 m across bent into a wheel 1.8 km across. The land runs round the
// bottom of the tube and climbs its sides; the ceiling facing the hub has the windows, and six
// spokes lead up to the hub.
namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity
namespace gpu = StarshipSimulator::gpu;

constexpr double kFloor  = 895.0;
constexpr double kTube   = 65.0;
constexpr double kCentre = kFloor - kTube;

HabitatSpec stanfordTorus()
{
    const auto scenario =
        loadScenario(pathFromUtf8(STARSHIPSIMULATOR_DATA_DIR) / "presets" / "stanford_torus.toml");
    EXPECT_TRUE(scenario.has_value()) << scenario.error().describe();
    return scenario ? scenario->habitat : HabitatSpec{};
}

/// The preset's world, generated once for all the tests.
struct World
{
    HabitatGeometry       geometry{stanfordTorus()};
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
};

const World& world()
{
    static const World kWorld;
    return kWorld;
}

/// The same torus with bare ground: no hills, no water.
const HabitatGeometry& bareTorus()
{
    static const HabitatGeometry kGeometry = [] {
        HabitatSpec spec             = stanfordTorus();
        spec.terrain.hillHeightM     = 0.0;
        spec.terrain.mountainHeightM = 0.0;
        spec.terrain.riverWidthM     = 0.0;
        spec.terrain.lakesPerValley  = 0;
        return HabitatGeometry(spec);
    }();
    return kGeometry;
}

/// z of the floor at a tube angle (degrees from its lowest line).
double tubeZ(double degrees)
{
    return kTube * std::sin(degreesToRadians(degrees));
}

TEST(Torus, TheFloorIsTheTubesOuterHalf)
{
    const HabitatGeometry& geometry = world().geometry;
    const MeridianProfile& profile  = geometry.profile();
    EXPECT_EQ(geometry.kind(), HabitatKind::STANFORD_TORUS);
    EXPECT_NEAR(profile.radiusAt(0.0).value_or(0.0), kFloor, 1e-9);
    EXPECT_NEAR(profile.zMax(), kTube, 1e-9);
    EXPECT_NEAR(profile.zMin(), -kTube, 1e-9);
    EXPECT_NEAR(profile.radiusAt(profile.zMax()).value_or(0.0), kCentre, 1e-6);
    for (const double z : {10.0, 40.0, 60.0})
    {
        EXPECT_NEAR(profile.arcAt(z) - profile.arcAt(0.0), profile.arcAt(0.0) - profile.arcAt(-z),
                    1e-9);
    }
    EXPECT_NEAR(profile.length(), kPi * kTube, 0.05);
    // The land reaches 40 degrees up either side.
    EXPECT_NEAR(geometry.floorZMax(), tubeZ(40.0), 1e-9);
    EXPECT_NEAR(geometry.floorZMin(), -tubeZ(40.0), 1e-9);
    EXPECT_NEAR(geometry.spanAcrossM(), 2.0 * kTube, 1e-9);
}

TEST(Torus, RegionsGoAcrossTheTube)
{
    const HabitatGeometry& geometry = world().geometry;
    for (const double theta : {0.0, 2.0, 4.0})
    {
        for (const double z : {0.0, 40.0, -40.0})
        {
            EXPECT_EQ(geometry.regionAt(z, theta).kind, RegionKind::LAND) << z;
        }
        for (const double z : {45.0, -45.0, 64.0, -64.0})
        {
            EXPECT_EQ(geometry.regionAt(z, theta).kind, RegionKind::ENDCAP) << z;
        }
        for (const double z : {66.0, -66.0})
        {
            EXPECT_EQ(geometry.regionAt(z, theta).kind, RegionKind::OUTSIDE) << z;
        }
    }
    ASSERT_EQ(geometry.bandCount(), 1);
    const LandBand& band = geometry.band(0);
    EXPECT_EQ(band.axis, BandAxis::AROUND);
    EXPECT_TRUE(band.wraps);
    EXPECT_NEAR(band.radiusM, kFloor, 1e-9);
    EXPECT_NEAR(band.halfWidthM, kTube * degreesToRadians(40.0), 0.05);
    EXPECT_NEAR(band.alongLengthM(), 2.0 * kPi * kFloor, 1e-6);
}

TEST(Torus, TheGroundClimbsTheTubesSides)
{
    const HabitatGeometry& geometry = bareTorus();
    for (const double degrees : {10.0, 25.0, 35.0, -30.0})
    {
        for (const double theta : {0.3, 3.3})
        {
            const GroundSample sample =
                geometry.ground(geometry.surfacePoint(tubeZ(degrees), theta));
            EXPECT_NEAR(radiansToDegrees(sample.slopeRadians), std::abs(degrees), 1.0) << degrees;
        }
    }
    // Up at the land's edge the floor is 15 m nearer the axis, and gravity a little weaker.
    const double edge = geometry.floorRadiusAt(tubeZ(40.0));
    EXPECT_NEAR(kFloor - edge, kTube * (1.0 - std::cos(degreesToRadians(40.0))), 0.01);
    EXPECT_NEAR(geometry.gravityAt(edge) / units::kStandardGravity, edge / kFloor, 1e-3);
}

TEST(Torus, MeasuresUpToTheStudy)
{
    const HabitatMetrics m = computeMetrics(world().geometry.spec());
    EXPECT_NEAR(m.periodS, 60.0, 0.1);
    EXPECT_NEAR(m.rimSpeed, 93.7, 0.1);
    EXPECT_NEAR(m.coriolisWalkingRatio, 0.0299, 5e-4);
    EXPECT_NEAR(m.landAreaM2 / 1e6, 0.5075, 2e-3);
    EXPECT_NEAR(m.population, 10000.0, 100.0);
}

TEST(Torus, TheAxisPointsAtTheEclipticsPole)
{
    EXPECT_FALSE(axisPointsAtSun(HabitatKind::STANFORD_TORUS));
    EXPECT_EQ(daylightKindFor(HabitatKind::STANFORD_TORUS), DaylightKind::OVERHEAD_MIRROR);
    EXPECT_TRUE(habitatKindBuilt(HabitatKind::STANFORD_TORUS));
}

TEST(Torus, TheCeilingHasItsWindowsAndTheHullFacesOut)
{
    const HabitatGeometry& geometry = world().geometry;
    const TorusShape       torus    = geometry.enclosure().torus().value_or(TorusShape{});
    const HabitatMeshes    meshes =
        buildHabitatMeshes(geometry, MeshingSettings{.cellSizeM      = 5.6,
                                                     .chunkSizeM     = 224.0,
                                                     .glassCellSizeM = 112.0,
                                                     .threads        = 0,
                                                     .terrain        = false});
    std::size_t glass = 0;
    std::size_t hull  = 0;
    std::size_t metal = 0;
    for (const MeshChunk& chunk : meshes.chunks)
    {
        ASSERT_FALSE(chunk.mesh.vertices.empty());
        for (const Vertex& vertex : chunk.mesh.vertices)
        {
            const Vec3d p = chunk.origin + Vec3d(vertex.position);
            ASSERT_FALSE(std::isnan(p.x) || std::isnan(vertex.normal.x));
            EXPECT_NEAR(glm::length(vertex.normal), 1.0F, 1e-4F);
            if (vertex.material == material::kGlass)
            {
                // On the tube's circle, round the ceiling's innermost line, facing into the tube.
                const double t = std::hypot(p.x, p.y) - kCentre;
                EXPECT_NEAR(std::hypot(t, p.z), kTube, 0.05);
                EXPECT_LE(std::abs(torus.tubeAngleOf(p) - kPi), torus.windowHalfAngle + 1e-6);
                const Vec3d towardMiddle = (HabitatGeometry::localUp(p) * t) - Vec3d(0.0, 0.0, p.z);
                EXPECT_GT(glm::dot(Vec3d(vertex.normal), glm::normalize(towardMiddle)), 0.99);
            }
        }
        glass += chunk.kind == ChunkKind::GLASS ? 1U : 0U;
        hull += chunk.mesh.vertices.front().material == material::kHull ? 1U : 0U;
        metal += chunk.mesh.vertices.front().material == material::kMetal ? 1U : 0U;
        // Counter-clockwise seen from the side the normals face.
        const auto& vertices = chunk.mesh.vertices;
        for (std::size_t i = 0; i + 2 < chunk.mesh.indices.size(); i += 3)
        {
            const Vertex& a = vertices[chunk.mesh.indices[i]];
            const Vertex& b = vertices[chunk.mesh.indices[i + 1]];
            const Vertex& c = vertices[chunk.mesh.indices[i + 2]];
            const Vec3d   n =
                glm::cross(Vec3d(b.position - a.position), Vec3d(c.position - a.position));
            if (glm::length(n) > 1e-3)
            {
                EXPECT_GT(glm::dot(n, Vec3d(a.normal + b.normal + c.normal)), 0.0);
            }
        }
    }
    EXPECT_GT(glass, 20U);
    EXPECT_GT(hull, 20U);
    EXPECT_GT(metal, 20U);
}

}  // namespace

// ---- Lifts and the ceiling ---------------------------------------------------------------------

namespace
{

/// A small torus, so the lifts are short: 300 m out to the floor, a 60 m tube, four spokes.
HabitatSpec smallTorus()
{
    HabitatSpec spec                = {};
    spec.kind                       = HabitatKind::STANFORD_TORUS;
    spec.radiusM                    = 300.0;
    spec.torus.tubeRadiusM          = 30.0;
    spec.torus.hubRadiusM           = 20.0;
    spec.torus.spokes               = 4;
    spec.torus.spokeRadiusM         = 5.0;
    spec.torus.landHalfAngleDeg     = 40.0;
    spec.torus.sections             = 0;
    spec.terrain.hillHeightM        = 0.0;
    spec.terrain.mountainHeightM    = 0.0;
    spec.terrain.riverWidthM        = 0.0;
    spec.terrain.lakesPerValley     = 0;
    spec.settlements.townsPerValley = 0;
    spec.settlements.farmsPerValley = 0;
    return spec;
}

struct SmallTorus
{
    std::shared_ptr<const HabitatGeometry> geometry =
        std::make_shared<const HabitatGeometry>(smallTorus());
    std::shared_ptr<TerrainGrid> grid =
        std::make_shared<TerrainGrid>(sampleTerrain(*geometry, 1.0));
    std::vector<TramLine>         lines;
    std::unique_ptr<PhysicsWorld> physics;

    SmallTorus()
    {
        lines = planTramLines(*geometry, *grid);
        gradeForTrack(*grid, lines);
        addTramStops(lines, Settlements{});
        physics = std::make_unique<PhysicsWorld>(geometry, grid, PhysicsSettings{.threads = 0});
        physics->addColliders(enclosureColliders(*geometry, buildEnclosureMeshes(*geometry, 2.0)));
    }

    [[nodiscard]] const TramLine& lift() const
    {
        const auto found = std::ranges::find(lines, LineKind::SPOKE, &TramLine::kind);
        EXPECT_NE(found, lines.end());
        return *found;
    }
};

}  // namespace

TEST(Torus, EachSpokeHasALiftFromTheFloorToTheHub)
{
    const World& w     = world();
    int          lifts = 0;
    for (const TramLine& line : w.lines)
    {
        if (line.kind != LineKind::SPOKE)
        {
            continue;
        }
        ++lifts;
        // Straight up the radius at the spoke's angle, from the floor to the hub's floor.
        const Vec3d foot = line.track.front().position;
        const Vec3d top  = line.track.back().position;
        EXPECT_NEAR(HabitatGeometry::angularDistance(HabitatGeometry::angleOf(foot), line.theta),
                    0.0, 1e-9);
        EXPECT_NEAR(glm::length(glm::cross(glm::normalize(foot), glm::normalize(top))), 0.0, 1e-9);
        EXPECT_NEAR(std::hypot(top.x, top.y), 65.0, 1e-6);
        EXPECT_GT(std::hypot(foot.x, foot.y), kFloor - 5.0);
        ASSERT_EQ(line.stops.size(), 2U);
        EXPECT_EQ(line.stops.back().name, "the hub");
        // Every point of it in the air: the tube, then the spoke.
        for (const TrackPoint& point : line.track)
        {
            EXPECT_TRUE(w.geometry.enclosure().contains(
                point.position + (HabitatGeometry::localUp(point.position) * 1.0)));
        }
    }
    EXPECT_EQ(lifts, 6);
    // The lift waits on the floor, climbs, waits at the hub and comes down; gently at the hub.
    const TramLine& line    = *std::ranges::find(w.lines, LineKind::SPOKE, &TramLine::kind);
    double          last    = 0.0;
    double          fastest = 0.0;
    for (int step = 0; step < 800; ++step)
    {
        const double            t     = 0.5 * step;
        const std::vector<Tram> trams = tramsAt({line}, t);
        ASSERT_EQ(trams.size(), 1U);
        EXPECT_TRUE(trams.front().lift);
        const double along = trams.front().alongM;
        EXPECT_GE(along, -1e-9);
        EXPECT_LE(along, line.lengthM + 1e-9);
        fastest = std::max(fastest, std::abs(along - last) / 0.5);
        last    = along;
    }
    EXPECT_LE(fastest, 10.0 + 1e-6);
}

TEST(Torus, TheLiftCarriesYouUpToTheHub)
{
    SmallTorus      torus;
    const TramLine& lift = torus.lift();
    // On the cabin's floor while it waits at the foot.
    PlayerController player;
    player.setMover(&torus.physics->character());
    const Vec3d foot = lift.track.front().position;
    player.teleport(foot + (HabitatGeometry::localUp(foot) * (player.settings.eyeHeight + 0.05)));
    const LookRig look(HabitatGeometry::localUp(foot), Vec3d(0.0, 0.0, 1.0));
    const double  step = 1.0 / 60.0;
    double        top  = 1e9;
    for (int i = 0; i < 60 * 90; ++i)
    {
        const double seconds = 1.0 + (i * step);
        torus.physics->setTrams(tramsAt(torus.lines, seconds), player.eyePosition());
        player.step(MoveIntent{}, look, *torus.geometry, step);
        torus.physics->step(step, player.eyePosition());
        top = std::min(top, std::hypot(player.eyePosition().x, player.eyePosition().y));
    }
    // Carried up the spoke to the hub, still standing on the cabin's floor there.
    EXPECT_LT(top, 20.0);
    EXPECT_GT(top, 20.0 - player.settings.eyeHeight - 0.5);
    EXPECT_TRUE(torus.geometry->enclosure().contains(player.eyePosition()));
}

TEST(Torus, ABallThrownUpHitsTheCeiling)
{
    SmallTorus  torus;
    const Vec3d floor = torus.geometry->surfacePoint(0.0, 0.3);
    const Vec3d up    = HabitatGeometry::localUp(floor);
    const Quatd pose  = floorOrientation(floor);
    // Seen from outside, the ball flies straight: with the floor's own speed round the axis and
    // 60 m/s up it would pass 200 m from the axis, but the ceiling is 240 m from it.
    const std::size_t ball = torus.physics->addProp(
        {.kind = PropKind::BALL, .position = floor + (up * 1.0), .orientation = pose}, up * 60.0);
    double nearest = 1e9;
    for (int i = 0; i < 60 * 12; ++i)
    {
        torus.physics->step(1.0 / 60.0, torus.physics->props()[ball].position);
        const Vec3d at = torus.physics->props()[ball].position;
        nearest        = std::min(nearest, std::hypot(at.x, at.y));
        // Never out past the ceiling (the floor's half is the ground's business).
        EXPECT_EQ(torus.geometry->enclosure().clampInside(at, 0.0), at) << i;
    }
    EXPECT_GT(nearest, 239.5);
    EXPECT_LT(nearest, 243.0);
}

// ---- Light from the hub ------------------------------------------------------------------------

TEST(Torus, TheLightComesDownFromTheHub)
{
    const HabitatGeometry& geometry = world().geometry;
    const auto             noon     = sunBeams(geometry, degreesToRadians(45.0));
    ASSERT_EQ(noon.size(), 1U);
    ASSERT_TRUE(noon.front().hubTilt.has_value());
    EXPECT_NEAR(noon.front().hubTilt.value_or(-1.0), 0.0, 1e-12);
    EXPECT_NEAR(noon.front().intensity, 0.9, 1e-12);
    // At noon, straight up everywhere: toward the hub.
    for (const double theta : {0.0, 1.0, 4.0})
    {
        const Vec3d floor = geometry.surfacePoint(0.0, theta);
        EXPECT_NEAR(glm::dot(towardSunFrom(noon.front(), floor), HabitatGeometry::localUp(floor)),
                    1.0, 1e-12);
        // Lit from the bottom of the tube to the land's edges on both sides.
        for (const double z : {0.0, 30.0, -30.0, geometry.floorZMax(), geometry.floorZMin()})
        {
            EXPECT_DOUBLE_EQ(beamReach(geometry, geometry.surfacePoint(z, theta), noon.front()),
                             1.0)
                << z;
        }
    }
    // High on the walls the ceiling's metal is in the way: only the glass facing the hub lets the
    // light in.
    const Vec3d high = geometry.surfacePoint(tubeZ(80.0), 0.5);
    EXPECT_DOUBLE_EQ(beamReach(geometry, high, noon.front()), 0.0);
    // In the evening the light leans across the tube, toward +z, and shades the +z side first.
    const auto evening = sunBeams(geometry, degreesToRadians(85.0));
    EXPECT_GT(evening.front().hubTilt.value_or(-1.0), degreesToRadians(8.0));
    EXPECT_LE(evening.front().hubTilt.value_or(99.0), degreesToRadians(12.0) + 1e-12);
    const Vec3d plusEdge = geometry.surfacePoint(tubeZ(55.0), 0.5);
    EXPECT_DOUBLE_EQ(beamReach(geometry, plusEdge, evening.front()), 0.0);
    EXPECT_DOUBLE_EQ(beamReach(geometry, geometry.surfacePoint(-tubeZ(55.0), 0.5), evening.front()),
                     1.0);
    // No light at all inside a spoke or the hub, nor at night.
    EXPECT_DOUBLE_EQ(beamReach(geometry, Vec3d(400.0, 0.0, 0.0), noon.front()), 0.0);
    EXPECT_DOUBLE_EQ(sunBeams(geometry, degreesToRadians(100.0)).front().intensity, 0.0);
}

TEST(Torus, TheHubLightMatchesABruteForceMarch)
{
    // March from random points in the tube toward the hub until the line leaves the tube, and see
    // whether it left through the glass: beamReach must agree away from the glass's edges.
    const HabitatGeometry& geometry = world().geometry;
    const TorusShape       torus    = geometry.enclosure().torus().value_or(TorusShape{});
    SplitMix64             random(2045);
    for (const double alpha : {45.0, 70.0, 85.0})
    {
        const SunBeam beam = sunBeams(geometry, degreesToRadians(alpha)).front();
        for (int i = 0; i < 400; ++i)
        {
            const double angle  = random.uniform(-kPi, kPi);
            const double across = random.uniform(0.0, kTube - 1.0);
            const double theta  = random.uniform(0.0, 2.0 * kPi);
            const Vec3d  p      = torus.tubePoint(theta, angle, across - kTube);
            const Vec3d  d      = towardSunFrom(beam, p);
            Vec3d        q      = p;
            while (geometry.enclosure().contains(q) &&
                   std::hypot(std::hypot(q.x, q.y) - kCentre, q.z) < kTube)
            {
                q += d * 0.05;
            }
            const double off   = std::abs(torus.tubeAngleOf(q) - kPi);
            const double reach = beamReach(geometry, p, beam);
            if (off < torus.windowHalfAngle - 0.03)
            {
                EXPECT_GT(reach, 0.99) << alpha << " " << i;
            }
            else if (off > torus.windowHalfAngle + 0.03)
            {
                EXPECT_LT(reach, 0.01) << alpha << " " << i;
            }
        }
    }
}

TEST(Torus, TheGpuSeesTheTubeAndTheLightFromTheHub)
{
    const gpu::HabitatUniforms habitat =
        gpu::makeHabitatUniforms(world().geometry, degreesToRadians(45.0), gpu::LightingSettings{},
                                 Weather{}, gpu::CloudSettings{});
    EXPECT_FLOAT_EQ(habitat.light.x, 2.0F);  // light from the hub
    EXPECT_FLOAT_EQ(habitat.light.y, 1.0F);
    EXPECT_FLOAT_EQ(habitat.light.z, static_cast<float>(kCentre));
    EXPECT_FLOAT_EQ(habitat.light.w, 0.0F);  // not a sphere
    EXPECT_FLOAT_EQ(habitat.band.x, 1.0F);   // the land runs round
    EXPECT_FLOAT_EQ(habitat.band.w, static_cast<float>(kTube));
    EXPECT_NEAR(habitat.shape.w, kPi * 0.333, 1e-6);
    EXPECT_FLOAT_EQ(habitat.strips.y, 0.0F);  // no window strips along a hull
    // Straight up at noon, at full strength.
    EXPECT_NEAR(habitat.beams.at(0).x, 1.0F, 1e-6F);
    EXPECT_NEAR(habitat.beams.at(0).z, 0.0F, 1e-6F);
    EXPECT_NEAR(habitat.beams.at(0).w, 0.9F, 1e-6F);
    EXPECT_EQ(habitat.beams.at(1).w, 0.0F);
    // The ring of mirrors round the hub: how far they lean the light, the hub, the spokes.
    EXPECT_FLOAT_EQ(habitat.mirror.y, 0.0F);
    EXPECT_FLOAT_EQ(habitat.mirror.z, 65.0F);
    EXPECT_FLOAT_EQ(habitat.mirror.w, 6.0F);

    // Every other kind leaves the torus's lanes empty, which is how the shaders tell.
    const HabitatGeometry      island{HabitatSpec{}};
    const gpu::HabitatUniforms other = gpu::makeHabitatUniforms(
        island, degreesToRadians(45.0), gpu::LightingSettings{}, Weather{}, gpu::CloudSettings{});
    EXPECT_EQ(other.band.w, 0.0F);
    EXPECT_EQ(other.light.x, 0.0F);
}

// ---- Land --------------------------------------------------------------------------------------

TEST(Torus, TownsStandBesideTheSpokesAndFarmsBetweenThem)
{
    const World&     w     = world();
    const TorusSpec& spec  = w.geometry.spec().torus;
    const TorusShape torus = w.geometry.enclosure().torus().value_or(TorusShape{});
    int              towns = 0;
    int              farms = 0;
    for (const Settlement& place : w.settlements.places)
    {
        const Vec3d  centre  = place.plane.point(Vec2d(0.0), 0.0);
        const double theta   = HabitatGeometry::angleOf(centre);
        const int    section = torusSectionAt(spec, theta);
        if (place.kind == SettlementKind::TOWN)
        {
            ++towns;
            EXPECT_EQ(section % 2, 0) << place.name;
            // Its lift station just beyond its end.
            double nearest = 1e9;
            for (int k = 0; k < torus.spokes; ++k)
            {
                nearest = std::min(
                    nearest, HabitatGeometry::angularDistance(theta, torus.spokeAngle(k)) * kFloor);
            }
            EXPECT_GT(nearest, place.radiusM) << place.name;
            EXPECT_LT(nearest, place.radiusM + 60.0) << place.name;
        }
        else
        {
            ++farms;
            EXPECT_EQ(section % 2, 1) << place.name;
        }
    }
    EXPECT_EQ(towns, 3);
    EXPECT_GE(farms, 3);
    // Every building on the land, none on a lift's foot.
    for (const Building& building : w.settlements.buildings)
    {
        const SurfaceSpot spot =
            w.settlements.places.at(building.settlement).plane.surface(building.centre);
        EXPECT_TRUE(w.geometry.onLand(spot.z, spot.theta));
        EXPECT_FALSE(nearTrack(w.lines, spot.z, spot.theta, 0.0) && std::abs(spot.z) < 8.0 &&
                     std::ranges::any_of(w.lines, [&](const TramLine& line) {
                         return line.kind == LineKind::SPOKE &&
                                HabitatGeometry::angularDistance(spot.theta, line.theta) * kFloor <
                                    8.0;
                     }));
    }
}

TEST(Torus, TheWallsAreTerraced)
{
    // The preset's terraces on otherwise bare ground.
    HabitatSpec spec         = stanfordTorus();
    spec.terrain.hillHeightM = 0.0;
    const HabitatGeometry geometry(spec);
    const double          step = geometry.spec().terrain.mountainHeightM;
    ASSERT_GT(step, 1.0);
    // Half way up the wall the ground stands on level shelves: whole steps from the axis.
    int shelves = 0;
    for (int i = 0; i < 48; ++i)
    {
        const double degrees = 50.0 + (0.25 * i);
        const double z       = tubeZ(degrees);
        const double h       = geometry.terraceHeight(z);
        EXPECT_GE(h, 0.0);
        EXPECT_LT(h, step);
        const double r = geometry.groundRadius(z, 0.7).value_or(0.0);
        if (std::abs(std::remainder(r, step)) < 1e-6)
        {
            ++shelves;
        }
    }
    EXPECT_GT(shelves, 40);
    // None on the land, and none where the wall stands upright under the ceiling.
    EXPECT_EQ(geometry.terraceHeight(0.0), 0.0);
    EXPECT_EQ(geometry.terraceHeight(tubeZ(35.0)), 0.0);
    EXPECT_EQ(geometry.terraceHeight(tubeZ(80.0)), 0.0);
}
