#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/core/utf8_path.h"

// Kalpana One: a short cylinder whose land runs once round the axis. Everything that is laid out
// on a band of land -- the river, the lakes, the towns, the farms and the tramway -- has to work on
// a band that closes on itself.
namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

HabitatSpec kalpanaOne()
{
    const auto scenario =
        loadScenario(pathFromUtf8(STARSHIPSIMULATOR_DATA_DIR) / "presets" / "kalpana_one.toml");
    EXPECT_TRUE(scenario.has_value()) << scenario.error().describe();
    return scenario ? scenario->habitat : HabitatSpec{};
}

/// The preset's world, generated once for all the tests.
struct World
{
    HabitatGeometry       geometry{kalpanaOne()};
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

TEST(Kalpana, TheLandIsOneBandRoundTheAxis)
{
    const HabitatGeometry& geometry = world().geometry;
    EXPECT_EQ(geometry.kind(), HabitatKind::KALPANA_CYLINDER);
    ASSERT_EQ(geometry.bandCount(), 1);
    const LandBand& band = geometry.band(0);
    EXPECT_EQ(band.axis, BandAxis::AROUND);
    EXPECT_TRUE(band.wraps);
    EXPECT_NEAR(band.alongLengthM(), 2.0 * kPi * 250.0, 1e-6);
    EXPECT_NEAR(band.halfWidthM, 162.5, 1e-6);
    // No window strips: land all the way round, end wall to end wall.
    for (int i = 0; i < 12; ++i)
    {
        EXPECT_TRUE(geometry.onLand(0.0, i * kPi / 6.0));
        EXPECT_EQ(geometry.regionAt(100.0, i * kPi / 6.0).kind, RegionKind::LAND);
    }
}

TEST(Kalpana, TheRiverRunsRoundAndBackIntoItself)
{
    const Landscape& landscape = world().geometry.landscape();
    const LandBand&  band      = world().geometry.band(0);
    ASSERT_TRUE(landscape.hasRivers());
    const double length = band.alongLengthM();
    for (int step = 0; step * 37.0 < length; ++step)
    {
        const double along = band.alongMinM + (step * 37.0);
        const Vec2d  here  = landscape.riverAcross(along);
        const Vec2d  again = landscape.riverAcross(along + length);
        EXPECT_NEAR(here.x, again.x, 1e-9);
        EXPECT_LT(std::abs(here.x), band.halfWidthM - 30.0) << "the river keeps off the end walls";
        // Water in the river's bed.
        const SurfaceSpot spot = band.toSurface(Vec2d(here.x, along));
        EXPECT_LT(landscape.shoreDistance(spot.z, spot.theta), 0.0);
    }
}

TEST(Kalpana, TownsAndFarmsStandOnTheLoop)
{
    const World&       w = world();
    const Settlements& s = w.settlements;
    EXPECT_GE(s.townCount(), 2U);
    EXPECT_GE(s.places.size() - s.townCount(), 2U) << "farmsteads";
    for (const Building& b : s.buildings)
    {
        const Settlement& place = s.places[b.settlement];
        EXPECT_EQ(place.plane.axis, BandAxis::AROUND);
        const SurfaceSpot spot = place.plane.surface(b.centre);
        EXPECT_TRUE(w.geometry.onLand(spot.z, spot.theta));
        EXPECT_GT(w.geometry.landscape().shoreDistance(spot.z, spot.theta), 4.0);
        EXPECT_LT(std::abs(spot.z), 162.5 - 20.0) << "clear of the end walls";
    }
}

TEST(Kalpana, AFloorPlaneOnTheLoopKeepsTheValleysHandedness)
{
    const Settlement& place = world().settlements.places.front();
    const Vec3d       here  = place.plane.point(Vec2d(0.0), 0.0);
    const Vec3d       x     = place.plane.point(Vec2d(1.0, 0.0), 0.0) - here;
    const Vec3d       y     = place.plane.point(Vec2d(0.0, 1.0), 0.0) - here;
    EXPECT_NEAR(glm::length(x), 1.0, 1e-3);
    EXPECT_NEAR(glm::length(y), 1.0, 1e-3);
    EXPECT_LT(glm::dot(glm::cross(x, y), HabitatGeometry::localUp(here)), -0.999);
    EXPECT_NEAR(glm::dot(glm::normalize(y), place.plane.alongDirection(Vec2d(0.0))), 1.0, 1e-5);
    const Vec2d back = place.plane.toPlan(place.plane.surface(Vec2d(30.0, -40.0)).z,
                                          place.plane.surface(Vec2d(30.0, -40.0)).theta);
    EXPECT_NEAR(back.x, 30.0, 1e-6);
    EXPECT_NEAR(back.y, -40.0, 1e-6);
}

TEST(Kalpana, TheTramwayGoesOnceRoundAndCallsAtTheTowns)
{
    const World& w = world();
    ASSERT_EQ(w.lines.size(), 1U) << "a loop, and no funicular";
    const TramLine& line = w.lines.front();
    EXPECT_EQ(line.kind, LineKind::LOOP);
    EXPECT_NEAR(line.lengthM, 2.0 * kPi * line.radiusM, 0.02 * line.lengthM);
    EXPECT_LT(glm::distance(line.track.front().position, line.track.back().position), 1e-9)
        << "the loop closes";
    // Every town has a stop, in order round the loop.
    EXPECT_GE(line.stops.size(), w.settlements.townCount());
    for (std::size_t i = 1; i < line.stops.size(); ++i)
    {
        EXPECT_LT(line.stops[i - 1].alongM, line.stops[i].alongM);
    }
    // Round and round.
    const TrackPoint once  = pointAlong(line, 100.0);
    const TrackPoint twice = pointAlong(line, 100.0 + line.lengthM);
    EXPECT_LT(glm::distance(once.position, twice.position), 1e-6);
}

TEST(Kalpana, TramsAlwaysGoTheSameWayRound)
{
    const TramLine& line = world().lines.front();
    for (int step = 0; step * 7.3 < 2.0 * line.journeyS; ++step)
    {
        const double t = step * 7.3;
        for (const Tram& tram : tramsAt(world().lines, t))
        {
            const Vec3d around =
                world().geometry.band(0).alongDirection(HabitatGeometry::angleOf(tram.position));
            EXPECT_GT(glm::dot(tram.forward, around), 0.9) << "at " << t << " s";
        }
    }
}

TEST(Kalpana, TheTrackLiesOnTheGradedGround)
{
    const World&    w    = world();
    const TramLine& line = w.lines.front();
    for (const TrackPoint& point : line.track)
    {
        if (point.carried)
        {
            continue;
        }
        const double theta  = HabitatGeometry::angleOf(point.position);
        const double ground = w.grid.groundHeight(point.position.z, theta);
        EXPECT_NEAR(point.railHeightM - 0.28, ground, 0.35) << "at " << point.alongM << " m";
    }
}

}  // namespace
