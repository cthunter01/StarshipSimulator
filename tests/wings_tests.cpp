#include <cmath>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/PlayerController.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

const HabitatGeometry& island()
{
    static const HabitatGeometry kGeometry{HabitatSpec{}};
    return kGeometry;
}

/// Flying at radius `r` in valley 0, aimed along the axis and `climbDeg` above the path.
struct Flight
{
    PlayerController player;
    LookRig          look;
    double           seconds = 0.0;
};

Flight launch(double radius, double speed, double climbDeg)
{
    const HabitatGeometry& geometry = island();
    const double           theta    = geometry.landCenter(0);
    const Vec3d            at(radius * std::cos(theta), radius * std::sin(theta), 0.0);
    const Vec3d            up = HabitatGeometry::localUp(at);

    Flight flight;
    flight.player.setLocomotion(Locomotion::WINGS);
    flight.player.teleport(at);
    flight.look = LookRig(up, Vec3d(0.0, 0.0, 1.0));
    flight.look.setAngles(0.0, degreesToRadians(climbDeg));
    // Launched along the axis: the look rig's north, with no climb of its own.
    flight.player.launch(Vec3d(0.0, 0.0, speed));
    return flight;
}

/// Flies on for `seconds`, returning how much height was gained (metres toward the axis).
double fly(Flight& flight, double seconds, bool flapping = false)
{
    const double before  = std::hypot(flight.player.eyePosition().x, flight.player.eyePosition().y);
    constexpr double kDt = 1.0 / 120.0;
    const MoveIntent intent{.jump = flapping};
    for (int step = 0; step < static_cast<int>(seconds / kDt); ++step)
    {
        flight.look.setFrame(flight.player.viewUp(), Vec3d(0.0, 0.0, 1.0));
        flight.player.step(intent, flight.look, island(), kDt);
    }
    const double after = std::hypot(flight.player.eyePosition().x, flight.player.eyePosition().y);
    return before - after;  // up is toward the axis, so a smaller radius is higher
}

TEST(Wings, TheAirThinsTowardTheAxis)
{
    const HabitatSpec& spec = island().spec();
    const double floor = airDensityAt(spec.atmosphere, island().omega(), island().radius(), 4000.0);
    const double axis  = airDensityAt(spec.atmosphere, island().omega(), island().radius(), 0.0);
    EXPECT_NEAR(floor, 1.2, 0.1);            // about sea level
    EXPECT_NEAR(axis / floor, 0.792, 0.01);  // the metrics' axis pressure ratio
}

TEST(Wings, CannotBeKeptUpAtAFullGravity)
{
    // The wings glide perfectly well down in the valley, as a hang glider does: what a person
    // cannot do at one gravity is pay for the drag, so however hard they flap they come down.
    Flight       flight = launch(3960.0, 14.0, 6.0);
    const double gained = fly(flight, 20.0, true);
    EXPECT_LT(gained, -5.0) << "flapping at one gravity should still lose height";
}

TEST(Wings, CarryYouNearTheAxisWhereGravityIsWeak)
{
    // A tenth of a gravity, 400 m out from the axis: now the same muscles are enough to climb.
    const double radius = 0.1 * island().radius();
    Flight       flight = launch(radius, 12.0, 8.0);
    const double gained = fly(flight, 20.0, true);
    EXPECT_GT(gained, 5.0) << "a flap should climb in a tenth of a gravity";
    EXPECT_GT(flight.player.wings().airspeed, 4.0);
    // Flying along, the wings carry about their own weight: that is what level flight is.
    EXPECT_GT(flight.player.wings().liftOverWeight, 0.5);
    EXPECT_LT(flight.player.wings().liftOverWeight, 4.0);
}

TEST(Wings, SinkFasterTheLowerYouGo)
{
    // The same glide, high up and low down: gravity makes the difference, not the air.
    Flight high = launch(0.12 * island().radius(), 13.0, 3.0);
    Flight low  = launch(0.6 * island().radius(), 13.0, 3.0);
    EXPECT_GT(fly(high, 8.0, false), fly(low, 8.0, false));
}

TEST(Wings, GlidingTradesHeightForDistance)
{
    // No flapping: drag has to be paid for out of height.
    const double radius = 0.25 * island().radius();
    Flight       flight = launch(radius, 16.0, 0.0);
    const double gained = fly(flight, 6.0, false);
    EXPECT_LT(gained, 0.0);
    EXPECT_GT(gained, -60.0) << "a wing should glide, not drop like a stone";
    EXPECT_GT(flight.player.wings().airspeed, 2.0);
}

TEST(Wings, StallWhenAimedTooFarAboveThePath)
{
    Flight flight = launch(0.15 * island().radius(), 10.0, 45.0);
    fly(flight, 1.0, false);
    EXPECT_TRUE(flight.player.wings().stalled);

    Flight gentle = launch(0.15 * island().radius(), 10.0, 5.0);
    fly(gentle, 1.0, false);
    EXPECT_FALSE(gentle.player.wings().stalled);
}

TEST(Wings, EverythingStaysFinite)
{
    Flight flight = launch(0.2 * island().radius(), 0.0, 0.0);  // dropped from a standstill
    fly(flight, 20.0, true);
    const Vec3d at = flight.player.eyePosition();
    EXPECT_TRUE(std::isfinite(at.x) && std::isfinite(at.y) && std::isfinite(at.z));
    EXPECT_LT(glm::length(flight.player.velocity()), 200.0);
}

}  // namespace
