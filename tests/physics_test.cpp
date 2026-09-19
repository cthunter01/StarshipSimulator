#include <algorithm>
#include <cmath>
#include <optional>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/player_controller.h"
#include "StarshipSimulator/core/physics/rotating_frame.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

constexpr double kStep = 1.0 / 120.0;

OneillCylinderSpec flatSpec(double gravityG = 1.0)
{
    OneillCylinderSpec spec;
    spec.surfaceGravityG         = gravityG;
    spec.terrain.hillHeightM     = 0.0;
    spec.terrain.mountainHeightM = 0.0;
    spec.terrain.riverWidthM     = 0.0;  // flat: no rivers or lakes either
    spec.terrain.lakesPerValley  = 0;
    return spec;
}

/// A point on the valley floor at theta = pi (up is +x), height h above it.
Vec3d valleyPoint(double height, double z = 0.0)
{
    return {-4000.0 + height, 0.0, z};
}

void expectNear(const Vec3d& actual, const Vec3d& expected, double tolerance)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

/// Look rig for a player at theta = pi: up +x, north +z.
LookRig valleyLook(double yawDeg = 0.0)
{
    LookRig look(Vec3d(1.0, 0.0, 0.0), Vec3d(0.0, 0.0, 1.0));
    look.setAngles(degreesToRadians(yawDeg), 0.0);
    return look;
}

// ---- Rotating frame ----------------------------------------------------------------------------

TEST(RotatingFrame, FictitiousForces)
{
    const RotatingFrame frame(0.05);
    expectNear(frame.centrifugal(Vec3d(4000.0, 0.0, 7.0)), Vec3d(10.0, 0.0, 0.0), 1e-12);
    expectNear(frame.coriolis(Vec3d(0.0, 1.0, 0.0)), Vec3d(0.1, 0.0, 0.0), 1e-12);
    expectNear(frame.coriolis(Vec3d(0.0, 0.0, 5.0)), Vec3d(0.0), 1e-12);  // along the axis: none
}

TEST(RotatingFrame, InertialRoundTrip)
{
    const RotatingFrame frame(0.049514);
    const BodyState     state{.position = Vec3d(-3000.0, 1200.0, 55.0),
                              .velocity = Vec3d(3.0, -2.0, 1.0)};
    const BodyState     back = frame.fromInertial(frame.toInertial(state, 1.234), 1.234);
    expectNear(back.position, state.position, 1e-9);
    expectNear(back.velocity, state.velocity, 1e-9);
    // Standing still on the floor means moving at the rim speed in the inertial frame.
    const BodyState standing{.position = Vec3d(4000.0, 0.0, 0.0), .velocity = Vec3d(0.0)};
    EXPECT_NEAR(glm::length(frame.toInertial(standing, 0.0).velocity), 198.056, 1e-3);
}

TEST(RotatingFrame, FreeFlightIsAStraightLineSeenFromOutside)
{
    const RotatingFrame frame(0.049514);
    const BodyState     start{.position = valleyPoint(1.5), .velocity = Vec3d(2.0, 3.0, -1.0)};
    const Vec3d         p0 = frame.toInertial(start, 0.0).position;
    Vec3d               previousStep(0.0);
    for (int i = 1; i <= 5; ++i)
    {
        const double    t        = 0.2 * i;
        const BodyState moved    = propagateFreeFlight(frame, start, t);
        const Vec3d     inertial = frame.toInertial(moved, frame.omega() * t).position;
        const Vec3d     step     = inertial - p0;
        if (i > 1)
        {
            // Same direction and proportional length: uniform straight-line motion.
            EXPECT_NEAR(glm::length(glm::cross(glm::normalize(step), glm::normalize(previousStep))),
                        0.0, 1e-9);
        }
        EXPECT_NEAR(glm::length(step) / t, glm::length(frame.toInertial(start, 0.0).velocity),
                    1e-9);
        previousStep = step;
    }
}

TEST(RotatingFrame, JacobiIntegralIsConservedInFreeFlight)
{
    const RotatingFrame frame(0.049514);
    const BodyState     start{.position = valleyPoint(1.0), .velocity = Vec3d(5.0, 7.0, 1.0)};
    const double        j0 = frame.jacobi(start);
    for (const double t : {0.5, 1.0, 2.0})
    {
        EXPECT_NEAR(frame.jacobi(propagateFreeFlight(frame, start, t)) / j0, 1.0, 1e-9);
    }
}

TEST(RotatingFrame, DroppedObjectLandsAntispinward)
{
    // Island Three: dropped from 1.5 m it lands 2.7 cm behind the spot below it.
    const HabitatGeometry geometry(flatSpec());
    const RotatingFrame   frame(geometry.omega());
    const BodyState       start{.position = valleyPoint(1.5), .velocity = Vec3d(0.0)};
    const auto            impact = predictImpact(frame, geometry, start, 5.0);
    ASSERT_TRUE(impact.has_value());
    const Impact hit         = impact.value_or(Impact{});
    const double startAngle  = HabitatGeometry::angleOf(start.position);
    const double impactAngle = HabitatGeometry::angleOf(hit.state.position);
    const double drift       = std::remainder(impactAngle - startAngle, 2.0 * kPi) * 4000.0;
    EXPECT_NEAR(drift, -0.0274, 0.0005);  // negative: against the spin
    EXPECT_NEAR(hit.time, std::sqrt(2.0 * 1.5 / 9.80665), 0.01);
}

TEST(RotatingFrame, SpinwardThrowsLandSooner)
{
    const HabitatGeometry geometry(flatSpec());
    const RotatingFrame   frame(geometry.omega());
    // At theta = pi the spin direction is -y.
    const Vec3d up(1.0, 0.0, 0.0);
    const Vec3d spinward(0.0, -1.0, 0.0);
    const auto  flightTime = [&](const Vec3d& horizontal) {
        const BodyState start{.position = valleyPoint(1.5),
                              .velocity = (horizontal * 20.0) + (up * 10.0)};
        return predictImpact(frame, geometry, start, 10.0).value_or(Impact{}).time;
    };
    EXPECT_LT(flightTime(spinward), flightTime(-spinward));
}

TEST(FreeBody, StepsFollowTheExactTrajectory)
{
    const RotatingFrame frame(0.049514);
    const BodyState     start{.position = valleyPoint(1.0), .velocity = Vec3d(4.0, 6.0, 1.0)};
    BodyState           state = start;
    for (int i = 0; i < 120; ++i)
    {
        stepFreeBody(state, frame, Vec3d(0.0), kStep);
    }
    expectNear(state.position, propagateFreeFlight(frame, start, 1.0).position, 1e-6);

    // A minute of floating near the axis: the Jacobi integral does not drift.
    BodyState    floating{.position = Vec3d(200.0, 0.0, 0.0), .velocity = Vec3d(0.0, 3.0, 0.5)};
    const double j0 = frame.jacobi(floating);
    for (int i = 0; i < 60 * 120; ++i)
    {
        stepFreeBody(floating, frame, Vec3d(0.0), kStep);
    }
    EXPECT_NEAR(frame.jacobi(floating) / j0, 1.0, 1e-9);
}

TEST(FreeBody, ComfortModeKeepsEnergyWithoutCoriolis)
{
    const RotatingFrame frame(0.049514);
    BodyState           state{.position = valleyPoint(1.0), .velocity = Vec3d(3.0, 0.0, 0.0)};
    const double        j0       = frame.jacobi(state);
    double              maxDrift = 0.0;
    for (int i = 0; i < 120; ++i)
    {
        stepFreeBody(state, frame, Vec3d(0.0), kStep, false);
        maxDrift = std::max(maxDrift, std::abs((frame.jacobi(state) - j0) / j0));
    }
    EXPECT_LT(maxDrift, 1e-6);
    EXPECT_NEAR(state.position.y, 0.0, 1e-9);  // straight up and down: no sideways drift
}

// ---- Player controller -------------------------------------------------------------------------

TEST(PlayerController, StandingStillStaysPut)
{
    const HabitatGeometry geometry(flatSpec());
    PlayerController      player;
    player.placeOnGround(geometry, 0.0, kPi);
    const Vec3d start = player.eyePosition();
    for (int i = 0; i < 1200; ++i)
    {
        player.step({}, valleyLook(), geometry, kStep);
    }
    expectNear(player.eyePosition(), start, 1e-6);
    EXPECT_TRUE(player.grounded());
}

TEST(PlayerController, WalksAlongTheValleyAtEyeHeight)
{
    const HabitatGeometry geometry(flatSpec());
    PlayerController      player;
    player.placeOnGround(geometry, 0.0, kPi);
    for (int i = 0; i < 1200; ++i)
    {
        player.step({.forward = 1.0}, valleyLook(), geometry, kStep);
    }
    EXPECT_NEAR(player.eyePosition().z, 1.4 * 10.0, 0.3);
    EXPECT_NEAR(std::hypot(player.eyePosition().x, player.eyePosition().y), 4000.0 - 1.7, 1e-6);
}

TEST(PlayerController, WalksAroundTheCurveOnTheFarSide)
{
    const HabitatGeometry geometry(flatSpec());
    PlayerController      player;
    player.placeOnGround(geometry, 0.0, geometry.landCenter(0));
    const Vec3d up = HabitatGeometry::localUp(player.eyePosition());
    LookRig     look(up, Vec3d(0.0, 0.0, 1.0));
    look.setAngles(degreesToRadians(-90.0), 0.0);  // face across the valley
    for (int i = 0; i < 1200; ++i)
    {
        look.setFrame(HabitatGeometry::localUp(player.eyePosition()), Vec3d(0.0, 0.0, 1.0));
        player.step({.forward = 1.0, .fast = true}, look, geometry, kStep);
    }
    EXPECT_NEAR(std::hypot(player.eyePosition().x, player.eyePosition().y), 4000.0 - 1.7, 1e-6);
    const double moved =
        HabitatGeometry::angularDistance(HabitatGeometry::angleOf(player.eyePosition()),
                                         geometry.landCenter(0)) *
        4000.0;
    EXPECT_NEAR(moved, 5.0 * 10.0, 1.0);
}

double jumpApex(double gravityG, bool comfort, double* drift = nullptr)
{
    const HabitatGeometry geometry(flatSpec(gravityG));
    PlayerController      player;
    player.settings.comfortMode = comfort;
    player.placeOnGround(geometry, 0.0, kPi);
    const Vec3d start = player.eyePosition();
    double      apex  = 0.0;
    player.step({.jump = true}, valleyLook(), geometry, kStep);
    for (int i = 0; i < 1000 && !player.grounded(); ++i)
    {
        player.step({}, valleyLook(), geometry, kStep);
        apex = std::max(apex, geometry.ground(player.eyePosition()).heightAboveGround - 1.7);
    }
    if (drift != nullptr)
    {
        *drift = glm::distance(player.eyePosition(), start);
    }
    return apex;
}

TEST(PlayerController, JumpsGoHigherInWeakerGravity)
{
    const double v = PlayerSettings{}.jumpSpeed;
    EXPECT_NEAR(jumpApex(1.0, false), v * v / (2.0 * 9.80665), 0.02);
    EXPECT_NEAR(jumpApex(0.5, false), v * v / (2.0 * 0.5 * 9.80665), 0.04);
}

TEST(PlayerController, CoriolisDeflectsJumpsUnlessInComfortMode)
{
    double withSpin = 0.0;
    double comfort  = 0.0;
    jumpApex(1.0, false, &withSpin);
    jumpApex(1.0, true, &comfort);
    EXPECT_GT(withSpin, 0.02);  // about 3 cm for a 3.5 m/s jump
    EXPECT_LT(comfort, 0.001);
}

TEST(PlayerController, TooSteepSlopesCannotBeWalkedUp)
{
    const HabitatGeometry geometry(flatSpec());
    const double          rampZ = geometry.floorZMin() - 1500.0;  // on the 25 degree ramp
    for (const double maxSlope : {20.0, 38.0})
    {
        PlayerController player;
        player.settings.maxSlopeDeg = maxSlope;
        player.placeOnGround(geometry, rampZ, kPi);
        const double startZ = player.eyePosition().z;
        for (int i = 0; i < 600; ++i)
        {
            // Yaw 180: face the anti-sunward end, uphill.
            LookRig look(HabitatGeometry::localUp(player.eyePosition()), Vec3d(0.0, 0.0, 1.0));
            look.setAngles(kPi, 0.0);
            player.step({.forward = 1.0}, look, geometry, kStep);
        }
        if (maxSlope < 25.0)
        {
            EXPECT_NEAR(player.eyePosition().z, startZ, 0.05);
        }
        else
        {
            EXPECT_LT(player.eyePosition().z, startZ - 5.0);
        }
    }
}

TEST(PlayerController, GravityFallsLinearlyUpTheRamp)
{
    const HabitatGeometry geometry(flatSpec());
    PlayerController      player;
    const double rampTopZ = geometry.floorZMin() - (2000.0 / std::tan(degreesToRadians(25.0)));
    player.placeOnGround(geometry, rampTopZ, kPi);
    const double r = std::hypot(player.eyePosition().x, player.eyePosition().y) + 1.7;
    EXPECT_NEAR(r, 2000.0, 0.5);
    EXPECT_NEAR(geometry.gravityAt(r) / 9.80665, 0.5, 1e-3);
}

TEST(PlayerController, FlyingStaysAboveGroundAndInside)
{
    const HabitatGeometry geometry(flatSpec());
    PlayerController      player;
    player.setLocomotion(Locomotion::Fly);
    player.placeOnGround(geometry, 0.0, kPi);
    LookRig look = valleyLook();
    look.setAngles(0.0, degreesToRadians(-80.0));  // dive into the ground
    for (int i = 0; i < 600; ++i)
    {
        player.step({.forward = 1.0}, look, geometry, kStep);
    }
    EXPECT_GE(geometry.ground(player.eyePosition()).heightAboveGround, 1.7 - 1e-9);
    player.teleport(Vec3d(0.0, 0.0, 25000.0));  // beyond the dome
    player.step({}, look, geometry, kStep);
    EXPECT_LE(player.eyePosition().z, geometry.walkableZMax());
}

}  // namespace
