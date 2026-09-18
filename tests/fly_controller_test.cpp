#include "StarshipSimulator/core/fly_controller.h"

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/math.h"

namespace
{

using StarshipSimulator::degreesToRadians;
using StarshipSimulator::FlyController;
using StarshipSimulator::Locomotion;
using StarshipSimulator::LookRig;
using StarshipSimulator::MoveIntent;
using StarshipSimulator::Vec3d;

constexpr double kStep = 1.0 / 120.0;

void run(FlyController& controller, const MoveIntent& intent, const LookRig& look, double seconds)
{
    const auto steps = static_cast<int>(std::ceil(seconds / kStep));
    for (int step = 0; step < steps; ++step)
    {
        controller.step(intent, look, kStep);
    }
}

TEST(FlyController, WalkerFallsToTheGroundAndStandsAtEyeHeight)
{
    FlyController controller(Vec3d(0.0, 0.0, 10.0));
    run(controller, {}, LookRig(), 3.0);
    EXPECT_TRUE(controller.grounded());
    EXPECT_DOUBLE_EQ(controller.eyePosition().z, controller.settings.eyeHeight);
}

TEST(FlyController, WalksHorizontallyAtWalkingSpeedEvenWhenLookingUp)
{
    FlyController controller;
    LookRig       look;
    look.setAngles(0.0, degreesToRadians(60.0));
    run(controller, {.forward = 1.0}, look, 2.0);
    EXPECT_NEAR(controller.velocity().y, controller.settings.walkSpeed, 0.01);
    EXPECT_NEAR(controller.velocity().x, 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(controller.eyePosition().z, controller.settings.eyeHeight);
}

TEST(FlyController, DiagonalInputIsNotFaster)
{
    FlyController controller;
    run(controller, {.forward = 1.0, .right = 1.0, .fast = true}, LookRig(), 3.0);
    EXPECT_NEAR(glm::length(controller.velocity()), controller.settings.runSpeed, 0.01);
}

TEST(FlyController, JumpReachesTheBallisticApex)
{
    FlyController controller;
    run(controller, {}, LookRig(), 0.5);  // settle on the ground
    const double ground = controller.eyePosition().z;
    double       apex   = ground;
    controller.step({.jump = true}, LookRig(), kStep);
    for (int i = 0; i < 240; ++i)
    {
        controller.step({}, LookRig(), kStep);
        apex = std::max(apex, controller.eyePosition().z);
    }
    const double v        = controller.settings.jumpSpeed;
    const double expected = v * v / (2.0 * controller.settings.gravity);
    EXPECT_NEAR(apex - ground, expected, 0.03);
    EXPECT_TRUE(controller.grounded());
}

TEST(FlyController, FlyingFollowsTheViewDirectionWithoutGravity)
{
    FlyController controller(Vec3d(0.0, 0.0, 100.0));
    controller.setLocomotion(Locomotion::Fly);
    LookRig look;
    look.setAngles(0.0, degreesToRadians(30.0));
    run(controller, {.forward = 1.0}, look, 3.0);
    const Vec3d direction = glm::normalize(controller.velocity());
    EXPECT_NEAR(direction.y, std::cos(degreesToRadians(30.0)), 1e-3);
    EXPECT_NEAR(direction.z, std::sin(degreesToRadians(30.0)), 1e-3);
    EXPECT_NEAR(glm::length(controller.velocity()), controller.settings.flySpeed, 0.01);

    run(controller, {}, look, 3.0);  // let go: glide to a stop, no falling
    EXPECT_NEAR(glm::length(controller.velocity()), 0.0, 1e-3);
    EXPECT_GT(controller.eyePosition().z, 100.0);
}

TEST(FlyController, TeleportStopsAllMotion)
{
    FlyController controller;
    controller.setLocomotion(Locomotion::Fly);
    run(controller, {.forward = 1.0}, LookRig(), 1.0);
    controller.teleport(Vec3d(1.0e6, 0.0, 1.7));
    EXPECT_EQ(controller.eyePosition(), Vec3d(1.0e6, 0.0, 1.7));
    EXPECT_EQ(controller.velocity(), Vec3d(0.0));
}

}  // namespace
