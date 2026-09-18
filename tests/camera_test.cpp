#include "StarshipSimulator/core/camera.h"

#include <cmath>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/math.h"

namespace
{

using StarshipSimulator::Camera;
using StarshipSimulator::cameraRelative;
using StarshipSimulator::degreesToRadians;
using StarshipSimulator::LookRig;
using StarshipSimulator::Mat4d;
using StarshipSimulator::reverseZInfinitePerspective;
using StarshipSimulator::Vec3d;
using StarshipSimulator::Vec3f;
using StarshipSimulator::Vec4d;

double depthAt(const Mat4d& projection, double viewDistance)
{
    const Vec4d clip = projection * Vec4d(0.0, 0.0, -viewDistance, 1.0);
    return clip.z / clip.w;
}

void expectNear(const Vec3d& actual, const Vec3d& expected, double tolerance)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

TEST(ReverseZ, NearPlaneMapsToOneAndInfinityToZero)
{
    const Mat4d projection = reverseZInfinitePerspective(degreesToRadians(70.0), 16.0 / 9.0, 0.05);
    EXPECT_NEAR(depthAt(projection, 0.05), 1.0, 1e-12);
    EXPECT_NEAR(depthAt(projection, 1.0e12), 0.0, 1e-12);
    EXPECT_GT(depthAt(projection, 1.0e12), 0.0);
}

TEST(ReverseZ, DepthDecreasesWithDistanceOutToThousandsOfKilometres)
{
    const Mat4d projection = reverseZInfinitePerspective(degreesToRadians(70.0), 1.0, 0.05);
    double      previous   = 1.0;
    double      distance   = 0.1;
    for (int i = 0; i < 40; ++i, distance *= 1.5)  // 0.1 m to beyond 1000 km
    {
        const auto depth = static_cast<float>(depthAt(projection, distance));  // as stored in D32F
        EXPECT_LT(depth, previous) << "at " << distance << " m";
        previous = static_cast<double>(depth);
    }
}

TEST(CameraRelative, StaysMillimetrePreciseFarFromTheOrigin)
{
    for (const double offset : {0.0, 1000.0, 16000.0, 1.0e6})
    {
        const Vec3d camera(offset, -offset, 1.7);
        const Vec3d point    = camera + Vec3d(0.1234, 2.0, -0.0987);
        const Vec3f relative = cameraRelative(point, camera);
        expectNear(Vec3d(relative), Vec3d(0.1234, 2.0, -0.0987), 1e-6);
    }
}

TEST(LookRig, DefaultLooksNorthWithRightToTheEast)
{
    const LookRig look;
    expectNear(look.forward(), Vec3d(0.0, 1.0, 0.0), 1e-12);
    expectNear(look.right(), Vec3d(1.0, 0.0, 0.0), 1e-12);
    expectNear(look.up(), Vec3d(0.0, 0.0, 1.0), 1e-12);
}

TEST(LookRig, PositiveYawTurnsLeftAndPositivePitchLooksUp)
{
    LookRig look;
    look.setAngles(degreesToRadians(90.0), 0.0);
    expectNear(look.forward(), Vec3d(-1.0, 0.0, 0.0), 1e-12);

    look.setAngles(0.0, degreesToRadians(45.0));
    expectNear(look.forward(), Vec3d(0.0, std::sqrt(0.5), std::sqrt(0.5)), 1e-12);
    expectNear(look.horizontalForward(), Vec3d(0.0, 1.0, 0.0), 1e-12);
}

TEST(LookRig, PitchIsClampedShortOfStraightUp)
{
    LookRig look;
    look.applyLook(0.0, degreesToRadians(200.0));
    EXPECT_DOUBLE_EQ(look.pitch(), LookRig::kMaxPitch);
    look.applyLook(0.0, degreesToRadians(-400.0));
    EXPECT_DOUBLE_EQ(look.pitch(), -LookRig::kMaxPitch);
}

TEST(LookRig, OrientationMapsCameraAxesToWorldDirections)
{
    LookRig look;
    look.setAngles(degreesToRadians(30.0), degreesToRadians(-20.0));
    const auto orientation = look.orientation();
    expectNear(orientation * Vec3d(0.0, 0.0, -1.0), look.forward(), 1e-12);
    expectNear(orientation * Vec3d(1.0, 0.0, 0.0), look.right(), 1e-12);
}

TEST(LookRig, WorksWithAnyUpVector)
{
    // Standing inside a spin habitat at +X: up points toward the axis (-X), north is the axis (+Z).
    LookRig look(Vec3d(-1.0, 0.0, 0.0), Vec3d(0.0, 0.0, 1.0));
    expectNear(look.forward(), Vec3d(0.0, 0.0, 1.0), 1e-12);
    look.setAngles(0.0, degreesToRadians(60.0));  // looking up, toward the axis
    expectNear(look.forward(),
               Vec3d(-std::sin(degreesToRadians(60.0)), 0.0, std::cos(degreesToRadians(60.0))),
               1e-12);

    // Changing the frame keeps yaw and pitch.
    look.setAngles(degreesToRadians(10.0), degreesToRadians(5.0));
    look.setFrame(Vec3d(0.0, -1.0, 0.0), Vec3d(0.0, 0.0, 1.0));
    EXPECT_NEAR(look.yaw(), degreesToRadians(10.0), 1e-12);
    EXPECT_NEAR(look.pitch(), degreesToRadians(5.0), 1e-12);
    EXPECT_NEAR(glm::dot(look.forward(), look.up()), std::sin(degreesToRadians(5.0)), 1e-12);
}

TEST(Camera, ViewProjectionPutsPointsAheadInTheCentre)
{
    Camera camera;
    camera.position    = Vec3d(1.0e6, 0.0, 1.7);
    camera.orientation = LookRig().orientation();
    const Mat4d viewProjection =
        StarshipSimulator::cameraRelativeViewProjection(camera, 16.0 / 9.0);
    const Vec4d clip = viewProjection * Vec4d(Vec3d(0.0, 10.0, 0.0), 1.0);  // 10 m north
    EXPECT_NEAR(clip.x / clip.w, 0.0, 1e-12);
    EXPECT_NEAR(clip.y / clip.w, 0.0, 1e-12);
    EXPECT_NEAR(clip.z / clip.w, camera.nearPlaneMeters / 10.0, 1e-12);
}

}  // namespace
