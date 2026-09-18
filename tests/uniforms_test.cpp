#include "StarshipSimulator/core/gpu_abi/uniforms.h"

#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace
{

namespace gpu = StarshipSimulator::gpu;
using StarshipSimulator::Camera;
using StarshipSimulator::degreesToRadians;
using StarshipSimulator::HabitatGeometry;
using StarshipSimulator::Mat4f;
using StarshipSimulator::OneillCylinderSpec;
using StarshipSimulator::Vec3d;
using StarshipSimulator::Vec4f;

TEST(FrameUniforms, InverseViewProjectionUndoesViewProjection)
{
    Camera camera;
    camera.orientation               = glm::angleAxis(0.7, glm::normalize(Vec3d(0.2, 0.3, 1.0)));
    const gpu::FrameUniforms frame   = gpu::makeFrameUniforms(camera, 1280, 720);
    const Mat4f              product = frame.viewProjection * frame.inverseViewProjection;
    for (int column = 0; column < 4; ++column)
    {
        for (int row = 0; row < 4; ++row)
        {
            EXPECT_NEAR(product[column][row], column == row ? 1.0F : 0.0F, 1e-5F);
        }
    }
}

TEST(FrameUniforms, ViewportAndCamera)
{
    Camera camera;
    camera.position                = Vec3d(-3998.3, 12.5, -2000.0);
    const gpu::FrameUniforms frame = gpu::makeFrameUniforms(camera, 1600, 900);
    EXPECT_FLOAT_EQ(frame.viewport.x, 1600.0F);
    EXPECT_FLOAT_EQ(frame.viewport.w, 1.0F / 900.0F);
    EXPECT_NEAR(static_cast<double>(frame.cameraPosition.x), -3998.3, 1e-3);
    EXPECT_FLOAT_EQ(frame.cameraPosition.w, static_cast<float>(camera.nearPlaneMeters));

    const gpu::FrameUniforms empty = gpu::makeFrameUniforms(Camera{}, 0, 0);  // minimized window
    EXPECT_TRUE(std::isfinite(empty.viewProjection[0][0]));
}

TEST(HabitatUniforms, DescribeShapeLightAndAir)
{
    const HabitatGeometry      geometry{OneillCylinderSpec{}};
    const gpu::HabitatUniforms habitat =
        gpu::makeHabitatUniforms(geometry, degreesToRadians(60.0), gpu::LightingSettings{});
    EXPECT_FLOAT_EQ(habitat.shape.x, 4000.0F);
    EXPECT_FLOAT_EQ(habitat.strips.y, 3.0F);
    for (int i = 0; i < 3; ++i)
    {
        const Vec4f beam = habitat.beams.at(static_cast<std::size_t>(i));
        EXPECT_NEAR(beam.w, 0.9F, 1e-6F);
        EXPECT_NEAR(beam.z, -0.5F, 1e-6F);  // afternoon: sun toward the anti-sunward end
    }
    EXPECT_EQ(habitat.beams.at(3).w, 0.0F);
    // Density falls to 0.792 at the axis: k R^2 = 0.233.
    EXPECT_NEAR(static_cast<double>(habitat.atmosphere.x) * 4000.0 * 4000.0, 0.2332, 1e-3);
    EXPECT_FLOAT_EQ(habitat.atmosphere.w, 1.0F);  // full daylight

    const gpu::HabitatUniforms night =
        gpu::makeHabitatUniforms(geometry, degreesToRadians(100.0), gpu::LightingSettings{});
    EXPECT_EQ(night.beams.at(0).w, 0.0F);
    EXPECT_EQ(night.atmosphere.w, 0.0F);
}

TEST(SkyUniforms, StarsTurnAgainstTheSpin)
{
    const gpu::SkyUniforms sky = gpu::makeSkyUniforms(degreesToRadians(90.0), 1.0);
    // A star toward +x appears toward -y after the habitat turned a quarter turn.
    const Vec4f star = sky.habitatFromInertial * Vec4f(1.0F, 0.0F, 0.0F, 0.0F);
    EXPECT_NEAR(star.x, 0.0F, 1e-6F);
    EXPECT_NEAR(star.y, -1.0F, 1e-6F);
}

}  // namespace
