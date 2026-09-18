#include "StarshipSimulator/core/gpu_abi/uniforms.h"

#include <cmath>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/math.h"

namespace
{

namespace gpu = StarshipSimulator::gpu;
using StarshipSimulator::Camera;
using StarshipSimulator::Mat4f;
using StarshipSimulator::Vec3d;

/// True if a and b differ by a whole number of grid periods (within tolerance).
bool congruent(double a, double b, double tolerance)
{
    const double remainder = std::remainder(a - b, gpu::kGridPeriodMeters);
    return std::abs(remainder) < tolerance;
}

TEST(FrameUniforms, GridOriginIsPreciseThousandsOfKilometresOut)
{
    Camera camera;
    camera.position                = Vec3d(1.0e6 + 12.5, -3.0e6 + 7.25, 1.7);
    const gpu::FrameUniforms frame = gpu::makeFrameUniforms(camera, 1920, 1080);
    EXPECT_TRUE(congruent(static_cast<double>(frame.gridOrigin.x), 12.5, 1e-3));
    EXPECT_TRUE(congruent(static_cast<double>(frame.gridOrigin.y), 7.25, 1e-3));
    EXPECT_NEAR(static_cast<double>(frame.gridOrigin.z), 1.7, 1e-6);
    EXPECT_FLOAT_EQ(frame.gridOrigin.w, static_cast<float>(camera.nearPlaneMeters));
}

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

TEST(FrameUniforms, ViewportHoldsSizeAndReciprocal)
{
    const gpu::FrameUniforms frame = gpu::makeFrameUniforms(Camera{}, 1600, 900);
    EXPECT_FLOAT_EQ(frame.viewport.x, 1600.0F);
    EXPECT_FLOAT_EQ(frame.viewport.y, 900.0F);
    EXPECT_FLOAT_EQ(frame.viewport.z, 1.0F / 1600.0F);
    EXPECT_FLOAT_EQ(frame.viewport.w, 1.0F / 900.0F);

    const gpu::FrameUniforms empty = gpu::makeFrameUniforms(Camera{}, 0, 0);  // minimized window
    EXPECT_TRUE(std::isfinite(empty.viewProjection[0][0]));
}

}  // namespace
