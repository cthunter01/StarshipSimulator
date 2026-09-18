#include "StarshipSimulator/core/gpu_abi/uniforms.h"

#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
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
    namespace astro            = StarshipSimulator::astro;
    const Vec3d            sun = glm::normalize(Vec3d(0.3, 0.9, 0.4));
    const Vec4f            star(0.6F, -0.48F, 0.64F, 0.0F);
    const gpu::SkyUniforms at0 = gpu::makeSkyUniforms(astro::habitatFromEqj(sun, 0.0), 1.0, 0.5);
    const gpu::SkyUniforms later =
        gpu::makeSkyUniforms(astro::habitatFromEqj(sun, degreesToRadians(90.0)), 1.0, 0.5);
    EXPECT_FLOAT_EQ(later.params.x, 1.0F);
    EXPECT_FLOAT_EQ(later.params.y, 0.5F);
    // After the habitat turned a quarter turn about +Z, a fixed star has turned a quarter turn
    // backwards: (x, y) -> (y, -x).
    const Vec4f before = at0.habitatFromInertial * star;
    const Vec4f after  = later.habitatFromInertial * star;
    EXPECT_NEAR(after.x, before.y, 1e-6F);
    EXPECT_NEAR(after.y, -before.x, 1e-6F);
    EXPECT_NEAR(after.z, before.z, 1e-6F);
}

TEST(BodyUniforms, EarthSeenFromL5)
{
    namespace astro           = StarshipSimulator::astro;
    const astro::SkyState sky = astro::computeSky(
        astro::Location::EarthMoonL5, astro::parseIsoTime("2045-06-15T09:00:00Z").value());
    const StarshipSimulator::Mat3d habitatFromInertial =
        astro::habitatFromEqj(sky.sunDirection, 0.3);
    for (const astro::VisibleBody& body : sky.bodies)
    {
        if (body.body != astro::Body::Earth)
        {
            continue;
        }
        const gpu::BodyUniforms earth = gpu::makeBodyUniforms(body, habitatFromInertial, {});
        EXPECT_NEAR(glm::length(StarshipSimulator::Vec3f(earth.direction)), 1.0F, 1e-6F);
        EXPECT_NEAR(earth.direction.w, body.angularRadius, 1e-7);
        EXPECT_GT(earth.params.x, 0.0F);  // atmosphere
        // Body-fixed coordinates: the direction from Earth toward the Sun has a small latitude
        // (below the tropics, 23.4 degrees).
        const Vec4f sunInBody =
            earth.bodyFromHabitat * Vec4f(StarshipSimulator::Vec3f(earth.towardSun), 0.0F);
        EXPECT_LT(std::abs(sunInBody.z), std::sin(degreesToRadians(23.5)));
    }
}

TEST(PlanetUniforms, CarriesTheSevenPlanets)
{
    namespace astro           = StarshipSimulator::astro;
    const astro::SkyState sky = astro::computeSky(
        astro::Location::EarthMoonL5, astro::parseIsoTime("2045-06-15T09:00:00Z").value());
    const gpu::PlanetUniforms planets = gpu::makePlanetUniforms(sky);
    EXPECT_EQ(planets.count.x, 7.0F);
    for (std::size_t i = 0; i < 7; ++i)
    {
        EXPECT_NEAR(glm::length(StarshipSimulator::Vec3f(planets.stars.at(2 * i))), 1.0F, 1e-5F);
        EXPECT_GT(planets.stars.at((2 * i) + 1).y, 0.0F);
    }
}

}  // namespace
