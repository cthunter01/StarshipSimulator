#include "StarshipSimulator/core/gpu_abi/uniforms.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/star_catalog.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/landscape.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/terrain_lod.h"
#include "StarshipSimulator/core/units.h"

namespace StarshipSimulator::gpu
{

namespace
{

constexpr double kSunAngularRadius = degreesToRadians(0.2666);  // at 1 AU
// Warm white sunlight after the mirror and glass; the ambient light from the sunlit far side and
// the glowing windows overhead, and light bounced up from the ground.
constexpr Vec3d kSunColor(1.0, 0.94, 0.84);
constexpr Vec3d kFarSideColor(0.30, 0.36, 0.28);
constexpr Vec3d kGroundBounce(0.10, 0.10, 0.07);

/// Colour temperatures that give the planets their familiar tints as points of light.
double planetColorKelvin(astro::Body body)
{
    switch (body)
    {
        case astro::Body::Mars:
            return 3300.0;  // ochre
        case astro::Body::Jupiter:
        case astro::Body::Saturn:
            return 5000.0;  // cream
        case astro::Body::Uranus:
        case astro::Body::Neptune:
            return 9000.0;  // blue-green
        default:
            return 5800.0;
    }
}

}  // namespace

FrameUniforms makeFrameUniforms(const Camera& camera, std::uint32_t width, std::uint32_t height,
                                double animationSeconds)
{
    const double safeWidth      = std::max(1.0, static_cast<double>(width));
    const double safeHeight     = std::max(1.0, static_cast<double>(height));
    const Mat4d  viewProjection = cameraRelativeViewProjection(camera, safeWidth / safeHeight);

    FrameUniforms frame;
    frame.viewProjection        = Mat4f(viewProjection);
    frame.inverseViewProjection = Mat4f(glm::inverse(viewProjection));
    frame.cameraPosition        = Vec4f(Vec4d(camera.position, camera.nearPlaneMeters));
    frame.viewport = Vec4f(Vec4d(safeWidth, safeHeight, 1.0 / safeWidth, 1.0 / safeHeight));
    frame.time = Vec4f(static_cast<float>(std::fmod(animationSeconds, 3600.0)), 0.0F, 0.0F, 0.0F);
    return frame;
}

HabitatUniforms makeHabitatUniforms(const HabitatGeometry& geometry, double openingAngle,
                                    const LightingSettings& lighting)
{
    const OneillCylinderSpec& spec     = geometry.spec();
    const double              daylight = daylightFactor(openingAngle);

    HabitatUniforms habitat;
    habitat.shape     = Vec4f(Vec4d(geometry.radius(), geometry.floorZMin(), geometry.floorZMax(),
                                    geometry.windowHalfAngle()));
    habitat.strips    = Vec4f(Vec4d(geometry.stripAngle(), geometry.stripCount(),
                                    geometry.profile().zMin(), geometry.profile().zMax()));
    std::size_t index = 0;
    for (const SunBeam& beam : sunBeams(geometry, openingAngle))
    {
        if (index < kMaxSunBeams)
        {
            habitat.beams.at(index) = Vec4f(Vec4d(beam.towardSun, beam.intensity));
            ++index;
        }
    }
    habitat.sunColor    = Vec4f(Vec4d(kSunColor * lighting.sunIntensity, 1.0));
    habitat.ambientUp   = Vec4f(Vec4d(kFarSideColor * (lighting.ambient * daylight), 0.0));
    habitat.ambientDown = Vec4f(Vec4d(kGroundBounce * (lighting.ambient * daylight), 0.0));

    // Isothermal air in spin gravity: density ~ exp(-k (R^2 - r^2)).
    const double omega = geometry.omega();
    const double falloff =
        (omega * omega) / (2.0 * units::kSpecificGasConstantAir * spec.atmosphere.temperatureK);
    const double pressure = spec.atmosphere.surfacePressurePa / units::kStandardAtmosphere;
    habitat.atmosphere    = Vec4f(Vec4d(falloff, pressure, lighting.haze, daylight));
    habitat.mirror        = Vec4f(Vec4d(openingAngle, geometry.floorZMax() - geometry.floorZMin(),
                                        geometry.radius() * std::sin(geometry.windowHalfAngle()),
                                        geometry.floorZMin()));
    habitat.sun           = Vec4f(0.0F, 0.0F, 1.0F, static_cast<float>(kSunAngularRadius));
    return habitat;
}

SkyUniforms makeSkyUniforms(const Mat3d& habitatFromInertial, double starBrightness,
                            double milkyWayBrightness)
{
    SkyUniforms sky;
    sky.habitatFromInertial = Mat4f(Mat4d(habitatFromInertial));
    sky.params              = Vec4f(Vec4d(starBrightness, milkyWayBrightness, 0.0, 0.0));
    return sky;
}

LandscapeUniforms makeLandscapeUniforms(const TerrainGrid&               grid,
                                        const std::vector<TerrainMorph>& morphs)
{
    const TerrainGridLayout& layout = grid.layout;
    LandscapeUniforms        uniforms;
    uniforms.grid    = Vec4f(Vec4d(layout.columns, layout.rows(), layout.cellU,
                                   2.0 * kPi / static_cast<double>(layout.columns)));
    uniforms.extent  = Vec4f(grid.zMin, grid.zMax, 0.0F, 0.0F);
    uniforms.heights = Vec4f(grid.heightMin, grid.heightMin + grid.heightRange,
                             static_cast<float>(kWaterLevelM), static_cast<float>(layout.radiusM));
    for (std::size_t level = 0; level < std::min(morphs.size(), kMaxTerrainLevels); ++level)
    {
        const TerrainMorph& m    = morphs[level];
        const double        band = m.end - m.start;
        uniforms.morph.at(level) = Vec4f(Vec4d(m.start, m.end, band > 0.0 ? 1.0 / band : 0.0, 0.0));
    }
    return uniforms;
}

ShadowUniforms makeShadowUniforms(const Vec3d& cameraPosition, const Vec3d& towardSun, int window,
                                  double halfExtentM, std::uint32_t resolution)
{
    constexpr double kDepthHalfRangeM = 1500.0;  // casters this far toward the sun still count
    const Vec3d      travel           = -glm::normalize(towardSun);  // the way the light goes
    const Vec3d helper = std::abs(travel.z) < 0.9 ? Vec3d(0.0, 0.0, 1.0) : Vec3d(1.0, 0.0, 0.0);
    const Vec3d right  = glm::normalize(glm::cross(travel, helper));
    const Vec3d up     = glm::cross(right, travel);

    // Keep the texel grid fixed in the habitat: shift the box by the camera's sub-texel offset.
    const double texel  = 2.0 * halfExtentM / static_cast<double>(resolution);
    const double alongR = glm::dot(cameraPosition, right);
    const double alongU = glm::dot(cameraPosition, up);
    const Vec3d  centre = -((alongR - (std::floor(alongR / texel) * texel)) * right) -
                          ((alongU - (std::floor(alongU / texel) * texel)) * up);

    const Mat4d view       = glm::lookAt(centre - (travel * kDepthHalfRangeM), centre, up);
    const Mat4d projection = glm::ortho(-halfExtentM, halfExtentM, -halfExtentM, halfExtentM, 0.0,
                                        2.0 * kDepthHalfRangeM);
    ShadowUniforms shadow;
    shadow.lightFromCameraRelative = Mat4f(projection * view);
    shadow.params = Vec4f(Vec4d(1.0, window, texel, 0.6 / (2.0 * kDepthHalfRangeM)));
    return shadow;
}

BodyUniforms makeBodyUniforms(const astro::VisibleBody& body, const Mat3d& habitatFromInertial,
                              const LightingSettings& lighting)
{
    const bool   earth = body.body == astro::Body::Earth;
    BodyUniforms uniforms;
    uniforms.direction =
        Vec4f(Vec4d(glm::normalize(habitatFromInertial * body.direction), body.angularRadius));
    uniforms.towardSun = Vec4f(Vec4d(glm::normalize(habitatFromInertial * body.towardSun), 0.0));
    // The same Sun that lights the habitat (before the mirrors) lights Earth and the Moon. NASA's
    // Earth image averages an albedo of 0.36, about Earth's own; it is dimmed a little so the
    // clouds keep some detail. The Moon map averages 0.30, the Moon itself only 0.12.
    uniforms.sunlight = Vec4f(Vec4d(kSunColor * lighting.sunIntensity, earth ? 0.6 : 0.4));
    uniforms.params   = earth ? Vec4f(1.0F, 0.02F, 0.0F, 0.0F) : Vec4f(0.0F, 0.0F, 0.0015F, 0.0F);
    uniforms.bodyFromHabitat = Mat4f(Mat4d(body.bodyFromEqj * glm::transpose(habitatFromInertial)));
    return uniforms;
}

PlanetUniforms makePlanetUniforms(const astro::SkyState& sky)
{
    PlanetUniforms planets;
    std::size_t    count = 0;
    for (const astro::VisibleBody& body : sky.bodies)
    {
        if (body.body == astro::Body::Earth || body.body == astro::Body::Moon ||
            count >= kMaxPlanets)
        {
            continue;
        }
        const GpuStar star =
            astro::gpuStar(body.direction, body.magnitude, planetColorKelvin(body.body));
        planets.stars.at(2 * count)       = star.direction;
        planets.stars.at((2 * count) + 1) = star.color;
        ++count;
    }
    planets.count = Vec4f(static_cast<float>(count), 0.0F, 0.0F, 0.0F);
    return planets;
}

}  // namespace StarshipSimulator::gpu
