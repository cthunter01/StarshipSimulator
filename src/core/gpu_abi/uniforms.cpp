#include "StarshipSimulator/core/gpu_abi/uniforms.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/math.h"
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

}  // namespace

FrameUniforms makeFrameUniforms(const Camera& camera, std::uint32_t width, std::uint32_t height)
{
    const double safeWidth      = std::max(1.0, static_cast<double>(width));
    const double safeHeight     = std::max(1.0, static_cast<double>(height));
    const Mat4d  viewProjection = cameraRelativeViewProjection(camera, safeWidth / safeHeight);

    FrameUniforms frame;
    frame.viewProjection        = Mat4f(viewProjection);
    frame.inverseViewProjection = Mat4f(glm::inverse(viewProjection));
    frame.cameraPosition        = Vec4f(Vec4d(camera.position, camera.nearPlaneMeters));
    frame.viewport = Vec4f(Vec4d(safeWidth, safeHeight, 1.0 / safeWidth, 1.0 / safeHeight));
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

SkyUniforms makeSkyUniforms(double spinPhase, double brightness)
{
    SkyUniforms sky;
    // The habitat turns by +phase about +Z, so fixed stars appear turned by -phase.
    sky.habitatFromInertial = Mat4f(glm::rotate(Mat4d(1.0), -spinPhase, Vec3d(0.0, 0.0, 1.0)));
    sky.params              = Vec4f(static_cast<float>(brightness), 0.0F, 0.0F, 0.0F);
    return sky;
}

}  // namespace StarshipSimulator::gpu
