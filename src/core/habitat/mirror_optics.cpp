#include "StarshipSimulator/core/habitat/mirror_optics.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kTwilight = degreesToRadians(4.0);

Vec3d outward(double angle)
{
    return {std::cos(angle), std::sin(angle), 0.0};
}

}  // namespace

Vec3d beamTravelDirection(double windowAngle, double openingAngle)
{
    // Sunlight travels along -Z; reflecting it off a mirror tilted by alpha turns it by 2 alpha.
    const double twice = 2.0 * openingAngle;
    return (-std::sin(twice) * outward(windowAngle)) - Vec3d(0.0, 0.0, std::cos(twice));
}

Vec3d apparentSunDirection(double windowAngle, double openingAngle)
{
    return -beamTravelDirection(windowAngle, openingAngle);
}

Vec3d mirrorNormal(double windowAngle, double openingAngle)
{
    return (-std::cos(openingAngle) * outward(windowAngle)) +
           Vec3d(0.0, 0.0, std::sin(openingAngle));
}

Vec3d mirrorDirection(double windowAngle, double openingAngle)
{
    return (std::sin(openingAngle) * outward(windowAngle)) +
           Vec3d(0.0, 0.0, std::cos(openingAngle));
}

double sunElevation(double openingAngle)
{
    const double twice = 2.0 * openingAngle;
    return std::atan2(std::sin(twice), std::abs(std::cos(twice)));
}

double daylightFactor(double openingAngle)
{
    return glm::smoothstep(0.0, kTwilight, openingAngle) *
           (1.0 - glm::smoothstep((kPi / 2.0) - kTwilight, kPi / 2.0, openingAngle));
}

std::vector<SunBeam> sunBeams(const HabitatGeometry& geometry, double openingAngle)
{
    std::vector<SunBeam> beams;
    beams.reserve(static_cast<std::size_t>(geometry.stripCount()));
    const double intensity = daylightFactor(openingAngle) * geometry.spec().mirrors.reflectivity;
    for (int i = 0; i < geometry.stripCount(); ++i)
    {
        beams.push_back({.window    = i,
                         .towardSun = apparentSunDirection(geometry.windowCenter(i), openingAngle),
                         .intensity = intensity});
    }
    return beams;
}

}  // namespace StarshipSimulator
