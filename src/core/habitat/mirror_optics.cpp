#include "StarshipSimulator/core/habitat/mirror_optics.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kTwilight = degreesToRadians(4.0);
// How high the sun images through the end caps stand above the floor at the near end: the
// mirrors outside can only fold the light in so steeply, and at dusk no lower than this.
constexpr double kEndCapHighest = degreesToRadians(40.0);
constexpr double kEndCapLowest  = degreesToRadians(15.0);
// A sphere's polar windows: from the equator an image beyond a pole shows through the window only
// while it stands higher than half the window's latitude above the rim (an inscribed angle), so the
// light is held a little above that, and at most this high.
constexpr double kPolarHighest = degreesToRadians(45.0);
constexpr double kPolarMargin  = degreesToRadians(3.0);

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

double endCapElevation(double openingAngle)
{
    return std::clamp(sunElevation(openingAngle), kEndCapLowest, kEndCapHighest);
}

double polarWindowElevation(double openingAngle, double windowLatitudeDeg)
{
    const double lowest  = (0.5 * degreesToRadians(windowLatitudeDeg)) + kPolarMargin;
    const double highest = std::max(kPolarHighest, lowest + degreesToRadians(5.0));
    return std::clamp(sunElevation(openingAngle), lowest, highest);
}

double daylightFactor(double openingAngle)
{
    return glm::smoothstep(0.0, kTwilight, openingAngle) *
           (1.0 - glm::smoothstep((kPi / 2.0) - kTwilight, kPi / 2.0, openingAngle));
}

namespace
{

/// The two sun images through a windowless cylinder's glass end caps, or through a sphere's polar
/// windows: points on the axis beyond the glass, as high above its rim as the mirrors put them.
std::vector<SunBeam> endCapBeams(const HabitatGeometry& geometry, double openingAngle)
{
    const HabitatSpec& spec      = geometry.spec();
    const double       intensity = daylightFactor(openingAngle) * spec.mirrors.reflectivity;
    const bool         sphere    = geometry.kind() == HabitatKind::BERNAL_SPHERE;
    const double       rim =
        sphere ? geometry.floorRadiusAt(geometry.profile().zMax()) : geometry.radius();
    const double elevation = sphere
                                 ? polarWindowElevation(openingAngle, spec.sphere.windowLatitudeDeg)
                                 : endCapElevation(openingAngle);
    const double beyond    = rim / std::tan(elevation);
    std::vector<SunBeam> beams;
    for (int end = 0; end < 2; ++end)
    {
        const double sign  = end == 0 ? 1.0 : -1.0;
        const double glass = end == 0 ? geometry.profile().zMax() : geometry.profile().zMin();
        beams.push_back({.window    = end,
                         .towardSun = Vec3d(0.0, 0.0, sign),
                         .intensity = intensity,
                         .image     = Vec3d(0.0, 0.0, glass + (sign * beyond))});
    }
    return beams;
}

}  // namespace

std::vector<SunBeam> sunBeams(const HabitatGeometry& geometry, double openingAngle)
{
    const DaylightKind daylight = daylightKindFor(geometry.kind());
    if (daylight == DaylightKind::END_CAPS || daylight == DaylightKind::POLAR_WINDOWS)
    {
        return endCapBeams(geometry, openingAngle);
    }
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

Vec3d towardSunFrom(const SunBeam& beam, const Vec3d& p)
{
    return beam.image ? glm::normalize(*beam.image - p) : beam.towardSun;
}

double beamReach(const HabitatGeometry& geometry, const Vec3d& p, const SunBeam& beam)
{
    if (beam.image && geometry.kind() == HabitatKind::BERNAL_SPHERE)
    {
        // The sphere is convex and its walls opaque: the ray toward the image leaves it once, and
        // the light gets in if that is through the polar window on the image's side.
        const Vec3d  d    = towardSunFrom(beam, p);
        const double r    = geometry.radius();
        const double b    = glm::dot(p, d);
        const double c    = glm::dot(p, p) - (r * r);
        const double disc = (b * b) - c;
        if (disc < 0.0)
        {
            return 0.0;
        }
        const Vec3d  exit   = p + (d * std::max(-b + std::sqrt(disc), 0.0));
        const double rimZ   = geometry.profile().zMax();
        const double toward = beam.image->z > 0.0 ? exit.z : -exit.z;
        return glm::smoothstep(rimZ - 2.0, rimZ + 2.0, toward);
    }
    if (beam.image)
    {
        // Seen from anywhere inside, an image on the axis beyond the glass shines through it.
        return p.z >= geometry.profile().zMin() && p.z <= geometry.profile().zMax() ? 1.0 : 0.0;
    }
    const Vec3d& s = beam.towardSun;
    const double a = (s.x * s.x) + (s.y * s.y);
    if (a < 1e-8)
    {
        return 0.0;
    }
    const double r    = geometry.radius();
    const double b    = (p.x * s.x) + (p.y * s.y);
    const double c    = (p.x * p.x) + (p.y * p.y) - (r * r);
    const double disc = (b * b) - (a * c);
    if (disc < 0.0)
    {
        return 0.0;
    }
    const Vec3d  exit   = p + (s * ((-b + std::sqrt(disc)) / a));
    const double off    = HabitatGeometry::angularDistance(HabitatGeometry::angleOf(exit),
                                                           geometry.windowCenter(beam.window));
    const double half   = geometry.windowHalfAngle();
    const double across = 1.0 - glm::smoothstep(half - 0.004, half, off);
    const double along =
        glm::smoothstep(geometry.floorZMin(), geometry.floorZMin() + 40.0, exit.z) *
        (1.0 - glm::smoothstep(geometry.floorZMax() - 40.0, geometry.floorZMax(), exit.z));
    return across * along;
}

std::optional<int> dominantBeam(const HabitatGeometry& geometry, double openingAngle,
                                const Vec3d& p)
{
    const Vec3d        up = HabitatGeometry::localUp(p);
    std::optional<int> best;
    double             strongest = 0.0;
    for (const SunBeam& beam : sunBeams(geometry, openingAngle))
    {
        const double light = beam.intensity * beamReach(geometry, p, beam) *
                             std::max(glm::dot(up, towardSunFrom(beam, p)), 0.05);
        if (light > strongest)
        {
            strongest = light;
            best      = beam.window;
        }
    }
    return best;
}

}  // namespace StarshipSimulator
