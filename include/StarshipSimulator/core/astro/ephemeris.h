#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/math.h"

// Where things are in the solar system, via Astronomy Engine. Everything is in the J2000 equatorial
// frame (EQJ, aligned with ICRF): +X toward the March equinox, +Z toward the celestial north pole.
// Positions are heliocentric, in astronomical units.
namespace StarshipSimulator::astro
{

inline constexpr double kKmPerAu = 149597870.7;

enum class Body : std::uint8_t
{
    SUN,
    MERCURY,
    VENUS,
    EARTH,
    MOON,
    MARS,
    JUPITER,
    SATURN,
    URANUS,
    NEPTUNE,
};

[[nodiscard]] const char*         bodyName(Body body);
[[nodiscard]] std::optional<Body> bodyFromName(std::string_view name);  // any letter case
[[nodiscard]] double              bodyRadiusKm(Body body);

[[nodiscard]] Vec3d heliocentricPosition(Body body, SimTime time);

/// Brightness as seen from near Earth (fine for the planets anywhere in cislunar space).
[[nodiscard]] double visualMagnitude(Body body, SimTime time);

/// A body's rotation: its north pole, and a rotation taking EQJ directions to body-fixed
/// coordinates
/// (+Z north, +X through the prime meridian, e.g. Greenwich), for texturing and day/night.
struct BodyOrientation
{
    Vec3d north{0.0, 0.0, 1.0};
    Mat3d bodyFromEqj{1.0};
};

[[nodiscard]] BodyOrientation bodyOrientation(Body body, SimTime time);

/// The constellation containing a direction (EQJ), e.g. "Canis Major".
[[nodiscard]] std::string constellationAt(const Vec3d& directionEqj);

// ---- Where the habitat is ----------------------------------------------------------------------

enum class Location : std::uint8_t
{
    EARTH_MOON_L4,  // 60 degrees ahead of the Moon in its orbit, 384,000 km from Earth
    EARTH_MOON_L5,  // 60 degrees behind the Moon: O'Neill's proposed site
    SUN_EARTH_L4,
    SUN_EARTH_L5,
    SUN_MARS_L4,
    SUN_MARS_L5,
};

[[nodiscard]] const char*             locationKey(Location location);   // "earth_moon_l5"
[[nodiscard]] const char*             locationName(Location location);  // "Earth-Moon L5"
[[nodiscard]] std::optional<Location> locationFromKey(std::string_view key);
[[nodiscard]] std::vector<Location>   allLocations();

/// Heliocentric position of a Lagrange point.
[[nodiscard]] Vec3d locationPosition(Location location, SimTime time);

// ---- The view from the habitat -----------------------------------------------------------------

/// A body as seen from the habitat.
struct VisibleBody
{
    Body   body = Body::EARTH;
    Vec3d  direction{0.0, 0.0, 1.0};  // unit, EQJ
    double distanceKm    = 0.0;
    double angularRadius = 0.0;       // radians
    Vec3d  towardSun{0.0, 0.0, 1.0};  // unit, EQJ, from the body toward the Sun (lighting)
    Mat3d  bodyFromEqj{1.0};
    double magnitude = 0.0;
};

/// Everything the sky needs for one moment: where we are, where the Sun is, and the other bodies.
struct SkyState
{
    SimTime                  time;
    Location                 location = Location::EARTH_MOON_L5;
    Vec3d                    positionAu{0.0};
    Vec3d                    sunDirection{0.0, 0.0, 1.0};  // unit, EQJ
    double                   sunDistanceAu = 1.0;
    std::vector<VisibleBody> bodies;  // Earth, Moon and the planets
};

[[nodiscard]] SkyState computeSky(Location location, SimTime time);

/// The habitat's orientation: its spin axis (+Z) points at the Sun; at spin phase 0 its +X lies
/// along the ecliptic (perpendicular to the ecliptic north pole). Returns the rotation taking EQJ
/// directions into the (spinning) habitat frame.
[[nodiscard]] Mat3d habitatFromEqj(const Vec3d& sunDirection, double spinPhase);

}  // namespace StarshipSimulator::astro
