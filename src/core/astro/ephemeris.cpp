#include "StarshipSimulator/core/astro/ephemeris.h"

#include <astronomy.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator::astro
{

namespace
{

constexpr double kObliquity = degreesToRadians(23.4392911);  // J2000 mean obliquity of the ecliptic

astro_body_t toAstronomy(Body body)
{
    switch (body)
    {
        case Body::SUN:
            return BODY_SUN;
        case Body::MERCURY:
            return BODY_MERCURY;
        case Body::VENUS:
            return BODY_VENUS;
        case Body::EARTH:
            return BODY_EARTH;
        case Body::MOON:
            return BODY_MOON;
        case Body::MARS:
            return BODY_MARS;
        case Body::JUPITER:
            return BODY_JUPITER;
        case Body::SATURN:
            return BODY_SATURN;
        case Body::URANUS:
            return BODY_URANUS;
        case Body::NEPTUNE:
            return BODY_NEPTUNE;
    }
    return BODY_EARTH;
}

astro_time_t astronomyTime(SimTime time)
{
    return Astronomy_TimeFromDays(time.days());
}

Vec3d toVec3(const astro_vector_t& v)
{
    return {v.x, v.y, v.z};
}

struct LagrangeSystem
{
    int         point;
    Body        major;
    Body        minor;
    const char* key;
    const char* name;
};

LagrangeSystem systemOf(Location location)
{
    switch (location)
    {
        case Location::EARTH_MOON_L4:
            return {.point = 4,
                    .major = Body::EARTH,
                    .minor = Body::MOON,
                    .key   = "earth_moon_l4",
                    .name  = "Earth-Moon L4"};
        case Location::EARTH_MOON_L5:
            return {.point = 5,
                    .major = Body::EARTH,
                    .minor = Body::MOON,
                    .key   = "earth_moon_l5",
                    .name  = "Earth-Moon L5"};
        case Location::SUN_EARTH_L4:
            return {.point = 4,
                    .major = Body::SUN,
                    .minor = Body::EARTH,
                    .key   = "sun_earth_l4",
                    .name  = "Sun-Earth L4"};
        case Location::SUN_EARTH_L5:
            return {.point = 5,
                    .major = Body::SUN,
                    .minor = Body::EARTH,
                    .key   = "sun_earth_l5",
                    .name  = "Sun-Earth L5"};
        case Location::SUN_MARS_L4:
            return {.point = 4,
                    .major = Body::SUN,
                    .minor = Body::MARS,
                    .key   = "sun_mars_l4",
                    .name  = "Sun-Mars L4"};
        case Location::SUN_MARS_L5:
            return {.point = 5,
                    .major = Body::SUN,
                    .minor = Body::MARS,
                    .key   = "sun_mars_l5",
                    .name  = "Sun-Mars L5"};
    }
    return systemOf(Location::EARTH_MOON_L5);
}

}  // namespace

const char* bodyName(Body body)
{
    switch (body)
    {
        case Body::SUN:
            return "Sun";
        case Body::MERCURY:
            return "Mercury";
        case Body::VENUS:
            return "Venus";
        case Body::EARTH:
            return "Earth";
        case Body::MOON:
            return "Moon";
        case Body::MARS:
            return "Mars";
        case Body::JUPITER:
            return "Jupiter";
        case Body::SATURN:
            return "Saturn";
        case Body::URANUS:
            return "Uranus";
        case Body::NEPTUNE:
            return "Neptune";
    }
    return "?";
}

std::optional<Body> bodyFromName(std::string_view name)
{
    constexpr std::array kBodies{Body::SUN,    Body::MERCURY, Body::VENUS,   Body::EARTH,
                                 Body::MOON,   Body::MARS,    Body::JUPITER, Body::SATURN,
                                 Body::URANUS, Body::NEPTUNE};
    const auto           lower = [](char c) {
        return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
    };
    for (const Body body : kBodies)
    {
        const std::string_view candidate = bodyName(body);
        if (std::ranges::equal(candidate, name, {}, lower, lower))
        {
            return body;
        }
    }
    return std::nullopt;
}

double bodyRadiusKm(Body body)
{
    switch (body)
    {
        case Body::SUN:
            return 695700.0;
        case Body::MERCURY:
            return 2439.7;
        case Body::VENUS:
            return 6051.8;
        case Body::EARTH:
            return 6371.0;
        case Body::MOON:
            return 1737.4;
        case Body::MARS:
            return 3389.5;
        case Body::JUPITER:
            return 69911.0;
        case Body::SATURN:
            return 58232.0;
        case Body::URANUS:
            return 25362.0;
        case Body::NEPTUNE:
            return 24622.0;
    }
    return 1.0;
}

Vec3d heliocentricPosition(Body body, SimTime time)
{
    const astro_time_t t = astronomyTime(time);
    if (body == Body::MOON)
    {
        return toVec3(Astronomy_HelioVector(BODY_EARTH, t)) + toVec3(Astronomy_GeoMoon(t));
    }
    return toVec3(Astronomy_HelioVector(toAstronomy(body), t));
}

double visualMagnitude(Body body, SimTime time)
{
    if (body == Body::EARTH)
    {
        return -3.9;  // roughly, from the Moon's distance; Earth is drawn as a disk anyway
    }
    const astro_illum_t illumination =
        Astronomy_Illumination(toAstronomy(body), astronomyTime(time));
    return illumination.status == ASTRO_SUCCESS ? illumination.mag : 0.0;
}

BodyOrientation bodyOrientation(Body body, SimTime time)
{
    astro_time_t       t    = astronomyTime(time);
    const astro_axis_t axis = Astronomy_RotationAxis(toAstronomy(body), &t);
    BodyOrientation    orientation;
    if (axis.status != ASTRO_SUCCESS)
    {
        return orientation;
    }
    // IAU convention: the prime meridian is W degrees east of the ascending node of the body's
    // equator on the ICRF equator.
    const Vec3d north = glm::normalize(toVec3(axis.north));
    Vec3d       node  = glm::cross(Vec3d(0.0, 0.0, 1.0), north);
    node              = glm::dot(node, node) > 1e-20 ? glm::normalize(node) : Vec3d(1.0, 0.0, 0.0);
    const double w    = degreesToRadians(axis.spin);
    const Vec3d  x    = (std::cos(w) * node) + (std::sin(w) * glm::cross(north, node));
    const Vec3d  y    = glm::cross(north, x);
    orientation.north = north;
    orientation.bodyFromEqj = glm::transpose(Mat3d(x, y, north));  // rows: body axes in EQJ
    return orientation;
}

std::string constellationAt(const Vec3d& directionEqj)
{
    const Vec3d  d   = glm::normalize(directionEqj);
    const double ra  = std::fmod((std::atan2(d.y, d.x) / (2.0 * kPi) * 24.0) + 24.0, 24.0);
    const double dec = radiansToDegrees(std::asin(std::clamp(d.z, -1.0, 1.0)));
    const astro_constellation_t constellation = Astronomy_Constellation(ra, dec);
    return constellation.status == ASTRO_SUCCESS ? std::string(constellation.name) : std::string();
}

const char* locationKey(Location location)
{
    return systemOf(location).key;
}

const char* locationName(Location location)
{
    return systemOf(location).name;
}

std::vector<Location> allLocations()
{
    return {Location::EARTH_MOON_L4, Location::EARTH_MOON_L5, Location::SUN_EARTH_L4,
            Location::SUN_EARTH_L5,  Location::SUN_MARS_L4,   Location::SUN_MARS_L5};
}

std::optional<Location> locationFromKey(std::string_view key)
{
    for (const Location location : allLocations())
    {
        if (key == locationKey(location))
        {
            return location;
        }
    }
    return std::nullopt;
}

Vec3d locationPosition(Location location, SimTime time)
{
    const LagrangeSystem       system = systemOf(location);
    const astro_state_vector_t state  = Astronomy_LagrangePoint(
        system.point, astronomyTime(time), toAstronomy(system.major), toAstronomy(system.minor));
    const Vec3d relative(state.x, state.y, state.z);  // relative to the major body
    return heliocentricPosition(system.major, time) + relative;
}

SkyState computeSky(Location location, SimTime time)
{
    SkyState sky;
    sky.time          = time;
    sky.location      = location;
    sky.positionAu    = locationPosition(location, time);
    sky.sunDistanceAu = glm::length(sky.positionAu);
    sky.sunDirection  = -sky.positionAu / sky.sunDistanceAu;

    constexpr std::array kShown{Body::EARTH,  Body::MOON,   Body::MERCURY,
                                Body::VENUS,  Body::MARS,   Body::JUPITER,
                                Body::SATURN, Body::URANUS, Body::NEPTUNE};
    for (const Body body : kShown)
    {
        const Vec3d  position = heliocentricPosition(body, time);
        const Vec3d  offset   = position - sky.positionAu;
        const double distance = glm::length(offset) * kKmPerAu;
        sky.bodies.push_back(
            {.body          = body,
             .direction     = glm::normalize(offset),
             .distanceKm    = distance,
             .angularRadius = std::asin(std::min(1.0, bodyRadiusKm(body) / distance)),
             .towardSun     = -glm::normalize(position),
             .bodyFromEqj   = bodyOrientation(body, time).bodyFromEqj,
             .magnitude     = visualMagnitude(body, time)});
    }
    return sky;
}

Mat3d habitatFromEqj(const Vec3d& sunDirection, double spinPhase)
{
    const Vec3d eclipticNorth(0.0, -std::sin(kObliquity), std::cos(kObliquity));
    const Vec3d z = glm::normalize(sunDirection);
    Vec3d       x = glm::cross(eclipticNorth, z);
    x             = glm::dot(x, x) > 1e-20 ? glm::normalize(x) : Vec3d(1.0, 0.0, 0.0);
    const Vec3d y = glm::cross(z, x);
    // Rows are the habitat axes at phase 0; the habitat then turns by +phase about its +Z, so fixed
    // directions appear turned by -phase.
    const Mat3d  atRest = glm::transpose(Mat3d(x, y, z));
    const double c      = std::cos(spinPhase);
    const double s      = std::sin(spinPhase);
    const Mat3d  unspin(Vec3d(c, -s, 0.0), Vec3d(s, c, 0.0), Vec3d(0.0, 0.0, 1.0));  // Rz(-phase)
    return unspin * atRest;
}

Mat3d habitatFromEqj(const Vec3d& sunDirection, double spinPhase, SpinAxis axis)
{
    if (axis == SpinAxis::TOWARD_SUN)
    {
        return habitatFromEqj(sunDirection, spinPhase);
    }
    const Vec3d z(0.0, -std::sin(kObliquity), std::cos(kObliquity));  // the ecliptic's north pole
    Vec3d       x  = glm::normalize(sunDirection) - (z * glm::dot(glm::normalize(sunDirection), z));
    x              = glm::dot(x, x) > 1e-20 ? glm::normalize(x) : Vec3d(1.0, 0.0, 0.0);
    const Vec3d  y = glm::cross(z, x);
    const Mat3d  atRest = glm::transpose(Mat3d(x, y, z));
    const double c      = std::cos(spinPhase);
    const double s      = std::sin(spinPhase);
    const Mat3d  unspin(Vec3d(c, -s, 0.0), Vec3d(s, c, 0.0), Vec3d(0.0, 0.0, 1.0));  // Rz(-phase)
    return unspin * atRest;
}

}  // namespace StarshipSimulator::astro
