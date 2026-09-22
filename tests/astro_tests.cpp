#include <array>
#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator::astro
{
namespace
{

constexpr double kArcminute = degreesToRadians(1.0 / 60.0);

double angleBetween(const Vec3d& a, const Vec3d& b)
{
    return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
}

// ---- Time ---------------------------------------------------------------------------------------

TEST(SimTime, J2000IsNoonOnNewYearsDay2000)
{
    const SimTime j2000 =
        fromCalendar({.year = 2000, .month = 1, .day = 1, .hour = 12, .minute = 0, .second = 0.0});
    EXPECT_EQ(j2000.microseconds, 0);
    EXPECT_DOUBLE_EQ(j2000.days(), 0.0);
}

TEST(SimTime, CalendarRoundTripIsExact)
{
    for (const CalendarTime& calendar :
         {CalendarTime{.year = 2045, .month = 6, .day = 21, .hour = 9, .minute = 0, .second = 0.0},
          CalendarTime{
              .year = 1969, .month = 7, .day = 20, .hour = 20, .minute = 17, .second = 40.0},
          CalendarTime{
              .year = 2100, .month = 2, .day = 28, .hour = 23, .minute = 59, .second = 59.0},
          CalendarTime{.year = 2000, .month = 1, .day = 1, .hour = 0, .minute = 0, .second = 0.0}})
    {
        const CalendarTime back = toCalendar(fromCalendar(calendar));
        EXPECT_EQ(back.year, calendar.year);
        EXPECT_EQ(back.month, calendar.month);
        EXPECT_EQ(back.day, calendar.day);
        EXPECT_EQ(back.hour, calendar.hour);
        EXPECT_EQ(back.minute, calendar.minute);
        EXPECT_DOUBLE_EQ(back.second, calendar.second);
    }
}

TEST(SimTime, DaysMatchAstronomyEngineCalendar)
{
    // 2045-06-21 09:00 UT is 16607.875 days after J2000.
    const SimTime time =
        fromCalendar({.year = 2045, .month = 6, .day = 21, .hour = 9, .minute = 0, .second = 0.0});
    EXPECT_NEAR(time.days(), 16607.875, 1e-9);
}

TEST(SimTime, ParsesAndFormatsIso8601)
{
    const auto time = parseIsoTime("2045-06-21T06:30:15Z");
    ASSERT_TRUE(time.has_value());
    EXPECT_EQ(formatIsoTime(time.value()), "2045-06-21T06:30:15Z");
    EXPECT_EQ(formatIsoTime(parseIsoTime("2045-06-21 06:30").value()), "2045-06-21T06:30:00Z");
    EXPECT_EQ(formatIsoTime(parseIsoTime("2045-06-21").value()), "2045-06-21T00:00:00Z");
    EXPECT_EQ(formatIsoTime(parseIsoTime("1999-12-31T23:59:59").value()), "1999-12-31T23:59:59Z");
}

TEST(SimTime, RejectsMalformedDates)
{
    for (const char* text : {"", "2045", "2045-13-01", "2045-02-30", "2045-06-21T25:00",
                             "2045-06-21T06:30:1", "2045/06/21", "2045-06-21X06:30"})
    {
        EXPECT_FALSE(parseIsoTime(text).has_value()) << text;
    }
}

TEST(SimTime, HourOfDayFollowsUtcOffset)
{
    const SimTime time = parseIsoTime("2045-06-21T09:30:00Z").value();
    EXPECT_NEAR(hourOfDay(time, 0.0), 9.5, 1e-9);
    EXPECT_NEAR(hourOfDay(time, -10.0), 23.5, 1e-9);
    EXPECT_NEAR(hourOfDay(time, 15.0), 0.5, 1e-9);
    EXPECT_NEAR(hourOfDay(time.plusSeconds(3600.0), 0.0), 10.5, 1e-9);
}

// ---- Ephemeris vs JPL Horizons ------------------------------------------------------------------

// JPL Horizons vector tables (DE441), ICRF, AU: heliocentric (Sun centre) for the planets and
// Earth, geocentric for the Moon. 1900-01-01, 1950-01-01, J2000, 2050-01-01, 2100-01-01.
constexpr std::array<double, 5> kDates{2415020.5, 2433282.5, 2451545.0, 2469807.5, 2488070.0};

constexpr std::array<Vec3d, 5> kMars{{
    {4.353672637661962E-01, -1.225360138887019E+00, -5.738531779562993E-01},
    {-1.395553774894103E+00, 8.085346814182592E-01, 4.087222243372503E-01},
    {1.390715921746351E+00, 1.401217628018087E-03, -3.696016719545011E-02},
    {-1.543231687780398E+00, -4.728546592448734E-01, -1.753591215834255E-01},
    // NOLINTNEXTLINE(modernize-use-std-numbers): measured data, not a mathematical constant
    {6.035098605298050E-01, 1.264237827282167E+00, 5.637492109687610E-01},
}};
constexpr std::array<Vec3d, 5> kJupiter{{
    {-3.016039428434643E+00, -4.126277988552982E+00, -1.695438766522865E+00},
    {3.406605247558555E+00, -3.425997624196318E+00, -1.551719750032203E+00},
    {4.001177435589426E+00, 2.736578429014779E+00, 1.075512145491845E+00},
    {-2.391045955156108E+00, 4.265693057695593E+00, 1.886424761712596E+00},
    {-5.373176142913723E+00, -8.860506518501108E-01, -2.491177533542291E-01},
}};
constexpr std::array<Vec3d, 5> kEarth{{
    {-1.968875343395437E-01, 8.837734032391863E-01, 3.833958478884268E-01},
    {-1.827171265684942E-01, 8.863522545045851E-01, 3.843984874215380E-01},
    {-1.771350992727098E-01, 8.874285223255191E-01, 3.847428990882070E-01},
    {-1.716121680953062E-01, 8.884038408444199E-01, 3.850503466899418E-01},
    {-1.660304915630282E-01, 8.893408366007594E-01, 3.853395180058480E-01},
}};
constexpr std::array<Vec3d, 5> kMoonGeocentric{{
    {1.635379676047432E-04, -2.272654330602060E-03, -9.340055635873918E-04},
    {1.246753512591357E-03, 2.091184782379517E-03, 1.098962434346118E-03},
    {-1.949281649686695E-03, -1.782891912873099E-03, -5.087137066222156E-04},
    {2.403647813223150E-03, 6.554283236619424E-04, 4.472719300783614E-04},
    {-2.372924215891485E-03, 6.567996739319745E-04, 3.072558140263113E-04},
}};

SimTime atJulianDate(double julianDateTdb)
{
    return SimTime::fromTerrestrialDays(julianDateTdb - 2451545.0);
}

void expectClose(const Vec3d& actual, const Vec3d& expected, double julianDate)
{
    EXPECT_LT(angleBetween(actual, expected), kArcminute) << "JD " << julianDate;
    EXPECT_NEAR(glm::length(actual) / glm::length(expected), 1.0, 1e-4) << "JD " << julianDate;
}

TEST(Ephemeris, PlanetsMatchHorizonsWithinAnArcminute)
{
    for (std::size_t i = 0; i < kDates.size(); ++i)
    {
        const SimTime time = atJulianDate(kDates.at(i));
        expectClose(heliocentricPosition(Body::MARS, time), kMars.at(i), kDates.at(i));
        expectClose(heliocentricPosition(Body::JUPITER, time), kJupiter.at(i), kDates.at(i));
        expectClose(heliocentricPosition(Body::EARTH, time), kEarth.at(i), kDates.at(i));
    }
}

TEST(Ephemeris, MoonMatchesHorizonsWithinAnArcminute)
{
    for (std::size_t i = 0; i < kDates.size(); ++i)
    {
        const SimTime time = atJulianDate(kDates.at(i));
        const Vec3d   moon =
            heliocentricPosition(Body::MOON, time) - heliocentricPosition(Body::EARTH, time);
        expectClose(moon, kMoonGeocentric.at(i), kDates.at(i));
    }
}

// ---- Where the habitat is -----------------------------------------------------------------------

TEST(Ephemeris, EarthMoonL5TrailsTheMoonBySixtyDegrees)
{
    const SimTime time  = parseIsoTime("2045-06-21T09:00:00Z").value();
    const Vec3d   earth = heliocentricPosition(Body::EARTH, time);
    const Vec3d   moon  = heliocentricPosition(Body::MOON, time) - earth;
    const Vec3d   l5    = locationPosition(Location::EARTH_MOON_L5, time) - earth;
    const Vec3d   l4    = locationPosition(Location::EARTH_MOON_L4, time) - earth;

    EXPECT_NEAR(radiansToDegrees(angleBetween(moon, l5)), 60.0, 0.5);
    EXPECT_NEAR(radiansToDegrees(angleBetween(moon, l4)), 60.0, 0.5);
    EXPECT_NEAR(glm::length(l5) / glm::length(moon), 1.0, 0.01);  // an equilateral triangle
    EXPECT_NEAR(glm::length(l5 - moon) / glm::length(moon), 1.0, 0.01);

    // Trailing: the Moon's orbital motion carries it away from L5 and toward L4.
    const Vec3d later       = heliocentricPosition(Body::MOON, time.plusSeconds(3600.0)) -
                              heliocentricPosition(Body::EARTH, time.plusSeconds(3600.0));
    const Vec3d orbitNormal = glm::cross(moon, later - moon);
    EXPECT_GT(glm::dot(glm::cross(l5, moon), orbitNormal), 0.0);
    EXPECT_LT(glm::dot(glm::cross(l4, moon), orbitNormal), 0.0);
}

TEST(Ephemeris, SkyFromL5ShowsEarthAboutTwoDegreesAcross)
{
    const SkyState sky =
        computeSky(Location::EARTH_MOON_L5, parseIsoTime("2045-06-21T09:00:00Z").value());
    EXPECT_NEAR(sky.sunDistanceAu, 1.0, 0.03);
    EXPECT_NEAR(glm::length(sky.sunDirection), 1.0, 1e-12);
    bool sawEarth = false;
    bool sawMoon  = false;
    for (const VisibleBody& body : sky.bodies)
    {
        EXPECT_NEAR(glm::length(body.direction), 1.0, 1e-12);
        if (body.body == Body::EARTH)
        {
            sawEarth = true;
            EXPECT_NEAR(radiansToDegrees(2.0 * body.angularRadius), 1.9, 0.12);
            EXPECT_NEAR(body.distanceKm, 384400.0, 30000.0);
        }
        if (body.body == Body::MOON)
        {
            sawMoon = true;
            EXPECT_NEAR(radiansToDegrees(2.0 * body.angularRadius), 0.52, 0.05);
        }
    }
    EXPECT_TRUE(sawEarth);
    EXPECT_TRUE(sawMoon);
}

TEST(Ephemeris, LocationKeysRoundTrip)
{
    for (const Location location : allLocations())
    {
        EXPECT_EQ(locationFromKey(locationKey(location)), location);
    }
    EXPECT_FALSE(locationFromKey("mars_orbit").has_value());
}

TEST(Ephemeris, BodyOrientationPutsNorthOnZ)
{
    const SimTime         time        = parseIsoTime("2045-06-21T09:00:00Z").value();
    const BodyOrientation orientation = bodyOrientation(Body::EARTH, time);
    const Vec3d           north       = orientation.bodyFromEqj * orientation.north;
    EXPECT_NEAR(north.z, 1.0, 1e-9);
    // Earth's pole is within a degree of the celestial pole (precession since J2000 is small).
    EXPECT_LT(radiansToDegrees(angleBetween(orientation.north, Vec3d(0.0, 0.0, 1.0))), 1.0);
    EXPECT_NEAR(glm::determinant(orientation.bodyFromEqj), 1.0, 1e-9);
}

TEST(Ephemeris, ConstellationLookup)
{
    // Sirius: RA 6h45m, Dec -16.7 degrees.
    const double ra  = degreesToRadians(101.287);
    const double dec = degreesToRadians(-16.716);
    const Vec3d  sirius(std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra), std::sin(dec));
    EXPECT_EQ(constellationAt(sirius), "Canis Major");
}

// ---- The habitat's orientation ------------------------------------------------------------------

TEST(HabitatFromEqj, SpinAxisPointsAtTheSun)
{
    const Vec3d sun = glm::normalize(Vec3d(0.3, -0.8, 0.2));
    for (const double phase : {0.0, 1.0, 4.0})
    {
        const Mat3d m = habitatFromEqj(sun, phase);
        const Vec3d z = m * sun;
        EXPECT_NEAR(z.z, 1.0, 1e-12);
        EXPECT_NEAR(glm::determinant(m), 1.0, 1e-12);
        const Mat3d identity = m * glm::transpose(m);
        for (int c = 0; c < 3; ++c)
        {
            for (int r = 0; r < 3; ++r)
            {
                EXPECT_NEAR(identity[c][r], c == r ? 1.0 : 0.0, 1e-12);
            }
        }
    }
}

TEST(HabitatFromEqj, FixedDirectionsTurnAgainstTheSpin)
{
    const Vec3d sun  = glm::normalize(Vec3d(-0.2, 0.9, 0.39));
    const Vec3d star = glm::normalize(Vec3d(0.5, 0.1, -0.3));
    const Vec3d at0  = habitatFromEqj(sun, 0.0) * star;
    const Vec3d atQ  = habitatFromEqj(sun, kPi / 2.0) * star;
    // After a quarter turn of the habitat, the star has turned a quarter turn backwards.
    EXPECT_NEAR(atQ.x, at0.y, 1e-12);
    EXPECT_NEAR(atQ.y, -at0.x, 1e-12);
    EXPECT_NEAR(atQ.z, at0.z, 1e-12);
}

TEST(HabitatFromEqj, PhaseZeroXLiesInTheEcliptic)
{
    const double obliquity = degreesToRadians(23.4392911);
    const Vec3d  eclipticNorth(0.0, -std::sin(obliquity), std::cos(obliquity));
    const Vec3d  sun = glm::normalize(Vec3d(0.2, 0.9, 0.39));
    const Mat3d  m   = habitatFromEqj(sun, 0.0);
    const Vec3d  x   = glm::transpose(m) * Vec3d(1.0, 0.0, 0.0);  // habitat +X in EQJ
    EXPECT_NEAR(glm::dot(x, eclipticNorth), 0.0, 1e-12);
}

}  // namespace
}  // namespace StarshipSimulator::astro
