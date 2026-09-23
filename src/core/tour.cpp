#include "StarshipSimulator/core/tour.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

/// Eased 0..1: slow away, slow in, so the camera never starts or stops with a jerk.
double ease(double t)
{
    const double x = std::clamp(t, 0.0, 1.0);
    return x * x * (3.0 - (2.0 * x));
}

/// The shorter way round from one heading to another, in degrees.
double turnTo(double from, double to)
{
    return from + std::remainder(to - from, 360.0);
}

/// A point `height` metres above the ground at (z, theta). Up is toward the axis, so standing
/// higher means a smaller radius.
///
/// A tour must not stop with a tree trunk filling the frame or the camera in a lake, and where the
/// woods and the water fall is up to the seed, so the spot is nudged to the nearest open ground:
/// rings of candidates out to about 120 m, the clearest one wins. Ground that is already open
/// (a field, a street, a window strip) is left exactly where it is.
Vec3d place(const HabitatGeometry& geometry, double z, double theta, double height)
{
    const auto clutter = [&geometry](double alongZ, double aroundTheta) {
        return geometry.forestDensity(alongZ, aroundTheta) +
               (geometry.waterDepth(alongZ, aroundTheta) > 0.0 ? 10.0 : 0.0);
    };
    constexpr int kRings = 6;
    constexpr int kSteps = 12;
    double        best   = clutter(z, theta);
    double        bestZ  = z;
    double        bestT  = theta;
    for (int ring = 1; ring <= kRings && best > 0.05; ++ring)
    {
        for (int step = 0; step < kSteps; ++step)
        {
            const double angle  = (2.0 * kPi * step) / kSteps;
            const double reach  = 25.0 * ring;
            const double alongZ = std::clamp(z + (reach * std::cos(angle)), geometry.walkableZMin(),
                                             geometry.walkableZMax());
            const double aroundTheta = theta + ((reach * std::sin(angle)) / geometry.radius());
            if (const double here = clutter(alongZ, aroundTheta); here < best)
            {
                best  = here;
                bestZ = alongZ;
                bestT = aroundTheta;
            }
        }
    }
    const double radius = geometry.groundRadius(bestZ, bestT).value_or(geometry.radius()) - height;
    return {radius * std::cos(bestT), radius * std::sin(bestT), bestZ};
}

Vec3d onAxis(double z, double theta, double radius)
{
    return {radius * std::cos(theta), radius * std::sin(theta), z};
}

}  // namespace

double Tour::lengthS() const
{
    double total = 0.0;
    for (const TourStop& stop : stops)
    {
        total += stop.travelS + stop.holdS;
    }
    return total;
}

TourFrame tourAt(const Tour& tour, double seconds)
{
    TourFrame frame;
    if (tour.stops.empty())
    {
        frame.finished = true;
        return frame;
    }
    double left = std::max(seconds, 0.0);
    for (std::size_t i = 0; i < tour.stops.size(); ++i)
    {
        const TourStop& stop = tour.stops[i];
        const TourStop& from = tour.stops[i > 0 ? i - 1 : 0];
        frame.stop           = i;
        if (left < stop.travelS)
        {
            // On the way: eased from the stop before, with nothing to read yet.
            const double t    = ease(stop.travelS > 0.0 ? left / stop.travelS : 1.0);
            frame.eye         = glm::mix(from.eye, stop.eye, t);
            frame.yawDeg      = std::lerp(from.yawDeg, turnTo(from.yawDeg, stop.yawDeg), t);
            frame.pitchDeg    = std::lerp(from.pitchDeg, stop.pitchDeg, t);
            frame.caption     = stop.caption;
            frame.captionFade = std::clamp((t - 0.55) / 0.35, 0.0, 1.0);
            return frame;
        }
        left -= stop.travelS;
        if (left < stop.holdS)
        {
            // Standing still, reading, turning slowly if the stop asks for it.
            frame.eye         = stop.eye;
            frame.yawDeg      = stop.yawDeg + (stop.panDegS * left);
            frame.pitchDeg    = stop.pitchDeg;
            frame.caption     = stop.caption;
            frame.captionFade = std::clamp(std::min(left, stop.holdS - left) / 0.6, 0.0, 1.0);
            return frame;
        }
        left -= stop.holdS;
    }
    const TourStop& last = tour.stops.back();
    frame.eye            = last.eye;
    frame.yawDeg         = last.yawDeg;
    frame.pitchDeg       = last.pitchDeg;
    frame.stop           = tour.stops.size() - 1;
    frame.captionFade    = 0.0;
    frame.finished       = true;
    return frame;
}

namespace
{

/// A stop, written out in the order it reads: what to say, where to stand, which way to look.
TourStop at(std::string caption, const Vec3d& eye, double yawDeg, double pitchDeg, double travelS,
            double holdS, double panDegS = 0.0)
{
    TourStop stop;
    stop.caption  = std::move(caption);
    stop.eye      = eye;
    stop.yawDeg   = yawDeg;
    stop.pitchDeg = pitchDeg;
    stop.travelS  = travelS;
    stop.holdS    = holdS;
    stop.panDegS  = panDegS;
    return stop;
}

TourStop lit(TourStop stop, double mirrorAngleDeg)
{
    stop.mirrorAngleDeg = mirrorAngleDeg;
    return stop;
}

/// "8.0 kilometres" or "500 metres", whichever reads better at that size.
std::string spanText(double metres)
{
    return metres < 2000.0 ? std::format("{:.0f} metres", metres)
                           : std::format("{:.1f} kilometres", metres / 1000.0);
}

/// "31 seconds" or "2.1 minutes".
std::string periodText(double seconds)
{
    return seconds < 90.0 ? std::format("{:.0f} seconds", seconds)
                          : std::format("{:.1f} minutes", seconds / 60.0);
}

/// "one town" or "12 towns".
std::string counted(int number, std::string_view one, std::string_view many)
{
    return number == 1 ? std::format("one {}", one) : std::format("{} {}", number, many);
}

/// A tour holds the weather from its first stop on: it is a presentation, and the far side is the
/// point of the place, so it should not be lost behind an overcast deck.
TourStop under(TourStop stop, std::string weather)
{
    stop.weather = std::move(weather);
    return stop;
}

/// Kalpana One's tours: the same three as a valley's, but its land runs round the axis between
/// two glass ends, the light comes in through them, and its axis points at the ecliptic's pole.
std::vector<Tour> kalpanaTours(const HabitatGeometry& geometry, std::string_view name)
{
    const HabitatSpec& spec   = geometry.spec();
    const double       middle = 0.5 * (geometry.floorZMin() + geometry.floorZMax());
    const double       radius = geometry.radius();
    const double       across = 2.0 * radius;
    const double       length = geometry.floorZMax() - geometry.floorZMin();
    const double       round  = 2.0 * kPi * radius;
    const double       period = 2.0 * kPi / geometry.omega();
    const double       glass  = geometry.floorZMax() - std::min(30.0, 0.1 * length);
    const double       step   = std::min(0.4, 150.0 / radius);  // radians round, between stops
    const int          towns  = spec.settlements.townsPerValley;
    constexpr double   kAlong = 90.0;  // yaw along the land: spinward, round the axis

    Tour first;
    first.name  = std::format("{} in five minutes", name);
    first.blurb = "The whole place, from its loop of land to the axis.";
    first.stops = {
        under(lit(at(std::format("You are standing inside a cylinder {} across and only {} long, "
                                 "turning once every {}. The land runs once round the inside: "
                                 "you could walk all the way round in about {:.0f} minutes.",
                                 spanText(across), spanText(length), periodText(period),
                                 round / 1.4 / 60.0),
                     place(geometry, middle, 0.0, 1.7), kAlong, 2.0, 0.5, 10.0, 2.0),
                  50.0),
              "fair"),
        at(std::format("There is no sky over you. That is the far side of the floor, {} up, and "
                       "the blue between is the habitat's own air.",
                       spanText(across)),
           place(geometry, middle, 0.0, 26.0), kAlong, 72.0, 4.0, 9.0),
        at("The ends are the windows. Mirrors outside each glass end fold the sunlight in along "
           "the axis, so the light comes from both ends at once.",
           place(geometry, middle, 0.0, 30.0), 0.0, 20.0, 6.0, 9.0),
        at(std::format("People live here: {} beside a river that runs all the way round and back "
                       "into itself, a tram that only ever goes one way round, and farms "
                       "between them.",
                       counted(towns, "village", "villages")),
           place(geometry, middle, step, 40.0), kAlong, -22.0, 8.0, 9.0, 4.0),
        at("Climb toward the axis and the gravity fades: it is made by the spin, and the spin "
           "reaches you through the floor.",
           onAxis(middle, 2.0 * step, 0.45 * radius), kAlong, -10.0, 10.0, 8.0, 5.0),
        at("At the axis there is none left at all. This is where you could fly under your own "
           "power, with a pair of wings.",
           onAxis(middle, 3.0 * step, 0.12 * radius), 0.0, 0.0, 9.0, 10.0, 9.0),
    };

    Tour day;
    day.name  = "How the shutters make a day";
    day.blurb = "Sunrise to nightfall from one spot.";
    TourStop morning =
        lit(at("Morning. The axis points at the pole of the ecliptic, square to the sunlight: "
               "mirrors outside the ends catch the Sun and fold it in through the glass.",
               place(geometry, middle, 0.0, 24.0), kAlong, 18.0, 0.5, 8.0),
            80.0);
    morning.weather = std::string("fair");
    day.stops       = {
        std::move(morning),
        lit(at("Noon. The mirrors stand at their steepest and the light comes in high from both "
               "ends; there is no east or west in here, only the two ends.",
               place(geometry, middle, 0.0, 24.0), kAlong, 30.0, 4.0, 8.0),
            45.0),
        lit(at("Evening. The light falls low along the axis and the shadows reach in toward the "
               "middle from both ends.",
               place(geometry, middle, 0.0, 24.0), 0.0, 12.0, 4.0, 8.0),
            88.0),
        lit(at("Past ninety degrees the shutters close, and the glass ends show the real stars, "
               "circling the middle of each end as the habitat turns.",
               place(geometry, glass, 0.0, 1.7), 0.0, 35.0, 6.0, 12.0),
            110.0),
    };

    Tour sky;
    sky.name  = "The sky outside";
    sky.blurb = "What you see through the glass ends, and why it circles.";
    sky.stops = {
        under(lit(at(std::format("The ends are glass. Stand near one and the stars wheel round "
                                 "its middle, a full turn every {}: the habitat's axis points "
                                 "at the pole of the ecliptic.",
                                 periodText(period)),
                     place(geometry, glass, 0.0, 1.7), 0.0, 40.0, 0.5, 14.0),
                  115.0),
              "clear"),
        at("Those are the real stars for the date on the clock. The Sun, Earth and the planets "
           "lie beside the habitat, near the ecliptic, where the hull hides them.",
           place(geometry, glass, 0.0, 12.0), 0.0, 25.0, 5.0, 14.0, 6.0),
    };
    return {std::move(first), std::move(day), std::move(sky)};
}

/// A Bernal sphere's tours: its land is a belt round the equator, the ground climbs away from it
/// toward the poles, and the light comes in through the polar windows.
std::vector<Tour> sphereTours(const HabitatGeometry& geometry, std::string_view name)
{
    const HabitatSpec& spec   = geometry.spec();
    const double       radius = geometry.radius();
    const double       across = 2.0 * radius;
    const double       belt   = 2.0 * geometry.band(0).halfWidthM;
    const double       round  = 2.0 * kPi * radius;
    const double       period = 2.0 * kPi / geometry.omega();
    const double       edge   = geometry.floorZMax();
    const double       climb  = radius - geometry.floorRadiusAt(edge);
    const double       weight = geometry.floorRadiusAt(edge) / radius;
    const double       rimZ   = geometry.profile().zMax();
    const double       rimR   = geometry.floorRadiusAt(rimZ);
    const double       lowest = radiansToDegrees(
        polarWindowElevation(degreesToRadians(90.0), spec.sphere.windowLatitudeDeg));
    const double highest = radiansToDegrees(
        polarWindowElevation(degreesToRadians(45.0), spec.sphere.windowLatitudeDeg));
    const double     step   = std::min(0.4, 150.0 / radius);  // radians round, between stops
    const int        towns  = spec.settlements.townsPerValley;
    constexpr double kAlong = 90.0;  // yaw along the land: spinward, round the axis
    // By the axis, just inside the sunward window.
    const Vec3d window = onAxis(rimZ - 15.0, 0.0, 20.0);

    Tour first;
    first.name  = std::format("{} in five minutes", name);
    first.blurb = "The whole place, from the belt of land round its equator to the axis.";
    first.stops = {
        under(lit(at(std::format("You are standing on the equator of a sphere {} across, turning "
                                 "once every {}. A belt of land {} wide runs round the inside, "
                                 "{} round: you could walk it in about {:.0f} minutes.",
                                 spanText(across), periodText(period), spanText(belt),
                                 spanText(round), round / 1.4 / 60.0),
                     place(geometry, 0.0, 0.0, 1.7), kAlong, 2.0, 0.5, 10.0, 2.0),
                  50.0),
              "fair"),
        at(std::format("There is no sky over you. That is the far side of the land, {} away "
                       "across the middle, and the blue between is the habitat's own air.",
                       spanText(across)),
           place(geometry, 0.0, 0.0, 26.0), kAlong, 72.0, 4.0, 9.0),
        at(std::format("The ground climbs {} to the polar slopes on either hand. Walk up it and "
                       "you weigh less: the spin makes the gravity, and at the top of the land it "
                       "is only {:.2f} of what it is at the equator.",
                       spanText(climb), weight),
           place(geometry, 0.8 * edge, step, 1.7), 0.0, 15.0, 6.0, 10.0),
        at(std::format("People live here: {} beside a river that runs round the equator and back "
                       "into itself, a tram that only ever goes one way round, and farms up the "
                       "slopes.",
                       counted(towns, "village", "villages")),
           place(geometry, 0.0, 2.0 * step, 40.0), kAlong, -22.0, 8.0, 9.0, 4.0),
        at("A funicular climbs the polar slope to the rim of the window. Beyond the glass, mirrors "
           "fold the sunlight in over the pole.",
           onAxis(rimZ - 20.0, 3.0 * step, rimR - 20.0), 0.0, 40.0, 8.0, 9.0),
        at("Climb toward the axis and the gravity fades: it is made by the spin, and the spin "
           "reaches you through the floor.",
           onAxis(0.0, 4.0 * step, 0.45 * radius), kAlong, -10.0, 9.0, 8.0, 5.0),
        at("At the axis there is none left at all. This is where you could fly under your own "
           "power, with a pair of wings.",
           onAxis(0.0, 5.0 * step, 0.12 * radius), 0.0, 0.0, 9.0, 10.0, 9.0),
    };

    Tour day;
    day.name  = "How the polar mirrors make a day";
    day.blurb = "Sunrise to nightfall from one spot.";
    TourStop morning =
        lit(at(std::format("Morning. The axis points at the Sun, and mirrors outside each polar "
                           "window fold its light in over the pole. It never comes in lower than "
                           "{:.0f} degrees: any lower and none of it would reach the equator.",
                           lowest),
               place(geometry, 0.0, 0.0, 24.0), kAlong, 18.0, 0.5, 9.0),
            80.0);
    morning.weather = std::string("fair");
    day.stops       = {
        std::move(morning),
        lit(at(std::format("Noon. The light stands {:.0f} degrees above each window's rim and "
                           "comes from both poles at once, each lighting mostly the far half of "
                           "the land.",
                           highest),
               place(geometry, 0.0, 0.0, 24.0), kAlong, 30.0, 4.0, 8.0),
            45.0),
        lit(at("Evening. The light comes in as low as the windows allow, and the shadows reach up "
               "the slopes toward the nearer pole.",
               place(geometry, 0.0, 0.0, 24.0), 0.0, 12.0, 4.0, 8.0),
            88.0),
        lit(at("Past ninety degrees the shutters close, and the polar windows show the real stars, "
               "wheeling round the pole as the habitat turns.",
               window, 0.0, 0.0, 8.0, 12.0),
            110.0),
    };

    Tour sky;
    sky.name  = "The sky outside";
    sky.blurb = "What you see through the polar windows, and why it circles.";
    sky.stops = {
        under(lit(at(std::format("The poles are glass. From here by the axis the stars wheel round "
                                 "the middle of the window, a full turn every {}: the axis points "
                                 "at the Sun, which the mirrors hide.",
                                 periodText(period)),
                     window, 0.0, 0.0, 0.5, 14.0),
                  115.0),
              "clear"),
        at("Those are the real stars for the date on the clock. Earth, the Moon and the planets "
           "lie far off to the side, behind the sphere's walls.",
           onAxis(rimZ - 30.0, 0.0, 60.0), 0.0, 20.0, 5.0, 14.0, 6.0),
    };
    return {std::move(first), std::move(day), std::move(sky)};
}

}  // namespace

std::vector<Tour> habitatTours(const HabitatGeometry& geometry, std::string_view name)
{
    if (geometry.kind() == HabitatKind::KALPANA_CYLINDER)
    {
        return kalpanaTours(geometry, name);
    }
    if (geometry.kind() == HabitatKind::BERNAL_SPHERE)
    {
        return sphereTours(geometry, name);
    }
    const HabitatSpec& spec   = geometry.spec();
    const double       valley = geometry.landCenter(0);
    const double       window = geometry.windowCenter(0);
    const double       middle = 0.5 * (geometry.floorZMin() + geometry.floorZMax());
    const double       radius = geometry.radius();
    // Everything a stop says and every distance it moves comes from this habitat, so the same
    // tours work in a 32 km cylinder and in a 250 m one.
    const double across = 2.0 * radius;
    const double period = 2.0 * kPi / geometry.omega();
    const double reach  = std::min(1500.0, 0.15 * (geometry.floorZMax() - geometry.floorZMin()));
    const int    towns  = spec.settlements.townsPerValley * spec.stripPairs;

    Tour first;
    first.name  = std::format("{} in five minutes", name);
    first.blurb = "The whole place, from the floor of a valley to the axis.";
    first.stops = {
        under(lit(at(std::format("You are standing inside a cylinder {} across, turning once "
                                 "every {}. The turning is what holds you down.",
                                 spanText(across), periodText(period)),
                     place(geometry, middle, valley, 1.7), 0.0, 2.0, 0.5, 9.0, 2.0),
                  50.0),
              "fair"),
        at(std::format("There is no sky over you. That is the far side of the floor, {} up, and "
                       "the blue between is the habitat's own air.",
                       spanText(across)),
           place(geometry, middle, valley, 26.0), 0.0, 72.0, 4.0, 9.0),
        at("The dark band across the middle is a window, running the length of the hull. Outside "
           "it a mirror leans in and throws the sunlight onto the valley opposite.",
           place(geometry, middle, valley, 30.0), 90.0, 24.0, 7.0, 9.0),
        at(std::format("People live here: {} along the rivers, a tramway between them, and farms "
                       "out in the fields.",
                       counted(towns, "town", "towns")),
           place(geometry, middle - reach, valley, 40.0), 0.0, -22.0, 8.0, 9.0, 4.0),
        at("Climb toward the axis and the gravity fades: it is made by the spin, and the spin "
           "reaches you through the floor.",
           onAxis(middle - (1.7 * reach), valley, 0.45 * radius), 0.0, -10.0, 10.0, 8.0, 5.0),
        at("At the axis there is none left at all. This is where you could fly under your own "
           "power, with a pair of wings.",
           onAxis(middle - (2.7 * reach), valley, 0.12 * radius), 0.0, 0.0, 9.0, 10.0, 9.0),
    };

    Tour day;
    day.name  = "How the mirrors make a day";
    day.blurb = "Sunrise to nightfall from one spot.";
    TourStop morning =
        lit(at("Morning. The axis points at the sun, so the light comes in along it, and the "
               "mirrors turn it down onto the land.",
               place(geometry, middle, valley, 24.0), 0.0, 18.0, 0.5, 8.0),
            80.0);
    morning.weather = std::string("fair");
    day.stops       = {
        std::move(morning),
        lit(at("Noon. The mirrors have closed to forty-five degrees and the sun's image stands "
               "high over the anti-sunward end.",
               place(geometry, middle, valley, 24.0), 0.0, 30.0, 4.0, 8.0),
            45.0),
        lit(at("Evening. It sets over the same end it rose from: there is no east or west in "
               "here, only sunward and anti-sunward.",
               place(geometry, middle, valley, 24.0), 0.0, 12.0, 4.0, 8.0),
            88.0),
        lit(at("Past ninety degrees the mirrors shut the light out altogether, and the windows "
               "show the real stars, wheeling past as the habitat turns.",
               place(geometry, middle + (0.15 * reach), window, 1.7), 0.0, -40.0, 6.0, 12.0),
            110.0),
    };

    Tour sky;
    sky.name  = "The sky outside";
    sky.blurb = "What you see through the windows, and why it sweeps.";
    sky.stops = {
        under(lit(at(std::format("The windows are walkable glass. Stand on one and the stars go "
                                 "past under your feet: a full turn every {}.",
                                 periodText(period)),
                     place(geometry, middle + (0.15 * reach), window, 1.7), 0.0, -60.0, 0.5, 14.0),
                  115.0),
              "clear"),
        at("Those are the real stars for the date on the clock, seen from where this habitat "
           "really is, with the planets and the Moon in their true places.",
           place(geometry, middle + (0.15 * reach), window, 12.0), 0.0, -25.0, 5.0, 14.0, 6.0),
    };
    return {std::move(first), std::move(day), std::move(sky)};
}

}  // namespace StarshipSimulator
