#include "StarshipSimulator/core/tour.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
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

}  // namespace

std::vector<Tour> habitatTours(const HabitatGeometry& geometry, std::string_view name)
{
    const OneillCylinderSpec& spec   = geometry.spec();
    const double              valley = geometry.landCenter(0);
    const double              window = geometry.windowCenter(0);
    const double              middle = 0.5 * (geometry.floorZMin() + geometry.floorZMax());
    const double              radius = geometry.radius();
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
