#include "StarshipSimulator/core/almanac.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <string>
#include <vector>

#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/sky_objects.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/RotatingFrame.h"
#include "StarshipSimulator/core/units.h"

namespace StarshipSimulator
{

namespace
{

/// Flies a body from the floor until it comes back to it, and returns how far around the floor it
/// landed from where it started: positive spinward.
double landedOffBy(const HabitatGeometry& geometry, const Vec3d& start, const Vec3d& velocity)
{
    const RotatingFrame frame(geometry.omega());
    const double        radius = geometry.radius();
    const BodyState     from{.position = start, .velocity = velocity};
    const auto          outside = [&](double t) {
        const BodyState at = propagateFreeFlight(frame, from, t);
        return std::hypot(at.position.x, at.position.y) >= radius;
    };
    // Bracket the landing, then close in on it.
    double high = 0.05;
    while (!outside(high) && high < 600.0)
    {
        high *= 2.0;
    }
    double low = 0.0;
    for (int step = 0; step < 80; ++step)
    {
        const double middle            = 0.5 * (low + high);
        (outside(middle) ? high : low) = middle;
    }
    const BodyState landed = propagateFreeFlight(frame, from, high);
    // Both points are at angle 0 to start with, so the landing angle is the whole story.
    return std::remainder(std::atan2(landed.position.y, landed.position.x), 2.0 * kPi) * radius;
}

std::string metres(double value)
{
    if (std::abs(value) < 1.0)
    {
        return std::format("{:.1f} cm", value * 100.0);
    }
    return std::abs(value) < 1000.0 ? std::format("{:.1f} m", value)
                                    : std::format("{:.2f} km", value / 1000.0);
}

std::string hoursMinutes(double hour)
{
    const double wrapped = std::fmod(std::fmod(hour, 24.0) + 24.0, 24.0);
    return std::format("{:02}:{:02}", static_cast<int>(wrapped),
                       static_cast<int>((wrapped - std::floor(wrapped)) * 60.0));
}

/// Where you are: the numbers that change as you walk about.
AlmanacPage standingHere(const AlmanacState& state)
{
    const HabitatGeometry& geometry = *state.geometry;
    const double           radius   = std::hypot(state.eye.x, state.eye.y);
    const double           gravity  = geometry.gravityAt(radius);
    const double           floor    = geometry.gravityAt(geometry.radius());
    const double           pressure = geometry.spec().atmosphere.surfacePressurePa *
                                      pressureRatioAt(geometry.omega(), geometry.radius(), radius,
                                                      geometry.spec().atmosphere.temperatureK);
    const double           speed    = glm::length(state.velocity);
    const double           coriolis = 2.0 * geometry.omega() * speed;

    AlmanacPage page;
    page.title = "Standing where you are";
    page.story =
        "Gravity here is made by the spin, and the spin only reaches you through the floor. Climb "
        "toward the axis and it fades away in proportion to how far out you are: at the axis "
        "there is none at all, and the air is thinner because nothing is holding it down.";
    page.facts.push_back({.label = "Distance from the axis",
                          .value = metres(radius),
                          .note  = std::format("{:.0f}% of the way out to the floor",
                                               100.0 * radius / geometry.radius())});
    page.facts.push_back({.label = "Gravity",
                          .value = std::format("{:.3f} g", gravity / units::kStandardGravity),
                          .note  = floor > 0.0 ? std::format("{:.0f}% of what it is on the floor",
                                                             100.0 * gravity / floor)
                                               : std::string{}});
    page.facts.push_back({.label = "Air pressure",
                          .value = std::format("{:.1f} kPa", pressure / 1000.0),
                          .note  = "it thins toward the axis, as on a mountain"});
    page.facts.push_back(
        {.label = "Coriolis on you now",
         .value = std::format("{:.3f} m/s^2", coriolis),
         .note = speed > 0.05 && gravity > 0.0
                     ? std::format("moving at {:.1f} m/s: {:.1f}% of your weight, pushing sideways",
                                   speed, 100.0 * coriolis / gravity)
                     : std::string("stand still and there is none; it only acts on what moves")});
    return page;
}

AlmanacPage theHabitat(const AlmanacState& state)
{
    const OneillCylinderSpec& spec    = state.geometry->spec();
    const HabitatMetrics&     metrics = state.metrics;

    AlmanacPage page;
    page.title = "This habitat";
    page.story =
        "A cylinder turning in sunlight, with the land on the inside of it. Three strips of ground "
        "alternate with three windows the length of the hull, and outside each window a mirror "
        "leans in to throw the sun onto the valley opposite. There is no dome over your head: what "
        "is up there is more of the same ground, eight kilometres away.";
    page.facts.push_back(
        {.label = "Size",
         .value = std::format("{:.1f} km across, {:.1f} km long", 2.0 * spec.radiusM / 1000.0,
                              spec.lengthM / 1000.0),
         .note  = std::format("{} valleys and {} windows", spec.stripPairs, spec.stripPairs)});
    page.facts.push_back(
        {.label = "Land",
         .value = std::format("{:.0f} km^2", metrics.landAreaM2 / 1e6),
         .note  = std::format("room for about {:.1f} million people", metrics.population / 1e6)});
    page.facts.push_back(
        {.label = "Volume of air",
         .value = std::format("{:.0f} km^3", metrics.volumeM3 / 1e9),
         .note  = "enough for weather of its own: cloud, rain and mist over the valleys"});
    if (state.towns > 0)
    {
        page.facts.push_back({.label = "Settled",
                              .value = std::format("{} towns, {} farms", state.towns, state.farms),
                              .note  = std::format("{} buildings and {:.1f} million trees",
                                                   state.buildings, state.treeMillions)});
    }
    if (state.tramLines > 0)
    {
        page.facts.push_back(
            {.label = "Transit",
             .value = std::format("{:.0f} km of track", state.trackKm),
             .note  = std::format("{} lines and {} stops, counting the lifts up to the hub",
                                  state.tramLines, state.tramStops)});
    }
    return page;
}

AlmanacPage spinAndGravity(const AlmanacState& state)
{
    const HabitatGeometry& geometry = *state.geometry;
    const HabitatMetrics&  metrics  = state.metrics;
    const double           drop     = dropDeflection(geometry, 1.5);
    const double           jump     = jumpDeflection(geometry, 3.5);
    const double           half     = radiusForGravity(geometry, 0.5);

    AlmanacPage page;
    page.title = "Spin and gravity";
    page.story =
        "Nothing here falls, strictly speaking: the floor accelerates up into you. That is enough "
        "to stand on and to pour a drink with, but it is not quite gravity, and anything that "
        "leaves the floor gives the difference away. Let go of something and it lands a little "
        "behind you; jump, and you come down a little in front.";
    page.facts.push_back(
        {.label = "Spin",
         .value = std::format("{:.3f} rpm, one turn every {:.0f} s", metrics.rpm, metrics.periodS),
         .note  = std::format("the floor is moving at {:.0f} m/s", metrics.rimSpeed)});
    page.facts.push_back(
        {.label = "Floor gravity",
         .value = std::format("{:.2f} g", metrics.floorGravity / units::kStandardGravity),
         .note  = std::format("{:.1f}% weaker at your head than at your feet",
                              100.0 * metrics.headToFootGradient)});
    page.facts.push_back({.label = "Half gravity",
                          .value = metres(geometry.radius() - half),
                          .note  = "climb that far toward the axis and you weigh half as much"});
    page.facts.push_back({.label = "Drop something from 1.5 m",
                          .value = metres(std::abs(drop)),
                          .note  = drop < 0.0
                                       ? "it lands that far behind the spin, not straight down"
                                       : "it lands that far ahead of the spin"});
    page.facts.push_back({.label = "Jump at 3.5 m/s",
                          .value = metres(std::abs(jump)),
                          .note  = jump > 0.0 ? "you come down that far ahead of where you left"
                                              : "you come down that far behind where you left"});
    page.facts.push_back(
        {.label = "Walking",
         .value = std::format("{:.1f}% of your weight", 100.0 * metrics.coriolisWalkingRatio),
         .note  = "the sideways push at a walking pace; you feel it on stairs"});
    return page;
}

AlmanacPage theMirrors(const AlmanacState& state)
{
    const double angle    = state.mirrorAngleRad;
    const double daylight = daylightFactor(angle);
    const double sunUp    = radiansToDegrees(sunElevation(angle));

    AlmanacPage page;
    page.title = "The mirrors and the day";
    page.story =
        "The axis points at the sun, so sunlight arrives along it, end on. Outside each window a "
        "mirror hinged at the far end leans in and throws that light down the length of the "
        "habitat onto the valley opposite. Swing the mirror and the sun's image walks along the "
        "sky from one end to the other: morning, noon, evening. It is the same end every time -- "
        "there is no east or west in here, only sunward and anti-sunward.";
    page.facts.push_back({.label = "Mirror angle now",
                          .value = std::format("{:.0f} degrees", radiansToDegrees(angle)),
                          .note  = std::format("45 is noon, 90 sunset; past 90 no light gets in")});
    page.facts.push_back(
        {.label = "The sun's image",
         .value = daylight > 0.0 ? std::format("{:.0f} degrees up", sunUp) : std::string("below"),
         .note  = daylight > 0.0 ? "toward the anti-sunward end, where it rises and sets"
                                 : "the mirrors are shut and the windows show the stars"});
    page.facts.push_back(
        {.label = "Local time",
         .value = hoursMinutes(state.localHour),
         .note  = std::format("sunrise {}, sunset {}, a {:.1f} hour day",
                              hoursMinutes(state.day.sunriseHour),
                              hoursMinutes(sunsetHour(state.day)), state.weather.dayLengthHours)});
    page.facts.push_back({.label = "Daylight",
                          .value = std::format("{:.0f}%", 100.0 * daylight),
                          .note  = "of what the mirrors can deliver at noon"});
    page.facts.push_back(
        {.label = "Windows",
         .value = std::format("{:.0f} km^2 of glass", state.metrics.windowAreaM2 / 1e6),
         .note  = "you can walk out onto them and watch the stars sweep past under your feet"});
    return page;
}

AlmanacPage theHull(const AlmanacState& state)
{
    const HabitatMetrics& metrics = state.metrics;

    AlmanacPage page;
    page.title = "What it is made of";
    page.story =
        "A spinning hoop has to hold itself together against the weight of its own rotation, and "
        "the sum that decides whether it can be built does not care how big it is: only how fast "
        "the rim is moving. That is why a cylinder at one gravity can be made of steel, and why a "
        "ring the size of a continent cannot be made of anything that exists.";
    page.facts.push_back(
        {.label = "The hull must hold",
         .value = std::format("{:.3f} MJ/kg", metrics.hoopSpecificStrength / 1e6),
         .note =
             "strength divided by density: the same number for any size of hoop at this speed"});
    page.facts.push_back({.label = "Which means",
                          .value = materialClassName(metrics.material),
                          .note  = buildableToday(metrics.material)
                                       ? "buildable with materials we have today"
                                       : "needs materials that do not exist yet"});
    page.facts.push_back({.label = "Rim speed",
                          .value = std::format("{:.0f} m/s", metrics.rimSpeed),
                          .note  = "what the hull is holding on to"});
    return page;
}

AlmanacPage theAir(const AlmanacState& state)
{
    const HabitatGeometry& geometry   = *state.geometry;
    const AtmosphereSpec&  atmosphere = geometry.spec().atmosphere;
    const HabitatMetrics&  metrics    = state.metrics;

    AlmanacPage page;
    page.title = "The air and the view";
    page.story =
        "Eight kilometres of air lies between you and the ground overhead -- about as much as you "
        "look through straight up on Earth. That is why the far side is not sharp: it is behind a "
        "veil of the same blue that makes a distant hill blue, and it is lit by sunlight that has "
        "come the same distance. The whole inside of the habitat is its own weather system.";
    page.facts.push_back({.label = "Pressure at the floor",
                          .value = std::format("{:.1f} kPa", atmosphere.surfacePressurePa / 1000.0),
                          .note  = std::format("{:.0f}% of it still there at the axis",
                                               100.0 * metrics.axisPressureRatio)});
    page.facts.push_back(
        {.label = "Temperature at the axis",
         .value = std::format("{:.0f} K colder", metrics.axisTemperatureDropK),
         .note  = "if the air were left to mix freely, as it is in a tall column on Earth"});
    page.facts.push_back({.label = "Across the habitat",
                          .value = metres(2.0 * geometry.radius()),
                          .note  = "of air between you and the land overhead"});
    page.facts.push_back(
        {.label = "The weather now",
         .value = std::format("{:.0f}% cloud", 100.0 * state.weather.cloudCover),
         .note =
             state.weather.rain > 0.05
                 ? std::format("raining; the ground is {:.0f}% wet", 100.0 * state.weather.wetness)
                 : std::format("wind {:.1f} m/s along the valley", state.weather.windAlongMS)});
    return page;
}

AlmanacPage theSkyOutside(const AlmanacState& state)
{
    const HabitatMetrics& metrics = state.metrics;

    AlmanacPage page;
    page.title = "The sky outside";
    page.story =
        "The stars outside are the real ones, in their real places, for the date on the clock. "
        "They do not rise and set: they sweep. The habitat turns once every couple of minutes, so "
        "the whole sky wheels past the windows at a pace you can watch, around the point the axis "
        "is aimed at. Everything else -- the planets, the Earth, the Moon -- keeps its own time.";
    page.facts.push_back({.label = "Where this is",
                          .value = astro::locationName(state.location),
                          .note  = "the habitat holds station here"});
    page.facts.push_back(
        {.label = "The sky sweeps past at",
         .value = std::format("{:.2f} degrees a second", 360.0 / metrics.periodS),
         .note  = std::format("a full turn every {:.0f} seconds", metrics.periodS)});
    if (state.sky != nullptr)
    {
        for (const astro::VisibleBody& body : state.sky->bodies)
        {
            if (body.body == astro::Body::Earth || body.body == astro::Body::Moon)
            {
                page.facts.push_back(
                    {.label = body.body == astro::Body::Earth ? "Earth" : "The Moon",
                     .value = std::format("{:.2f} degrees across",
                                          radiansToDegrees(2.0 * body.angularRadius)),
                     .note  = std::format("{:.0f}% lit ({}), {:.0f} thousand km away",
                                          100.0 * astro::illuminatedFraction(body),
                                          astro::phaseName(astro::illuminatedFraction(body)),
                                          body.distanceKm / 1e3)});
            }
        }
    }
    if (state.partner)
    {
        page.facts.push_back(
            {.label = "The partner cylinder",
             .value = metres(state.partnerSeparationM),
             .note  = "turning the other way, so the pair has no net spin to fight"});
    }
    return page;
}

}  // namespace

double radiusForGravity(const HabitatGeometry& geometry, double fraction)
{
    return geometry.radius() * std::clamp(fraction, 0.0, 1.0);  // gravity is w^2 r: linear in r
}

double dropDeflection(const HabitatGeometry& geometry, double heightM)
{
    const double radius = geometry.radius() - heightM;
    return landedOffBy(geometry, Vec3d(radius, 0.0, 0.0), Vec3d(0.0));
}

double jumpDeflection(const HabitatGeometry& geometry, double speedMS)
{
    // Straight up is toward the axis, which from (R, 0, 0) is -x.
    return landedOffBy(geometry, Vec3d(geometry.radius(), 0.0, 0.0), Vec3d(-speedMS, 0.0, 0.0));
}

std::vector<AlmanacPage> almanacPages(const AlmanacState& state)
{
    if (state.geometry == nullptr)
    {
        return {};
    }
    return {theHabitat(state), standingHere(state), spinAndGravity(state), theMirrors(state),
            theAir(state),     theHull(state),      theSkyOutside(state)};
}

}  // namespace StarshipSimulator
