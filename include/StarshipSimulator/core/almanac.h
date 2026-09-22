#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/weather.h"
#include "StarshipSimulator/core/math.h"

// The almanac: what this place is, in numbers and plain words. Everything here is worked out from
// the habitat the player is actually standing in and the moment they are standing there, so the
// figures quoted are the ones the simulation is using, not a table copied out of a book.
namespace StarshipSimulator
{

/// One measured fact: what it is, what it comes to, and what that means.
struct AlmanacFact
{
    std::string label;
    std::string value;
    std::string note;  // may be empty
};

/// A page: a heading, a paragraph, and the numbers behind it.
struct AlmanacPage
{
    std::string              title;
    std::string              story;
    std::vector<AlmanacFact> facts;
};

/// The habitat and the moment the almanac describes.
struct AlmanacState
{
    const HabitatGeometry* geometry = nullptr;  // required; everything else may be left alone
    HabitatMetrics         metrics;
    Weather                weather;
    DayScheduleSpec        day;
    double                 mirrorAngleRad = 0.0;
    double                 localHour      = 0.0;
    Vec3d                  eye{0.0};
    Vec3d                  velocity{0.0};
    astro::Location        location = astro::Location::EarthMoonL5;
    const astro::SkyState* sky      = nullptr;  // null while the sky data is still loading
    bool                   partner  = false;
    double                 partnerSeparationM = 0.0;
    std::size_t            towns              = 0;
    std::size_t            farms              = 0;
    std::size_t            buildings          = 0;
    std::size_t            tramLines          = 0;
    std::size_t            tramStops          = 0;
    double                 trackKm            = 0.0;
    double                 treeMillions       = 0.0;
};

/// The pages, in reading order.
[[nodiscard]] std::vector<AlmanacPage> almanacPages(const AlmanacState& state);

/// How far from straight down something dropped from `heightM` lands, in metres around the floor:
/// negative is antispinward (behind the spin). Flown with the simulation's own exact free flight,
/// so the almanac and the world cannot disagree.
[[nodiscard]] double dropDeflection(const HabitatGeometry& geometry, double heightM);

/// The same for a jump straight up at `speedMS`: positive is spinward.
[[nodiscard]] double jumpDeflection(const HabitatGeometry& geometry, double speedMS);

/// The radius at which the spin gravity has fallen to `fraction` of the floor's.
[[nodiscard]] double radiusForGravity(const HabitatGeometry& geometry, double fraction);

}  // namespace StarshipSimulator
