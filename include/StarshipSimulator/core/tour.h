#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"

// Guided tours: a camera flown along a set path with something to read at each stop. The point is
// to be able to show someone the place in a few minutes without them having to learn the controls.
namespace StarshipSimulator
{

/// One stop on a tour: where to stand, what to say, and how to get there from the last one.
struct TourStop
{
    std::string caption;
    Vec3d       eye{0.0};
    double      yawDeg   = 0.0;
    double      pitchDeg = 0.0;
    double      travelS  = 6.0;  // flying here from the stop before
    double      holdS    = 7.0;  // standing here reading
    double      panDegS  = 0.0;  // turning slowly while held, degrees a second
    // What the habitat should be doing here, if the tour wants to set it.
    std::optional<double>      mirrorAngleDeg;
    std::optional<std::string> weather;
    std::optional<double>      timeScale;
};

struct Tour
{
    std::string           name;
    std::string           blurb;
    std::vector<TourStop> stops;

    [[nodiscard]] double lengthS() const;
};

/// Where the camera is partway through a tour.
struct TourFrame
{
    Vec3d       eye{0.0};
    double      yawDeg   = 0.0;
    double      pitchDeg = 0.0;
    std::string caption;
    double      captionFade = 1.0;  // 0 while moving between stops, 1 while standing and reading
    std::size_t stop        = 0;
    bool        finished    = false;
};

/// The camera `seconds` into a tour: eased between stops, holding still to be read at each.
[[nodiscard]] TourFrame tourAt(const Tour& tour, double seconds);

/// The tours on offer in a habitat, named after it. Everything they say and every distance they
/// move comes from the habitat itself, so they work in any one.
[[nodiscard]] std::vector<Tour> habitatTours(const HabitatGeometry& geometry,
                                             std::string_view       name);

}  // namespace StarshipSimulator
