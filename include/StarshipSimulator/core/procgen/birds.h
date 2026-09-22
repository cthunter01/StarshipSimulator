#pragma once

#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"

// Birds over the valleys. Flocks wheel above fixed places on the floor, so the same habitat at the
// same moment always has the same birds; none of it is simulated and nothing is remembered between
// frames.
namespace StarshipSimulator
{

/// One bird as it is drawn: where it is, where it is heading, and where its wings are.
struct Bird
{
    Vec3d  position{0.0};
    Vec3d  forward{0.0, 0.0, 1.0};  // unit, in the habitat frame
    double wingBeat  = 0.0;         // -1 wings down, +1 wings up
    double wingspanM = 0.9;
};

/// How thick the birds are, and how far off they are still drawn.
struct BirdSettings
{
    double rangeM      = 650.0;  // birds nearer than this to the camera are drawn
    double flockCellM  = 300.0;  // at most one flock to a cell of the valley floor
    double flockChance = 0.6;    // how many of those cells hold one
    double windMS      = 0.0;    // the flocks drift downwind along the axis
    int    maxBirds    = 1200;
};

/// The birds within `settings.rangeM` of a point, at a moment. `seconds` is wall-clock time (the
/// birds fly in real time, like the spin), and `seed` is the habitat's terrain seed.
[[nodiscard]] std::vector<Bird> birdsNear(const HabitatGeometry& geometry, const Vec3d& camera,
                                          double seconds, std::uint64_t seed,
                                          const BirdSettings& settings = {});

}  // namespace StarshipSimulator
