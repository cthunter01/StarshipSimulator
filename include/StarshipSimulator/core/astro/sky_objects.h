#pragma once

#include <optional>
#include <string>

#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/star_catalog.h"
#include "StarshipSimulator/core/math.h"

// Describing what is in the sky, for people: phases, and naming what someone points at.
namespace StarshipSimulator::astro
{

/// Fraction of a body's disk that is sunlit as seen from the habitat: 0 new, 1 full.
[[nodiscard]] double illuminatedFraction(const VisibleBody& body);

/// "new", "crescent", "half lit", "gibbous" or "full".
[[nodiscard]] const char* phaseName(double illuminatedFraction);

/// "Earth: 1.91 degrees across, 384,400 km away, 73% lit (gibbous)"
[[nodiscard]] std::string describeBody(const VisibleBody& body);

struct Identified
{
    std::string name;                      // "Sirius", "Jupiter", "Earth"
    std::string details;                   // designation, constellation, brightness, distance...
    Vec3d       direction{0.0, 0.0, 1.0};  // EQJ, where the label goes
};

/// The star, planet, Earth or Moon someone looking along `directionEqj` is most likely pointing
/// at, or nothing if there is nothing there. The catalog may be null (not loaded).
[[nodiscard]] std::optional<Identified> identifyInSky(const Vec3d&       directionEqj,
                                                      const SkyState&    sky,
                                                      const StarCatalog* catalog);

}  // namespace StarshipSimulator::astro
