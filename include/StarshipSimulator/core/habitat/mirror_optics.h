#pragma once

#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"

// Sunlight in an O'Neill cylinder. The spin axis points at the Sun (+Z), so sunlight runs parallel
// to the hull and never shines straight in. Flat mirrors hinged at the anti-sunward end of each
// window, opened by angle alpha from the hull, reflect it through the window. The reflected beam
// crosses the axis and lights the valley opposite that window. Because the Sun sits on the spin
// axis, the sun image is fixed in the habitat frame (only the stars sweep past) and moves in the
// plane containing the axis: alpha = 45 degrees puts it overhead, smaller angles toward the sunward
// end (+Z), larger toward the anti-sunward end. Beyond 90 degrees no light enters: night.
namespace StarshipSimulator
{

/// Direction reflected sunlight travels after the mirror of the window at windowAngle.
[[nodiscard]] Vec3d beamTravelDirection(double windowAngle, double openingAngle);

/// Direction toward the sun's image as seen from the valley that beam lights.
[[nodiscard]] Vec3d apparentSunDirection(double windowAngle, double openingAngle);

/// Unit normal of the mirror's reflective face (toward the window and the Sun).
[[nodiscard]] Vec3d mirrorNormal(double windowAngle, double openingAngle);

/// Direction from the hinge along the mirror toward its free edge.
[[nodiscard]] Vec3d mirrorDirection(double windowAngle, double openingAngle);

/// Sun elevation above the valley horizon in radians (pi/2 at alpha = 45 degrees).
[[nodiscard]] double sunElevation(double openingAngle);

/// 1 in full daylight, fading to 0 as the mirrors open toward 90 degrees or close toward 0.
[[nodiscard]] double daylightFactor(double openingAngle);

struct SunBeam
{
    int    window = 0;
    Vec3d  towardSun{0.0, 0.0, 1.0};  // light direction for shading
    double intensity = 0.0;           // fraction of full sunlight (mirror reflectivity, daylight)
};

/// One beam per window.
[[nodiscard]] std::vector<SunBeam> sunBeams(const HabitatGeometry& geometry, double openingAngle);

}  // namespace StarshipSimulator
