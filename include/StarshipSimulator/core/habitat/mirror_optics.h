#pragma once

#include <optional>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"

// Sunlight in an O'Neill cylinder. The spin axis points at the Sun (+Z), so sunlight runs parallel
// to the hull and never shines straight in. Flat mirrors hinged at the anti-sunward end of each
// window, opened by angle alpha from the hull, reflect it through the window. The reflected beam
// crosses the axis and lights the valley opposite that window. Because the Sun sits on the spin
// axis, the sun image is fixed in the habitat frame (only the stars sweep past) and moves in the
// plane containing the axis: alpha = 45 degrees puts it overhead, smaller angles toward the sunward
// end (+Z), larger toward the anti-sunward end. Beyond 90 degrees no light enters: night.
//
// Kalpana One has no windows along its sides: mirrors outside its glass end caps send the light in
// along the axis, so each end shows an image of the Sun, a point on the axis beyond the glass. The
// same day schedule sets them: the lower the image, the farther out it lies, and past 90 degrees
// the shutters are closed.
//
// A Bernal sphere is lit the same way through its polar windows, but its walls between the land
// and the glass are opaque: an image beyond one pole lights only the places from which the ray
// toward it leaves through that pole's window, mostly the far half of the band.
//
// A Stanford torus's axis points at the ecliptic's pole, square to the sunlight. A mirror over the
// hub, fixed at 45 degrees and turned once a year to face the Sun, sends the light down the axis
// onto a ring of mirrors round the hub, which throw it straight out along every radius, through
// the windows in the ceiling of the tube. So everywhere in the tube the light comes from straight
// up, the direction of the hub, leaning a little across the tube in the morning and evening; the
// ceiling's metal shades whatever cannot see the hub through its glass.
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

/// How high the light through a glass end cap stands above the floor at that end, in radians: the
/// mirrors outside can only fold it in so steeply, and at dusk hold it no lower than 15 degrees.
[[nodiscard]] double endCapElevation(double openingAngle);

/// How high the light through a sphere's polar window stands above the window's rim, in radians:
/// never lower than half the window's latitude (and a little), or no light from beyond one pole
/// would reach the equator, and at most 45 degrees.
[[nodiscard]] double polarWindowElevation(double openingAngle, double windowLatitudeDeg);

/// How far the light thrown out from a torus's hub leans from straight up, toward +Z (radians):
/// none at noon, a little across the tube as the day's mirror angle goes toward dawn or dusk.
[[nodiscard]] double hubLightTilt(double openingAngle);

/// 1 in full daylight, fading to 0 as the mirrors open toward 90 degrees or close toward 0.
[[nodiscard]] double daylightFactor(double openingAngle);

struct SunBeam
{
    int    window = 0;                // or the end it comes in at (0: +Z, 1: -Z)
    Vec3d  towardSun{0.0, 0.0, 1.0};  // light direction for shading (an image at a point: nominal)
    double intensity = 0.0;           // fraction of full sunlight (mirror reflectivity, daylight)
    // A sun image at a point (habitat frame): the light comes from there, a different direction
    // at every point.
    std::optional<Vec3d> image = std::nullopt;
    // Light thrown out from a torus's hub: from straight up at every point, leaning toward +Z by
    // this angle (radians). towardSun is its direction at angle 0 round the axis.
    std::optional<double> hubTilt = std::nullopt;
};

/// One beam per window (or per glass end cap, or per polar window; one from a torus's hub).
[[nodiscard]] std::vector<SunBeam> sunBeams(const HabitatGeometry& geometry, double openingAngle);

/// The direction toward a beam's sun image from p.
[[nodiscard]] Vec3d towardSunFrom(const SunBeam& beam, const Vec3d& p);

/// How much of a beam reaches point p inside the habitat, 0..1: the ray toward its sun image must
/// leave through its window, between the mirror's hinge and the far end of the window. Matches
/// beamAperture() in shaders/include/habitat.glsl.
[[nodiscard]] double beamReach(const HabitatGeometry& geometry, const Vec3d& p,
                               const SunBeam& beam);

/// The window whose beam lights p most strongly (on ground facing up), if any does.
[[nodiscard]] std::optional<int> dominantBeam(const HabitatGeometry& geometry, double openingAngle,
                                              const Vec3d& p);

}  // namespace StarshipSimulator
