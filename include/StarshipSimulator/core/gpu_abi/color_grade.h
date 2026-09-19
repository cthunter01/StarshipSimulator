#pragma once

#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"

// The final colour grade: the palette of the 1970s NASA habitat paintings (Rick Guidice, Don
// Davis): warm, golden light, slightly cool shadows, sap-green foliage, rich but not garish colour.
namespace StarshipSimulator
{

struct GradeSettings
{
    double warmth        = 0.05;  // golden highlights
    double coolShadows   = 0.02;  // blue-violet in the darks
    double saturation    = 1.12;  // overall (eased off in the highlights)
    double contrast      = 0.12;  // blend toward an S-curve
    double greenShiftDeg = 9.0;   // greens lean toward yellow (sap and olive greens)
};

/// Grades one display-referred (sRGB-encoded, 0..1) colour.
[[nodiscard]] Vec3d gradeColor(const Vec3d& display, const GradeSettings& settings);

/// The grade as a size^3 RGBA8 lookup table (red varies fastest, then green, then blue), indexed
/// by display-referred colour, for a 3D texture.
[[nodiscard]] std::vector<std::uint8_t> makeGradeLut(const GradeSettings& settings,
                                                     std::uint32_t        size);

}  // namespace StarshipSimulator
