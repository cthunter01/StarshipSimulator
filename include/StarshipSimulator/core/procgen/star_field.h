#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// A star as the renderer needs it. Layout matches the storage buffer in shaders/stars.vert.
struct GpuStar
{
    Vec4f direction;  // xyz: unit direction in the inertial frame, w: size in pixels
    Vec4f color;      // rgb: linear radiance, a: unused
};
static_assert(sizeof(GpuStar) == 32);

/// Placeholder sky until the real star catalog arrives (M2): random directions with a realistic
/// spread of brightness (many faint stars, few bright ones) and star colours from 3000 to 12000 K.
[[nodiscard]] std::vector<GpuStar> generateStarField(std::uint64_t seed, std::size_t count);

/// Approximate linear RGB colour of a black body, normalized to a maximum of 1.
[[nodiscard]] Vec3f blackbodyColor(double kelvin);

}  // namespace StarshipSimulator
