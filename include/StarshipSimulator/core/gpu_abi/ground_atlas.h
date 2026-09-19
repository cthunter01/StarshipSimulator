#pragma once

#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/settlements.h"

namespace StarshipSimulator::gpu
{

/// The towns' ground maps packed side by side into one texture, and where each lies, for the
/// landscape shader. Must match the TownGround buffer in shaders/include/town_ground.glsl:
/// records[0].x is the number of towns; then three records per town:
///   (z0, theta0, floor radius, 0): the town plan's origin
///   (x0, y0, x1, y1): the plan area its map covers (m)
///   (u0, v0, du/dx, dv/dy): where that area lies in the atlas (texture coordinates)
struct GroundAtlas
{
    std::uint32_t             width  = 1;
    std::uint32_t             height = 1;
    std::vector<std::uint8_t> pixels;  // RGBA8, as GroundMap
    std::vector<Vec4f>        records;
};

inline constexpr std::uint32_t kGroundAtlasWidth   = 2048;
inline constexpr std::uint32_t kGroundAtlasPadding = 4;  // texels between maps (for mipmaps)

[[nodiscard]] GroundAtlas packGroundAtlas(const Settlements& settlements);

}  // namespace StarshipSimulator::gpu
