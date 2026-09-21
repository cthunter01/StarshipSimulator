#pragma once

#include <cstdint>
#include <vector>

// The habitat's clouds as a map over its whole inside surface: the deck of cloud a few hundred
// metres above the valleys, seen from below, from inside and from the far side. The map wraps
// around the axis and along it, and the renderer scrolls it with the wind.
namespace StarshipSimulator
{

/// Cloud cover over the habitat, two channels (RG8): the big shapes and the detail on them.
/// x runs once around the axis, y once along it.
struct CloudMap
{
    std::uint32_t             width  = 0;
    std::uint32_t             height = 0;
    std::vector<std::uint8_t> texels;
};

/// Draws the map. `featureM` is the size of a cloud (m) and `aroundM`/`alongM` the habitat's
/// circumference and length, so clouds come out the same size in any habitat. Deterministic.
[[nodiscard]] CloudMap makeCloudMap(std::uint64_t seed, double aroundM, double alongM,
                                    double featureM = 900.0, std::uint32_t width = 1024,
                                    std::uint32_t height = 1024, unsigned threads = 0);

}  // namespace StarshipSimulator
