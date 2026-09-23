#include "StarshipSimulator/core/gpu_abi/ground_atlas.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/settlements.h"

namespace StarshipSimulator::gpu
{

GroundAtlas packGroundAtlas(const Settlements& settlements)
{
    struct Slot
    {
        const Settlement* town = nullptr;
        std::uint32_t     x    = 0;
        std::uint32_t     y    = 0;
    };
    std::vector<Slot> slots;
    for (const Settlement& place : settlements.places)
    {
        if (place.kind == SettlementKind::TOWN && place.ground.width > 0 && place.ground.height > 0)
        {
            slots.push_back({.town = &place});
        }
    }
    // Shelves, tallest maps first.
    std::ranges::stable_sort(slots, std::ranges::greater{},
                             [](const Slot& s) { return s.town->ground.height; });
    GroundAtlas atlas;
    if (slots.empty())
    {
        atlas.pixels  = {255, 0, 0, 0};
        atlas.records = {Vec4f(0.0F)};
        return atlas;
    }
    atlas.width = kGroundAtlasWidth;
    for (const Slot& slot : slots)
    {
        atlas.width = std::max(atlas.width, slot.town->ground.width + (2 * kGroundAtlasPadding));
    }
    std::uint32_t x     = kGroundAtlasPadding;
    std::uint32_t y     = kGroundAtlasPadding;
    std::uint32_t shelf = 0;
    for (Slot& slot : slots)
    {
        const GroundMap& map = slot.town->ground;
        if (x + map.width + kGroundAtlasPadding > atlas.width)
        {
            x = kGroundAtlasPadding;
            y += shelf + kGroundAtlasPadding;
            shelf = 0;
        }
        slot.x = x;
        slot.y = y;
        x += map.width + kGroundAtlasPadding;
        shelf = std::max(shelf, map.height);
    }
    atlas.height = std::max<std::uint32_t>(1, y + shelf + kGroundAtlasPadding);

    // Unpainted texels: far from any paving, not built up, no light.
    atlas.pixels.resize(static_cast<std::size_t>(atlas.width) * atlas.height * 4);
    for (std::size_t i = 0; i < atlas.pixels.size(); i += 4)
    {
        atlas.pixels[i] = 255;
    }
    atlas.records.emplace_back(static_cast<float>(slots.size()), 0.0F, 0.0F, 0.0F);
    for (const Slot& slot : slots)
    {
        const Settlement& town = *slot.town;
        const GroundMap&  map  = town.ground;
        for (std::uint32_t row = 0; row < map.height; ++row)
        {
            std::memcpy(
                &atlas
                     .pixels[((static_cast<std::size_t>(slot.y + row) * atlas.width) + slot.x) * 4],
                &map.texels[static_cast<std::size_t>(row) * map.width * 4],
                static_cast<std::size_t>(map.width) * 4);
        }
        const Vec2d size = Vec2d(map.width, map.height) * map.texelM;
        // A plan along a valley is placed by (z0, theta0); one running round the axis by its
        // profile arc length u0 instead, and flagged in w.
        const bool around = town.plane.axis == BandAxis::AROUND;
        atlas.records.emplace_back(Vec4d(around ? town.plane.u0 : town.plane.z0, town.plane.theta0,
                                         town.plane.radius, around ? 1.0 : 0.0));
        atlas.records.emplace_back(Vec4d(map.origin, map.origin + size));
        atlas.records.emplace_back(Vec4d(
            static_cast<double>(slot.x) / atlas.width, static_cast<double>(slot.y) / atlas.height,
            1.0 / (map.texelM * atlas.width), 1.0 / (map.texelM * atlas.height)));
    }
    return atlas;
}

}  // namespace StarshipSimulator::gpu
