#include "StarshipSimulator/core/procgen/clouds.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/noise.h"
#include "StarshipSimulator/core/rng.h"

namespace StarshipSimulator
{

namespace
{

constexpr int kShapeOctaves  = 4;
constexpr int kDetailOctaves = 3;

std::uint8_t toByte(double value)
{
    return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0, 1.0) * 255.0));
}

}  // namespace

CloudMap makeCloudMap(std::uint64_t seed, double aroundM, double alongM, double featureM,
                      std::uint32_t width, std::uint32_t height, unsigned threads)
{
    CloudMap map;
    map.width  = std::max(4U, width);
    map.height = std::max(4U, height);
    map.texels.resize(static_cast<std::size_t>(map.width) * map.height * 2);

    const SimplexNoise shape(hashSeed(seed, 0xC10D));
    const SimplexNoise detail(hashSeed(seed, 0xC10E));
    // The map is sampled on a circle for x (so it wraps around the axis); y is blended with a
    // shifted copy of itself so it wraps along the axis too.
    const double radius = aroundM / (2.0 * kPi) / featureM;
    const double along  = alongM / featureM;

    const auto row = [&](std::uint32_t y) {
        const double v = static_cast<double>(y) / map.height;
        for (std::uint32_t x = 0; x < map.width; ++x)
        {
            const double angle = 2.0 * kPi * static_cast<double>(x) / map.width;
            const Vec2d  ring(radius * std::cos(angle), radius * std::sin(angle));
            const auto   blended = [&](const SimplexNoise& noise, int octaves, double scale) {
                const Vec3d here(ring / scale, (v * along) / scale);
                const Vec3d wrapped(ring / scale, ((v - 1.0) * along) / scale);
                return std::lerp(noise.fbm(here, octaves), noise.fbm(wrapped, octaves), v);
            };
            const double      big   = 0.5 + (0.6 * blended(shape, kShapeOctaves, 1.0));
            const double      small = 0.5 + (0.7 * blended(detail, kDetailOctaves, 0.28));
            const std::size_t at    = ((static_cast<std::size_t>(y) * map.width) + x) * 2;
            map.texels[at]          = toByte(big);
            map.texels[at + 1]      = toByte(small);
        }
    };

    std::atomic<std::uint32_t> next{0};
    const auto                 work = [&]() {
        for (std::uint32_t y = next++; y < map.height; y = next++)
        {
            row(y);
        }
    };
    const unsigned workers =
        threads > 0 ? threads : std::max(1U, std::thread::hardware_concurrency());
    std::vector<std::jthread> pool;
    pool.reserve(workers - 1);
    for (unsigned t = 1; t < workers; ++t)
    {
        pool.emplace_back(work);
    }
    work();
    return map;
}

}  // namespace StarshipSimulator
