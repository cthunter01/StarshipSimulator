#include "StarshipSimulator/core/procgen/clouds.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

#include <gtest/gtest.h>

namespace
{

using StarshipSimulator::CloudMap;
using StarshipSimulator::makeCloudMap;

constexpr double kAround = 2.0 * std::numbers::pi * 4000.0;
constexpr double kAlong  = 32000.0;

double at(const CloudMap& map, std::uint32_t x, std::uint32_t y, int channel)
{
    const std::size_t index =
        (((static_cast<std::size_t>(y) * map.width) + x) * 2) + static_cast<std::size_t>(channel);
    return static_cast<double>(map.texels[index]) / 255.0;
}

/// The biggest step between neighbouring texels, and the step across the join.
struct Steps
{
    double inside = 0.0;
    double join   = 0.0;
};

Steps stepsAcross(const CloudMap& map, bool along)
{
    Steps               steps;
    const std::uint32_t last = (along ? map.height : map.width) - 1;
    const std::uint32_t wide = along ? map.width : map.height;
    for (std::uint32_t other = 0; other < wide; ++other)
    {
        const auto read = [&](std::uint32_t i) {
            return along ? at(map, other, i, 0) : at(map, i, other, 0);
        };
        for (std::uint32_t i = 0; i < last; ++i)
        {
            steps.inside = std::max(steps.inside, std::abs(read(i + 1) - read(i)));
        }
        steps.join = std::max(steps.join, std::abs(read(0) - read(last)));
    }
    return steps;
}

TEST(CloudMap, WrapsBothWaysWithNoSeam)
{
    const CloudMap map = makeCloudMap(1975, kAround, kAlong, 900.0, 256, 256, 1);
    ASSERT_EQ(map.width, 256U);
    ASSERT_EQ(map.height, 256U);
    ASSERT_EQ(map.texels.size(), 256U * 256U * 2U);
    // The join is no sharper than the map is anywhere else: around the axis, and along it (so the
    // deck has no seam over the endcaps).
    const Steps around = stepsAcross(map, false);
    const Steps along  = stepsAcross(map, true);
    EXPECT_LE(around.join, around.inside * 1.2);
    EXPECT_LE(along.join, along.inside * 1.2);
    EXPECT_GT(around.inside, 0.0);
}

TEST(CloudMap, IsTheSameEveryTimeAndDiffersWithTheSeed)
{
    const CloudMap a    = makeCloudMap(7, kAround, kAlong, 900.0, 64, 64, 1);
    const CloudMap same = makeCloudMap(7, kAround, kAlong, 900.0, 64, 64, 4);  // threads: no effect
    const CloudMap other = makeCloudMap(8, kAround, kAlong, 900.0, 64, 64, 1);
    EXPECT_EQ(a.texels, same.texels);
    EXPECT_NE(a.texels, other.texels);
}

TEST(CloudMap, CoversSomeOfTheSkyButNotAllOfIt)
{
    const CloudMap map  = makeCloudMap(1975, kAround, kAlong, 900.0, 256, 256, 0);
    double         sum  = 0.0;
    double         low  = 1.0;
    double         high = 0.0;
    for (std::uint32_t y = 0; y < map.height; ++y)
    {
        for (std::uint32_t x = 0; x < map.width; ++x)
        {
            const double value = at(map, x, y, 0);
            sum += value;
            low  = std::min(low, value);
            high = std::max(high, value);
        }
    }
    const double mean = sum / (map.width * map.height);
    EXPECT_NEAR(mean, 0.5, 0.1);  // the threshold the shader uses sits in the middle
    EXPECT_LT(low, 0.25);
    EXPECT_GT(high, 0.75);
}

}  // namespace
