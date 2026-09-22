#include "StarshipSimulator/core/procgen/birds.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

const HabitatGeometry& island()
{
    static const HabitatGeometry kGeometry{OneillCylinderSpec{}};
    return kGeometry;
}

/// Standing in the middle of a valley.
Vec3d standing(double z = -2000.0)
{
    const HabitatGeometry& geometry = island();
    const double           theta    = geometry.landCenter(0);
    const double           r        = geometry.groundRadius(z, theta).value_or(3990.0) - 1.7;
    return {r * std::cos(theta), r * std::sin(theta), z};
}

TEST(Birds, FlyOverTheValleyInFlocks)
{
    const std::vector<Bird> birds = birdsNear(island(), standing(), 12.0, 1975);
    ASSERT_FALSE(birds.empty());
    EXPECT_LT(birds.size(), 1300U);
    for (const Bird& bird : birds)
    {
        const double radius = std::hypot(bird.position.x, bird.position.y);
        EXPECT_LT(radius, 4000.0);  // inside the hull
        EXPECT_GT(radius, 3000.0);  // and not up by the axis
        EXPECT_LE(glm::distance(bird.position, standing()), 651.0);
        EXPECT_NEAR(glm::length(bird.forward), 1.0, 1e-9);
        EXPECT_GE(bird.wingBeat, -1.0);
        EXPECT_LE(bird.wingBeat, 1.0);
        EXPECT_GT(bird.wingspanM, 0.1);
        EXPECT_LT(bird.wingspanM, 3.0);
    }
}

TEST(Birds, AreTheSameEveryTimeAndFlyOn)
{
    const std::vector<Bird> a = birdsNear(island(), standing(), 12.0, 1975);
    const std::vector<Bird> b = birdsNear(island(), standing(), 12.0, 1975);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i)
    {
        EXPECT_EQ(a[i].position, b[i].position);
    }
    // A second later they have moved on, but not far: a flock wheels round in half a minute or
    // more. (Birds at the edge of the range come and go, so they are matched by position.)
    const std::vector<Bird> later = birdsNear(island(), standing(), 13.0, 1975);
    EXPECT_NEAR(static_cast<double>(later.size()), static_cast<double>(a.size()),
                (0.1 * static_cast<double>(a.size())) + 4.0);
    double furthest = 0.0;
    for (const Bird& was : a)
    {
        if (glm::distance(was.position, standing()) > BirdSettings{}.rangeM - 40.0)
        {
            continue;  // it may have flown out of range
        }
        double nearest = 1e9;
        for (const Bird& now : later)
        {
            nearest = std::min(nearest, glm::distance(was.position, now.position));
        }
        furthest = std::max(furthest, nearest);
        EXPECT_GT(nearest, 0.05) << "a bird has not moved at all";
    }
    EXPECT_LT(furthest, 25.0);  // no bird flies faster than about 16 m/s
}

TEST(Birds, KeepOffTheWindowsAndTheEnds)
{
    const HabitatGeometry& geometry = island();
    // Over a window strip there is no ground, so no flock wheels there.
    const std::vector<Bird> birds = birdsNear(geometry, standing(), 3.0, 4242);
    for (const Bird& bird : birds)
    {
        const double theta = HabitatGeometry::angleOf(bird.position);
        EXPECT_TRUE(geometry.groundRadius(bird.position.z, theta).has_value())
            << "bird over nothing at z " << bird.position.z;
    }
}

TEST(Birds, ThinOutWhenAskedTo)
{
    const BirdSettings few{.rangeM = 200.0, .flockCellM = 500.0, .flockChance = 0.1};
    const BirdSettings many{.rangeM = 650.0, .flockCellM = 300.0, .flockChance = 0.9};
    EXPECT_LT(birdsNear(island(), standing(), 5.0, 77, few).size(),
              birdsNear(island(), standing(), 5.0, 77, many).size());
}

}  // namespace
