#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/SimplexNoise.h"
#include "StarshipSimulator/core/procgen/star_field.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

TEST(Rng, IsDeterministicAndUniform)
{
    SplitMix64 a(42);
    SplitMix64 b(42);
    double     sum = 0.0;
    for (int i = 0; i < 10000; ++i)
    {
        const double x = a.uniform();
        EXPECT_EQ(x, b.uniform());
        EXPECT_GE(x, 0.0);
        EXPECT_LT(x, 1.0);
        sum += x;
    }
    EXPECT_NEAR(sum / 10000.0, 0.5, 0.02);
    EXPECT_NE(hashSeed(1, 2), hashSeed(1, 3));
    EXPECT_NE(hashSeed(1, 2), hashSeed(2, 2));
    // A fixed value guards against accidental changes to the generator (shared worlds depend on
    // it).
    EXPECT_EQ(SplitMix64(0).next(), 0xE220A8397B1DCDAFULL);
}

TEST(Noise, IsDeterministicBoundedAndSmooth)
{
    const SimplexNoise noise(1975);
    const SimplexNoise same(1975);
    const SimplexNoise other(1976);
    double             low       = 0.0;
    double             high      = 0.0;
    bool               different = false;
    for (int i = 0; i < 2000; ++i)
    {
        const Vec3d  p(0.37 * i, -0.21 * i, 0.113 * i);
        const double v = noise.sample(p);
        EXPECT_EQ(v, same.sample(p));
        different = different || v != other.sample(p);
        low       = std::min(low, v);
        high      = std::max(high, v);
        // Continuous: a tiny step changes the value only a little.
        EXPECT_NEAR(noise.sample(p + Vec3d(1e-4)), v, 1e-2);
    }
    EXPECT_TRUE(different);
    EXPECT_GE(low, -1.05);
    EXPECT_LE(high, 1.05);
    EXPECT_LT(low, -0.5);
    EXPECT_GT(high, 0.5);

    for (int i = 0; i < 200; ++i)
    {
        const Vec3d p(1.3 * i, 0.7 * i, -0.4 * i);
        EXPECT_LE(std::abs(noise.fbm(p, 5)), 1.05);
        const double r = noise.ridged(p, 5);
        EXPECT_GE(r, 0.0);
        EXPECT_LE(r, 1.0);
    }
}

TEST(StarField, UnitDirectionsManyFaintFewBright)
{
    const auto stars = generateStarField(7, 5000);
    ASSERT_EQ(stars.size(), 5000U);
    std::size_t bright = 0;
    for (const GpuStar& star : stars)
    {
        EXPECT_NEAR(glm::length(Vec3f(star.direction)), 1.0F, 1e-5F);
        bright += star.color.y > 0.5F ? 1U : 0U;
    }
    EXPECT_GT(bright, 0U);
    EXPECT_LT(bright, 100U);

    const auto again = generateStarField(7, 5000);
    EXPECT_EQ(std::memcmp(stars.data(), again.data(), stars.size() * sizeof(GpuStar)), 0);
}

TEST(StarField, BlackbodyColoursRunFromRedToBlue)
{
    const Vec3f cool = blackbodyColor(3000.0);
    const Vec3f sun  = blackbodyColor(5800.0);
    const Vec3f hot  = blackbodyColor(12000.0);
    EXPECT_GT(cool.x, cool.z);
    EXPECT_GT(hot.z, hot.x);
    EXPECT_NEAR(sun.x, 1.0F, 1e-6F);
    EXPECT_GT(sun.z, 0.6F);
}

}  // namespace
