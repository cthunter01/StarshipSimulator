#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/clouds.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/core/utf8_path.h"

// A habitat is shared as a small text file and built again from it, so the same file has to make
// the same world everywhere: on this machine, on someone else's, and under either compiler. These
// tests hash a whole generated world and check it against a value written down here. When a change
// to generation is deliberate, bump kGeneratorVersion in scenario.cpp and put the new hash in.
namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

/// A hash that does not depend on the compiler, the platform or the standard library.
class Digest
{
public:
    void add(std::uint64_t value) { state_ = hashSeed(state_ ^ value, 0x9E3779B97F4A7C15ULL); }
    /// Doubles go in as their exact bits, rounded first to a nanometre so that a difference too
    /// small to see cannot fail the test.
    void add(double value) { add(static_cast<std::uint64_t>(std::llround(value * 1e6))); }
    void add(const Vec3d& value)
    {
        add(value.x);
        add(value.y);
        add(value.z);
    }
    void add(const std::string& text)
    {
        for (const char letter : text)
        {
            add(static_cast<std::uint64_t>(static_cast<unsigned char>(letter)));
        }
    }
    [[nodiscard]] std::uint64_t value() const { return state_; }

private:
    std::uint64_t state_ = 0xA11CEULL;
};

/// Everything generation produces for a habitat, boiled down to one number.
std::uint64_t worldHash(const HabitatSpec& spec)
{
    const HabitatGeometry geometry{spec};
    TerrainGrid           grid  = sampleTerrain(geometry, 2.0 * kPi * spec.radiusM / 1024.0);
    std::vector<TramLine> lines = planTramLines(geometry, grid);
    gradeForTrack(grid, lines);
    const Settlements settlements = planSettlements(geometry, grid);
    addTramStops(lines, settlements);

    Digest digest;
    for (std::size_t i = 0; i < grid.heights.size(); i += 97)  // every 97th, over the whole grid
    {
        digest.add(std::uint64_t{grid.heights[i]});
    }
    for (std::size_t i = 0; i < grid.cover.size(); i += 101)
    {
        digest.add(std::uint64_t{grid.cover[i]});
    }
    for (const Settlement& place : settlements.places)
    {
        digest.add(place.name);
        digest.add(place.plane.z0);
        digest.add(place.plane.theta0);
        digest.add(std::uint64_t{place.streets.size()});
    }
    for (const Building& building : settlements.buildings)
    {
        digest.add(building.centre.x);
        digest.add(building.centre.y);
        digest.add(building.angle);
        digest.add(static_cast<std::uint64_t>(std::max(building.storeys, 0)));
    }
    for (const TramLine& line : lines)
    {
        digest.add(line.lengthM);
        digest.add(std::uint64_t{line.track.size()});
        digest.add(std::uint64_t{line.stops.size()});
        for (std::size_t i = 0; i < line.track.size(); i += 13)
        {
            digest.add(line.track[i].position);
        }
    }
    const CloudMap clouds =
        makeCloudMap(spec.terrain.seed, 2.0 * kPi * spec.radiusM, spec.lengthM, 900.0, 64, 64, 1);
    for (std::size_t i = 0; i < clouds.texels.size(); i += 7)
    {
        digest.add(std::uint64_t{clouds.texels[i]});
    }
    return digest.value();
}

/// A habitat small enough to generate quickly, but with everything in it.
HabitatSpec sample()
{
    HabitatSpec spec;
    spec.radiusM = 400.0;
    spec.lengthM = 3000.0;
    return spec;
}

TEST(Determinism, TheSameHabitatFileAlwaysMakesTheSameWorld)
{
    // If this fails after a deliberate change to generation, bump kGeneratorVersion and update it.
    // Checked against both GCC and Clang. If this fails after a deliberate change to generation,
    // bump Scenario::kGeneratorVersion and put the new value here; if it fails without one, some
    // piece of generation has stopped being reproducible and a shared habitat file no longer
    // makes the same world on someone else's machine.
    constexpr std::uint64_t kWorldHash = 0x10BC37DF85F4E0BFULL;
    EXPECT_EQ(worldHash(sample()), kWorldHash);
    EXPECT_EQ(worldHash(sample()), worldHash(sample()));
}

TEST(Determinism, TheCoriolisPlaygroundAlwaysMakesTheSameWorld)
{
    // The second preset, pinned the same way: a small cylinder without rivers, so a change that
    // only shows up in one like it is caught too.
    const auto playground = loadScenario(pathFromUtf8(STARSHIPSIMULATOR_DATA_DIR) / "presets" /
                                         "coriolis_playground.toml");
    ASSERT_TRUE(playground.has_value()) << playground.error().describe();
    constexpr std::uint64_t kPlaygroundHash = 0xECB25B5C7F435F98ULL;
    EXPECT_EQ(worldHash(playground->habitat), kPlaygroundHash);
}

TEST(Determinism, KalpanaOneAlwaysMakesTheSameWorld)
{
    // A habitat whose land runs round the axis, pinned the same way: its river, towns, farms and
    // looping tramway are all laid out on a band that closes on itself.
    const auto kalpana =
        loadScenario(pathFromUtf8(STARSHIPSIMULATOR_DATA_DIR) / "presets" / "kalpana_one.toml");
    ASSERT_TRUE(kalpana.has_value()) << kalpana.error().describe();
    constexpr std::uint64_t kKalpanaHash = 0x763F0776F282C6B5ULL;
    EXPECT_EQ(worldHash(kalpana->habitat), kKalpanaHash);
}

TEST(Determinism, IslandOneAlwaysMakesTheSameWorld)
{
    // A sphere, pinned the same way: its floor slopes up from the equator, its water lies level at
    // one radius, and its funicular climbs to the window's rim.
    const auto islandOne =
        loadScenario(pathFromUtf8(STARSHIPSIMULATOR_DATA_DIR) / "presets" / "island_one.toml");
    ASSERT_TRUE(islandOne.has_value()) << islandOne.error().describe();
    constexpr std::uint64_t kSphereHash = 0xBEC708DA0F7D0D45ULL;
    EXPECT_EQ(worldHash(islandOne->habitat), kSphereHash);
}

TEST(Determinism, TheThreadCountDoesNotChangeTheTerrain)
{
    const HabitatGeometry geometry{sample()};
    const double          cell = 2.0 * kPi * sample().radiusM / 512.0;
    const TerrainGrid     one  = sampleTerrain(geometry, cell, 1);
    const TerrainGrid     many = sampleTerrain(geometry, cell, 8);
    EXPECT_EQ(one.heights, many.heights);
    EXPECT_EQ(one.cover, many.cover);
}

TEST(Determinism, ADifferentSeedMakesADifferentWorld)
{
    HabitatSpec other  = sample();
    other.terrain.seed = sample().terrain.seed + 1;
    EXPECT_NE(worldHash(sample()), worldHash(other));
}

TEST(Determinism, AHabitatSurvivesBeingWrittenOutAndReadBackIn)
{
    Scenario original;
    original.habitat = sample();
    // A habitat this small needs a cloud deck to match: the default one would reach its axis.
    original.climate.cloudBaseM = 90.0;
    original.climate.cloudTopM  = 160.0;
    const auto parsed           = parseScenario(serializeScenario(original));
    ASSERT_TRUE(parsed.has_value()) << parsed.error().describe();
    EXPECT_EQ(worldHash(original.habitat), worldHash(parsed->habitat))
        << "a habitat read back from its own file makes a different world";
}

}  // namespace
