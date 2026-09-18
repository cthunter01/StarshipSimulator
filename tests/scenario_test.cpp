#include "StarshipSimulator/core/scenario/scenario.h"

#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/habitat_spec.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

std::filesystem::path presets()
{
    return std::filesystem::path(STARSHIPSIMULATOR_DATA_DIR) / "presets";
}

TEST(Scenario, RoundTripsThroughToml)
{
    Scenario original;
    original.title                                  = R"(Test "quotes" and \ slashes)";
    original.description                            = "two\nlines";
    original.habitat.radiusM                        = 1234.5;
    original.habitat.lengthM                        = 20000.0;
    original.habitat.stripPairs                     = 4;
    original.habitat.surfaceGravityG                = 0.8;
    original.habitat.sunwardEndcap                  = makeEndcap(EndcapShape::Flat);
    original.habitat.antisunwardEndcap.rampSlopeDeg = 20.0;
    original.habitat.mirrors.openingAngleDeg        = 33.3;
    original.habitat.atmosphere.surfacePressurePa   = 50662.5;
    original.habitat.terrain.seed                   = 123456789012345ULL;
    original.start = {.valley = 2, .zM = -123.25, .headingDeg = 45.0};

    const auto parsed = parseScenario(serializeScenario(original));
    ASSERT_TRUE(parsed.has_value()) << parsed.error().describe();
    const Scenario& copy = parsed.value();
    EXPECT_EQ(copy.title, original.title);
    EXPECT_EQ(copy.description, original.description);
    EXPECT_EQ(copy.habitat.radiusM, 1234.5);
    EXPECT_EQ(copy.habitat.stripPairs, 4);
    EXPECT_EQ(copy.habitat.surfaceGravityG, 0.8);
    EXPECT_EQ(copy.habitat.sunwardEndcap.shape, EndcapShape::Flat);
    EXPECT_EQ(copy.habitat.antisunwardEndcap.shape, EndcapShape::ConicalRamp);
    EXPECT_EQ(copy.habitat.antisunwardEndcap.rampSlopeDeg, 20.0);
    EXPECT_EQ(copy.habitat.mirrors.openingAngleDeg, 33.3);
    EXPECT_DOUBLE_EQ(copy.habitat.atmosphere.surfacePressurePa, 50662.5);
    EXPECT_EQ(copy.habitat.terrain.seed, 123456789012345ULL);
    EXPECT_EQ(copy.start.valley, 2);
    EXPECT_EQ(copy.start.zM, -123.25);
    EXPECT_EQ(serializeScenario(copy), serializeScenario(original));
}

TEST(Scenario, PresetsLoad)
{
    const auto island = loadScenario(presets() / "island_three.toml");
    ASSERT_TRUE(island.has_value()) << island.error().describe();
    EXPECT_EQ(island->title, "Island Three");
    EXPECT_EQ(island->habitat.radiusM, 4000.0);
    EXPECT_EQ(island->habitat.antisunwardEndcap.shape, EndcapShape::ConicalRamp);

    const auto playground = loadScenario(presets() / "coriolis_playground.toml");
    ASSERT_TRUE(playground.has_value()) << playground.error().describe();
    EXPECT_EQ(playground->habitat.radiusM, 250.0);
}

TEST(Scenario, MissingKeysKeepDefaults)
{
    const auto parsed = parseScenario("title = \"Minimal\"\n[habitat]\nradius_m = 3000\n");
    ASSERT_TRUE(parsed.has_value()) << parsed.error().describe();
    EXPECT_EQ(parsed->habitat.radiusM, 3000.0);  // integers are fine for numbers
    EXPECT_EQ(parsed->habitat.lengthM, OneillCylinderSpec{}.lengthM);
}

TEST(Scenario, ErrorsSayWhatAndWhere)
{
    const auto typo =
        parseScenario("title = \"x\"\n[habitat]\nradius_m = 4000.0\nraduis_m = 1.0\n");
    ASSERT_FALSE(typo.has_value());
    EXPECT_EQ(typo.error().line, 4);
    EXPECT_NE(typo.error().message.find("raduis_m"), std::string::npos);

    const auto type = parseScenario("[habitat]\nstrip_pairs = \"three\"\n");
    ASSERT_FALSE(type.has_value());
    EXPECT_EQ(type.error().line, 2);

    const auto syntax = parseScenario("title = \"x\"\n\n[habitat\n");
    ASSERT_FALSE(syntax.has_value());
    EXPECT_EQ(syntax.error().line, 3);

    const auto invalid = parseScenario("[habitat]\nradius_m = -10.0\n");
    ASSERT_FALSE(invalid.has_value());
    EXPECT_NE(invalid.error().message.find("radius"), std::string::npos);

    const auto shape = parseScenario("[habitat.sunward_endcap]\nshape = \"cube\"\n");
    ASSERT_FALSE(shape.has_value());
    EXPECT_EQ(shape.error().line, 2);

    const auto future = parseScenario("format_version = 99\n");
    ASSERT_FALSE(future.has_value());
    EXPECT_NE(future.error().message.find("newer"), std::string::npos);

    EXPECT_FALSE(loadScenario(presets() / "does_not_exist.toml").has_value());
}

}  // namespace
