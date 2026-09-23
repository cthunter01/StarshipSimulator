#include "StarshipSimulator/core/scenario/scenario.h"

#include <filesystem>
#include <format>
#include <string>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/utf8_path.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

std::filesystem::path presets()
{
    return pathFromUtf8(STARSHIPSIMULATOR_DATA_DIR) / "presets";
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
    original.habitat.sunwardEndcap                  = makeEndcap(EndcapShape::FLAT);
    original.habitat.antisunwardEndcap.rampSlopeDeg = 20.0;
    original.habitat.mirrors.openingAngleDeg        = 33.3;
    original.habitat.atmosphere.surfacePressurePa   = 50662.5;
    original.habitat.terrain.seed                   = 123456789012345ULL;
    original.habitat.settlements = {.townsPerValley = 2, .townRadiusM = 180.5, .farmsPerValley = 9};
    original.start = {.band = 2, .alongM = -123.25, .acrossM = 0.0, .headingDeg = 45.0};
    original.habitat.partner.separationM = 90000.0;
    original.sky.location                = astro::Location::SUN_MARS_L4;
    original.sky.start                   = astro::parseIsoTime("2061-07-28T18:45:30Z").value();
    original.sky.utcOffsetHours          = -5.5;
    original.day.enabled                 = false;
    original.day.dayLengthHours          = 16.5;
    original.day.noonAngleDeg            = 50.0;
    original.climate                     = {.cloudBaseM       = 300.0,
                                            .cloudTopM        = 650.0,
                                            .cloudiness       = 0.7,
                                            .raininess        = 0.1,
                                            .windSpeedMS      = 5.5,
                                            .mistiness        = 0.25,
                                            .yearDays         = 365.25,
                                            .seasonSwingHours = 3.0,
                                            .seasonAtEpoch    = 0.625};

    const auto parsed = parseScenario(serializeScenario(original));
    ASSERT_TRUE(parsed.has_value()) << parsed.error().describe();
    const Scenario& copy = parsed.value();
    EXPECT_EQ(copy.title, original.title);
    EXPECT_EQ(copy.description, original.description);
    EXPECT_EQ(copy.habitat.radiusM, 1234.5);
    EXPECT_EQ(copy.habitat.stripPairs, 4);
    EXPECT_EQ(copy.habitat.surfaceGravityG, 0.8);
    EXPECT_EQ(copy.habitat.sunwardEndcap.shape, EndcapShape::FLAT);
    EXPECT_EQ(copy.habitat.antisunwardEndcap.shape, EndcapShape::CONICAL_RAMP);
    EXPECT_EQ(copy.habitat.antisunwardEndcap.rampSlopeDeg, 20.0);
    EXPECT_EQ(copy.habitat.mirrors.openingAngleDeg, 33.3);
    EXPECT_DOUBLE_EQ(copy.habitat.atmosphere.surfacePressurePa, 50662.5);
    EXPECT_EQ(copy.habitat.terrain.seed, 123456789012345ULL);
    EXPECT_EQ(copy.habitat.settlements.townsPerValley, 2);
    EXPECT_EQ(copy.habitat.settlements.townRadiusM, 180.5);
    EXPECT_EQ(copy.habitat.settlements.farmsPerValley, 9);
    EXPECT_EQ(copy.start.band, 2);
    EXPECT_EQ(copy.start.alongM, -123.25);
    EXPECT_TRUE(copy.habitat.partner.enabled);
    EXPECT_EQ(copy.habitat.partner.separationM, 90000.0);
    EXPECT_EQ(copy.sky.location, astro::Location::SUN_MARS_L4);
    EXPECT_EQ(copy.sky.start, original.sky.start);
    EXPECT_EQ(copy.sky.utcOffsetHours, -5.5);
    EXPECT_FALSE(copy.day.enabled);
    EXPECT_EQ(copy.day.dayLengthHours, 16.5);
    EXPECT_EQ(copy.day.noonAngleDeg, 50.0);
    EXPECT_EQ(copy.climate.cloudBaseM, 300.0);
    EXPECT_EQ(copy.climate.cloudTopM, 650.0);
    EXPECT_EQ(copy.climate.cloudiness, 0.7);
    EXPECT_EQ(copy.climate.raininess, 0.1);
    EXPECT_EQ(copy.climate.windSpeedMS, 5.5);
    EXPECT_EQ(copy.climate.mistiness, 0.25);
    EXPECT_EQ(copy.climate.yearDays, 365.25);
    EXPECT_EQ(copy.climate.seasonSwingHours, 3.0);
    EXPECT_EQ(copy.climate.seasonAtEpoch, 0.625);
    EXPECT_EQ(serializeScenario(copy), serializeScenario(original));
}

TEST(Scenario, EveryKindRoundTrips)
{
    for (const HabitatKind kind : allHabitatKinds())
    {
        Scenario original;
        original.habitat.kind    = kind;
        original.habitat.radiusM = kind == HabitatKind::BISHOP_RING ? 1.0e6 : 900.5;
        // A ring is 450 km wide; an O'Neill cylinder needs room for its ramps; the rest are short.
        original.habitat.lengthM = 350.0;
        if (kind == HabitatKind::BISHOP_RING)
        {
            original.habitat.lengthM = 4.5e5;
        }
        if (kind == HabitatKind::ONEILL_CYLINDER)
        {
            original.habitat.lengthM = 8000.0;
        }
        original.habitat.torus.tubeRadiusM        = 70.5;
        original.habitat.torus.spokes             = 4;
        original.habitat.torus.ceilingWindowShare = 0.25;
        original.habitat.torus.sections           = 8;
        original.habitat.sphere.landLatitudeDeg   = 33.0;
        original.habitat.sphere.windowLatitudeDeg = 60.0;
        original.habitat.ring.wallHeightM         = 150000.0;
        original.habitat.ring.sunTiltDeg          = 12.5;
        original.start = {.band = 0, .alongM = 42.0, .acrossM = -7.5, .headingDeg = 90.0};
        original.climate.cloudTopM  = 105.0;  // fits the smallest headroom here
        original.climate.cloudBaseM = 50.0;

        const std::string text = serializeScenario(original);
        EXPECT_NE(text.find(std::format("type = \"{}\"", habitatKindKey(kind))), std::string::npos);
        const auto parsed = parseScenario(text);
        ASSERT_TRUE(parsed.has_value())
            << habitatKindKey(kind) << ": " << parsed.error().describe();
        EXPECT_EQ(parsed->habitat.kind, kind);
        EXPECT_EQ(parsed->start.acrossM, -7.5);
        EXPECT_EQ(serializeScenario(*parsed), text) << habitatKindKey(kind);
        // Only the kind's own tables are written.
        EXPECT_EQ(text.contains("[habitat.torus]"), kind == HabitatKind::STANFORD_TORUS);
        EXPECT_EQ(text.contains("strip_pairs"), kind == HabitatKind::ONEILL_CYLINDER);
        if (kind == HabitatKind::STANFORD_TORUS)
        {
            EXPECT_EQ(parsed->habitat.torus.tubeRadiusM, 70.5);
            EXPECT_EQ(parsed->habitat.torus.spokes, 4);
            EXPECT_EQ(parsed->habitat.torus.sections, 8);
        }
        if (kind == HabitatKind::BERNAL_SPHERE)
        {
            EXPECT_EQ(parsed->habitat.sphere.windowLatitudeDeg, 60.0);
        }
        if (kind == HabitatKind::BISHOP_RING)
        {
            EXPECT_EQ(parsed->habitat.ring.sunTiltDeg, 12.5);
        }
    }
}

TEST(Scenario, ReadsTheNamesFromBeforeHabitatKinds)
{
    // Files saved before M8 have no type, and start in a valley at an axial position.
    const auto old = parseScenario("[start]\nvalley = 2\nz_m = -750.0\n");
    ASSERT_TRUE(old.has_value()) << old.error().describe();
    EXPECT_EQ(old->habitat.kind, HabitatKind::ONEILL_CYLINDER);
    EXPECT_EQ(old->start.band, 2);
    EXPECT_EQ(old->start.alongM, -750.0);

    const auto unknown = parseScenario("[habitat]\ntype = \"dyson_sphere\"\n");
    ASSERT_FALSE(unknown.has_value());
    EXPECT_EQ(unknown.error().line, 2);
    EXPECT_NE(unknown.error().message.find("stanford_torus"), std::string::npos)
        << unknown.error().message;
}

TEST(Scenario, PresetsLoad)
{
    const auto island = loadScenario(presets() / "island_three.toml");
    ASSERT_TRUE(island.has_value()) << island.error().describe();
    EXPECT_EQ(island->title, "Island Three");
    EXPECT_EQ(island->habitat.radiusM, 4000.0);
    EXPECT_EQ(island->habitat.antisunwardEndcap.shape, EndcapShape::CONICAL_RAMP);

    EXPECT_EQ(island->sky.location, astro::Location::EARTH_MOON_L5);
    EXPECT_TRUE(island->habitat.partner.enabled);
    EXPECT_TRUE(island->day.enabled);

    EXPECT_EQ(island->climate.cloudBaseM, 420.0);
    EXPECT_GT(island->climate.seasonAtEpoch, 0.0);

    const auto playground = loadScenario(presets() / "coriolis_playground.toml");
    ASSERT_TRUE(playground.has_value()) << playground.error().describe();
    EXPECT_EQ(playground->habitat.radiusM, 250.0);
    EXPECT_LT(playground->climate.cloudTopM, 0.8 * playground->habitat.radiusM);

    // The presets are written the way serializeScenario writes them.
    EXPECT_EQ(serializeScenario(island.value()),
              serializeScenario(parseScenario(serializeScenario(island.value())).value()));
}

TEST(Scenario, MissingKeysKeepDefaults)
{
    const auto parsed = parseScenario("title = \"Minimal\"\n[habitat]\nradius_m = 3000\n");
    ASSERT_TRUE(parsed.has_value()) << parsed.error().describe();
    EXPECT_EQ(parsed->habitat.radiusM, 3000.0);  // integers are fine for numbers
    EXPECT_EQ(parsed->habitat.lengthM, HabitatSpec{}.lengthM);
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

    const auto place = parseScenario("[sky]\nlocation = \"mars_orbit\"\n");
    ASSERT_FALSE(place.has_value());
    EXPECT_EQ(place.error().line, 2);
    EXPECT_NE(place.error().message.find("earth_moon_l5"), std::string::npos);

    const auto date = parseScenario("[sky]\nstart = \"2045-02-30\"\n");
    ASSERT_FALSE(date.has_value());
    EXPECT_EQ(date.error().line, 2);

    const auto flag = parseScenario("[day]\nenabled = 1\n");
    ASSERT_FALSE(flag.has_value());
    EXPECT_EQ(flag.error().line, 2);

    const auto clouds = parseScenario("[climate]\ncloud_base_m = 900.0\ncloud_top_m = 800.0\n");
    ASSERT_FALSE(clouds.has_value());
    EXPECT_NE(clouds.error().message.find("cloud"), std::string::npos);

    // The deck must stay well clear of the axis: a small habitat cannot have Earth's clouds.
    const auto high = parseScenario(
        "[habitat]\nradius_m = 250.0\n[climate]\ncloud_base_m = 150.0\ncloud_top_m = 240.0\n");
    ASSERT_FALSE(high.has_value());

    const auto night = parseScenario("[day]\nnight_angle_deg = 80.0\n");
    ASSERT_FALSE(night.has_value());
    EXPECT_NE(night.error().message.find("night"), std::string::npos);

    const auto future = parseScenario("format_version = 99\n");
    ASSERT_FALSE(future.has_value());
    EXPECT_NE(future.error().message.find("newer"), std::string::npos);

    EXPECT_FALSE(loadScenario(presets() / "does_not_exist.toml").has_value());
}

TEST(Scenario, DescribesAHabitatInOneLine)
{
    const std::string line = describeHabitat(HabitatSpec{});  // Island Three
    EXPECT_NE(line.find("8.0 km across"), std::string::npos) << line;
    EXPECT_NE(line.find("32 km long"), std::string::npos) << line;
    EXPECT_NE(line.find("1.00 g"), std::string::npos) << line;
    EXPECT_NE(line.find("structural steel"), std::string::npos) << line;

    HabitatSpec huge;
    huge.radiusM = 1000000.0;  // a Bishop ring's radius: hoop stress no known material takes
    EXPECT_NE(describeHabitat(huge).find("future materials"), std::string::npos);

    HabitatSpec torus;
    torus.kind    = HabitatKind::STANFORD_TORUS;
    torus.radiusM = 895.0;
    EXPECT_NE(describeHabitat(torus).find("a 1.8 km wheel with a 130 m tube"), std::string::npos)
        << describeHabitat(torus);
    HabitatSpec sphere;
    sphere.kind    = HabitatKind::BERNAL_SPHERE;
    sphere.radiusM = 250.0;
    EXPECT_NE(describeHabitat(sphere).find("a 500 m sphere"), std::string::npos)
        << describeHabitat(sphere);
    HabitatSpec kalpana;
    kalpana.kind    = HabitatKind::KALPANA_CYLINDER;
    kalpana.radiusM = 250.0;
    kalpana.lengthM = 325.0;
    EXPECT_NE(describeHabitat(kalpana).find("500 m across, 325 m long"), std::string::npos)
        << describeHabitat(kalpana);
}

TEST(Scenario, SaysWhichKindsCannotBeBuiltYet)
{
    Scenario torus;
    torus.habitat.kind       = HabitatKind::STANFORD_TORUS;
    torus.habitat.radiusM    = 895.0;
    torus.climate.cloudBaseM = 50.0;
    torus.climate.cloudTopM  = 102.0;  // inside the 130 m tube
    torus.start.band         = 0;
    const auto problems      = validateScenario(torus);
    ASSERT_EQ(problems.size(), 1U) << problems.front();
    EXPECT_NE(problems.front().find("cannot be built yet"), std::string::npos);
}

TEST(Scenario, ValidateNamesEverythingWrongAtOnce)
{
    // The editor shows the whole list while you drag sliders, so one problem must not hide
    // another.
    EXPECT_TRUE(validateScenario(Scenario{}).empty());

    Scenario broken;
    broken.habitat.radiusM    = 50.0;  // too small to hold together
    broken.day.nightAngleDeg  = 80.0;  // sunlight would still get in at midnight
    broken.day.dayLengthHours = 30.0;  // longer than a day
    broken.start.band         = 7;     // there is no seventh valley
    const auto problems       = validateScenario(broken);
    EXPECT_GE(problems.size(), 4U);
    for (const std::string& problem : problems)
    {
        EXPECT_FALSE(problem.empty());
    }
}

TEST(Scenario, EveryPresetIsReadyToOpen)
{
    for (const char* name : {"island_three.toml", "coriolis_playground.toml"})
    {
        const auto loaded = loadScenario(presets() / name);
        ASSERT_TRUE(loaded.has_value()) << loaded.error().describe();
        EXPECT_TRUE(validateScenario(*loaded).empty()) << validateScenario(*loaded).front();
        EXPECT_FALSE(describeHabitat(loaded->habitat).empty());
        EXPECT_FALSE(loaded->description.empty()) << name << " needs a line saying what it is";
    }
}

}  // namespace
