#include "StarshipSimulator/core/app_options.h"

#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/habitat/weather.h"

namespace
{

using StarshipSimulator::parseAppOptions;

auto parse(std::vector<std::string_view> args)
{
    return parseAppOptions(args);
}

TEST(AppOptions, DefaultsWithoutArguments)
{
    const auto parsed = parse({});
    ASSERT_TRUE(parsed.has_value());
    const auto* options = &parsed.value();
    EXPECT_FALSE(options->showHelp);
    EXPECT_TRUE(options->vsync);
    EXPECT_FALSE(options->gpuDebug.has_value());
    EXPECT_FALSE(options->capturePath.has_value());
}

TEST(AppOptions, ParsesEveryOption)
{
    const auto parsed = parse({"--size", "1280x720", "--camera", "1000000,2.5,1.7,90,-10",
                               "--no-vsync", "--gpu-debug", "--capture", "out/shot.png",
                               "--capture-frames", "30", "--capture-ui"});
    ASSERT_TRUE(parsed.has_value()) << parsed.error();
    const StarshipSimulator::AppOptions& options = parsed.value();

    const auto size = options.windowSize.value_or(StarshipSimulator::WindowSize{});
    EXPECT_EQ(size.width, 1280);
    EXPECT_EQ(size.height, 720);
    const auto camera = options.camera.value_or(StarshipSimulator::CameraPose{});
    EXPECT_DOUBLE_EQ(camera.x, 1.0e6);
    EXPECT_DOUBLE_EQ(camera.pitchDeg, -10.0);
    EXPECT_FALSE(options.vsync);
    EXPECT_EQ(options.gpuDebug, true);
    EXPECT_EQ(options.capturePath, "out/shot.png");
    EXPECT_EQ(options.captureFrames, 30);
    EXPECT_TRUE(options.captureUi);
}

TEST(AppOptions, ReportsProblemsClearly)
{
    EXPECT_NE(parse({"--frobnicate"}).error().find("unknown option"), std::string::npos);
    EXPECT_NE(parse({"--size"}).error().find("needs a value"), std::string::npos);
    EXPECT_NE(parse({"--size", "1280"}).error().find("WIDTHxHEIGHT"), std::string::npos);
    EXPECT_NE(parse({"--size", "0x720"}).error().find("WIDTHxHEIGHT"), std::string::npos);
    EXPECT_NE(parse({"--camera", "1,2,3"}).error().find("x,y,z,yaw,pitch"), std::string::npos);
    EXPECT_NE(parse({"--camera", "1,2,3,4,five"}).error().find("x,y,z"), std::string::npos);
    EXPECT_NE(parse({"--capture-frames", "0"}).error().find("positive"), std::string::npos);
}

TEST(AppOptions, HabitatOptions)
{
    const auto parsed =
        parse({"--scenario", "my.toml", "--view", "lookup", "--mirror", "80", "--benchmark"});
    ASSERT_TRUE(parsed.has_value()) << parsed.error();
    EXPECT_EQ(parsed->scenarioPath, "my.toml");
    EXPECT_EQ(parsed->view, "lookup");
    EXPECT_EQ(parsed->mirrorAngleDeg, 80.0);
    EXPECT_TRUE(parsed->benchmark);
    EXPECT_FALSE(parse({"--mirror", "200"}).has_value());
    // A sphere's polar window is a viewpoint too, and the help says so.
    EXPECT_EQ(parse({"--view", "pole"}).value_or(StarshipSimulator::AppOptions{}).view, "pole");
    EXPECT_NE(StarshipSimulator::appUsage().find("pole"), std::string::npos);
}

TEST(AppOptions, SkyOptions)
{
    const auto parsed = parse(
        {"--time", "2045-06-15T21:30", "--time-scale", "3600", "--look-at", "Moon", "--fov", "10"});
    ASSERT_TRUE(parsed.has_value()) << parsed.error();
    ASSERT_TRUE(parsed->startTime.has_value());
    EXPECT_EQ(StarshipSimulator::astro::formatIsoTime(
                  parsed->startTime.value_or(StarshipSimulator::astro::SimTime{})),
              "2045-06-15T21:30:00Z");
    EXPECT_EQ(parsed->timeScale, 3600.0);
    EXPECT_EQ(parsed->lookAt, "Moon");
    EXPECT_EQ(parsed->fieldOfViewDeg, 10.0);
    EXPECT_FALSE(parse({"--fov", "0"}).has_value());
    EXPECT_EQ(parse({"--look-at", "partner"})->lookAt, "partner");
    EXPECT_NE(parse({"--time", "tomorrow"}).error().find("--time"), std::string::npos);
    EXPECT_NE(parse({"--look-at", "sun"}).error().find("--look-at"), std::string::npos);
    EXPECT_FALSE(parse({"--time-scale", "-1"}).has_value());
}

TEST(AppOptions, WeatherAndSound)
{
    const auto parsed = parse({"--weather", "rain", "--mute"});
    ASSERT_TRUE(parsed.has_value()) << parsed.error();
    EXPECT_EQ(parsed->weather, "rain");
    EXPECT_TRUE(parsed->mute);
    EXPECT_FALSE(parse({})->mute);
    EXPECT_NE(parse({"--weather", "snow"}).error().find("--weather"), std::string::npos);

    // Each name holds weather that looks like it.
    using StarshipSimulator::Weather;
    Weather cloudy;  // what a missing name would give: fails the checks below
    cloudy.cloudCover   = 1.0;
    const Weather clear = StarshipSimulator::weatherNamed("clear").value_or(cloudy);
    const Weather storm = StarshipSimulator::weatherNamed("storm").value_or(Weather{});
    const Weather mist  = StarshipSimulator::weatherNamed("mist").value_or(Weather{});
    EXPECT_LT(clear.cloudCover, 0.1);
    EXPECT_EQ(clear.rain, 0.0);
    EXPECT_GT(storm.cloudCover, 0.9);
    EXPECT_GT(storm.rain, 0.9);
    EXPECT_GE(storm.wetness, storm.rain);
    EXPECT_GT(mist.mist, 0.5);
    for (const auto* name : {"fair", "cloudy", "overcast"})
    {
        EXPECT_TRUE(StarshipSimulator::weatherNamed(name).has_value()) << name;
    }
    EXPECT_FALSE(StarshipSimulator::weatherNamed("hail").has_value());
}

TEST(AppOptions, PanelsAndTours)
{
    const auto parsed = parse({"--panel", "editor", "--panel", "gallery", "--tour", "mirrors"});
    ASSERT_TRUE(parsed.has_value()) << parsed.error();
    EXPECT_EQ(parsed->panels, (std::vector<std::string>{"editor", "gallery"}));
    EXPECT_EQ(parsed->tour, "mirrors");
    EXPECT_TRUE(parse({})->panels.empty());
    EXPECT_FALSE(parse({})->tour.has_value());
    EXPECT_NE(parse({"--panel", "kitchen"}).error().find("--panel"), std::string::npos);
    EXPECT_NE(parse({"--tour"}).error().find("needs a value"), std::string::npos);
}

TEST(AppOptions, HelpFlag)
{
    EXPECT_TRUE(parse({"-h"})->showHelp);
    EXPECT_TRUE(parse({"--help"})->showHelp);
    EXPECT_FALSE(StarshipSimulator::appUsage().empty());
}

}  // namespace
