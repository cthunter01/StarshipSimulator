#include "StarshipSimulator/core/app_options.h"

#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

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
}

TEST(AppOptions, HelpFlag)
{
    EXPECT_TRUE(parse({"-h"})->showHelp);
    EXPECT_TRUE(parse({"--help"})->showHelp);
    EXPECT_FALSE(StarshipSimulator::appUsage().empty());
}

}  // namespace
