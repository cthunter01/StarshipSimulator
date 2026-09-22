#pragma once

#include <expected>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/habitat/weather.h"

namespace StarshipSimulator
{

struct WindowSize
{
    int width  = 0;
    int height = 0;
};

/// A starting viewpoint in the habitat frame: eye position in metres and view angles in degrees
/// (yaw 0 faces the sunward end, positive yaw turns left; pitch up is positive).
struct CameraPose
{
    double x        = 0.0;
    double y        = 0.0;
    double z        = 0.0;
    double yawDeg   = 0.0;
    double pitchDeg = 0.0;
};

/// Command-line options of the StarshipSimulator executable.
struct AppOptions
{
    bool                                 showHelp = false;
    std::optional<bool>                  gpuDebug;  // unset: on in Debug builds
    bool                                 vsync = true;
    std::optional<WindowSize>            windowSize;
    std::optional<CameraPose>            camera;
    std::optional<std::filesystem::path> scenarioPath;    // default: the Island Three preset
    std::optional<std::string>           view;            // a named viewpoint, see appUsage()
    std::optional<double>                mirrorAngleDeg;  // fixed mirror angle (no day schedule)
    std::optional<astro::SimTime>        startTime;       // overrides the scenario's start
    std::optional<double>                timeScale;       // simulated seconds per real second
    std::optional<std::string>           lookAt;  // a planet, "moon", a star's name or "partner"
    std::optional<double>                fieldOfViewDeg;  // vertical
    std::optional<std::string>           weather;         // holds the weather: see weatherNamed()
    bool                                 benchmark = false;
    bool                                 mute      = false;
    std::optional<std::filesystem::path> capturePath;  // render, save a PNG, then exit
    int                                  captureFrames = 90;
    bool                                 captureUi     = false;
    std::vector<std::string>             panels;  // opened at startup: editor, gallery, almanac
    std::optional<std::string>           tour;    // a guided tour to set off on: a number or name
};

/// The weather a `--weather` name asks for ("clear", "fair", "cloudy", "overcast", "rain",
/// "storm", "mist"), or nothing if the name is not one of them.
[[nodiscard]] std::optional<Weather> weatherNamed(std::string_view name);

/// Parses the arguments after the program name.
[[nodiscard]] std::expected<AppOptions, std::string> parseAppOptions(
    std::span<const std::string_view> args);

[[nodiscard]] std::string appUsage();

}  // namespace StarshipSimulator
