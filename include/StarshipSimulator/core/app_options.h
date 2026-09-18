#pragma once

#include <expected>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace StarshipSimulator
{

struct WindowSize
{
    int width  = 0;
    int height = 0;
};

/// A starting viewpoint: eye position in metres and view angles in degrees.
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
    std::optional<std::filesystem::path> capturePath;  // render, save a PNG, then exit
    int                                  captureFrames = 90;
    bool                                 captureUi     = false;
};

/// Parses the arguments after the program name.
[[nodiscard]] std::expected<AppOptions, std::string> parseAppOptions(
    std::span<const std::string_view> args);

[[nodiscard]] std::string appUsage();

}  // namespace StarshipSimulator
