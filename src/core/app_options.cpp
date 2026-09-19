#include "StarshipSimulator/core/app_options.h"

#include <charconv>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"

namespace StarshipSimulator
{

namespace
{

template <typename T>
std::optional<T> parseNumber(std::string_view text)
{
    const char* const first = std::to_address(text.begin());
    const char* const last  = std::to_address(text.end());
    T                 value{};
    const auto [end, error] = std::from_chars(first, last, value);
    if (error != std::errc{} || end != last || text.empty())
    {
        return std::nullopt;
    }
    return value;
}

std::vector<std::string_view> split(std::string_view text, char separator)
{
    std::vector<std::string_view> parts;
    std::size_t                   start = 0;
    while (true)
    {
        const std::size_t end = text.find(separator, start);
        parts.push_back(text.substr(start, end == std::string_view::npos ? end : end - start));
        if (end == std::string_view::npos)
        {
            return parts;
        }
        start = end + 1;
    }
}

std::expected<WindowSize, std::string> parseWindowSize(std::string_view text)
{
    const auto parts = split(text, 'x');
    if (parts.size() == 2)
    {
        const auto width  = parseNumber<int>(parts[0]);
        const auto height = parseNumber<int>(parts[1]);
        if (width && height && *width > 0 && *height > 0)
        {
            return WindowSize{.width = *width, .height = *height};
        }
    }
    return std::unexpected(
        std::format("--size expects WIDTHxHEIGHT, e.g. 1920x1080, got '{}'", text));
}

std::expected<CameraPose, std::string> parseCameraPose(std::string_view text)
{
    const auto          parts = split(text, ',');
    std::vector<double> values;
    for (const std::string_view part : parts)
    {
        const auto value = parseNumber<double>(part);
        if (!value)
        {
            break;
        }
        values.push_back(*value);
    }
    if (values.size() != 5 || parts.size() != 5)
    {
        return std::unexpected(
            std::format("--camera expects x,y,z,yaw,pitch (metres, degrees), got '{}'", text));
    }
    return CameraPose{
        .x = values[0], .y = values[1], .z = values[2], .yawDeg = values[3], .pitchDeg = values[4]};
}

bool isSkyOption(std::string_view name)
{
    return name == "--time" || name == "--time-scale" || name == "--look-at" || name == "--fov";
}

/// Options about the sky and the view of it.
std::expected<void, std::string> applySkyOption(AppOptions& options, std::string_view name,
                                                std::string_view value)
{
    if (name == "--time")
    {
        auto time = astro::parseIsoTime(value);
        if (!time)
        {
            return std::unexpected(std::format("--time: {}", time.error()));
        }
        options.startTime = *time;
    }
    else if (name == "--time-scale")
    {
        const auto scale = parseNumber<double>(value);
        if (!scale || *scale < 0.0 || *scale > 1.0e7)
        {
            return std::unexpected(
                std::format("--time-scale expects a factor of 0..10000000, got '{}'", value));
        }
        options.timeScale = scale;
    }
    else if (name == "--look-at")
    {
        if (value.empty() || astro::bodyFromName(value) == astro::Body::Sun)
        {
            return std::unexpected(std::format(
                "--look-at expects a planet, the Moon, a star or 'partner', got '{}' (the Sun "
                "is always behind the mirrors)",
                value));
        }
        options.lookAt = std::string(value);
    }
    else if (name == "--fov")
    {
        const auto fov = parseNumber<double>(value);
        if (!fov || *fov < 1.0 || *fov > 120.0)
        {
            return std::unexpected(std::format(
                "--fov expects a vertical field of view of 1..120 degrees, got '{}'", value));
        }
        options.fieldOfViewDeg = fov;
    }
    return {};
}

std::expected<void, std::string> applyValueOption(AppOptions& options, std::string_view name,
                                                  std::string_view value)
{
    if (name == "--size")
    {
        auto size = parseWindowSize(value);
        if (!size)
        {
            return std::unexpected(size.error());
        }
        options.windowSize = *size;
    }
    else if (name == "--camera")
    {
        auto pose = parseCameraPose(value);
        if (!pose)
        {
            return std::unexpected(pose.error());
        }
        options.camera = *pose;
    }
    else if (name == "--capture")
    {
        options.capturePath = std::filesystem::path(value);
    }
    else if (name == "--scenario")
    {
        options.scenarioPath = std::filesystem::path(value);
    }
    else if (name == "--view")
    {
        options.view = std::string(value);
    }
    else if (name == "--mirror")
    {
        const auto angle = parseNumber<double>(value);
        if (!angle || *angle < 0.0 || *angle > 180.0)
        {
            return std::unexpected(
                std::format("--mirror expects an angle in degrees (0..180), got '{}'", value));
        }
        options.mirrorAngleDeg = angle;
    }
    else if (isSkyOption(name))
    {
        return applySkyOption(options, name, value);
    }
    else if (name == "--capture-frames")
    {
        const auto frames = parseNumber<int>(value);
        if (!frames || *frames < 1)
        {
            return std::unexpected(
                std::format("--capture-frames expects a positive number, got '{}'", value));
        }
        options.captureFrames = *frames;
    }
    return {};
}

/// Applies a flag without a value; returns false if it is not one.
bool applyFlag(AppOptions& options, std::string_view name)
{
    if (name == "-h" || name == "--help")
    {
        options.showHelp = true;
    }
    else if (name == "--gpu-debug" || name == "--no-gpu-debug")
    {
        options.gpuDebug = name == "--gpu-debug";
    }
    else if (name == "--no-vsync")
    {
        options.vsync = false;
    }
    else if (name == "--capture-ui")
    {
        options.captureUi = true;
    }
    else if (name == "--benchmark")
    {
        options.benchmark = true;
    }
    else
    {
        return false;
    }
    return true;
}

bool takesValue(std::string_view name)
{
    return name == "--size" || name == "--camera" || name == "--capture" ||
           name == "--capture-frames" || name == "--scenario" || name == "--view" ||
           name == "--mirror" || isSkyOption(name);
}

}  // namespace

std::expected<AppOptions, std::string> parseAppOptions(std::span<const std::string_view> args)
{
    AppOptions options;
    for (std::size_t i = 0; i < args.size(); ++i)
    {
        const std::string_view arg = args[i];
        if (applyFlag(options, arg))
        {
            continue;
        }
        if (!takesValue(arg))
        {
            return std::unexpected(std::format("unknown option '{}' (see --help)", arg));
        }
        if (i + 1 >= args.size())
        {
            return std::unexpected(std::format("{} needs a value", arg));
        }
        ++i;
        if (auto applied = applyValueOption(options, arg, args[i]); !applied)
        {
            return std::unexpected(applied.error());
        }
    }
    return options;
}

std::string appUsage()
{
    return R"(Usage: StarshipSimulator [options]

Options:
  -h, --help               Show this help
  --scenario FILE.toml     Habitat to load (default: data/presets/island_three.toml)
  --view NAME              Start at a viewpoint: valley, lookup, window, river, lake, endcap, ramp,
                           sunward, axis, overview
  --camera x,y,z,yaw,pitch Start at this eye position (m, habitat frame) and view (degrees)
  --mirror DEG             Hold the mirrors at this angle instead of following the day schedule:
                           45 = noon, 90 = sunset, over 90 = night
  --time DATE              Start at this moment (UTC), e.g. 2045-06-15T21:30
  --time-scale N           Run the clock N times faster (0 pauses it; the spin stays real)
  --look-at NAME            Look out of a window at earth, moon, a planet, a star (e.g. Vega)
                           or the partner cylinder ("partner")
  --fov DEG                Vertical field of view (default 70; B toggles binoculars)
  --size WxH               Window size, e.g. 1920x1080
  --no-vsync               Present as fast as possible (mailbox or immediate)
  --gpu-debug              SDL_GPU debug mode and Vulkan validation (default in Debug builds)
  --no-gpu-debug           Disable it
  --capture FILE.png       Render, save a screenshot to FILE.png, then exit
  --capture-frames N       Frames to render before the capture (default 90)
  --capture-ui             Include the HUD in the capture
  --benchmark              Fly a fixed tour of viewpoints, print frame times, then exit

Controls:
  Click the view to capture the mouse, Esc to release it
  WASD move, Shift run, Space jump (walk) or rise (fly), Ctrl descend (fly)
  F walk/fly, G throw a ball, C comfort mode (no Coriolis on you), mouse wheel fly speed
  I identify the star, planet or moon under the crosshair, B binoculars
  P pause the clock, comma/period slower/faster time
  Tab habitat editor, F1 HUD, F5 reload shaders, F12 screenshot
)";
}

}  // namespace StarshipSimulator
