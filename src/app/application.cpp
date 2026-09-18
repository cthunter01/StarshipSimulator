#include "application.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_stdinc.h>
#include <SDL3/SDL_timer.h>
#include <SDL3/SDL_video.h>

#include "StarshipSimulator/core/app_options.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/fly_controller.h"
#include "StarshipSimulator/core/log.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/passes/marker_pass.h"
#include "StarshipSimulator/render/renderer.h"
#include "StarshipSimulator/render/shader_library.h"
#include "hud.h"
#include "sdl_input.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kLookSensitivity = 0.0022;  // radians per pixel of mouse motion
constexpr double kWheelStep       = 1.25;    // fly speed factor per scroll step
constexpr double kMinFlySpeed     = 0.5;     // m/s
constexpr double kMaxFlySpeed     = 1.0e5;   // m/s
constexpr double kStatsSmoothing  = 0.05;    // exponential moving average weight
constexpr int    kDefaultWidth    = 1600;
constexpr int    kDefaultHeight   = 900;

SDL_Window* createWindow(const AppOptions& options)
{
    const WindowSize size =
        options.windowSize.value_or(WindowSize{.width = kDefaultWidth, .height = kDefaultHeight});
    SDL_Window* window = SDL_CreateWindow("StarshipSimulator", size.width, size.height,
                                          SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (window == nullptr)
    {
        throw std::runtime_error(std::format("Cannot create the window: {}", SDL_GetError()));
    }
    return window;
}

GpuDeviceOptions deviceOptions(const AppOptions& options)
{
#ifdef NDEBUG
    constexpr bool kDebugBuild = false;
#else
    constexpr bool kDebugBuild = true;
#endif
    return {.debug = options.gpuDebug.value_or(kDebugBuild), .vsync = options.vsync};
}

/// M0 test scene: reference sites at growing distances from the origin. Each has a tall beacon and
/// a row of 4 cm cubes 2 m ahead of the spawn point; with 32-bit world coordinates they would
/// visibly jitter or merge at 16 km and 1000 km, with camera-relative rendering they stay rock
/// steady.
void buildTestScene(std::vector<Waypoint>& waypoints, std::vector<Marker>& markers)
{
    struct Site
    {
        const char* name;
        double      x;
        Vec3f       color;
    };
    const std::vector<Site> sites{
        {.name = "Origin", .x = 0.0, .color = Vec3f(0.9F, 0.9F, 0.9F)},
        {.name = "1 km", .x = 1000.0, .color = Vec3f(0.3F, 0.8F, 0.3F)},
        {.name = "16 km", .x = 16000.0, .color = Vec3f(0.9F, 0.6F, 0.2F)},
        {.name = "1000 km", .x = 1.0e6, .color = Vec3f(0.8F, 0.3F, 0.9F)},
    };
    for (const Site& site : sites)
    {
        waypoints.push_back({.name           = site.name,
                             .eyePosition    = Vec3d(site.x, 0.0, 1.7),
                             .beaconPosition = Vec3d(site.x, 40.0, 22.0)});
        markers.push_back({.position    = Vec3d(site.x, 40.0, 10.0),
                           .halfExtents = Vec3f(1.0F, 1.0F, 10.0F),
                           .color       = site.color,
                           .emission    = site.color * 0.05F});
        for (int k = -2; k <= 2; ++k)
        {
            markers.push_back({.position    = Vec3d(site.x + (0.1 * k), 2.0, 1.6),
                               .halfExtents = Vec3f(0.02F),
                               .color       = site.color,
                               .emission    = site.color * 0.3F});
        }
    }
    // Axis gizmo near the origin: +X red, +Y green, +Z blue.
    markers.push_back({.position    = Vec3d(-3.0 + 2.5, 6.0, 0.1),
                       .halfExtents = Vec3f(2.5F, 0.05F, 0.05F),
                       .color       = Vec3f(0.9F, 0.1F, 0.1F),
                       .emission    = Vec3f(0.0F)});
    markers.push_back({.position    = Vec3d(-3.0, 6.0 + 2.5, 0.1),
                       .halfExtents = Vec3f(0.05F, 2.5F, 0.05F),
                       .color       = Vec3f(0.1F, 0.9F, 0.1F),
                       .emission    = Vec3f(0.0F)});
    markers.push_back({.position    = Vec3d(-3.0, 6.0, 0.1 + 2.5),
                       .halfExtents = Vec3f(0.05F, 0.05F, 2.5F),
                       .color       = Vec3f(0.1F, 0.2F, 1.0F),
                       .emission    = Vec3f(0.0F)});
}

/// A timestamped file in ~/.local/share/StarshipSimulator/screenshots/.
std::filesystem::path nextScreenshotPath()
{
    std::filesystem::path directory = std::filesystem::current_path();
    if (char* prefPath = SDL_GetPrefPath("", "StarshipSimulator"); prefPath != nullptr)
    {
        directory = std::filesystem::path(prefPath);
        SDL_free(prefPath);
    }
    const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
    return directory / "screenshots" / std::format("StarshipSimulator-{:%Y%m%d-%H%M%S}.png", now);
}

}  // namespace

SdlContext::SdlContext()
{
    SDL_SetAppMetadata("StarshipSimulator", nullptr, "io.github.starshipsimulator");
    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        throw std::runtime_error(std::format("Cannot initialise SDL: {}", SDL_GetError()));
    }
}

SdlContext::~SdlContext()
{
    SDL_Quit();
}

void Application::WindowDeleter::operator()(SDL_Window* window) const noexcept
{
    SDL_DestroyWindow(window);
}

Application::Application(AppOptions options)
  : options_(std::move(options)),
    window_(createWindow(options_)),
    device_(window_.get(), deviceOptions(options_)),
    imgui_(window_.get(), device_.get(), device_.swapchainFormat()),
    renderer_(device_, defaultShaderDirectory()),
    input_(window_.get())
{
    buildTestScene(waypoints_, markers_);
    if (options_.camera)
    {
        const CameraPose& pose = *options_.camera;
        controller_.teleport(Vec3d(pose.x, pose.y, pose.z));
        look_.setAngles(degreesToRadians(pose.yawDeg), degreesToRadians(pose.pitchDeg));
        if (pose.z > controller_.settings.eyeHeight + 0.01)
        {
            controller_.setLocomotion(Locomotion::Fly);
        }
    }
    else
    {
        teleport(0);
    }
}

Camera Application::camera() const
{
    Camera camera;
    camera.position    = controller_.eyePosition();
    camera.orientation = look_.orientation();
    return camera;
}

void Application::teleport(std::size_t waypointIndex)
{
    controller_.teleport(waypoints_.at(waypointIndex).eyePosition);
    look_.setAngles(0.0, 0.0);
    status_ = std::format("Teleported to {}", waypoints_.at(waypointIndex).name);
}

void Application::applyInput(const InputFrame& input, const HudActions& hudActions)
{
    look_.applyLook(-input.lookDelta.x * kLookSensitivity, -input.lookDelta.y * kLookSensitivity);

    if (input.toggleLocomotion || hudActions.toggleLocomotion)
    {
        controller_.setLocomotion(controller_.locomotion() == Locomotion::Walk ? Locomotion::Fly
                                                                               : Locomotion::Walk);
    }
    if (input.toggleHud)
    {
        showHud_ = !showHud_;
    }
    if (input.reloadShaders || hudActions.reloadShaders)
    {
        const auto reloaded = renderer_.reloadShaders();
        status_             = reloaded ? std::string("Shaders reloaded")
                                       : "Shader reload failed:\n" + reloaded.error();
        if (!reloaded)
        {
            log::error("{}", status_);
        }
    }
    if (input.wheel != 0.0 && controller_.locomotion() == Locomotion::Fly)
    {
        double& speed = controller_.settings.flySpeed;
        speed = std::clamp(speed * std::pow(kWheelStep, input.wheel), kMinFlySpeed, kMaxFlySpeed);
    }
    if (hudActions.teleportTo)
    {
        teleport(*hudActions.teleportTo);
    }
}

void Application::updateStats(double realSeconds)
{
    if (realSeconds > 0.0)
    {
        frameMs_ += ((realSeconds * 1000.0) - frameMs_) * kStatsSmoothing;
        fps_ = frameMs_ > 0.0 ? 1000.0 / frameMs_ : 0.0;
    }
}

HudActions Application::drawUi()
{
    if (!showHud_)
    {
        return {};
    }
    const HudModel model{
        .gpu           = &device_.info(),
        .samples       = renderer_.sceneFormats().samples,
        .controller    = &controller_,
        .camera        = camera(),
        .waypoints     = waypoints_,
        .fps           = fps_,
        .frameMs       = frameMs_,
        .mouseCaptured = input_.mouseCaptured(),
        .status        = status_,
    };
    return drawHud(model, hudSettings_);
}

std::optional<int> Application::render(int frame, bool screenshotRequested, ImDrawData* ui)
{
    FrameOptions frameOptions;
    frameOptions.ui       = showHud_ ? ui : nullptr;
    const bool captureNow = options_.capturePath && frame + 1 >= options_.captureFrames;
    if (captureNow)
    {
        frameOptions.screenshot           = *options_.capturePath;
        frameOptions.screenshotIncludesUi = options_.captureUi;
    }
    else if (screenshotRequested)
    {
        frameOptions.screenshot = nextScreenshotPath();
    }

    const SceneView scene{
        .camera = camera(), .markers = markers_, .exposure = hudSettings_.exposure};
    const FrameResult result = renderer_.renderFrame(scene, frameOptions);
    if (!result.presented)
    {
        SDL_Delay(16);  // minimized: don't spin
    }
    if (!result.screenshot)
    {
        return std::nullopt;
    }
    const bool saved = result.screenshot->has_value();
    status_          = saved ? std::format("Saved {}", result.screenshot->value().string())
                             : result.screenshot->error();
    if (saved)
    {
        log::info("{}", status_);
    }
    else
    {
        log::error("{}", status_);
    }
    if (captureNow)
    {
        return saved ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    return std::nullopt;
}

int Application::run()
{
    std::uint64_t previous = SDL_GetTicksNS();
    for (int frame = 0;; ++frame)
    {
        const InputFrame input = input_.poll(imgui_);
        if (input.quit)
        {
            return EXIT_SUCCESS;
        }
        const std::uint64_t now         = SDL_GetTicksNS();
        const double        realSeconds = static_cast<double>(now - previous) * 1e-9;
        previous                        = now;
        updateStats(realSeconds);

        // The HUD is built before the simulation step so its button clicks apply this frame.
        imgui_.beginFrame();
        const HudActions hudActions = drawUi();
        ImDrawData*      ui         = imgui_.endFrame();

        applyInput(input, hudActions);
        const int steps = clock_.advance(realSeconds);
        for (int step = 0; step < steps; ++step)
        {
            controller_.step(input.move, look_, clock_.stepSeconds());
        }

        if (const auto exitCode = render(frame, input.screenshot, ui))
        {
            return *exitCode;
        }
    }
}

}  // namespace StarshipSimulator
