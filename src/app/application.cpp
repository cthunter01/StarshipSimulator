#include "application.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <future>
#include <memory>
#include <optional>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
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
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/log.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/player_controller.h"
#include "StarshipSimulator/core/physics/rotating_frame.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/gpu_world.h"
#include "StarshipSimulator/render/passes/marker_pass.h"
#include "StarshipSimulator/render/renderer.h"
#include "StarshipSimulator/render/shader_library.h"
#include "hud.h"
#include "sdl_input.h"

namespace StarshipSimulator
{

namespace
{

constexpr double        kLookSensitivity      = 0.0022;  // radians per pixel of mouse motion
constexpr double        kWheelStep            = 1.25;    // fly speed factor per scroll step
constexpr double        kMinFlySpeed          = 1.0;     // m/s
constexpr double        kMaxFlySpeed          = 3000.0;  // m/s
constexpr double        kStatsSmoothing       = 0.05;    // exponential moving average weight
constexpr int           kDefaultWidth         = 1600;
constexpr int           kDefaultHeight        = 900;
constexpr std::size_t   kStarCount            = 9000;
constexpr std::uint64_t kStarSeed             = 20260918;
constexpr double        kThrowSpeed           = 12.0;  // m/s
constexpr double        kBallRadius           = 0.11;  // m
constexpr double        kPathStep             = 0.05;  // s between trajectory dots
constexpr double        kMaxFlightTime        = 60.0;  // s
constexpr double        kBenchmarkViewSeconds = 3.0;
constexpr double        kBenchmarkWarmup      = 0.5;
constexpr double        kBenchmarkPanRate     = degreesToRadians(20.0);  // per second
constexpr std::array<std::string_view, 7> kBenchmarkViews{"valley",  "lookup", "window",  "ramp",
                                                          "sunward", "axis",   "overview"};
constexpr Vec3d kNorth(0.0, 0.0, 1.0);  // the "north" of the look rig: toward the sunward end

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

std::filesystem::path dataDirectory()
{
    const char* base = SDL_GetBasePath();
    return (base != nullptr ? std::filesystem::path(base) : std::filesystem::current_path()) /
           "data";
}

/// ~/.local/share/StarshipSimulator/
std::filesystem::path userDirectory()
{
    std::filesystem::path directory = std::filesystem::current_path();
    if (char* prefPath = SDL_GetPrefPath("", "StarshipSimulator"); prefPath != nullptr)
    {
        directory = std::filesystem::path(prefPath);
        SDL_free(prefPath);
    }
    return directory;
}

std::filesystem::path nextScreenshotPath()
{
    const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
    return userDirectory() / "screenshots" /
           std::format("StarshipSimulator-{:%Y%m%d-%H%M%S}.png", now);
}

Scenario loadInitialScenario(const AppOptions& options)
{
    const std::filesystem::path path =
        options.scenarioPath.value_or(dataDirectory() / "presets" / "island_three.toml");
    auto scenario = loadScenario(path);
    if (scenario)
    {
        return *scenario;
    }
    log::error("{}", scenario.error().describe());
    log::warn("Using the built-in Island Three instead");
    Scenario fallback;
    fallback.title = "Island Three";
    return fallback;
}

/// Builds geometry and meshes; safe to run on a worker thread.
GeneratedWorld generateWorld(const OneillCylinderSpec& spec)
{
    const auto     start = std::chrono::steady_clock::now();
    GeneratedWorld world;
    try
    {
        world.geometry = std::make_shared<const HabitatGeometry>(spec);
        // About 1000 cells around the circumference: 25 m for Island Three, finer for small ones.
        const double cell = std::clamp(2.0 * kPi * spec.radiusM / 1000.0, 2.0, 25.0);
        world.meshes =
            buildHabitatMeshes(*world.geometry, MeshingSettings{.cellSizeM      = cell,
                                                                .chunkSizeM     = 40.0 * cell,
                                                                .glassCellSizeM = 20.0 * cell,
                                                                .threads        = 0});
    }
    catch (const std::exception& e)
    {
        world.geometry.reset();
        world.error = e.what();
    }
    world.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return world;
}

/// A file name made from a habitat title.
std::string fileNameFor(std::string_view title)
{
    std::string name;
    for (const char c : title)
    {
        const bool keep = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                          (c >= '0' && c <= '9') || c == '-' || c == '_';
        if (keep)
        {
            name += c;
        }
        else if (c == ' ' && !name.empty() && name.back() != '_')
        {
            name += '_';
        }
    }
    return name.empty() ? std::string("habitat") : name;
}

void setSaveName(EditorState& editor, std::string_view title)
{
    editor.saveName.fill('\0');
    const std::size_t length = std::min(title.size(), editor.saveName.size() - 1);
    std::ranges::copy(title.substr(0, length), editor.saveName.begin());
}

Vec3d radial(double theta)
{
    return {std::cos(theta), std::sin(theta), 0.0};
}

void printBenchmark(std::vector<double> frameMs, const GpuInfo& gpu)
{
    if (frameMs.empty())
    {
        return;
    }
    std::ranges::sort(frameMs);
    const auto percentile = [&](double p) {
        const auto index = static_cast<std::size_t>(p * static_cast<double>(frameMs.size() - 1));
        return frameMs[index];
    };
    double sum = 0.0;
    for (const double ms : frameMs)
    {
        sum += ms;
    }
    std::println(
        "Benchmark on {}: {} frames, average {:.2f} ms ({:.0f} fps), p50 {:.2f} ms, "
        "p95 {:.2f} ms, p99 {:.2f} ms, worst {:.2f} ms",
        gpu.deviceName, frameMs.size(), sum / static_cast<double>(frameMs.size()),
        1000.0 * static_cast<double>(frameMs.size()) / sum, percentile(0.5), percentile(0.95),
        percentile(0.99), frameMs.back());
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
    renderer_(device_, defaultShaderDirectory(), generateStarField(kStarSeed, kStarCount)),
    input_(window_.get()),
    scenario_(loadInitialScenario(options_))
{
    hudSettings_.mirrorAngleDeg = static_cast<float>(
        options_.mirrorAngleDeg.value_or(scenario_.habitat.mirrors.openingAngleDeg));
    editor_.draft = scenario_.habitat;
    setSaveName(editor_, scenario_.title);
    refreshScenarioList();

    GeneratedWorld world = generateWorld(scenario_.habitat);
    if (!world.geometry)
    {
        throw std::runtime_error(std::format("Cannot build the habitat: {}", world.error));
    }
    adoptWorld(std::move(world), true);

    if (options_.camera)
    {
        applyCameraPose(*options_.camera);
    }
    else if (options_.view)
    {
        applyView(*options_.view);
    }
    if (options_.benchmark)
    {
        benchmark_.emplace();
        applyView(kBenchmarkViews.front());
    }
}

// ---- Habitats ----------------------------------------------------------------------------------

void Application::adoptWorld(GeneratedWorld world, bool placeAtStartPoint)
{
    auto        gpuWorld = std::make_unique<GpuWorld>(device_.get(), world.meshes);
    const Vec3d eye      = player_.eyePosition();
    const bool  hadWorld = geometry_ != nullptr;
    geometry_            = std::move(world.geometry);
    world_               = std::move(gpuWorld);
    metrics_             = computeMetrics(geometry_->spec());
    ball_.reset();
    status_ = std::format("Generated {} in {:.1f} s: {:.2f} M triangles", scenario_.title,
                          world.seconds, static_cast<double>(world_->triangleCount()) / 1e6);
    log::info("{}", status_);

    if (placeAtStartPoint || !hadWorld)
    {
        placeAtStart();
    }
    else if (player_.locomotion() == Locomotion::Walk)
    {
        player_.placeOnGround(*geometry_, eye.z, HabitatGeometry::angleOf(eye));
    }
    else
    {
        player_.teleport(eye);  // the next step keeps it inside the new habitat
    }
}

void Application::startGeneration(const OneillCylinderSpec& spec, bool placeAtStartPoint)
{
    if (pending_.valid())
    {
        return;  // one at a time
    }
    placeWhenReady_ = placeAtStartPoint;
    pending_        = std::async(std::launch::async, generateWorld, spec);
    status_         = "Generating the habitat...";
}

void Application::pollGeneration()
{
    if (!pending_.valid() ||
        pending_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
        return;
    }
    GeneratedWorld world = pending_.get();
    if (!world.geometry)
    {
        status_ = std::format("Could not build the habitat: {}", world.error);
        return;
    }
    try
    {
        adoptWorld(std::move(world), placeWhenReady_);
    }
    catch (const std::exception& e)
    {
        status_ = std::format("Could not upload the habitat: {}", e.what());
        log::error("{}", status_);
    }
}

void Application::loadScenarioFile(const std::filesystem::path& path)
{
    auto loaded = loadScenario(path);
    if (!loaded)
    {
        status_ = loaded.error().describe();
        return;
    }
    scenario_                   = std::move(*loaded);
    editor_.draft               = scenario_.habitat;
    hudSettings_.mirrorAngleDeg = static_cast<float>(scenario_.habitat.mirrors.openingAngleDeg);
    setSaveName(editor_, scenario_.title);
    startGeneration(scenario_.habitat, true);
}

void Application::saveDraft()
{
    const std::string title(editor_.saveName.data());
    Scenario          copy               = scenario_;
    copy.title                           = title.empty() ? scenario_.title : title;
    copy.habitat                         = editor_.draft;
    copy.habitat.mirrors.openingAngleDeg = static_cast<double>(hudSettings_.mirrorAngleDeg);
    const std::filesystem::path path =
        userDirectory() / "habitats" / (fileNameFor(copy.title) + ".toml");
    const auto saved = saveScenario(copy, path);
    status_          = saved ? std::format("Saved {}", path.string()) : saved.error();
    refreshScenarioList();
}

void Application::refreshScenarioList()
{
    editor_.scenarios.clear();
    for (const std::filesystem::path& directory :
         {dataDirectory() / "presets", userDirectory() / "habitats"})
    {
        std::error_code error;
        for (const auto& entry : std::filesystem::directory_iterator(directory, error))
        {
            if (entry.path().extension() == ".toml")
            {
                editor_.scenarios.push_back(entry.path());
            }
        }
    }
    std::ranges::sort(editor_.scenarios);
}

void Application::placeAtStart()
{
    const HabitatGeometry& geometry = *geometry_;
    const int              strips   = geometry.stripCount();
    const int              valley   = ((scenario_.start.valley % strips) + strips) % strips;
    player_.setLocomotion(Locomotion::Walk);
    player_.placeOnGround(geometry, scenario_.start.zM, geometry.landCenter(valley));
    look_.setFrame(player_.viewUp(), kNorth);
    look_.setAngles(degreesToRadians(scenario_.start.headingDeg), 0.0);
}

void Application::applyView(std::string_view name)
{
    const HabitatGeometry& geometry = *geometry_;
    const int              strips   = geometry.stripCount();
    const double           valley =
        geometry.landCenter(((scenario_.start.valley % strips) + strips) % strips);
    // Keep viewpoints on the floor, away from its ends (the margin shrinks for small habitats).
    const double margin = std::min(500.0, 0.25 * (geometry.floorZMax() - geometry.floorZMin()));
    const double startZ = std::clamp(scenario_.start.zM, geometry.floorZMin() + margin,
                                     geometry.floorZMax() - margin);
    const auto   walk   = [&](double z, double theta, double yawDeg, double pitchDeg) {
        player_.setLocomotion(Locomotion::Walk);
        player_.placeOnGround(geometry, z, theta);
        look_.setFrame(player_.viewUp(), kNorth);
        look_.setAngles(degreesToRadians(yawDeg), degreesToRadians(pitchDeg));
    };
    const auto fly = [&](const Vec3d& eye, double yawDeg, double pitchDeg) {
        player_.setLocomotion(Locomotion::Fly);
        player_.teleport(eye);
        look_.setFrame(player_.viewUp(), kNorth);
        look_.setAngles(degreesToRadians(yawDeg), degreesToRadians(pitchDeg));
    };

    if (name == "valley")
    {
        walk(startZ, valley, 0.0, 6.0);
    }
    else if (name == "lookup")
    {
        walk(startZ, valley, 0.0, 75.0);
    }
    else if (name == "window")
    {
        // Off the lattice ribs (every 80 m), looking down and ahead at the mirror.
        walk(startZ + 37.0, geometry.windowCenter(0) + (19.0 / geometry.radius()), 0.0, -55.0);
    }
    else if (name == "endcap")
    {
        walk(geometry.floorZMin() + 800.0, valley, 180.0, 10.0);
    }
    else if (name == "ramp")
    {
        walk(geometry.floorZMin() - 2500.0, valley, 180.0, 15.0);
    }
    else if (name == "sunward")
    {
        walk(geometry.floorZMax() - 1500.0, valley, 0.0, 20.0);
    }
    else if (name == "axis")
    {
        fly((radial(valley) * 20.0) + Vec3d(0.0, 0.0, geometry.walkableZMin() + 1500.0), 0.0, 0.0);
    }
    else if (name == "overview")
    {
        fly((radial(valley) * (0.35 * geometry.radius())) +
                Vec3d(0.0, 0.0, geometry.floorZMin() + 200.0),
            0.0, -8.0);
    }
    else
    {
        status_ = std::format("Unknown view '{}'", name);
        log::warn("{}", status_);
    }
}

void Application::applyCameraPose(const CameraPose& pose)
{
    const Vec3d eye(pose.x, pose.y, pose.z);
    player_.teleport(eye);
    const double height = geometry_->ground(eye).heightAboveGround - player_.settings.eyeHeight;
    if (height > 0.5)
    {
        player_.setLocomotion(Locomotion::Fly);
    }
    else
    {
        player_.setLocomotion(Locomotion::Walk);
        player_.placeOnGround(*geometry_, pose.z, HabitatGeometry::angleOf(eye));
    }
    look_.setFrame(player_.viewUp(), kNorth);
    look_.setAngles(degreesToRadians(pose.yawDeg), degreesToRadians(pose.pitchDeg));
}

// ---- Frames ------------------------------------------------------------------------------------

Camera Application::camera() const
{
    Camera camera;
    camera.position    = player_.eyePosition();
    camera.orientation = look_.orientation();
    return camera;
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
        .gpu            = &device_.info(),
        .samples        = renderer_.sceneFormats().samples,
        .fps            = fps_,
        .frameMs        = frameMs_,
        .chunksDrawn    = chunksDrawn_,
        .trianglesDrawn = trianglesDrawn_,
        .title          = scenario_.title,
        .geometry       = geometry_.get(),
        .metrics        = &metrics_,
        .player         = &player_,
        .mouseCaptured  = input_.mouseCaptured(),
        .generating     = pending_.valid(),
        .status         = status_,
        .throwReport    = ball_ ? std::optional<ThrowReport>(ball_->report) : std::nullopt,
    };
    return drawHud(model, hudSettings_, editor_, player_.settings);
}

void Application::applyInput(const InputFrame& input, const HudActions& actions)
{
    look_.setFrame(player_.viewUp(), kNorth);
    look_.applyLook(-input.lookDelta.x * kLookSensitivity, -input.lookDelta.y * kLookSensitivity);

    if (input.toggleLocomotion || actions.toggleLocomotion)
    {
        player_.setLocomotion(player_.locomotion() == Locomotion::Walk ? Locomotion::Fly
                                                                       : Locomotion::Walk);
    }
    if (input.toggleComfort)
    {
        player_.settings.comfortMode = !player_.settings.comfortMode;
        status_ = player_.settings.comfortMode ? "Comfort mode: no Coriolis force on you"
                                               : "Comfort mode off: jumps drift with the spin";
    }
    if (input.toggleEditor)
    {
        hudSettings_.showEditor = !hudSettings_.showEditor;
    }
    if (input.toggleHud)
    {
        showHud_ = !showHud_;
    }
    if (input.reloadShaders)
    {
        const auto reloaded = renderer_.reloadShaders();
        status_             = reloaded ? std::string("Shaders reloaded")
                                       : "Shader reload failed:\n" + reloaded.error();
    }
    if (input.wheel != 0.0 && player_.locomotion() == Locomotion::Fly)
    {
        double& speed = player_.settings.flySpeed;
        speed = std::clamp(speed * std::pow(kWheelStep, input.wheel), kMinFlySpeed, kMaxFlySpeed);
    }
    if (input.throwBall || actions.throwBall)
    {
        throwBall();
    }
    if (actions.regenerate)
    {
        scenario_.habitat = editor_.draft;
        startGeneration(editor_.draft, false);
    }
    if (actions.save)
    {
        saveDraft();
    }
    if (actions.load)
    {
        loadScenarioFile(*actions.load);
    }
}

void Application::simulate(const MoveIntent& intent, double realSeconds)
{
    const int           steps = clock_.advance(realSeconds);
    const double        dt    = clock_.stepSeconds();
    const RotatingFrame frame(geometry_->omega());
    for (int step = 0; step < steps; ++step)
    {
        look_.setFrame(player_.viewUp(), kNorth);
        player_.step(intent, look_, *geometry_, dt);
        spinPhase_ = std::fmod(spinPhase_ + (geometry_->omega() * dt), 2.0 * kPi);
        if (ball_ && !ball_->resting)
        {
            stepFreeBody(ball_->state, frame, Vec3d(0.0), dt);
            const GroundSample ground = geometry_->ground(ball_->state.position);
            if (ground.heightAboveGround < kBallRadius)
            {
                const Vec3d up = HabitatGeometry::localUp(ball_->state.position);
                ball_->state.position += up * (kBallRadius - ground.heightAboveGround);
                ball_->resting = true;
            }
        }
    }
}

void Application::throwBall()
{
    const HabitatGeometry& geometry = *geometry_;
    const RotatingFrame    frame(geometry.omega());
    const Vec3d            eye = player_.eyePosition();
    const Vec3d            up  = HabitatGeometry::localUp(eye);
    const BodyState start{.position = eye + (look_.forward() * 0.4) - (up * 0.3),  // from the hand
                          .velocity = (look_.forward() * kThrowSpeed) + player_.velocity()};
    const auto      impact = predictImpact(frame, geometry, start, kMaxFlightTime);

    ThrownBall ball;
    ball.state = start;
    if (impact)
    {
        const auto steps = static_cast<int>(impact->time / kPathStep);
        for (int i = 1; i <= steps; ++i)
        {
            ball.path.push_back(propagateFreeFlight(frame, start, i * kPathStep).position);
        }
        ball.path.push_back(impact->state.position);

        // The same throw on a planet with this gravity: a parabola over a flat plane.
        const double gravity = geometry.gravityAt(std::hypot(start.position.x, start.position.y));
        const double height  = geometry.ground(start.position).heightAboveGround;
        const double upSpeed = glm::dot(start.velocity, up);
        const double tGhost =
            (upSpeed + std::sqrt((upSpeed * upSpeed) + (2.0 * gravity * height))) / gravity;
        const auto ghostSteps = static_cast<int>(tGhost / kPathStep);
        for (int i = 1; i <= ghostSteps; ++i)
        {
            ball.ghostPath.push_back(
                propagateUniformGravity(start, up, gravity, i * kPathStep).position);
        }
        const Vec3d ghostImpact = propagateUniformGravity(start, up, gravity, tGhost).position;
        ball.ghostPath.push_back(ghostImpact);

        Vec3d horizontal = start.velocity - (glm::dot(start.velocity, up) * up);
        horizontal =
            glm::length(horizontal) > 1e-6 ? glm::normalize(horizontal) : look_.horizontalForward();
        const Vec3d left       = glm::cross(up, horizontal);
        const Vec3d difference = impact->state.position - ghostImpact;
        ball.report            = {.flightTime = impact->time,
                                  .range      = glm::distance(start.position, impact->state.position),
                                  .lateral    = glm::dot(difference, left),
                                  .along      = glm::dot(difference, horizontal)};
    }
    ball_ = std::move(ball);
}

std::vector<Marker> Application::markers() const
{
    std::vector<Marker> result;
    if (!ball_)
    {
        return result;
    }
    for (const Vec3d& point : ball_->ghostPath)
    {
        result.push_back({.position    = point,
                          .halfExtents = Vec3f(0.03F),
                          .color       = Vec3f(0.6F),
                          .emission    = Vec3f(0.35F),
                          .shape       = MarkerShape::Sphere});
    }
    for (const Vec3d& point : ball_->path)
    {
        result.push_back({.position    = point,
                          .halfExtents = Vec3f(0.035F),
                          .color       = Vec3f(0.2F, 0.5F, 1.0F),
                          .emission    = Vec3f(0.1F, 0.35F, 0.9F),
                          .shape       = MarkerShape::Sphere});
    }
    result.push_back({.position    = ball_->state.position,
                      .halfExtents = Vec3f(static_cast<float>(kBallRadius)),
                      .color       = Vec3f(0.9F, 0.35F, 0.1F),
                      .emission    = Vec3f(0.3F, 0.08F, 0.02F),
                      .shape       = MarkerShape::Sphere});
    return result;
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

    gpu::LightingSettings lighting;
    lighting.haze                    = static_cast<double>(hudSettings_.haze);
    const std::vector<Marker> shapes = markers();
    const SceneView           scene{
        .camera  = camera(),
        .world   = world_.get(),
        .habitat = gpu::makeHabitatUniforms(
            *geometry_, degreesToRadians(static_cast<double>(hudSettings_.mirrorAngleDeg)),
            lighting),
        .sky = gpu::makeSkyUniforms(spinPhase_, static_cast<double>(hudSettings_.starBrightness)),
        .markers  = shapes,
        .exposure = hudSettings_.exposure,
    };
    const FrameResult result = renderer_.renderFrame(scene, frameOptions);
    if (result.presented)
    {
        chunksDrawn_    = result.chunksDrawn;
        trianglesDrawn_ = result.trianglesDrawn;
    }
    else
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

std::optional<int> Application::stepBenchmark(double realSeconds)
{
    if (!benchmark_)
    {
        return std::nullopt;
    }
    Benchmark& benchmark = *benchmark_;
    benchmark.viewSeconds += realSeconds;
    if (benchmark.viewSeconds > kBenchmarkWarmup)
    {
        benchmark.frameMs.push_back(realSeconds * 1000.0);
    }
    look_.applyLook(kBenchmarkPanRate * realSeconds, 0.0);
    if (benchmark.viewSeconds >= kBenchmarkViewSeconds)
    {
        benchmark.viewSeconds = 0.0;
        if (++benchmark.view >= kBenchmarkViews.size())
        {
            printBenchmark(benchmark.frameMs, device_.info());
            return EXIT_SUCCESS;
        }
        applyView(kBenchmarkViews.at(benchmark.view));
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
        pollGeneration();
        if (const auto exitCode = stepBenchmark(realSeconds))
        {
            return *exitCode;
        }

        // The HUD is built before the simulation step so its button clicks apply this frame.
        imgui_.beginFrame();
        const HudActions actions = drawUi();
        ImDrawData*      ui      = imgui_.endFrame();
        applyInput(input, actions);
        simulate(benchmark_ ? MoveIntent{} : input.move, realSeconds);

        if (const auto exitCode = render(frame, input.screenshot, ui))
        {
            return *exitCode;
        }
    }
}

}  // namespace StarshipSimulator
