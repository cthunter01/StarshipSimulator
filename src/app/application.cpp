#include "application.h"

#include <imgui.h>

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
#include <functional>
#include <future>
#include <limits>
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

#include "StarshipSimulator/audio/audio_device.h"
#include "StarshipSimulator/core/app_options.h"
#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/sky_objects.h"
#include "StarshipSimulator/core/astro/star_catalog.h"
#include "StarshipSimulator/core/audio/soundscape.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/landscape.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/habitat/weather.h"
#include "StarshipSimulator/core/log.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/physics/player_controller.h"
#include "StarshipSimulator/core/physics/rotating_frame.h"
#include "StarshipSimulator/core/procgen/birds.h"
#include "StarshipSimulator/core/procgen/buildings.h"
#include "StarshipSimulator/core/procgen/clouds.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/hull_mesh.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/trees.h"
#include "StarshipSimulator/core/rng.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/physics/physics_world.h"
#include "StarshipSimulator/render/gpu_birds.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/gpu_landscape.h"
#include "StarshipSimulator/render/gpu_props.h"
#include "StarshipSimulator/render/gpu_settlements.h"
#include "StarshipSimulator/render/gpu_trees.h"
#include "StarshipSimulator/render/gpu_world.h"
#include "StarshipSimulator/render/passes/marker_pass.h"
#include "StarshipSimulator/render/renderer.h"
#include "StarshipSimulator/render/shader_library.h"
#include "StarshipSimulator/render/shadow_map.h"
#include "hud.h"
#include "sdl_input.h"
#include "sky_loader.h"

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
constexpr double        kBallRadius           = 0.12;  // m (PropKind::Ball)
constexpr std::size_t   kMaxThrownBalls       = 12;    // older ones vanish
constexpr double        kKickReach            = 2.5;   // m
constexpr double        kPathStep             = 0.05;  // s between trajectory dots
constexpr double        kMaxFlightTime        = 60.0;  // s
constexpr double        kBenchmarkViewSeconds = 3.0;
constexpr double        kBenchmarkWarmup      = 0.5;
constexpr double        kBenchmarkPanRate     = degreesToRadians(20.0);  // per second
constexpr std::array<std::string_view, 9> kBenchmarkViews{
    "valley", "river", "town", "lookup", "window", "ramp", "sunward", "axis", "overview"};
constexpr Vec3d  kNorth(0.0, 0.0, 1.0);       // the "north" of the look rig: toward the sunward end
constexpr double kStarMagnitudeLimit = 7.5;   // a little fainter than the naked eye in a dark sky
constexpr double kMilkyWayScale      = 0.02;  // NASA's map (0..1) to scene radiance
constexpr double kNightExposure      = 60.0;  // how far the view brightens in darkness
constexpr double kLabelSeconds       = 8.0;   // how long an identified name stays up
constexpr double kLabelFadeSeconds   = 2.0;
constexpr float  kBinocularsFovDeg   = 8.0F;
constexpr float  kNormalFovDeg       = 70.0F;
constexpr double kCaptureStepSeconds = 1.0 / 60.0;
constexpr double kShadowHalfExtentM  = 300.0;  // the trees' shadows reach this far around you

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

/// Places kept free of trees: where the visit starts and the viewpoints near it.
std::vector<Clearing> startClearings(const HabitatGeometry& geometry, const StartSpec& start)
{
    const int             strips = geometry.stripCount();
    const int             valley = ((start.valley % strips) + strips) % strips;
    const double          theta  = geometry.landCenter(valley);
    const double          z      = std::clamp(start.zM, geometry.floorZMin(), geometry.floorZMax());
    std::vector<Clearing> clearings{{.centre = geometry.surfacePoint(z, theta), .radiusM = 25.0}};
    if (geometry.landscape().hasRivers())
    {
        const double riverZ = z + 300.0;  // the "river" view
        const double bank =
            geometry.landscape().riverAngle(valley, riverZ) +
            (((0.5 * geometry.spec().terrain.riverWidthM) + 12.0) / geometry.radius());
        clearings.push_back({.centre = geometry.surfacePoint(riverZ, bank), .radiusM = 20.0});
    }
    return clearings;
}

/// Builds geometry and meshes; safe to run on a worker thread.
GeneratedWorld generateWorld(const OneillCylinderSpec& spec, const StartSpec& visitStart)
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
                                                                .threads        = 0,
                                                                .terrain        = false});
        world.hull = buildHullMesh(*world.geometry);
        // The terrain on a grid of about 4 m for Island Three (about 6000 cells around).
        const double terrainCell = 2.0 * kPi * spec.radiusM / 6144.0;
        world.terrain = std::make_shared<TerrainGrid>(sampleTerrain(*world.geometry, terrainCell));
        world.lod.emplace(*world.terrain, *world.geometry, 50.0 * world.terrain->layout.cellArcM);
        // Towns and farms, then the woods around them.
        world.settlements = planSettlements(*world.geometry, *world.terrain);
        stampSettlements(*world.terrain, world.settlements);
        const TreeSettings trees{
            .clearings = startClearings(*world.geometry, visitStart),
            .keepOff   = [&settlements = world.settlements](double z, double theta) {
                return settlements.keepsTreesOff(z, theta);
            }};
        world.trees =
            std::make_shared<TreeLayer>(plantTrees(*world.geometry, *world.terrain, trees));
        addStandingTrees(*world.trees, world.settlements);
        world.settlementMeshes = buildSettlementMeshes(world.settlements);
        // The clouds, wrapped once round the habitat and once along it.
        world.clouds =
            makeCloudMap(hashSeed(spec.terrain.seed, 0xC10D), 2.0 * kPi * world.geometry->radius(),
                         world.geometry->profile().zMax() - world.geometry->profile().zMin());
        // The physics: what the towns built, and everything lying about in them.
        world.physics = std::make_unique<PhysicsWorld>(world.geometry, world.terrain);
        world.physics->addColliders(settlementColliders(world.settlements));
        world.physics->setTrees(world.trees);
        for (const PropPlacement& prop : world.settlements.props)
        {
            world.physics->addProp(prop);
        }
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
    hudSettings_.followSchedule = scenario_.day.enabled && !options_.mirrorAngleDeg;
    if (!options_.mute && !options_.capturePath && !options_.benchmark)
    {
        audio_ = std::make_unique<AudioDevice>(scenario_.habitat.terrain.seed);
    }
    if (options_.weather)
    {
        const Weather held        = weatherNamed(*options_.weather).value_or(Weather{});
        hudSettings_.forceWeather = true;
        hudSettings_.cloudCover   = static_cast<float>(held.cloudCover);
        hudSettings_.rain         = static_cast<float>(held.rain);
        hudSettings_.wetness      = static_cast<float>(held.wetness);
        hudSettings_.mist         = static_cast<float>(held.mist);
        hudSettings_.windSpeedMS  = static_cast<float>(held.windAlongMS);
    }
    if (options_.timeScale)
    {
        hudSettings_.timePaused = *options_.timeScale <= 0.0;
        hudSettings_.timeScale  = hudSettings_.timePaused ? 1.0 : *options_.timeScale;
    }
    simTime_ = options_.startTime.value_or(scenario_.sky.start);
    hudSettings_.fieldOfViewDeg =
        static_cast<float>(options_.fieldOfViewDeg.value_or(kNormalFovDeg));
    editor_.draft = scenario_.habitat;
    setSaveName(editor_, scenario_.title);
    refreshScenarioList();

    GeneratedWorld world = generateWorld(scenario_.habitat, scenario_.start);
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
    startSkyLoad();
    updateSky();
    if (options_.lookAt)
    {
        lookAtName(*options_.lookAt);
    }
}

// ---- Habitats ----------------------------------------------------------------------------------

void Application::adoptWorld(GeneratedWorld world, bool placeAtStartPoint)
{
    if (!world.lod)
    {
        throw std::runtime_error("the habitat's terrain was not generated");
    }
    auto gpuWorld = std::make_unique<GpuWorld>(device_.get(), world.meshes);
    auto gpuLandscape =
        std::make_unique<GpuLandscape>(device_.get(), *world.terrain, std::move(*world.lod));
    auto gpuTrees = std::make_unique<GpuTrees>(device_.get(), *world.trees);
    auto gpuSettlements =
        std::make_unique<GpuSettlements>(device_.get(), world.settlementMeshes, world.settlements);
    if (!gpuProps_)
    {
        gpuProps_ = std::make_unique<GpuProps>(device_.get());
        gpuBirds_ = std::make_unique<GpuBirds>(device_.get());
    }
    renderer_.setHull(world.hull);
    renderer_.setCloudMap(world.clouds);
    const Vec3d eye      = player_.eyePosition();
    const bool  hadWorld = geometry_ != nullptr;
    geometry_            = std::move(world.geometry);
    world_               = std::move(gpuWorld);
    landscape_           = std::move(gpuLandscape);
    trees_               = std::move(gpuTrees);
    gpuSettlements_      = std::move(gpuSettlements);
    terrain_             = std::move(world.terrain);
    settlements_         = std::make_shared<const Settlements>(std::move(world.settlements));
    physics_             = std::move(world.physics);
    player_.setMover(&physics_->character());
    metrics_ = computeMetrics(geometry_->spec());
    ball_.reset();
    thrown_.clear();
    const auto counted = [](std::size_t n, std::string_view one, std::string_view many) {
        return std::format("{} {}", n, n == 1 ? one : many);
    };
    const auto trees = static_cast<double>(trees_->treeCount());
    status_          = std::format(
        "Generated {} in {:.1f} s: {} and {} ({}), {}", scenario_.title, world.seconds,
        counted(settlements_->townCount(), "town", "towns"),
        counted(settlements_->places.size() - settlements_->townCount(), "farm", "farms"),
        counted(settlements_->buildings.size(), "building", "buildings"),
        trees >= 1e6 ? std::format("{:.1f} million trees", trees / 1e6)
                     : counted(trees_->treeCount(), "tree", "trees"));
    log::info("{}", status_);
    log::info("Terrain, trees and towns use about {:.0f} MB of GPU memory",
              static_cast<double>(landscape_->memoryBytes() + trees_->memoryBytes() +
                                  gpuSettlements_->memoryBytes()) /
                  1e6);

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
    pending_        = std::async(std::launch::async, generateWorld, spec, scenario_.start);
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
    hudSettings_.followSchedule = scenario_.day.enabled;
    simTime_                    = scenario_.sky.start;
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
    copy.sky.start                       = simTime_;  // the visit resumes from this moment
    copy.day.enabled                     = hudSettings_.followSchedule;
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

void Application::walkTo(double z, double theta, double yawDeg, double pitchDeg)
{
    player_.setLocomotion(Locomotion::Walk);
    player_.placeOnGround(*geometry_, z, theta);
    look_.setFrame(player_.viewUp(), kNorth);
    look_.setAngles(degreesToRadians(yawDeg), degreesToRadians(pitchDeg));
}

void Application::flyTo(const Vec3d& eye, double yawDeg, double pitchDeg)
{
    player_.setLocomotion(Locomotion::Fly);
    player_.teleport(eye);
    look_.setFrame(player_.viewUp(), kNorth);
    look_.setAngles(degreesToRadians(yawDeg), degreesToRadians(pitchDeg));
}

int Application::startValley() const
{
    const int strips = geometry_->stripCount();
    return ((scenario_.start.valley % strips) + strips) % strips;
}

double Application::startViewZ() const
{
    // Keep viewpoints on the floor, away from its ends (the margin shrinks for small habitats).
    const HabitatGeometry& geometry = *geometry_;
    const double margin = std::min(500.0, 0.25 * (geometry.floorZMax() - geometry.floorZMin()));
    return std::clamp(scenario_.start.zM, geometry.floorZMin() + margin,
                      geometry.floorZMax() - margin);
}

void Application::applyView(std::string_view name)
{
    const HabitatGeometry& geometry = *geometry_;
    const double           valley   = geometry.landCenter(startValley());
    const double           startZ   = startViewZ();
    if (name == "river" || name == "lake")
    {
        applyWaterView(name);
    }
    else if (name == "town" || name == "rooftops" || name == "street")
    {
        applyTownView(name);
    }
    else if (name == "valley" || name == "lookup")
    {
        walkTo(startZ, valley, 0.0, name == "valley" ? 6.0 : 75.0);
    }
    else if (name == "window")
    {
        // Off the lattice ribs (every 80 m), looking down and ahead at the mirror.
        walkTo(startZ + 37.0, geometry.windowCenter(0) + (19.0 / geometry.radius()), 0.0, -55.0);
    }
    else if (name == "endcap")
    {
        walkTo(geometry.floorZMin() + 800.0, valley, 180.0, 10.0);
    }
    else if (name == "ramp")
    {
        walkTo(geometry.floorZMin() - 2500.0, valley, 180.0, 15.0);
    }
    else if (name == "sunward")
    {
        walkTo(geometry.floorZMax() - 1500.0, valley, 0.0, 20.0);
    }
    else if (name == "axis")
    {
        flyTo((radial(valley) * 20.0) + Vec3d(0.0, 0.0, geometry.walkableZMin() + 1500.0), 0.0,
              0.0);
    }
    else if (name == "overview")
    {
        flyTo((radial(valley) * (0.35 * geometry.radius())) +
                  Vec3d(0.0, 0.0, geometry.floorZMin() + 200.0),
              0.0, -8.0);
    }
    else
    {
        status_ = std::format("Unknown view '{}'", name);
        log::warn("{}", status_);
    }
}

void Application::applyWaterView(std::string_view name)
{
    // On the bank (or the lake shore), looking along the water.
    const HabitatGeometry& geometry  = *geometry_;
    const Landscape&       landscape = geometry.landscape();
    const int              valley    = startValley();
    const auto             lake      = std::ranges::find(landscape.lakes(), valley, &Lake::valley);
    if (name == "lake" && lake != landscape.lakes().end())
    {
        walkTo(lake->z - lake->halfLengthM - 25.0, lake->theta, 0.0, -2.0);
        return;
    }
    const double z     = startViewZ() + 300.0;
    const double theta = landscape.riverAngle(valley, z) +
                         (((0.5 * geometry.spec().terrain.riverWidthM) + 12.0) / geometry.radius());
    walkTo(z, theta, 15.0, -4.0);
}

void Application::applyTownView(std::string_view name)
{
    if (!settlements_ || settlements_->townCount() == 0)
    {
        status_ = "There are no towns in this habitat";
        return;
    }
    // The start valley's first town (or any): on its square facing the hall, on its main street,
    // or above it.
    const Settlements& plan = *settlements_;
    std::size_t        town = 0;
    for (std::size_t i = 0; i < plan.places.size(); ++i)
    {
        if (plan.places[i].kind == SettlementKind::Town && plan.places[i].valley == startValley())
        {
            town = i;
            break;
        }
    }
    const Settlement& place = plan.places[town];
    Vec2d             square(0.0);
    Vec2d             hall(0.0, 30.0);
    for (const Furniture& item : plan.furniture)
    {
        square = item.settlement == town && item.kind == FurnitureKind::Fountain ? item.position
                                                                                 : square;
    }
    for (const Building& b : plan.buildings)
    {
        hall = b.settlement == town && b.use == BuildingUse::Hall ? b.centre : hall;
    }
    const Vec2d away    = glm::normalize(square - hall);
    const auto  yawFrom = [](const Vec2d& from, const Vec2d& to) {
        // Yaw is measured from the axis (plan +y) toward the spin (plan +x).
        return radiansToDegrees(std::atan2(to.x - from.x, to.y - from.y));
    };
    if (name == "rooftops")
    {
        const Vec2d over = square + (away * 160.0);
        flyTo(place.plane.point(over, 70.0), yawFrom(over, square), -18.0);
        return;
    }
    // On the main street (the widest), a little way from the square, looking along it.
    const Street* main = nullptr;
    for (const Street& street : place.streets)
    {
        const double d = glm::distance(0.5 * (street.from + street.to), square);
        if (street.halfWidth > 5.0 && d > 40.0 &&
            (main == nullptr || d < glm::distance(0.5 * (main->from + main->to), square)))
        {
            main = &street;
        }
    }
    if (name == "street" && main != nullptr)
    {
        const Vec2d dir   = glm::normalize(main->to - main->from);
        const Vec2d stand = main->from + (Vec2d(-dir.y, dir.x) * 2.0);
        const bool  back  = glm::dot(dir, square - stand) > 0.0;  // away from the square
        walkTo(place.plane.z(stand.y), place.plane.theta(stand.x),
               yawFrom(stand, stand + (back ? -dir : dir)), 3.0);
        return;
    }
    const Vec2d stand = square + (away * 11.0);
    walkTo(place.plane.z(stand.y), place.plane.theta(stand.x), yawFrom(stand, hall), 6.0);
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

// ---- The sky and the clock -------------------------------------------------------------------

void Application::startSkyLoad()
{
    const std::filesystem::path directory = findSkyDataDirectory(dataDirectory());
    if (options_.capturePath || options_.benchmark)
    {
        adoptSky(loadSkyData(directory, kStarMagnitudeLimit));  // captures need the real sky
        return;
    }
    pendingSky_ = std::async(std::launch::async, loadSkyData, directory, kStarMagnitudeLimit);
}

void Application::pollSky()
{
    if (pendingSky_.valid() &&
        pendingSky_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        adoptSky(pendingSky_.get());
    }
}

void Application::adoptSky(SkyData data)
{
    try
    {
        renderer_.setSkyImages(data.images);
        if (data.catalog)
        {
            renderer_.setStars(astro::toGpuStars(*data.catalog));
            catalog_ = std::move(data.catalog);
        }
    }
    catch (const std::exception& e)
    {
        data.problems.emplace_back(e.what());
    }
    skyStatus_ = catalog_ ? std::format("{} stars from the HYG catalog", catalog_->stars.size())
                          : std::string("Placeholder stars: the star catalog is missing");
    if (!data.problems.empty())
    {
        skyStatus_ += std::format("; {} sky file(s) missing, see the log", data.problems.size());
        for (const std::string& problem : data.problems)
        {
            log::warn("Sky data: {}", problem);
        }
    }
    log::info("Sky loaded in {:.1f} s: {}", data.seconds, skyStatus_);
    if (pendingLookAt_ && catalog_)
    {
        const std::string name = *pendingLookAt_;
        pendingLookAt_.reset();
        lookAtName(name);
    }
}

void Application::advanceClock(double realSeconds)
{
    if (!hudSettings_.timePaused)
    {
        simTime_ = simTime_.plusSeconds(realSeconds * hudSettings_.timeScale);
    }
    identifiedAge_ += realSeconds;
    animationSeconds_ = std::fmod(animationSeconds_ + realSeconds, 3600.0);
}

void Application::updateWeather(double realSeconds)
{
    weather_ = weatherAt(scenario_.climate, scenario_.day, simTime_, scenario_.sky.utcOffsetHours,
                         scenario_.habitat.terrain.seed);
    // While the weather runs itself the sliders follow it, so holding it starts from what is
    // outside the window rather than jumping.
    if (!hudSettings_.forceWeather)
    {
        hudSettings_.cloudCover  = static_cast<float>(weather_.cloudCover);
        hudSettings_.rain        = static_cast<float>(weather_.rain);
        hudSettings_.wetness     = static_cast<float>(weather_.wetness);
        hudSettings_.mist        = static_cast<float>(weather_.mist);
        hudSettings_.windSpeedMS = static_cast<float>(weather_.windAlongMS);
    }
    else
    {
        weather_.cloudCover  = static_cast<double>(hudSettings_.cloudCover);
        weather_.rain        = static_cast<double>(hudSettings_.rain);
        weather_.wetness     = static_cast<double>(hudSettings_.wetness);
        weather_.mist        = static_cast<double>(hudSettings_.mist);
        weather_.windAlongMS = static_cast<double>(hudSettings_.windSpeedMS);
    }
    if (!hudSettings_.forceSeason)
    {
        hudSettings_.season = static_cast<float>(weather_.season);
    }
    else
    {
        weather_.season = static_cast<double>(hudSettings_.season);
    }
    weather_.look        = seasonLook(weather_.season);
    cloudSettings_.baseM = scenario_.climate.cloudBaseM;
    cloudSettings_.topM  = scenario_.climate.cloudTopM;
    // The clouds blow with the wind in real time, like the spin: the sky is not a clock.
    const double radius = std::max(geometry_->radius() - scenario_.climate.cloudBaseM, 1.0);
    cloudSettings_.driftM += weather_.windAlongMS * realSeconds;
    cloudSettings_.turnRad += (weather_.windAroundMS / radius) * realSeconds;
}

audio::SoundMix Application::soundMix() const
{
    audio::SoundMix mix;
    const Vec3d     eye    = player_.eyePosition();
    const double    theta  = HabitatGeometry::angleOf(eye);
    const double    radius = std::hypot(eye.x, eye.y);
    const auto      ground = geometry_->groundRadius(eye.z, theta);
    const double    height = ground ? *ground - radius : 1000.0;
    const double    speed  = glm::length(player_.velocity());

    // Wind: a breeze in the open, stronger up high, and the rush of air when you fly fast.
    const double breeze = weather_.windAlongMS / 10.0;
    mix.wind            = std::clamp(
        0.12 + breeze + (0.3 * glm::smoothstep(5.0, 250.0, height)) + (speed / 40.0), 0.0, 1.0);
    mix.windSpeedMS = weather_.windAlongMS + (0.5 * speed);

    // What is within earshot: woods and water on a ring of points around you.
    double woods = 0.0;
    double water = 0.0;
    for (const double reach : {0.0, 30.0, 70.0})
    {
        const double falloff = 1.0 - (reach / 100.0);
        for (int k = 0; k < (reach > 0.0 ? 6 : 1); ++k)
        {
            const double angle = (2.0 * kPi * k) / 6.0;
            const double z     = eye.z + (reach * std::cos(angle));
            const double around =
                theta + ((reach * std::sin(angle)) / std::max(geometry_->radius(), 1.0));
            woods = std::max(woods, geometry_->forestDensity(z, around) * falloff);
            water = std::max(water, geometry_->waterDepth(z, around) > 0.0 ? falloff : 0.0);
        }
    }
    const double onTheGround = 1.0 - glm::smoothstep(4.0, 90.0, height);
    const double daylight    = daylightFactor(mirrorAngle());
    mix.leaves               = std::clamp(woods * (0.45 + breeze), 0.0, 1.0) * onTheGround;
    mix.water                = water * onTheGround;
    mix.rain                 = weather_.rain;
    const bool inTown = settlements_ != nullptr && settlements_->townAt(eye.z, theta) != nullptr;
    mix.town          = inTown ? 0.8 * onTheGround : 0.0;
    mix.night         = 1.0 - glm::smoothstep(0.05, 0.4, daylight);
    // Birds sing in the woods and hedges and keep quiet in the rain; crickets take over at night.
    mix.birds  = (0.3 + (0.7 * woods)) * (1.0 - weather_.rain) * (0.4 + (0.6 * onTheGround)) *
                 (inTown ? 0.5 : 1.0);
    mix.master = static_cast<double>(hudSettings_.volume);
    return mix;
}

void Application::updateSound(double realSeconds)
{
    if (!audio_ || geometry_ == nullptr)
    {
        return;
    }
    audio_->setMix(soundMix());
    // Footsteps: one every stride while walking on the ground.
    if (player_.locomotion() == Locomotion::Walk && player_.grounded())
    {
        const Vec3d  up       = HabitatGeometry::localUp(player_.eyePosition());
        const Vec3d  velocity = player_.velocity();
        const double speed    = glm::length(velocity - (up * glm::dot(velocity, up)));
        const bool   running  = speed > 3.2;
        stride_ += speed * realSeconds;
        if (speed < 0.3)
        {
            stride_ = 0.0;
        }
        else if (stride_ >= (running ? 1.25 : 0.75))
        {
            stride_ = 0.0;
            audio_->play(running ? audio::Sound::Run : audio::Sound::Footstep,
                         0.7 + (0.3 * weather_.wetness));
        }
    }
}

void Application::updateSky()
{
    sky_            = astro::computeSky(scenario_.sky.location, simTime_);
    habitatFromSky_ = astro::habitatFromEqj(sky_.sunDirection, spinPhase_);
    if (hudSettings_.followSchedule)
    {
        const double hour  = astro::hourOfDay(simTime_, scenario_.sky.utcOffsetHours);
        const auto   today = seasonalDay(scenario_.day, scenario_.climate, weather_.season);
        hudSettings_.mirrorAngleDeg = static_cast<float>(scheduledMirrorAngleDeg(today, hour));
    }
}

void Application::setTime(astro::SimTime time)
{
    simTime_ = time;
    status_  = std::format("Time set to {}", astro::formatIsoTime(time));
    updateSky();
}

void Application::stepTimeScale(int steps)
{
    // The preset nearest the current scale, then that many presets faster or slower.
    std::size_t nearest = 0;
    for (std::size_t i = 0; i < kTimeScales.size(); ++i)
    {
        if (std::abs(std::log(kTimeScales.at(i) / hudSettings_.timeScale)) <
            std::abs(std::log(kTimeScales.at(nearest) / hudSettings_.timeScale)))
        {
            nearest = i;
        }
    }
    const auto index = std::clamp(static_cast<std::ptrdiff_t>(nearest) + steps, std::ptrdiff_t{0},
                                  static_cast<std::ptrdiff_t>(kTimeScales.size()) - 1);
    hudSettings_.timeScale  = kTimeScales.at(static_cast<std::size_t>(index));
    hudSettings_.timePaused = false;
    status_                 = std::format("Time runs at {}", timeScaleName(hudSettings_.timeScale));
}

void Application::identify()
{
    const Vec3d eye     = player_.eyePosition();
    const Vec3d forward = look_.forward();
    // Only what can be seen: the line of sight must leave the habitat through a window.
    if (const auto hit = geometry_->raycast(eye, forward, 4.0 * geometry_->spec().lengthM))
    {
        const Vec3d point = eye + (forward * *hit);
        const auto  kind  = geometry_->regionAt(point.z, HabitatGeometry::angleOf(point)).kind;
        if (kind != RegionKind::Window)
        {
            identified_.reset();
            status_ = "That is the land across the habitat. Look out through a window.";
            return;
        }
    }
    const Vec3d direction = glm::transpose(habitatFromSky_) * forward;
    identified_           = astro::identifyInSky(direction, sky_, catalog_ ? &*catalog_ : nullptr);
    identifiedAge_        = 0.0;
    status_ = identified_ ? std::format("{}: {}", identified_->name, identified_->details)
                          : std::string(
                                "Nothing bright there. Point the crosshair at a "
                                "star, a planet, Earth or the Moon.");
}

void Application::lookAtBody(astro::Body body)
{
    const auto found = std::ranges::find(sky_.bodies, body, &astro::VisibleBody::body);
    if (found == sky_.bodies.end())
    {
        status_ = std::format("{} is not in the sky model", astro::bodyName(body));
        return;
    }
    lookOut(found->direction, astro::bodyName(body));
}

void Application::lookAtName(std::string_view name)
{
    if (name == "partner")
    {
        lookAtPartner();
        return;
    }
    if (const auto body = astro::bodyFromName(name))
    {
        lookAtBody(*body);
        return;
    }
    if (!catalog_)
    {
        pendingLookAt_ = std::string(name);  // once the star catalog has loaded
        return;
    }
    if (const astro::CatalogStar* star = catalog_->find(name))
    {
        lookOut(star->direction, astro::displayName(*star));
        return;
    }
    status_ = std::format("There is no planet or named star called '{}'", name);
    log::warn("{}", status_);
}

void Application::lookAtPartner()
{
    // The partner lies along +X of the habitat at rest (spin phase 0).
    lookOut(glm::transpose(astro::habitatFromEqj(sky_.sunDirection, 0.0)) * Vec3d(1.0, 0.0, 0.0),
            "the partner cylinder");
}

void Application::lookOut(Vec3d directionEqj, std::string_view name)
{
    // directionEqj is a copy: it often points into sky_, which updateSky() below replaces.
    // Turn the habitat so the direction lies straight out from window 0 (angle 0) ...
    const Vec3d atRest = astro::habitatFromEqj(sky_.sunDirection, 0.0) * directionEqj;
    spinPhase_         = std::fmod(std::atan2(atRest.y, atRest.x) + (2.0 * kPi), 2.0 * kPi);
    updateSky();
    const Vec3d d = habitatFromSky_ * directionEqj;  // now in the x-z plane, x >= 0

    // ... and float just off the axis on the other side, looking out through the middle of it.
    const HabitatGeometry& geometry = *geometry_;
    constexpr double       kOffAxis = 60.0;
    const double           reach    = (geometry.radius() + kOffAxis) / std::max(d.x, 1e-3);
    const double           middle   = 0.5 * (geometry.floorZMin() + geometry.floorZMax());
    const double           z = std::clamp(middle - (reach * d.z), geometry.walkableZMin() + 500.0,
                                          geometry.walkableZMax() - 500.0);
    player_.setLocomotion(Locomotion::Fly);
    player_.teleport(Vec3d(-kOffAxis, 0.0, z));
    look_.setFrame(player_.viewUp(), kNorth);
    const Vec3d  up         = look_.up();
    const Vec3d  horizontal = d - (glm::dot(d, up) * up);
    const double yaw =
        std::atan2(glm::dot(horizontal, glm::cross(up, kNorth)), glm::dot(horizontal, kNorth));
    const double pitch = std::asin(std::clamp(glm::dot(d, up), -1.0, 1.0));
    look_.setAngles(yaw, pitch);

    const Vec3d exit = player_.eyePosition() + (d * reach);
    status_ = exit.z > geometry.floorZMin() && exit.z < geometry.floorZMax()
                  ? std::format(
                        "Looking at {} through a window; the spin carries it past every "
                        "{:.0f} s",
                        name, 2.0 * kPi / geometry.omega())
                  : std::format("{} is nearly along the spin axis: the endcaps hide it", name);
}

double Application::mirrorAngle() const
{
    return degreesToRadians(static_cast<double>(hudSettings_.mirrorAngleDeg));
}

double Application::autoExposure() const
{
    if (!hudSettings_.autoExposure)
    {
        return 1.0;
    }
    // Eyes adapt: about 1x in daylight, up to kNightExposure^0.8 in the dark. A cloudy day is
    // dimmer than a clear one, but it does not look it, so the eye opens for that too.
    const double daylight =
        daylightFactor(mirrorAngle()) * std::lerp(1.0, gpu::cloudShade(weather_.cloudCover), 0.45);
    return std::pow(1.0 / (daylight + (1.0 / kNightExposure)), 0.8);
}

std::optional<SkyLabel> Application::skyLabel() const
{
    if (!identified_ || identifiedAge_ > kLabelSeconds + kLabelFadeSeconds)
    {
        return std::nullopt;
    }
    const ImVec2 size   = ImGui::GetIO().DisplaySize;
    const double aspect = static_cast<double>(size.x) / std::max(1.0, static_cast<double>(size.y));
    const Vec4d  clip   = cameraRelativeViewProjection(camera(), aspect) *
                          Vec4d(habitatFromSky_ * identified_->direction, 0.0);
    if (clip.w <= 0.0)
    {
        return std::nullopt;  // behind the viewer
    }
    const double x = ((clip.x / clip.w) * 0.5) + 0.5;
    const double y = 0.5 - ((clip.y / clip.w) * 0.5);
    return SkyLabel{
        .screen  = ImVec2(static_cast<float>(x) * size.x, static_cast<float>(y) * size.y),
        .name    = identified_->name,
        .details = identified_->details,
        .fade    = static_cast<float>(std::clamp(
            (kLabelSeconds + kLabelFadeSeconds - identifiedAge_) / kLabelFadeSeconds, 0.0, 1.0))};
}

SkyModel Application::skyModel() const
{
    const astro::CalendarTime utc = astro::toCalendar(simTime_);
    SkyModel                  model;
    model.clock     = std::format("{:04}-{:02}-{:02} {:02}:{:02}:{:02} UTC", utc.year, utc.month,
                                  utc.day, utc.hour, utc.minute, static_cast<int>(utc.second));
    model.localHour = astro::hourOfDay(simTime_, scenario_.sky.utcOffsetHours);
    model.location  = astro::locationName(scenario_.sky.location);
    model.partner   = geometry_->spec().partner.enabled;
    model.sky       = &sky_;
    for (const astro::VisibleBody& body : sky_.bodies)
    {
        if (body.body == astro::Body::Earth)
        {
            model.earth = &body;
        }
        else if (body.body == astro::Body::Moon)
        {
            model.moon = &body;
        }
    }
    model.loading  = pendingSky_.valid();
    model.status   = skyStatus_;
    model.exposure = autoExposure();
    model.label    = skyLabel();
    return model;
}

std::vector<BodyDraw> Application::bodyDraws(const gpu::LightingSettings& lighting) const
{
    std::vector<const astro::VisibleBody*> disks;
    for (const astro::VisibleBody& body : sky_.bodies)
    {
        if (body.body == astro::Body::Earth || body.body == astro::Body::Moon)
        {
            disks.push_back(&body);
        }
    }
    std::ranges::sort(disks, std::ranges::greater{}, &astro::VisibleBody::distanceKm);

    // Sunlit Earth is as bright as sunlit fields; at night, with the view brightened for the dark
    // land, it would be a white blur. Like a photographer bracketing, keep the sunlit sides at
    // their daytime exposure (Earth's city lights still get the night brightening).
    const auto            compensation = static_cast<float>(1.0 / autoExposure());
    std::vector<BodyDraw> draws;
    for (const astro::VisibleBody* body : disks)
    {
        BodyDraw draw{.uniforms = gpu::makeBodyUniforms(*body, habitatFromSky_, lighting),
                      .textures = body->body == astro::Body::Earth ? BodyTextures::Earth
                                                                   : BodyTextures::Moon};
        draw.uniforms.sunlight =
            Vec4f(Vec3f(draw.uniforms.sunlight) * compensation, draw.uniforms.sunlight.w);
        draws.push_back(draw);
    }
    return draws;
}

// ---- Frames ------------------------------------------------------------------------------------

Camera Application::camera() const
{
    Camera camera;
    camera.position           = player_.eyePosition();
    camera.orientation        = look_.orientation();
    camera.verticalFovRadians = degreesToRadians(static_cast<double>(hudSettings_.fieldOfViewDeg));
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
        .trees          = trees_ ? trees_->treeCount() : 0,
        .towns          = settlements_ ? settlements_->townCount() : 0,
        .farms         = settlements_ ? settlements_->places.size() - settlements_->townCount() : 0,
        .buildings     = settlements_ ? settlements_->buildings.size() : 0,
        .movingProps   = physics_ ? physics_->awakeProps() : 0,
        .birds         = birdPoses_.size(),
        .place         = placeName(),
        .player        = &player_,
        .mouseCaptured = input_.mouseCaptured(),
        .generating    = pending_.valid(),
        .status        = status_,
        .throwReport   = ball_ ? std::optional<ThrowReport>(ball_->report) : std::nullopt,
        .sky           = skyModel(),
        .sound         = audio_ != nullptr && audio_->isOpen(),
        .weather       = weather_,
        .cloudBaseM    = scenario_.climate.cloudBaseM,
        .cloudTopM     = scenario_.climate.cloudTopM,
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
    if (input.kick)
    {
        kick();
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
    applySkyInput(input, actions);
}

void Application::applySkyInput(const InputFrame& input, const HudActions& actions)
{
    if (input.identify || actions.identify)
    {
        identify();
    }
    if (input.toggleZoom)
    {
        const bool zoomed           = hudSettings_.fieldOfViewDeg <= kBinocularsFovDeg + 0.5F;
        hudSettings_.fieldOfViewDeg = zoomed ? kNormalFovDeg : kBinocularsFovDeg;
    }
    if (input.togglePause)
    {
        hudSettings_.timePaused = !hudSettings_.timePaused;
        status_ =
            hudSettings_.timePaused ? "Time paused (the habitat keeps spinning)" : "Time running";
    }
    if (input.timeSteps != 0)
    {
        stepTimeScale(input.timeSteps);
    }
    if (actions.setTime)
    {
        setTime(*actions.setTime);
    }
    if (actions.lookAt)
    {
        lookAtBody(*actions.lookAt);
    }
    if (actions.lookAtPartner)
    {
        lookAtPartner();
    }
}

void Application::simulate(const MoveIntent& intent, double realSeconds)
{
    const int    steps = clock_.advance(realSeconds);
    const double dt    = clock_.stepSeconds();
    for (int step = 0; step < steps; ++step)
    {
        look_.setFrame(player_.viewUp(), kNorth);
        player_.step(intent, look_, *geometry_, dt);
        spinPhase_      = std::fmod(spinPhase_ + (geometry_->omega() * dt), 2.0 * kPi);
        const Vec3d eye = player_.eyePosition();
        physics_->step(dt, eye - (HabitatGeometry::localUp(eye) * player_.settings.eyeHeight));
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

    // The ball itself: a prop, flying (and bouncing, and rolling) in the physics world.
    const Quatd upright = floorOrientation(start.position);
    thrown_.push_back(
        physics_->addProp({.kind        = PropKind::Ball,
                           .position    = start.position - (upright * Vec3d(0.0, kBallRadius, 0.0)),
                           .orientation = upright,
                           .tint        = (static_cast<float>(thrown_.size() % 4) / 4.0F) + 0.1F},
                          start.velocity));
    while (thrown_.size() > kMaxThrownBalls)
    {
        physics_->removeProp(thrown_.front());
        thrown_.erase(thrown_.begin());
    }
    if (audio_)
    {
        audio_->play(audio::Sound::Throw);
    }
}

void Application::kick()
{
    const Vec3d eye     = player_.eyePosition();
    const Vec3d forward = look_.forward();
    const auto  hit     = physics_->raycast(eye, forward, kKickReach);
    if (!hit || !hit->prop)
    {
        status_ = "Nothing to kick: walk up to a ball, a crate or a chair and press E";
        return;
    }
    const PropKind kind = physics_->props()[*hit->prop].kind;
    // A kick is a quick push, a little upward: light things fly, heavy ones barely budge.
    const Vec3d  up        = HabitatGeometry::localUp(eye);
    const Vec3d  direction = glm::normalize(forward - (up * glm::dot(forward, up)) + (up * 0.35));
    const double impulse   = std::min(static_cast<double>(propInfo(kind).massKg) * 9.0, 45.0);
    physics_->push(*hit->prop, direction * impulse, hit->point);
    status_ = std::format("You kick the {}", propKindName(kind));
}

std::string Application::placeName() const
{
    if (!settlements_)
    {
        return {};
    }
    const Vec3d  eye   = player_.eyePosition();
    const double theta = HabitatGeometry::angleOf(eye);
    std::string  near;
    double       nearest = std::numeric_limits<double>::max();
    for (const Settlement& place : settlements_->places)
    {
        const Vec2d  at       = place.plane.toPlan(eye.z, theta);
        const double distance = glm::length(at);
        if (place.kind == SettlementKind::Farm)
        {
            if (distance < 60.0)
            {
                return "at a farmstead";
            }
            continue;
        }
        if (place.ground.sample(at).z > 0.3)
        {
            return std::format("in {}, {} buildings", place.name, place.buildingCount);
        }
        if (distance < place.radiusM + 800.0 && distance < nearest)
        {
            nearest = distance;
            near    = std::format("near {}, {:.1f} km away", place.name, distance / 1000.0);
        }
    }
    return near;
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
    lighting.haze                      = static_cast<double>(hudSettings_.haze);
    const std::vector<Marker>   shapes = markers();
    const std::vector<BodyDraw> bodies = bodyDraws(lighting);
    const OneillCylinderSpec&   spec   = geometry_->spec();
    // The trees' shadows, along the beam that lights the ground here.
    gpu::ShadowUniforms shadow;
    if (const auto beam = dominantBeam(*geometry_, mirrorAngle(), player_.eyePosition()))
    {
        const auto beams = sunBeams(*geometry_, mirrorAngle());
        shadow = gpu::makeShadowUniforms(player_.eyePosition(),
                                         beams.at(static_cast<std::size_t>(*beam)).towardSun, *beam,
                                         kShadowHalfExtentM, kShadowMapResolution);
    }
    propPoses_.clear();
    for (const PropState& prop : physics_->props())
    {
        if (!prop.removed)
        {
            propPoses_.push_back({.kind        = prop.kind,
                                  .position    = prop.position,
                                  .orientation = prop.orientation,
                                  .tint        = prop.tint});
        }
    }
    // The birds are worked out fresh each frame: flocks that wheel over fixed places, in real time.
    BirdSettings flocks;
    flocks.windMS = 0.35 * weather_.windAlongMS;
    birdPoses_    = birdsNear(*geometry_, player_.eyePosition(), animationSeconds_,
                              scenario_.habitat.terrain.seed, flocks);

    const SceneView scene{
        .camera      = camera(),
        .world       = world_.get(),
        .landscape   = landscape_.get(),
        .trees       = trees_.get(),
        .settlements = gpuSettlements_.get(),
        .props       = gpuProps_.get(),
        .propPoses   = propPoses_,
        .birds       = gpuBirds_.get(),
        .birdPoses   = birdPoses_,
        .shadow      = shadow,
        .habitat =
            gpu::makeHabitatUniforms(*geometry_, mirrorAngle(), lighting, weather_, cloudSettings_),
        .sky =
            gpu::makeSkyUniforms(habitatFromSky_, static_cast<double>(hudSettings_.starBrightness),
                                 kMilkyWayScale * static_cast<double>(hudSettings_.milkyWay)),
        .planets  = gpu::makePlanetUniforms(sky_),
        .bodies   = bodies,
        .partner  = spec.partner.enabled ? std::optional<Mat4d>(partnerTransform(
                                               spec.partner.separationM, spinPhase_))
                                         : std::nullopt,
        .markers  = shapes,
        .exposure = static_cast<float>(static_cast<double>(hudSettings_.exposure) * autoExposure()),
        .grade    = hudSettings_.grade,
        .animationSeconds = animationSeconds_,
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
        benchmark.viewMs.push_back(realSeconds * 1000.0);
    }
    look_.applyLook(kBenchmarkPanRate * realSeconds, 0.0);
    if (benchmark.viewSeconds >= kBenchmarkViewSeconds)
    {
        benchmark.viewSeconds = 0.0;
        if (!benchmark.viewMs.empty())
        {
            std::ranges::sort(benchmark.viewMs);
            double sum = 0.0;
            for (const double ms : benchmark.viewMs)
            {
                sum += ms;
            }
            std::println("  {:10} average {:5.2f} ms, p99 {:5.2f} ms",
                         kBenchmarkViews.at(benchmark.view),
                         sum / static_cast<double>(benchmark.viewMs.size()),
                         benchmark.viewMs.at((benchmark.viewMs.size() - 1) * 99 / 100));
            benchmark.viewMs.clear();
        }
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
        const std::uint64_t now = SDL_GetTicksNS();
        // Captures step a fixed 1/60 s per frame, so they show the same moment every time.
        const double realSeconds =
            options_.capturePath ? kCaptureStepSeconds : static_cast<double>(now - previous) * 1e-9;
        previous = now;
        updateStats(realSeconds);
        pollGeneration();
        pollSky();
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
        advanceClock(realSeconds);
        updateWeather(realSeconds);
        updateSky();
        updateSound(realSeconds);

        if (const auto exitCode = render(frame, input.screenshot, ui))
        {
            return *exitCode;
        }
    }
}

}  // namespace StarshipSimulator
