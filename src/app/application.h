#pragma once

#include <cstddef>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <SDL3/SDL_video.h>

#include "StarshipSimulator/core/app_options.h"
#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/sky_objects.h"
#include "StarshipSimulator/core/astro/star_catalog.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/physics/player_controller.h"
#include "StarshipSimulator/core/physics/rotating_frame.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/terrain_lod.h"
#include "StarshipSimulator/core/procgen/trees.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/core/sim_clock.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/gpu_landscape.h"
#include "StarshipSimulator/render/gpu_trees.h"
#include "StarshipSimulator/render/gpu_world.h"
#include "StarshipSimulator/render/imgui_layer.h"
#include "StarshipSimulator/render/passes/marker_pass.h"
#include "StarshipSimulator/render/renderer.h"
#include "hud.h"
#include "sdl_input.h"
#include "sky_loader.h"

struct ImDrawData;

namespace StarshipSimulator
{

/// SDL_Init / SDL_Quit for the lifetime of the application.
class SdlContext
{
public:
    SdlContext();
    ~SdlContext();
    SdlContext(const SdlContext&)            = delete;
    SdlContext& operator=(const SdlContext&) = delete;
    SdlContext(SdlContext&&)                 = delete;
    SdlContext& operator=(SdlContext&&)      = delete;
};

/// A habitat generated on the CPU (possibly on a worker thread), ready to upload.
struct GeneratedWorld
{
    std::shared_ptr<const HabitatGeometry> geometry;
    HabitatMeshes                          meshes;
    CpuMesh                                hull;  // seen from outside, for the partner cylinder
    TerrainGrid                            terrain;
    std::optional<TerrainLod>              lod;
    TreeLayer                              trees;
    std::string                            error;
    double                                 seconds = 0.0;
};

/// A ball thrown by the player, with its predicted path and the same throw without spin.
struct ThrownBall
{
    BodyState          state;
    bool               resting = false;
    std::vector<Vec3d> path;
    std::vector<Vec3d> ghostPath;
    ThrowReport        report;
};

/// Frame times collected while flying the benchmark tour.
struct Benchmark
{
    std::size_t         view        = 0;
    double              viewSeconds = 0.0;
    std::vector<double> frameMs;
};

/// The StarshipSimulator application: window, GPU, habitat, player, HUD.
class Application
{
public:
    explicit Application(AppOptions options);

    /// Runs until the window closes (or a capture/benchmark finishes). Returns the exit code.
    int run();

private:
    struct WindowDeleter
    {
        void operator()(SDL_Window* window) const noexcept;
    };

    // Habitats
    void adoptWorld(GeneratedWorld world, bool placeAtStart);
    void startGeneration(const OneillCylinderSpec& spec, bool placeAtStart);
    void pollGeneration();
    void loadScenarioFile(const std::filesystem::path& path);
    void saveDraft();
    void refreshScenarioList();
    void placeAtStart();
    void applyView(std::string_view name);
    void applyCameraPose(const CameraPose& pose);

    // The sky and the clock
    void                                  startSkyLoad();
    void                                  pollSky();
    void                                  adoptSky(SkyData data);
    void                                  advanceClock(double realSeconds);
    void                                  updateSky();
    void                                  setTime(astro::SimTime time);
    void                                  stepTimeScale(int steps);
    void                                  identify();
    void                                  lookAtBody(astro::Body body);
    void                                  lookAtPartner();
    void                                  lookAtName(std::string_view name);
    void                                  lookOut(Vec3d directionEqj, std::string_view name);
    [[nodiscard]] double                  mirrorAngle() const;  // radians
    [[nodiscard]] double                  autoExposure() const;
    [[nodiscard]] SkyModel                skyModel() const;
    [[nodiscard]] std::optional<SkyLabel> skyLabel() const;
    [[nodiscard]] std::vector<BodyDraw>   bodyDraws(const gpu::LightingSettings& lighting) const;

    // Frames
    void                     updateStats(double realSeconds);
    [[nodiscard]] HudActions drawUi();
    void                     applyInput(const InputFrame& input, const HudActions& actions);
    void                     applySkyInput(const InputFrame& input, const HudActions& actions);
    void                     simulate(const MoveIntent& intent, double realSeconds);
    void                     throwBall();
    [[nodiscard]] std::vector<Marker> markers() const;
    [[nodiscard]] Camera              camera() const;
    [[nodiscard]] std::optional<int>  render(int frame, bool screenshotRequested, ImDrawData* ui);
    [[nodiscard]] std::optional<int>  stepBenchmark(double realSeconds);

    AppOptions                                 options_;
    SdlContext                                 sdl_;
    std::unique_ptr<SDL_Window, WindowDeleter> window_;
    GpuDevice                                  device_;
    ImGuiLayer                                 imgui_;
    Renderer                                   renderer_;
    SdlInput                                   input_;

    Scenario                               scenario_;
    std::shared_ptr<const HabitatGeometry> geometry_;
    std::unique_ptr<GpuWorld>              world_;
    std::unique_ptr<GpuLandscape>          landscape_;
    std::unique_ptr<GpuTrees>              trees_;
    HabitatMetrics                         metrics_;
    std::future<GeneratedWorld>            pending_;
    bool                                   placeWhenReady_ = false;

    std::future<SkyData>              pendingSky_;
    std::optional<astro::StarCatalog> catalog_;
    std::string                       skyStatus_;
    astro::SimTime                    simTime_;
    astro::SkyState                   sky_;
    Mat3d                             habitatFromSky_{1.0};  // EQJ -> the spinning habitat frame
    std::optional<astro::Identified>  identified_;
    std::optional<std::string>        pendingLookAt_;  // a star to look at once the catalog loads
    double                            identifiedAge_ = 0.0;  // seconds since it was named

    PlayerController          player_;
    LookRig                   look_;
    SimClock                  clock_;
    double                    spinPhase_ = 0.0;
    std::optional<ThrownBall> ball_;
    std::optional<Benchmark>  benchmark_;

    HudSettings   hudSettings_;
    EditorState   editor_;
    bool          showHud_ = true;
    std::string   status_;
    double        fps_            = 0.0;
    double        frameMs_        = 0.0;
    std::uint32_t chunksDrawn_    = 0;
    std::uint64_t trianglesDrawn_ = 0;
};

}  // namespace StarshipSimulator
