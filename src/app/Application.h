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

#include "SdlInput.h"
#include "StarshipSimulator/audio/AudioDevice.h"
#include "StarshipSimulator/core/SimClock.h"
#include "StarshipSimulator/core/almanac.h"
#include "StarshipSimulator/core/app_options.h"
#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/sky_objects.h"
#include "StarshipSimulator/core/astro/star_catalog.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/weather.h"
#include "StarshipSimulator/core/physics/PlayerController.h"
#include "StarshipSimulator/core/physics/RotatingFrame.h"
#include "StarshipSimulator/core/procgen/TerrainLod.h"
#include "StarshipSimulator/core/procgen/birds.h"
#include "StarshipSimulator/core/procgen/buildings.h"
#include "StarshipSimulator/core/procgen/clouds.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/people.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/core/procgen/trees.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/core/tour.h"
#include "StarshipSimulator/physics/PhysicsWorld.h"
#include "StarshipSimulator/render/GpuBirds.h"
#include "StarshipSimulator/render/GpuDevice.h"
#include "StarshipSimulator/render/GpuLandscape.h"
#include "StarshipSimulator/render/GpuPeople.h"
#include "StarshipSimulator/render/GpuProps.h"
#include "StarshipSimulator/render/GpuSettlements.h"
#include "StarshipSimulator/render/GpuTransit.h"
#include "StarshipSimulator/render/GpuTrees.h"
#include "StarshipSimulator/render/GpuWorld.h"
#include "StarshipSimulator/render/ImGuiLayer.h"
#include "StarshipSimulator/render/Renderer.h"
#include "StarshipSimulator/render/passes/MarkerPass.h"
#include "hud.h"
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
    std::shared_ptr<TerrainGrid>           terrain;
    std::optional<TerrainLod>              lod;
    std::shared_ptr<TreeLayer>             trees;
    Settlements                            settlements;
    std::vector<SettlementMesh>            settlementMeshes;
    CloudMap                               clouds;
    std::vector<TramLine>                  tramLines;
    std::vector<TrackChunk>                track;
    std::unique_ptr<PhysicsWorld>          physics;
    std::string                            error;
    double                                 seconds = 0.0;
};

/// The latest ball thrown by the player: its predicted path, and the same throw without spin. (The
/// ball itself is a prop in the physics world.)
struct ThrownBall
{
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
    std::vector<double> viewMs;  // this view's frames
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
    void                 adoptWorld(GeneratedWorld world, bool placeAtStart);
    void                 startGeneration(const OneillCylinderSpec& spec, bool placeAtStart);
    void                 pollGeneration();
    void                 loadScenarioFile(const std::filesystem::path& path);
    void                 editScenarioFile(const std::filesystem::path& path);
    void                 buildDraft();
    void                 saveDraft();
    void                 refreshScenarioList();
    void                 placeAtStart();
    void                 applyView(std::string_view name);
    void                 applyWaterView(std::string_view name);
    void                 applyTownView(std::string_view name);
    void                 applyTransitView(std::string_view name);
    void                 walkTo(double z, double theta, double yawDeg, double pitchDeg);
    void                 flyTo(const Vec3d& eye, double yawDeg, double pitchDeg);
    [[nodiscard]] int    startValley() const;
    [[nodiscard]] double startViewZ() const;
    void                 applyCameraPose(const CameraPose& pose);

    // The sky and the clock
    void                          startSkyLoad();
    void                          pollSky();
    void                          adoptSky(SkyData data);
    void                          advanceClock(double realSeconds);
    void                          updateSky();
    void                          updateWeather(double realSeconds);
    void                          updateSound(double realSeconds);
    [[nodiscard]] audio::SoundMix soundMix() const;
    void                          setTime(astro::SimTime time);
    void                          stepTimeScale(int steps);
    void                          identify();
    void                          lookAtBody(astro::Body body);
    void                          lookAtPartner();
    void                          lookAtName(std::string_view name);
    void                          lookOut(Vec3d directionEqj, std::string_view name);
    [[nodiscard]] double          mirrorAngle() const;  // radians
    [[nodiscard]] double          autoExposure() const;
    [[nodiscard]] SkyModel        skyModel() const;
    [[nodiscard]] AlmanacState    almanacState() const;
    /// How big a picture taken now would come out, in pixels.
    [[nodiscard]] glm::uvec2              photoSize() const;
    [[nodiscard]] std::vector<TourName>   tourNames() const;
    void                                  startTour(std::size_t which);
    void                                  stepTour(double realSeconds);
    [[nodiscard]] std::optional<SkyLabel> skyLabel() const;
    [[nodiscard]] std::vector<BodyDraw>   bodyDraws(const gpu::LightingSettings& lighting) const;

    // Frames
    void                      updateStats(double realSeconds);
    [[nodiscard]] HudActions  drawUi();
    void                      applyInput(const InputFrame& input, const HudActions& actions);
    void                      applyMoveInput(const InputFrame& input, const HudActions& actions);
    void                      applySkyInput(const InputFrame& input, const HudActions& actions);
    void                      simulate(const MoveIntent& intent, double realSeconds);
    void                      throwBall();
    void                      kick();
    [[nodiscard]] std::string placeName() const;
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
    std::shared_ptr<const TerrainGrid>     terrain_;
    std::shared_ptr<const Settlements>     settlements_;
    std::unique_ptr<GpuSettlements>        gpuSettlements_;
    std::unique_ptr<GpuProps>              gpuProps_;
    std::unique_ptr<GpuBirds>              gpuBirds_;
    std::unique_ptr<GpuPeople>             gpuPeople_;
    std::unique_ptr<GpuTransit>            gpuTransit_;
    std::unique_ptr<PhysicsWorld>          physics_;
    std::vector<std::size_t>               thrown_;     // props: the balls thrown, oldest first
    std::vector<PropPlacement>             propPoses_;  // where the props are, for drawing
    std::vector<Bird>                      birdPoses_;
    std::vector<Person>                    peoplePoses_;
    std::vector<TramLine>                  tramLines_;
    std::vector<Tram>                      trams_;
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
    double                    spinPhase_        = 0.0;
    double                    animationSeconds_ = 0.0;  // wall clock, for ripples, rain and birds
    std::optional<ThrownBall> ball_;
    Weather                   weather_;
    std::unique_ptr<AudioDevice> audio_;         // null when silent
    double                       stride_ = 0.0;  // metres walked since the last footstep
    gpu::CloudSettings           cloudSettings_;
    std::optional<Benchmark>     benchmark_;
    std::vector<Tour>            tours_;
    std::optional<std::size_t>   touring_;  // which tour is running
    double                       tourSeconds_ = 0.0;
    std::string                  tourCaption_;
    double                       tourFade_ = 0.0;

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
