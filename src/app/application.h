#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <SDL3/SDL_video.h>

#include "StarshipSimulator/core/app_options.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/fly_controller.h"
#include "StarshipSimulator/core/sim_clock.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/imgui_layer.h"
#include "StarshipSimulator/render/passes/marker_pass.h"
#include "StarshipSimulator/render/renderer.h"
#include "hud.h"
#include "sdl_input.h"

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

/// The StarshipSimulator application: window, GPU, input, simulation and HUD.
class Application
{
public:
    explicit Application(AppOptions options);

    /// Runs until the window closes (or the capture is saved). Returns the process exit code.
    int run();

private:
    struct WindowDeleter
    {
        void operator()(SDL_Window* window) const noexcept;
    };

    void                     updateStats(double realSeconds);
    [[nodiscard]] HudActions drawUi();
    void                     applyInput(const InputFrame& input, const HudActions& hudActions);
    /// Renders and presents a frame. Returns an exit code when the app should stop (capture saved).
    [[nodiscard]] std::optional<int> render(int frame, bool screenshotRequested, ImDrawData* ui);
    void                             teleport(std::size_t waypointIndex);
    [[nodiscard]] Camera             camera() const;

    AppOptions                                 options_;
    SdlContext                                 sdl_;
    std::unique_ptr<SDL_Window, WindowDeleter> window_;
    GpuDevice                                  device_;
    ImGuiLayer                                 imgui_;
    Renderer                                   renderer_;
    SdlInput                                   input_;

    FlyController         controller_;
    LookRig               look_;
    SimClock              clock_;
    HudSettings           hudSettings_;
    std::vector<Waypoint> waypoints_;
    std::vector<Marker>   markers_;
    bool                  showHud_ = true;
    std::string           status_;
    double                fps_     = 0.0;
    double                frameMs_ = 0.0;
};

}  // namespace StarshipSimulator
