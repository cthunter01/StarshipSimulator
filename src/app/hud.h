#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/fly_controller.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/gpu_device.h"

namespace StarshipSimulator
{

/// A named place to teleport to, with a beacon that gets a floating label.
struct Waypoint
{
    std::string name;
    Vec3d       eyePosition{0.0};
    Vec3d       beaconPosition{0.0};
};

/// Read-only data the HUD shows.
struct HudModel
{
    const GpuInfo*            gpu        = nullptr;
    SDL_GPUSampleCount        samples    = SDL_GPU_SAMPLECOUNT_1;
    const FlyController*      controller = nullptr;
    Camera                    camera;
    std::span<const Waypoint> waypoints;
    double                    fps           = 0.0;
    double                    frameMs       = 0.0;
    bool                      mouseCaptured = false;
    std::string               status;  // last notice, e.g. "Saved screenshot ..."
};

/// Settings the HUD edits in place.
struct HudSettings
{
    float exposure = 1.0F;
    bool  showHelp = true;
};

/// What the user clicked this frame.
struct HudActions
{
    std::optional<std::size_t> teleportTo;
    bool                       toggleLocomotion = false;
    bool                       reloadShaders    = false;
};

/// Draws the HUD windows and floating waypoint labels. Call between ImGuiLayer begin/end frame.
[[nodiscard]] HudActions drawHud(const HudModel& model, HudSettings& settings);

}  // namespace StarshipSimulator
