#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/physics/player_controller.h"
#include "StarshipSimulator/render/gpu_device.h"

namespace StarshipSimulator
{

/// Where a thrown ball landed compared with the same throw without spin (constant gravity).
struct ThrowReport
{
    double flightTime = 0.0;  // s
    double range      = 0.0;  // m, from the throw to the landing point
    double lateral    = 0.0;  // m the spin moved the landing point to the left (negative: right)
    double along      = 0.0;  // m further (negative: shorter) than without spin
};

/// Read-only data the HUD shows.
struct HudModel
{
    const GpuInfo*             gpu            = nullptr;
    SDL_GPUSampleCount         samples        = SDL_GPU_SAMPLECOUNT_1;
    double                     fps            = 0.0;
    double                     frameMs        = 0.0;
    std::uint32_t              chunksDrawn    = 0;
    std::uint64_t              trianglesDrawn = 0;
    std::string                title;
    const HabitatGeometry*     geometry      = nullptr;
    const HabitatMetrics*      metrics       = nullptr;
    const PlayerController*    player        = nullptr;
    bool                       mouseCaptured = false;
    bool                       generating    = false;
    std::string                status;
    std::optional<ThrowReport> throwReport;
};

/// Settings the HUD edits in place.
struct HudSettings
{
    float mirrorAngleDeg = 60.0F;  // time of day
    float exposure       = 1.0F;
    float haze           = 1.0F;
    float starBrightness = 2.0F;
    bool  showHelp       = true;
    bool  showEditor     = false;
};

/// The habitat editor's working copy.
struct EditorState
{
    OneillCylinderSpec                 draft;
    std::string                        title;
    std::vector<std::filesystem::path> scenarios;  // presets and saved habitats
    int                                selected = 0;
    std::array<char, 256>              saveName{};
};

/// What the user clicked this frame.
struct HudActions
{
    bool                                 toggleLocomotion = false;
    bool                                 throwBall        = false;
    bool                                 regenerate       = false;
    bool                                 save             = false;
    std::optional<std::filesystem::path> load;
};

/// Draws the HUD and the editor. Call between ImGuiLayer::beginFrame and endFrame.
[[nodiscard]] HudActions drawHud(const HudModel& model, HudSettings& settings, EditorState& editor,
                                 PlayerSettings& player);

}  // namespace StarshipSimulator
