#pragma once

#include <imgui.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/weather.h"
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

/// A name drawn next to something in the sky (after pressing I).
struct SkyLabel
{
    ImVec2      screen;  // pixels from the top left
    std::string name;
    std::string details;
    float       fade = 1.0F;  // 1 visible, 0 gone
};

/// The simulated clock and the sky, as the HUD shows them.
struct SkyModel
{
    std::string               clock;  // "2045-06-15 09:00:00 UTC"
    double                    localHour = 0.0;
    std::string               location;
    bool                      partner = false;  // the scenario has a partner cylinder
    const astro::SkyState*    sky     = nullptr;
    const astro::VisibleBody* earth   = nullptr;
    const astro::VisibleBody* moon    = nullptr;
    bool                      loading = false;
    std::string               status;          // what sky data is in use
    double                    exposure = 1.0;  // the automatic part
    std::optional<SkyLabel>   label;
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
    const HabitatGeometry*     geometry    = nullptr;
    const HabitatMetrics*      metrics     = nullptr;
    std::size_t                trees       = 0;
    std::size_t                towns       = 0;
    std::size_t                farms       = 0;
    std::size_t                buildings   = 0;
    std::size_t                movingProps = 0;
    std::size_t                birds       = 0;
    std::string                place;  // the town (or farm) you are in or near
    const PlayerController*    player        = nullptr;
    bool                       mouseCaptured = false;
    bool                       generating    = false;
    std::string                status;
    std::optional<ThrowReport> throwReport;
    SkyModel                   sky;
    bool                       sound = false;  // an audio device is playing
    Weather                    weather;
    double                     cloudBaseM = 0.0;
    double                     cloudTopM  = 0.0;
};

/// Settings the HUD edits in place.
struct HudSettings
{
    float                mirrorAngleDeg = 60.0F;
    bool                 followSchedule = true;  // mirrors swing with the day schedule
    double               timeScale      = 1.0;   // simulated seconds per real second
    bool                 timePaused     = false;
    float                exposure       = 1.0F;  // on top of the automatic exposure
    bool                 autoExposure   = true;  // brighten the view at night
    float                haze           = 1.0F;
    float                starBrightness = 2.0F;
    float                milkyWay       = 1.0F;
    float                fieldOfViewDeg = 70.0F;  // vertical
    float                grade          = 1.0F;   // painterly colour grade
    float                volume         = 0.8F;
    bool                 forceWeather   = false;  // hold the weather still, for looking at it
    float                cloudCover     = 0.45F;
    float                rain           = 0.0F;
    float                wetness        = 0.0F;
    float                mist           = 0.0F;
    float                windSpeedMS    = 3.0F;
    bool                 forceSeason    = false;  // hold the year still as well
    float                season         = 0.2F;   // 0 spring, 0.25 summer, 0.5 autumn
    bool                 showHelp       = true;
    bool                 showEditor     = false;
    bool                 showCredits    = false;
    std::array<char, 32> dateText{};  // the "go to" date being typed
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
    bool                                 identify         = false;
    std::optional<std::filesystem::path> load;
    std::optional<astro::SimTime>        setTime;
    std::optional<astro::Location>       setLocation;
    std::optional<astro::Body>           lookAt;
    bool                                 lookAtPartner = false;
};

/// Simulated seconds per real second offered by the HUD and the comma/period keys.
inline constexpr std::array<double, 7> kTimeScales{1.0,    10.0,    60.0,   600.0,
                                                   3600.0, 21600.0, 86400.0};

/// "1 h/s" for a time scale.
[[nodiscard]] std::string timeScaleName(double scale);

/// Draws the HUD and the editor. Call between ImGuiLayer::beginFrame and endFrame.
[[nodiscard]] HudActions drawHud(const HudModel& model, HudSettings& settings, EditorState& editor,
                                 PlayerSettings& player);

}  // namespace StarshipSimulator
