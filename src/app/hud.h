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

#include "StarshipSimulator/core/almanac.h"
#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/weather.h"
#include "StarshipSimulator/core/physics/player_controller.h"
#include "StarshipSimulator/core/scenario/scenario.h"
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
    astro::SimTime            time;   // the same moment, to put in a habitat file
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

/// A tour on offer: its name and what it covers.
struct TourName
{
    std::string name;
    std::string blurb;
    double      lengthS = 0.0;
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
    std::size_t                people      = 0;  // near you, and drawn
    std::size_t                tramLines   = 0;
    std::size_t                tramStops   = 0;
    std::size_t                trams       = 0;
    double                     trackKm     = 0.0;
    std::string                place;  // the town (or farm) you are in or near
    const PlayerController*    player        = nullptr;
    bool                       mouseCaptured = false;
    bool                       generating    = false;
    std::string                status;
    std::optional<ThrowReport> throwReport;
    SkyModel                   sky;
    bool                       sound = false;  // an audio device is playing
    Weather                    weather;
    std::vector<AlmanacPage>   almanac;
    std::vector<TourName>      tours;        // what is on offer
    std::string                tourCaption;  // what the tour is saying now, if one is running
    double                     tourFade    = 0.0;
    bool                       touring     = false;
    std::uint32_t              photoWidth  = 0;  // what a picture would come out at
    std::uint32_t              photoHeight = 0;
    double                     cloudBaseM  = 0.0;
    double                     cloudTopM   = 0.0;
};

/// Settings the HUD edits in place.
struct HudSettings
{
    float  mirrorAngleDeg = 60.0F;
    bool   followSchedule = true;  // mirrors swing with the day schedule
    double timeScale      = 1.0;   // simulated seconds per real second
    bool   timePaused     = false;
    float  exposure       = 1.0F;  // on top of the automatic exposure
    bool   autoExposure   = true;  // brighten the view at night
    float  haze           = 1.0F;
    float  starBrightness = 2.0F;
    float  milkyWay       = 1.0F;
    float  fieldOfViewDeg = 70.0F;  // vertical
    float  grade          = 1.0F;   // painterly colour grade
    float  volume         = 0.8F;
    bool   forceWeather   = false;  // hold the weather still, for looking at it
    float  cloudCover     = 0.45F;
    float  rain           = 0.0F;
    float  wetness        = 0.0F;
    float  mist           = 0.0F;
    float  windSpeedMS    = 3.0F;
    bool   forceSeason    = false;  // hold the year still as well
    float  season         = 0.2F;   // 0 spring, 0.25 summer, 0.5 autumn
    bool   showHelp       = true;
    bool   showEditor     = false;
    bool   showGallery    = false;
    bool   showAlmanac    = false;
    int    almanacPage    = 0;
    // Photo mode: the HUD out of the way, and a long exposure to draw star trails with.
    bool                 photoMode    = false;
    bool                 trails       = false;
    bool                 trailsReset  = false;  // cleared once the renderer has acted on it
    int                  captureScale = 2;      // 1, 2, 3 or 4: how much bigger the picture is
    bool                 showCredits  = false;
    std::array<char, 32> dateText{};  // the "go to" date being typed
};

/// One habitat file on offer in the gallery, as it described itself when the list was read.
struct GalleryEntry
{
    std::filesystem::path path;
    std::string           title;
    std::string           description;
    std::string           summary;         // describeHabitat(), or empty when the file is broken
    std::string           problem;         // why it cannot be opened (empty when it is fine)
    bool                  preset = false;  // shipped with the program, rather than saved here
};

/// The habitat editor's working copy: a whole scenario, not just its shape.
struct EditorState
{
    Scenario                  draft;
    std::vector<GalleryEntry> gallery;  // presets and saved habitats
    int                       selected = 0;
    std::array<char, 64>      saveName{};
    std::array<char, 256>     description{};
};

/// What the user clicked this frame.
struct HudActions
{
    bool                                 toggleLocomotion = false;
    bool                                 toggleWings      = false;
    bool                                 toggleAlmanac    = false;
    bool                                 screenshot       = false;
    std::optional<std::size_t>           startTour;  // which tour to set off on
    bool                                 stopTour       = false;
    bool                                 throwBall      = false;
    bool                                 regenerate     = false;
    bool                                 save           = false;
    bool                                 refreshGallery = false;
    bool                                 identify       = false;
    std::optional<std::filesystem::path> load;
    std::optional<std::filesystem::path> editCopy;  // open this file in the editor, without going
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
