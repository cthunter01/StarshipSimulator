#include "hud.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <optional>
#include <string>
#include <utility>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/almanac.h"
#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/sky_objects.h"
#include "StarshipSimulator/core/habitat/Enclosure.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/habitat/weather.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/PlayerController.h"
#include "StarshipSimulator/core/physics/RotatingFrame.h"
#include "StarshipSimulator/core/scenario/scenario.h"
#include "StarshipSimulator/core/units.h"
#include "StarshipSimulator/render/GpuDevice.h"
#include "StarshipSimulator/render/ImGuiLayer.h"

namespace StarshipSimulator
{

namespace
{

int sampleCountValue(SDL_GPUSampleCount samples)
{
    switch (samples)
    {
        case SDL_GPU_SAMPLECOUNT_1:
            return 1;
        case SDL_GPU_SAMPLECOUNT_2:
            return 2;
        case SDL_GPU_SAMPLECOUNT_4:
            return 4;
        case SDL_GPU_SAMPLECOUNT_8:
            return 8;
    }
    return 1;
}

bool sliderDouble(const char* label, double& value, double min, double max, const char* format)
{
    return ImGui::SliderScalar(label, ImGuiDataType_Double, &value, &min, &max, format);
}

/// What the light is doing at this mirror angle. windowLatitudeDeg: a sphere's polar windows.
std::string describeSun(double openingAngleDeg, HabitatKind kind, double windowLatitudeDeg)
{
    const double alpha = degreesToRadians(openingAngleDeg);
    if (daylightKindFor(kind) == DaylightKind::OVERHEAD_MIRROR)
    {
        // The ring of mirrors round a torus's hub, and the louvres over its windows.
        if (daylightFactor(alpha) <= 0.0)
        {
            return "Night: the louvres are closed over the windows";
        }
        const double tilt = radiansToDegrees(hubLightTilt(alpha));
        return tilt < 0.5 ? std::string("Noon: the light comes straight down from the hub")
                          : std::format(
                                "The light comes down from the hub, leaning {:.0f} "
                                "degrees across the tube",
                                tilt);
    }
    if (daylightKindFor(kind) == DaylightKind::POLAR_WINDOWS)
    {
        // The mirrors outside the polar windows, shuttered at night.
        if (daylightFactor(alpha) <= 0.0)
        {
            return "Night: the shutters are closed over the poles";
        }
        const double lowest = polarWindowElevation(degreesToRadians(90.0), windowLatitudeDeg);
        return std::format(
            "The light comes in {:.0f} degrees up over each pole (never lower than {:.0f}, or "
            "none would reach the equator)",
            radiansToDegrees(polarWindowElevation(alpha, windowLatitudeDeg)),
            radiansToDegrees(lowest));
    }
    if (daylightKindFor(kind) == DaylightKind::END_CAPS)
    {
        // The mirrors outside the glass ends, shuttered at night.
        if (daylightFactor(alpha) <= 0.0)
        {
            return "Night: the shutters are closed over the ends";
        }
        return std::format("The light comes in {:.0f} degrees up, from both ends",
                           radiansToDegrees(endCapElevation(alpha)));
    }
    if (daylightFactor(alpha) <= 0.0)
    {
        return openingAngleDeg >= 90.0 ? "Night: the mirrors are open past the sun"
                                       : "Night: the mirrors are closed";
    }
    if (openingAngleDeg > 44.5 && openingAngleDeg < 45.5)
    {
        return "Noon: the sun stands straight overhead";
    }
    return std::format("Sun {:.0f} degrees up, toward the {} end",
                       radiansToDegrees(sunElevation(alpha)),
                       openingAngleDeg < 45.0 ? "sunward" : "anti-sunward");
}

/// A few words for what the sky is doing.
std::string skyName(const Weather& weather)
{
    if (weather.rain > 0.5)
    {
        return "pouring";
    }
    if (weather.rain > 0.05)
    {
        return "raining";
    }
    if (weather.mist > 0.4)
    {
        return "misty";
    }
    if (weather.cloudCover > 0.85)
    {
        return "overcast";
    }
    if (weather.cloudCover > 0.55)
    {
        return "cloudy";
    }
    if (weather.cloudCover > 0.2)
    {
        return "fair";
    }
    return "clear";
}

/// Where in the year the habitat is.
std::string seasonName(double season)
{
    constexpr std::array<const char*, 8> kNames{"spring", "late spring", "summer", "late summer",
                                                "autumn", "late autumn", "winter", "late winter"};
    const auto index = static_cast<std::size_t>(std::floor(season * 8.0)) % kNames.size();
    return kNames.at(index);
}

void drawThrowReport(const ThrowReport& report)
{
    ui::textWrapped(std::format(
        "Last throw: {:.1f} m in {:.2f} s. The spin moved the landing point {:.2f} m to the {} "
        "and {:.2f} m {} than on a planet with the same gravity.",
        report.range, report.flightTime, std::abs(report.lateral),
        report.lateral >= 0.0 ? "left" : "right", std::abs(report.along),
        report.along >= 0.0 ? "further" : "shorter"));
}

/// Where you are, in the habitat's own terms.
std::string describeWhere(const HabitatGeometry& geometry, const Vec3d& eye,
                          const GroundSample& ground)
{
    if (geometry.band(0).axis == BandAxis::AROUND)
    {
        // Land that runs round the axis: how far round it you are, and how far from its middle.
        const LandBand& band = geometry.band(0);
        const Vec2d     plan = band.toPlan(eye.z, HabitatGeometry::angleOf(eye));
        const double    round =
            std::fmod(plan.y - band.alongMinM + band.alongLengthM(), band.alongLengthM());
        if (const auto& torus = geometry.enclosure().torus())
        {
            // In a torus, how far up the tube's side from its lowest line; or up a spoke, or in
            // the hub.
            const double r = std::hypot(eye.x, eye.y);
            if (r < torus->hubRadiusM)
            {
                return std::format("in the hub, {:.0f} m from the axis", r);
            }
            if (r < torus->ceilingRadiusAt(eye.z))
            {
                return std::format("up a spoke, {:.0f} m from the axis", r);
            }
            const double up = torus->tubeAngleOf(eye);
            return std::format("{:.0f} m round the land, {:.0f} deg up the tube's side", round,
                               radiansToDegrees(up > kPi ? (2.0 * kPi) - up : up));
        }
        if (geometry.kind() == HabitatKind::BERNAL_SPHERE)
        {
            // On a sphere, how far north or south of the equator: the gravity goes with it.
            const double r = std::hypot(eye.x, eye.y);
            return std::format("{:.0f} m round the land, {:.0f} deg {}", round,
                               std::abs(radiansToDegrees(std::atan2(eye.z, r))),
                               eye.z >= 0.0 ? "sunward" : "anti-sunward");
        }
        return std::format("{:.0f} m round the land, {:.0f} m along the axis", round, eye.z);
    }
    std::string where = regionKindName(ground.region.kind);
    if (ground.region.index >= 0)
    {
        where += std::format(" {}", ground.region.index + 1);
    }
    return std::format("{}, {:.2f} km along, {:.0f} deg around", where, eye.z / 1000.0,
                       radiansToDegrees(HabitatGeometry::angleOf(eye)));
}

void drawLocation(const HudModel& model, HudSettings& settings, PlayerSettings& player,
                  HudActions& actions)
{
    const HabitatGeometry&  geometry = *model.geometry;
    const PlayerController& you      = *model.player;
    const Vec3d&            eye      = you.eyePosition();
    const GroundSample      ground   = geometry.ground(eye);
    const double            r        = std::hypot(eye.x, eye.y);
    const double            gravity  = geometry.gravityAt(r);
    const double            pressure = pressureRatioAt(geometry.omega(), geometry.radius(), r,
                                                       geometry.spec().atmosphere.temperatureK) *
                                       geometry.spec().atmosphere.surfacePressurePa / 1000.0;
    const double coriolis = glm::length(RotatingFrame(geometry.omega()).coriolis(you.velocity()));

    ui::field("Where", describeWhere(geometry, eye, ground));
    if (!model.place.empty())
    {
        ui::field("Place", model.place);
    }
    ui::field("Height",
              std::format("{:.1f} m above ground, {:.0f} m from the axis",
                          std::max(0.0, ground.heightAboveGround - player.eyeHeight) + 0.0, r));
    ui::field("Gravity", std::format("{:.3f} g", gravity / units::kStandardGravity));
    ui::field("Air", std::format("{:.1f} kPa", pressure));
    ui::field("Coriolis", std::format("{:.3f} m/s^2 ({:.1f}% of your weight)", coriolis,
                                      100.0 * coriolis / std::max(gravity, 1e-6)));
    const char* mode = "airborne";
    if (you.locomotion() == Locomotion::FLY)
    {
        mode = "flying";
    }
    else if (you.locomotion() == Locomotion::WINGS)
    {
        mode = you.grounded() ? "on foot, wings on" : "on the wing";
    }
    else if (you.grounded())
    {
        mode = "walking";
    }
    ui::field("Moving", std::format("{:.1f} m/s, {}", glm::length(you.velocity()), mode));
    if (you.locomotion() == Locomotion::FLY)
    {
        ui::field("Fly speed", std::format("{:.0f} m/s (mouse wheel)", player.flySpeed));
    }
    if (you.locomotion() == Locomotion::WINGS)
    {
        const PlayerController::WingState& wings = you.wings();
        ui::field("Wings", std::format("{:.1f} m/s through the air, lift {:.0f}% of your weight, "
                                       "{:+.1f} m/s{}",
                                       wings.airspeed, 100.0 * wings.liftOverWeight, wings.climbMS,
                                       wings.stalled ? ", STALLED" : ""));
    }

    if (ImGui::Button(you.locomotion() == Locomotion::WALK ? "Fly (F)" : "Walk (F)"))
    {
        actions.toggleLocomotion = true;
    }
    ImGui::SameLine();
    if (ImGui::Button(you.locomotion() == Locomotion::WINGS ? "Wings off (V)" : "Wings (V)"))
    {
        actions.toggleWings = true;
    }
    if (ImGui::BeginItemTooltip())
    {
        ImGui::TextUnformatted(
            "Strap-on wings. Space flaps; they only carry you where the "
            "gravity has fallen away, up near the axis.");
        ImGui::EndTooltip();
    }
    ImGui::SameLine();
    if (ImGui::Button("Throw a ball (G)"))
    {
        actions.throwBall = true;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Comfort (C)", &player.comfortMode);
    if (ImGui::BeginItemTooltip())
    {
        ImGui::TextUnformatted("No Coriolis force on you. Thrown objects still curve.");
        ImGui::EndTooltip();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Editor (Tab)", &settings.showEditor);
    ImGui::SameLine();
    ImGui::Checkbox("Almanac (K)", &settings.showAlmanac);
    if (ImGui::BeginItemTooltip())
    {
        ImGui::TextUnformatted("What this place is, in numbers: spin, air, the mirrors, the hull.");
        ImGui::EndTooltip();
    }
    if (model.throwReport)
    {
        drawThrowReport(*model.throwReport);
    }
}

void drawClock(const SkyModel& sky, HudSettings& settings, HudActions& actions)
{
    const int hour   = static_cast<int>(sky.localHour);
    const int minute = static_cast<int>((sky.localHour - hour) * 60.0);
    ui::field("Time", std::format("{}, local {:02}:{:02}", sky.clock, hour, minute));
    if (ImGui::Button(settings.timePaused ? "Run (P)" : "Pause (P)"))
    {
        settings.timePaused = !settings.timePaused;
    }
    for (const double scale : kTimeScales)
    {
        ImGui::SameLine();
        const bool current = settings.timeScale == scale;
        ImGui::BeginDisabled(current);
        if (ImGui::Button(timeScaleName(scale).c_str()))
        {
            settings.timeScale = scale;
        }
        ImGui::EndDisabled();
    }
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12.0F);
    const bool entered =
        ImGui::InputTextWithHint("##date", "2045-06-15T21:30", settings.dateText.data(),
                                 settings.dateText.size(), ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if (ImGui::Button("Go to date (UTC)") || entered)
    {
        if (const auto time = astro::parseIsoTime(settings.dateText.data()))
        {
            actions.setTime = *time;
        }
    }
    if (settings.dateText[0] != '\0' && !astro::parseIsoTime(settings.dateText.data()))
    {
        ui::textMuted("Dates look like 2045-06-15 or 2045-06-15T21:30 (UTC)");
    }
}

void drawSkyObjects(const SkyModel& sky, HudActions& actions)
{
    ui::field("Place", sky.location);
    if (sky.partner)
    {
        ImGui::SameLine();
        if (ImGui::SmallButton("Look at the partner"))
        {
            actions.lookAtPartner = true;
        }
    }
    for (const astro::VisibleBody* body : {sky.earth, sky.moon})
    {
        if (body == nullptr)
        {
            continue;
        }
        ui::textWrapped(astro::describeBody(*body));
        ImGui::SameLine();
        ImGui::PushID(static_cast<int>(body->body));
        if (ImGui::SmallButton("Look"))
        {
            actions.lookAt = body->body;
        }
        ImGui::PopID();
    }
    ui::textMuted(sky.loading ? "Loading the star catalog and sky maps..." : sky.status);
}

void drawTimeAndLook(const HudModel& model, HudSettings& settings, HudActions& actions)
{
    drawClock(model.sky, settings, actions);
    if (ImGui::SliderFloat("Mirror angle", &settings.mirrorAngleDeg, 0.0F, 120.0F, "%.1f deg"))
    {
        settings.followSchedule = false;  // the user took over
    }
    ImGui::Checkbox("Mirrors follow the day schedule", &settings.followSchedule);
    ui::textMuted(describeSun(static_cast<double>(settings.mirrorAngleDeg), model.geometry->kind(),
                              model.geometry->spec().sphere.windowLatitudeDeg));
    if (model.sky.sky != nullptr)
    {
        drawSkyObjects(model.sky, actions);
    }
    if (ImGui::Button("Identify (I)"))
    {
        actions.identify = true;
    }
    ImGui::SameLine();
    ui::textMuted("names what is under the crosshair");

    ImGui::SliderFloat("Exposure", &settings.exposure, 0.1F, 16.0F, "%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    ImGui::Checkbox("Auto", &settings.autoExposure);
    if (ImGui::BeginItemTooltip())
    {
        ui::text(std::format("Adapt to the dark at night (now {:.1f}x)", model.sky.exposure));
        ImGui::EndTooltip();
    }
    ImGui::SliderFloat("Haze", &settings.haze, 0.0F, 4.0F, "%.2f");
    ImGui::SliderFloat("Stars", &settings.starBrightness, 0.0F, 4.0F, "%.2f");
    ImGui::SliderFloat("Milky Way", &settings.milkyWay, 0.0F, 4.0F, "%.2f");
    ImGui::SliderFloat("Painted colours", &settings.grade, 0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Field of view", &settings.fieldOfViewDeg, 2.0F, 100.0F, "%.0f deg",
                       ImGuiSliderFlags_Logarithmic);
}

/// What the weather is doing, and the knobs to hold it still and look at it.
void drawWeather(const HudModel& model, HudSettings& settings)
{
    const Weather& now = model.weather;
    ui::field("Sky", std::format("{}, {:.0f}% cloud from {:.0f} to {:.0f} m up", skyName(now),
                                 100.0 * now.cloudCover, model.cloudBaseM, model.cloudTopM));
    ui::field("Ground", now.wetness > 0.02 ? std::format("wet ({:.0f}%)", 100.0 * now.wetness)
                                           : std::string("dry"));
    ui::field("Wind",
              std::format("{:.1f} m/s {}, {:.1f} m/s across", now.windAlongMS,
                          model.geometry->band(0).axis == BandAxis::AROUND ? "round the land"
                                                                           : "along the valley",
                          now.windAroundMS));
    ui::field("Year", std::format("{} ({:.0f}% through), days {:.1f} h long",
                                  seasonName(now.season), 100.0 * now.season, now.dayLengthHours));
    ui::field("About", std::format("{} people and {} birds near you", model.people, model.birds));
    if (model.sound)
    {
        ImGui::SliderFloat("Volume", &settings.volume, 0.0F, 1.0F, "%.2f");
    }
    else
    {
        ui::textMuted("No sound (--mute, a capture, or no audio device)");
    }

    ImGui::Checkbox("Hold the weather", &settings.forceWeather);
    ImGui::BeginDisabled(!settings.forceWeather);
    ImGui::SliderFloat("Cloud", &settings.cloudCover, 0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Rain", &settings.rain, 0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Wet ground", &settings.wetness, 0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Mist", &settings.mist, 0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Wind", &settings.windSpeedMS, 0.0F, 30.0F, "%.1f m/s");
    ImGui::EndDisabled();
    ImGui::Checkbox("Hold the season", &settings.forceSeason);
    ImGui::BeginDisabled(!settings.forceSeason);
    ImGui::SliderFloat("Season", &settings.season, 0.0F, 0.999F, "%.3f");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ui::textMuted("0 spring, 0.5 autumn");
}

void drawCredits()
{
    ui::textWrapped(
        "Stars: HYG database v4.4 by David Nash (astronexus), CC BY-SA 4.0, from Hipparcos, Yale "
        "Bright Star and Gliese catalogs.");
    ui::textWrapped(
        "Milky Way: NASA/Goddard Space Flight Center Scientific Visualization Studio, Deep Star "
        "Maps 2020, using Gaia DR2 (ESA/Gaia/DPAC).");
    ui::textWrapped(
        "Earth: NASA Visible Earth, Blue Marble (Reto Stockli, NASA GSFC) and Black Marble 2012 "
        "(NASA Earth Observatory, Suomi NPP VIIRS).");
    ui::textWrapped(
        "Moon: NASA SVS CGI Moon Kit, Lunar Reconnaissance Orbiter LROC (NASA/GSFC/ASU).");
    ui::textWrapped(
        "Positions: Astronomy Engine by Don Cross (MIT). Checked against JPL Horizons.");
    ui::textWrapped("Physics: Jolt Physics by Jorrit Rouwe (MIT).");
}

/// The name of what the user identified, next to it in the sky, with a crosshair in the middle.
void drawSkyLabel(const std::optional<SkyLabel>& label)
{
    const ImGuiIO& io     = ImGui::GetIO();
    ImDrawList*    draw   = ImGui::GetForegroundDrawList();
    const ImVec2   centre = ImVec2(io.DisplaySize.x * 0.5F, io.DisplaySize.y * 0.5F);
    if (!label || label->fade <= 0.0F)
    {
        return;
    }
    const auto      alpha = static_cast<int>(255.0F * std::clamp(label->fade, 0.0F, 1.0F));
    const ImU32     color = IM_COL32(255, 240, 200, alpha);
    constexpr float kArm  = 6.0F;
    draw->AddLine(ImVec2(centre.x - kArm, centre.y), ImVec2(centre.x + kArm, centre.y), color);
    draw->AddLine(ImVec2(centre.x, centre.y - kArm), ImVec2(centre.x, centre.y + kArm), color);
    draw->AddCircle(label->screen, 14.0F, color, 0, 1.5F);
    const ImVec2 textAt(label->screen.x + 20.0F, label->screen.y - ImGui::GetFontSize());
    draw->AddText(textAt, color, label->name.c_str());
    draw->AddText(ImVec2(textAt.x, textAt.y + ImGui::GetFontSize()),
                  IM_COL32(220, 220, 220, alpha * 3 / 4), label->details.c_str());
}

void drawMetrics(const HudModel& model, HudSettings& settings, HudActions& actions)
{
    const HabitatMetrics& m    = *model.metrics;
    const HabitatSpec&    spec = model.geometry->spec();
    if (spec.kind == HabitatKind::BERNAL_SPHERE)
    {
        ui::field("Size", std::format("a sphere {:.0f} m across", 2.0 * spec.radiusM));
    }
    else if (spec.kind == HabitatKind::STANFORD_TORUS)
    {
        ui::field("Size", std::format("a wheel {:.2f} km across, its tube {:.0f} m",
                                      2.0 * spec.radiusM / 1000.0, 2.0 * spec.torus.tubeRadiusM));
    }
    else
    {
        ui::field("Size", std::format("{:.1f} km across, {:.1f} km long",
                                      2.0 * spec.radiusM / 1000.0, spec.lengthM / 1000.0));
    }
    ui::field("Spin", std::format("{:.2f} rpm, one turn every {:.0f} s", m.rpm, m.periodS));
    ui::field("Floor", std::format("{:.2f} g, moving at {:.0f} m/s",
                                   m.floorGravity / units::kStandardGravity, m.rimSpeed));
    if (m.landAreaM2 < 1e7)
    {
        // A small habitat: hectares and people, not square kilometres and millions.
        ui::field("Land", std::format("{:.0f} hectares for {:.0f} people", m.landAreaM2 / 1e4,
                                      m.population));
    }
    else
    {
        ui::field("Land", std::format("{:.0f} km^2 for {:.1f} million people", m.landAreaM2 / 1e6,
                                      m.population / 1e6));
    }
    const Landscape& land = model.geometry->landscape();
    const char*      river =
        model.geometry->bandCount() > 1 ? ", a river in each valley" : ", a river round the land";
    ui::field("Nature", std::format("{:.1f} million trees, {} lakes{}",
                                    static_cast<double>(model.trees) / 1e6, land.lakes().size(),
                                    land.hasRivers() ? river : ""));
    ui::field("Towns", std::format("{} towns and {} farms, {} buildings", model.towns, model.farms,
                                   model.buildings));
    ui::field("Transit", std::format("{} lines, {:.0f} km of track, {} stops, {} cars running",
                                     model.tramLines, model.trackKm, model.tramStops, model.trams));
    ui::field("Axis air", std::format("{:.0f}% of floor pressure, {:.0f} K colder",
                                      100.0 * m.axisPressureRatio, m.axisTemperatureDropK));
    ui::field("Walking",
              std::format("Coriolis {:.1f}% of your weight", 100.0 * m.coriolisWalkingRatio));
    ui::field("Hull", std::format("needs {:.3f} MJ/kg: {}", m.hoopSpecificStrength / 1e6,
                                  materialClassName(m.material)));
    ui::textMuted(buildableToday(m.material) ? "Buildable with materials we have today."
                                             : "Needs materials that do not exist yet.");
    if (ImGui::Button("Change it"))
    {
        settings.showEditor = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Other habitats"))
    {
        settings.showGallery   = true;
        actions.refreshGallery = true;
    }
}

void drawRenderer(const HudModel& model)
{
    const GpuInfo& gpu = *model.gpu;
    ui::field("GPU", gpu.deviceName);
    ui::field("Driver", std::format("{} {}", gpu.driverName, gpu.driverVersion));
    ui::field("Present", std::format("{}, {}, MSAA {}x{}", gpu.presentMode, gpu.swapchainFormat,
                                     sampleCountValue(model.samples), gpu.debug ? ", debug" : ""));
    ui::field("Frame", std::format("{:.1f} fps, {:.2f} ms", model.fps, model.frameMs));
    ui::field("Drawn", std::format("{} chunks, {:.2f} M triangles", model.chunksDrawn,
                                   static_cast<double>(model.trianglesDrawn) / 1e6));
    ui::field("Physics", std::format("{} props moving", model.movingProps));
}

/// A slider for a value stored in one unit and shown in another (metres shown as kilometres).
bool sliderScaled(const char* label, double& value, double perUnit, double low, double high,
                  const char* format)
{
    double     shown   = value / perUnit;
    const bool changed = sliderDouble(label, shown, low, high, format);
    if (changed)
    {
        value = shown * perUnit;
    }
    return changed;
}

/// "06:30" for an hour of the day.
std::string clockHour(double hour)
{
    const double wrapped = std::fmod(std::fmod(hour, 24.0) + 24.0, 24.0);
    const int    minutes = static_cast<int>(std::lround(wrapped * 60.0)) % (24 * 60);
    return std::format("{:02}:{:02}", minutes / 60, minutes % 60);
}

void drawEditorEndcaps(HabitatSpec& spec)
{
    int antisunward = static_cast<int>(spec.antisunwardEndcap.shape);
    int sunward     = static_cast<int>(spec.sunwardEndcap.shape);
    ImGui::Combo("Anti-sunward end", &antisunward, "Flat wall\0Dome\0Mountain ramp\0");
    ImGui::Combo("Sunward end", &sunward, "Flat wall\0Dome\0Mountain ramp\0");
    spec.antisunwardEndcap.shape = static_cast<EndcapShape>(antisunward);
    spec.sunwardEndcap.shape     = static_cast<EndcapShape>(sunward);
    sliderDouble("Ramp slope (deg)", spec.antisunwardEndcap.rampSlopeDeg, 10.0, 40.0, "%.0f");
    sliderDouble("Ramp top (share of R)", spec.antisunwardEndcap.rampTopRadiusFraction, 0.1, 0.9,
                 "%.2f");
    sliderDouble("Hub opening (m)", spec.antisunwardEndcap.hubRadiusM, 10.0, 400.0, "%.0f");
    spec.sunwardEndcap.rampSlopeDeg          = spec.antisunwardEndcap.rampSlopeDeg;
    spec.sunwardEndcap.rampTopRadiusFraction = spec.antisunwardEndcap.rampTopRadiusFraction;
    spec.sunwardEndcap.hubRadiusM            = spec.antisunwardEndcap.hubRadiusM;
    const double depth = endcapDepthInside(spec.antisunwardEndcap, spec.radiusM) +
                         endcapDepthInside(spec.sunwardEndcap, spec.radiusM);
    ui::textWrapped(
        std::format("The ramps reach {:.0f} m into the cylinder, leaving {:.0f} m of "
                    "level valley between them.",
                    depth, std::max(spec.lengthM - depth, 0.0)));
}

void drawEditorPartner(HabitatSpec& spec)
{
    ImGui::Checkbox("Counter-rotating partner", &spec.partner.enabled);
    sliderScaled("Distance (km)", spec.partner.separationM, 1000.0, 10.0, 300.0, "%.0f");
    ui::textWrapped(std::format(
        "At least {:.0f} km, so the mirrors clear each other however the pair is turned.",
        minimumPartnerSeparation(spec) / 1000.0));
}

void drawEditorShape(HabitatSpec& spec)
{
    if (spec.kind == HabitatKind::KALPANA_CYLINDER)
    {
        // A short cylinder with glass ends: no strips, no endcaps to shape, no partner.
        sliderDouble("Radius (m)", spec.radiusM, 100.0, 1000.0, "%.0f");
        sliderDouble("Length (m)", spec.lengthM, 100.0, 2000.0, "%.0f");
        sliderDouble("Gravity (g)", spec.surfaceGravityG, 0.1, 1.5, "%.2f");
        sliderDouble("People per km2", spec.populationDensityPerKm2, 0.0, 20000.0, "%.0f");
        const HabitatMetrics preview = computeMetrics(spec);
        ui::textMuted(std::format("Spins at {:.2f} rpm; hull: {}", preview.rpm,
                                  materialClassName(preview.material)));
        return;
    }
    if (spec.kind == HabitatKind::STANFORD_TORUS)
    {
        // A torus: the wheel, its tube and hub, the spokes, how far the land climbs the tube's
        // sides, how much of the ceiling is glass, and how the ring is shared out.
        TorusSpec& torus = spec.torus;
        sliderDouble("Floor radius (m)", spec.radiusM, 200.0, 5000.0, "%.0f");
        sliderDouble("Tube radius (m)", torus.tubeRadiusM, 20.0,
                     std::min(600.0, 0.35 * spec.radiusM), "%.0f");
        sliderDouble("Hub radius (m)", torus.hubRadiusM, 5.0, 0.25 * spec.radiusM, "%.0f");
        ImGui::SliderInt("Spokes", &torus.spokes, 0, 12);
        sliderDouble("Spoke radius (m)", torus.spokeRadiusM, 1.0, torus.tubeRadiusM, "%.1f");
        sliderDouble("Land up the sides (deg)", torus.landHalfAngleDeg, 5.0, 60.0, "%.0f");
        sliderDouble("Window share", torus.ceilingWindowShare, 0.0, 0.9, "%.2f");
        ImGui::SliderInt("Sections", &torus.sections, 0, 24);
        sliderDouble("Gravity (g)", spec.surfaceGravityG, 0.1, 1.5, "%.2f");
        sliderDouble("People per km2", spec.populationDensityPerKm2, 0.0, 25000.0, "%.0f");
        const HabitatMetrics preview = computeMetrics(spec);
        ui::textMuted(std::format(
            "Spins at {:.2f} rpm; hull: {}. The ceiling is {:.0f} m up; the sections alternate "
            "towns and farmland round the ring.",
            preview.rpm, materialClassName(preview.material), 2.0 * torus.tubeRadiusM));
        return;
    }
    if (spec.kind == HabitatKind::BERNAL_SPHERE)
    {
        // A sphere: its size, how far the land reaches from the equator, where the glass begins.
        sliderDouble("Radius (m)", spec.radiusM, 100.0, 1000.0, "%.0f");
        sliderDouble("Land to latitude (deg)", spec.sphere.landLatitudeDeg, 5.0, 60.0, "%.0f");
        sliderDouble("Windows from latitude (deg)", spec.sphere.windowLatitudeDeg,
                     spec.sphere.landLatitudeDeg + 5.0, 85.0, "%.0f");
        sliderDouble("Gravity (g)", spec.surfaceGravityG, 0.1, 1.5, "%.2f");
        sliderDouble("People per km2", spec.populationDensityPerKm2, 0.0, 25000.0, "%.0f");
        const HabitatMetrics preview = computeMetrics(spec);
        ui::textMuted(std::format(
            "Spins at {:.2f} rpm; hull: {}. Gravity at the land's edge: {:.2f} of the equator's",
            preview.rpm, materialClassName(preview.material),
            std::cos(degreesToRadians(spec.sphere.landLatitudeDeg))));
        return;
    }
    sliderDouble("Radius (m)", spec.radiusM, 200.0, 10000.0, "%.0f");
    sliderDouble("Length (m)", spec.lengthM, 1000.0, 60000.0, "%.0f");
    sliderDouble("Gravity (g)", spec.surfaceGravityG, 0.1, 1.5, "%.2f");
    ImGui::SliderInt("Valleys", &spec.stripPairs, 1, 6);
    sliderDouble("Window share", spec.windowFraction, 0.2, 0.7, "%.2f");
    sliderDouble("People per km2", spec.populationDensityPerKm2, 0.0, 20000.0, "%.0f");
    const HabitatMetrics preview = computeMetrics(spec);
    ui::textMuted(std::format("Spins at {:.2f} rpm; hull: {}", preview.rpm,
                              materialClassName(preview.material)));
    if (ImGui::CollapsingHeader("Endcaps"))
    {
        drawEditorEndcaps(spec);
    }
    if (ImGui::CollapsingHeader("Partner cylinder"))
    {
        drawEditorPartner(spec);
    }
}

/// The mirrors and the day they make.
void drawEditorDay(HabitatSpec& spec, DayScheduleSpec& day)
{
    sliderDouble("Mirror angle (deg)", spec.mirrors.openingAngleDeg, 20.0, 150.0, "%.0f");
    ui::textMuted(
        describeSun(spec.mirrors.openingAngleDeg, spec.kind, spec.sphere.windowLatitudeDeg));
    sliderDouble("Reflectivity", spec.mirrors.reflectivity, 0.3, 1.0, "%.2f");
    ImGui::Separator();
    ImGui::Checkbox("The mirrors keep a daily schedule", &day.enabled);
    ImGui::BeginDisabled(!day.enabled);
    sliderDouble("Daylight (hours)", day.dayLengthHours, 1.0, 23.0, "%.1f");
    sliderDouble("Sunrise (local hour)", day.sunriseHour, 0.0, 23.9, "%.1f");
    sliderDouble("Noon angle (deg)", day.noonAngleDeg, 20.0, 85.0, "%.0f");
    sliderDouble("Midnight angle (deg)", day.nightAngleDeg, 90.0, 150.0, "%.0f");
    ImGui::EndDisabled();
    if (!day.enabled)
    {
        ui::textWrapped("The mirrors stand still at the angle above: the same hour, for ever.");
        return;
    }
    if (daylightKindFor(spec.kind) == DaylightKind::OVERHEAD_MIRROR)
    {
        ui::textWrapped(std::format(
            "Sunrise {}, sunset {}: {:.1f} hours of daylight and {:.1f} of night. Past ninety "
            "degrees the louvres close over the ceiling's windows, which then show the stars.",
            clockHour(day.sunriseHour), clockHour(sunsetHour(day)), day.dayLengthHours,
            24.0 - day.dayLengthHours));
        return;
    }
    if (daylightKindFor(spec.kind) != DaylightKind::MIRROR_STRIPS)
    {
        // The light comes in through glass at the ends or the poles; shutters close them at night.
        ui::textWrapped(std::format(
            "Sunrise {}, sunset {}: {:.1f} hours of daylight and {:.1f} of night. Past ninety "
            "degrees the shutters close over the glass, which then shows the stars.",
            clockHour(day.sunriseHour), clockHour(sunsetHour(day)), day.dayLengthHours,
            24.0 - day.dayLengthHours));
        return;
    }
    ui::textWrapped(std::format(
        "Sunrise {}, sunset {}: {:.1f} hours of daylight and {:.1f} of night. The sun climbs to "
        "{:.0f} degrees at midday, and at midnight the mirrors stand {:.0f} degrees open, so no "
        "sunlight gets in and the windows show the stars.",
        clockHour(day.sunriseHour), clockHour(sunsetHour(day)), day.dayLengthHours,
        24.0 - day.dayLengthHours,
        radiansToDegrees(sunElevation(degreesToRadians(day.noonAngleDeg))), day.nightAngleDeg));
}

void drawEditorLand(TerrainSpec& terrain, SettlementSpec& settlements)
{
    ImGui::InputScalar("Seed", ImGuiDataType_U64, &terrain.seed);
    ui::textMuted("The one number the whole landscape grows from.");
    sliderDouble("Hills (m)", terrain.hillHeightM, 0.0, 200.0, "%.0f");
    sliderDouble("Endcap relief (m)", terrain.mountainHeightM, 0.0, 600.0, "%.0f");
    sliderDouble("Hill spacing (m)", terrain.featureSizeM, 100.0, 3000.0, "%.0f");
    ImGui::Separator();
    sliderDouble("River width (m)", terrain.riverWidthM, 0.0, 120.0, "%.0f");
    ImGui::SliderInt("Lakes per valley", &terrain.lakesPerValley, 0, 8);
    sliderDouble("Lake size (m)", terrain.lakeRadiusM, 10.0, 800.0, "%.0f");
    sliderDouble("Woods", terrain.forestCover, 0.0, 0.9, "%.2f");
    ImGui::Separator();
    ImGui::SliderInt("Towns per valley", &settlements.townsPerValley, 0, 12);
    sliderDouble("Town size (m)", settlements.townRadiusM, 40.0, 800.0, "%.0f");
    ImGui::SliderInt("Farms per valley", &settlements.farmsPerValley, 0, 60);
    ui::textMuted("The tramway follows the towns, so moving them moves the line.");
}

void drawEditorAir(AtmosphereSpec& air, ClimateSpec& climate, double radiusM)
{
    sliderScaled("Air pressure (kPa)", air.surfacePressurePa, 1000.0, 20.0, 110.0, "%.1f");
    sliderDouble("Temperature (K)", air.temperatureK, 260.0, 310.0, "%.0f");
    ImGui::Separator();
    sliderDouble("Cloud base (m)", climate.cloudBaseM, 50.0, 0.7 * radiusM, "%.0f");
    sliderDouble("Cloud top (m)", climate.cloudTopM, 100.0, 0.8 * radiusM, "%.0f");
    sliderDouble("Cloudiness", climate.cloudiness, 0.0, 1.0, "%.2f");
    sliderDouble("Rain", climate.raininess, 0.0, 1.0, "%.2f");
    sliderDouble("Morning mist", climate.mistiness, 0.0, 1.0, "%.2f");
    sliderDouble("Wind (m/s)", climate.windSpeedMS, 0.0, 20.0, "%.1f");
    ImGui::Separator();
    sliderDouble("Year (days)", climate.yearDays, 4.0, 400.0, "%.0f");
    sliderDouble("Season swing (hours)", climate.seasonSwingHours, 0.0, 8.0, "%.1f");
    sliderDouble("Season at the epoch", climate.seasonAtEpoch, 0.0, 0.999, "%.3f");
    ui::textWrapped(std::format(
        "A deck {:.0f} m thick, with {:.0f} m of clear air above it before the far side. The "
        "season knob decides where in the year the first of January 2000 fell, and so which "
        "season a visit starting today begins in.",
        std::max(climate.cloudTopM - climate.cloudBaseM, 0.0),
        std::max(radiusM - climate.cloudTopM, 0.0)));
}

/// Where in the solar system the habitat flies, when the visit starts, and where you stand.
void drawEditorPlace(Scenario& draft, const SkyModel& sky)
{
    int location = static_cast<int>(draft.sky.location);
    ImGui::Combo("Where", &location,
                 "Earth-Moon L4\0Earth-Moon L5\0Sun-Earth L4\0Sun-Earth L5\0Sun-Mars L4\0"
                 "Sun-Mars L5\0");
    draft.sky.location = static_cast<astro::Location>(location);
    ui::field("Visit begins", astro::formatIsoTime(draft.sky.start));
    if (ImGui::Button("Use the moment outside"))
    {
        draft.sky.start = sky.time;
    }
    sliderDouble("Clock offset from UTC (h)", draft.sky.utcOffsetHours, -14.0, 14.0, "%.1f");
    ImGui::Separator();
    const HabitatSpec& habitat = draft.habitat;
    if (habitat.kind != HabitatKind::ONEILL_CYLINDER)
    {
        // One band of land, all the way round the axis: along it is round, across it toward the
        // ends (or the poles).
        draft.start.band    = 0;
        const double round  = kPi * habitat.radiusM;
        double       across = 0.5 * habitat.lengthM;
        if (habitat.kind == HabitatKind::BERNAL_SPHERE)
        {
            across = habitat.radiusM * degreesToRadians(habitat.sphere.landLatitudeDeg);
        }
        else if (habitat.kind == HabitatKind::STANFORD_TORUS)
        {
            across = habitat.torus.tubeRadiusM * degreesToRadians(habitat.torus.landHalfAngleDeg);
        }
        sliderDouble("Start round (m)", draft.start.alongM, -round, round, "%.0f");
        sliderDouble("Start across (m)", draft.start.acrossM, -across, across, "%.0f");
        sliderDouble("Facing (deg)", draft.start.headingDeg, 0.0, 360.0, "%.0f");
        ui::textMuted("0 degrees faces spinward, round the land; the angle turns you to the left.");
        return;
    }
    ImGui::SliderInt("Start in valley", &draft.start.band, 0,
                     std::max(draft.habitat.stripPairs - 1, 0));
    const double half = 0.5 * draft.habitat.lengthM / 1000.0;
    sliderScaled("Start along (km)", draft.start.alongM, 1000.0, -half, half, "%.2f");
    sliderDouble("Facing (deg)", draft.start.headingDeg, 0.0, 360.0, "%.0f");
    ui::textMuted("0 degrees faces the sunward end; the angle turns you to the left.");
}

void numberRow(const char* label, const std::string& value)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ui::textMuted(label);
    ImGui::TableNextColumn();
    ui::text(value);
}

/// What the habitat in the editor would be like to live in, worked out as you drag the sliders.
void drawEditorNumbers(const HabitatSpec& spec)
{
    const HabitatMetrics metrics = computeMetrics(spec);
    if (ImGui::BeginTable("numbers", 2, ImGuiTableFlags_SizingStretchProp))
    {
        numberRow("Spin",
                  std::format("{:.3f} rpm, one turn in {:.0f} s", metrics.rpm, metrics.periodS));
        numberRow("Floor speed", std::format("{:.0f} m/s", metrics.rimSpeed));
        numberRow("Gravity", std::format("{:.2f} m/s2 ({:.2f} g)", metrics.floorGravity,
                                         spec.surfaceGravityG));
        numberRow("Head to foot", std::format("{:.2f} % lighter at head height",
                                              100.0 * metrics.headToFootGradient));
        numberRow("Coriolis", std::format("{:.2f} % of gravity at walking pace",
                                          100.0 * metrics.coriolisWalkingRatio));
        numberRow("Air at the axis",
                  std::format("{:.0f} % of the floor's pressure, {:.1f} K colder",
                              100.0 * metrics.axisPressureRatio, metrics.axisTemperatureDropK));
        numberRow("Land", std::format("{:.1f} km2", metrics.landAreaM2 / 1e6));
        numberRow("Windows", std::format("{:.1f} km2", metrics.windowAreaM2 / 1e6));
        numberRow("Volume", std::format("{:.1f} km3", metrics.volumeM3 / 1e9));
        numberRow("Room for", std::format("{:.0f} people", metrics.population));
        numberRow("Hull", std::format("{:.3f} MJ/kg of hoop strength: {}",
                                      metrics.hoopSpecificStrength / 1e6,
                                      materialClassName(metrics.material)));
        ImGui::EndTable();
    }
    ui::textWrapped(buildableToday(metrics.material)
                        ? "Nothing here needs a material we cannot already make."
                        : "This one waits on materials nobody can make at scale yet, so it is a "
                          "picture of a further future.");
}

void drawEditorFiles(EditorState& editor, HudSettings& settings, HudActions& actions)
{
    ImGui::InputText("Name", editor.saveName.data(), editor.saveName.size());
    ui::textMuted("What it is, in a line or two:");
    ImGui::InputTextMultiline("##description", editor.description.data(), editor.description.size(),
                              ImVec2(0.0F, ImGui::GetFontSize() * 3.5F));
    if (ImGui::Button("Save it"))
    {
        actions.save = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Browse habitats..."))
    {
        settings.showGallery   = true;
        actions.refreshGallery = true;
    }
    ui::textWrapped(
        "Habitats you save go next to the presets, as small TOML files. Send one to "
        "someone and they walk the same world, down to the last tree.");
}

void drawEditorTabs(const HudModel& model, EditorState& editor, HudSettings& settings,
                    HudActions& actions)
{
    if (!ImGui::BeginTabBar("editor"))
    {
        return;
    }
    Scenario& draft = editor.draft;
    if (ImGui::BeginTabItem("Shape"))
    {
        drawEditorShape(draft.habitat);
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Day"))
    {
        drawEditorDay(draft.habitat, draft.day);
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Land"))
    {
        drawEditorLand(draft.habitat.terrain, draft.habitat.settlements);
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Air"))
    {
        drawEditorAir(draft.habitat.atmosphere, draft.climate, draft.habitat.radiusM);
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Place"))
    {
        drawEditorPlace(draft, model.sky);
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Numbers"))
    {
        drawEditorNumbers(draft.habitat);
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("File"))
    {
        drawEditorFiles(editor, settings, actions);
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
}

void drawEditor(const HudModel& model, EditorState& editor, HudSettings& settings,
                HudActions& actions)
{
    const float scale = ImGui::GetFontSize() / 13.0F;
    ImGui::SetNextWindowSize(ImVec2(430.0F * scale, 460.0F * scale), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Habitat editor", &settings.showEditor))
    {
        drawEditorTabs(model, editor, settings, actions);
        ImGui::Separator();
        const auto problems = validateScenario(editor.draft);
        for (const std::string& problem : problems)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.5F, 0.4F, 1.0F));
            ui::textWrapped(problem);
            ImGui::PopStyleColor();
        }
        ImGui::BeginDisabled(!problems.empty() || model.generating);
        if (ImGui::Button(model.generating ? "Building..." : "Build it"))
        {
            actions.regenerate = true;
        }
        ImGui::EndDisabled();
        ui::textMutedWrapped("Nothing outside changes until you build it.");
    }
    ImGui::End();
}

void drawGalleryEntry(const GalleryEntry& entry, int index, const HudModel& model,
                      EditorState& editor, HudActions& actions)
{
    ImGui::PushID(index);
    if (ImGui::Selectable(entry.title.c_str(), index == editor.selected))
    {
        editor.selected = index;
    }
    ImGui::Indent();
    if (entry.problem.empty())
    {
        ui::textWrapped(entry.description);
        ui::textMutedWrapped(entry.summary);
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.5F, 0.4F, 1.0F));
        ui::textWrapped(entry.problem);
        ImGui::PopStyleColor();
    }
    ImGui::BeginDisabled(!entry.problem.empty() || model.generating);
    if (ImGui::Button("Go there"))
    {
        actions.load = entry.path;
    }
    ImGui::SameLine();
    if (ImGui::Button("Edit a copy"))
    {
        actions.editCopy = entry.path;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ui::textMuted(entry.preset ? "came with the program" : "saved here");
    ImGui::Unindent();
    ImGui::Separator();
    ImGui::PopID();
}

/// Every habitat file this machine has: the presets and the ones saved here.
void drawGallery(const HudModel& model, EditorState& editor, HudSettings& settings,
                 HudActions& actions)
{
    const float scale = ImGui::GetFontSize() / 13.0F;
    ImGui::SetNextWindowSize(ImVec2(460.0F * scale, 430.0F * scale), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Habitats", &settings.showGallery))
    {
        ui::textWrapped(
            "The habitats that came with the program and the ones you have saved. "
            "Each is a small text file: give it to someone and it builds the same "
            "world on their machine.");
        if (ImGui::Button("Look again"))
        {
            actions.refreshGallery = true;
        }
        ImGui::Separator();
        if (editor.gallery.empty())
        {
            ui::textMuted("No habitat files found.");
        }
        for (std::size_t i = 0; i < editor.gallery.size(); ++i)
        {
            drawGalleryEntry(editor.gallery[i], static_cast<int>(i), model, editor, actions);
        }
    }
    ImGui::End();
}

void drawHelp(const HudModel& model, HudSettings& settings)
{
    ImGui::Checkbox("Show controls", &settings.showHelp);
    if (settings.showHelp)
    {
        ui::textMuted(model.mouseCaptured ? "Mouse captured: Esc to release"
                                          : "Click the view to look around");
        ui::textMuted("WASD move, Shift run, Space jump (fly: rise), Ctrl descend");
        ui::textMuted("F walk/fly, V wings, G throw a ball, E kick, C comfort, wheel fly speed");
        ui::textMuted("I identify, B binoculars, P pause time, comma/period slower/faster");
        ui::textMuted("K almanac, F2 photo mode, Tab editor, F1 HUD, F12 screenshot");
    }
}

/// The tours on offer, and the way out of one that is running.
void drawTours(const HudModel& model, HudActions& actions)
{
    if (model.touring)
    {
        ui::textMuted("A tour is running. Press Stop, or take the controls, to end it.");
        if (ImGui::Button("Stop the tour"))
        {
            actions.stopTour = true;
        }
        return;
    }
    ui::textMuted("Sit back and be shown around. Moving takes the controls back.");
    for (std::size_t i = 0; i < model.tours.size(); ++i)
    {
        const TourName& tour = model.tours[i];
        ImGui::PushID(static_cast<int>(i));
        if (ImGui::Button("Take it"))
        {
            actions.startTour = i;
        }
        ImGui::PopID();
        ImGui::SameLine();
        ui::text(std::format("{} ({:.0f} s)", tour.name, tour.lengthS));
        ImGui::Indent();
        ui::textMuted(tour.blurb);
        ImGui::Unindent();
    }
}

/// What the tour is saying, across the bottom of the screen.
void drawTourCaption(const HudModel& model)
{
    if (!model.touring || model.tourCaption.empty() || model.tourFade <= 0.01)
    {
        return;
    }
    const float  scale  = ImGui::GetFontSize() / 13.0F;
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    const float  width  = std::min(760.0F * scale, screen.x - (40.0F * scale));
    ImGui::SetNextWindowPos(ImVec2(0.5F * (screen.x - width), screen.y - (140.0F * scale)),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, 0.0F), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.72F * static_cast<float>(model.tourFade));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav;
    if (ImGui::Begin("##tour", nullptr, flags))
    {
        ImGui::PushStyleColor(ImGuiCol_Text,
                              ImVec4(1.0F, 1.0F, 1.0F, static_cast<float>(model.tourFade)));
        ui::textWrapped(model.tourCaption);
        ImGui::PopStyleColor();
    }
    ImGui::End();
}

/// The almanac: a window of pages about the place, with the numbers it is really running on.
void drawAlmanac(const HudModel& model, HudSettings& settings)
{
    const float  scale  = ImGui::GetFontSize() / 13.0F;
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    const ImVec2 size(std::min(660.0F * scale, screen.x - (24.0F * scale)),
                      std::min(470.0F * scale, screen.y - (60.0F * scale)));
    ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(
        ImVec2(std::max(12.0F * scale, screen.x - size.x - (16.0F * scale)), 60.0F * scale),
        ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Almanac", &settings.showAlmanac))
    {
        ImGui::End();
        return;
    }
    if (model.almanac.empty())
    {
        ui::textMuted("Nothing to describe yet.");
        ImGui::End();
        return;
    }
    settings.almanacPage =
        std::clamp(settings.almanacPage, 0, static_cast<int>(model.almanac.size()) - 1);

    // The contents down the left, the page itself on the right.
    ImGui::BeginChild("contents", ImVec2(196.0F * scale, 0.0F), ImGuiChildFlags_Borders);
    for (std::size_t i = 0; i < model.almanac.size(); ++i)
    {
        const bool chosen = std::cmp_equal(i, settings.almanacPage);
        if (ImGui::Selectable(model.almanac[i].title.c_str(), chosen))
        {
            settings.almanacPage = static_cast<int>(i);
        }
    }
    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::BeginChild("page");
    const AlmanacPage& page = model.almanac[static_cast<std::size_t>(settings.almanacPage)];
    ImGui::SeparatorText(page.title.c_str());
    ui::textWrapped(page.story);
    ImGui::Spacing();
    for (const AlmanacFact& fact : page.facts)
    {
        // The labels here are whole phrases, so they get a line of their own with the number
        // after them, and the note under both.
        ui::textMuted(fact.label);
        ImGui::SameLine();
        ImGui::TextUnformatted(fact.value.c_str());
        if (!fact.note.empty())
        {
            ImGui::Indent(16.0F * scale);
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
            ui::textWrapped(fact.note);
            ImGui::PopStyleColor();
            ImGui::Unindent(16.0F * scale);
        }
        ImGui::Spacing();
    }
    ImGui::EndChild();
    ImGui::End();
}

/// Photo mode: the controls that matter for a picture, and nothing else.
void drawPhotoMode(const HudModel& model, HudSettings& settings, HudActions& actions)
{
    const float scale = ImGui::GetFontSize() / 13.0F;
    ImGui::SetNextWindowPos(ImVec2(12.0F * scale, 12.0F * scale), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(330.0F * scale, 0.0F), ImGuiCond_Always);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
    if (!ImGui::Begin("Photo mode (F2)", nullptr, flags))
    {
        ImGui::End();
        return;
    }
    ui::textMuted("Fly where you like; the HUD stays out of the picture.");
    ImGui::SliderFloat("Exposure", &settings.exposure, 0.1F, 16.0F, "%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Field of view", &settings.fieldOfViewDeg, 2.0F, 100.0F, "%.0f deg",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Painted colours", &settings.grade, 0.0F, 1.0F, "%.2f");

    ImGui::Separator();
    if (ImGui::Checkbox("Long exposure", &settings.trails))
    {
        settings.trailsReset = true;
    }
    if (ImGui::BeginItemTooltip())
    {
        ImGui::TextUnformatted(
            "Keeps the brightest each pixel has been. Hold still and the stars "
            "draw arcs as the habitat turns: a full circle every two minutes.");
        ImGui::EndTooltip();
    }
    ImGui::BeginDisabled(!settings.trails);
    ImGui::SameLine();
    if (ImGui::Button("Start again"))
    {
        settings.trailsReset = true;
    }
    ImGui::EndDisabled();

    ImGui::BeginDisabled(settings.trails);
    ImGui::SliderInt("Picture size", &settings.captureScale, 1, 4,
                     settings.captureScale > 1 ? "%dx the window" : "the window");
    ImGui::EndDisabled();
    ui::textMuted(settings.trails
                      ? std::format("{} x {} pixels (an exposure is held at the window's size)",
                                    model.photoWidth, model.photoHeight)
                      : std::format("{} x {} pixels", model.photoWidth, model.photoHeight));
    if (ImGui::Button("Take the picture (F12)"))
    {
        actions.screenshot = true;
    }
    if (!model.status.empty())
    {
        ui::textWrapped(model.status);
    }
    ImGui::End();
}

}  // namespace

std::string timeScaleName(double scale)
{
    if (scale >= 86400.0)
    {
        return std::format("{:g} d/s", scale / 86400.0);
    }
    if (scale >= 3600.0)
    {
        return std::format("{:g} h/s", scale / 3600.0);
    }
    if (scale >= 60.0)
    {
        return std::format("{:g} min/s", scale / 60.0);
    }
    return std::format("{:g}x", scale);
}

HudActions drawHud(const HudModel& model, HudSettings& settings, EditorState& editor,
                   PlayerSettings& player)
{
    HudActions  actions;
    const float scale = ImGui::GetFontSize() / 13.0F;
    if (settings.photoMode)
    {
        drawPhotoMode(model, settings, actions);
        drawTourCaption(model);
        return actions;
    }
    if (model.touring)
    {
        // A tour shows the place, not the instruments: just the caption and a way to stop.
        drawTourCaption(model);
        ImGui::SetNextWindowPos(ImVec2(12.0F * scale, 12.0F * scale), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.5F);
        const ImGuiWindowFlags bare = ImGuiWindowFlags_NoDecoration |
                                      ImGuiWindowFlags_AlwaysAutoResize |
                                      ImGuiWindowFlags_NoSavedSettings;
        if (ImGui::Begin("##touring", nullptr, bare))
        {
            if (ImGui::Button("Stop the tour"))
            {
                actions.stopTour = true;
            }
        }
        ImGui::End();
        return actions;
    }
    ImGui::SetNextWindowPos(ImVec2(12.0F * scale, 12.0F * scale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(440.0F * scale, 0.0F), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(model.title.c_str()))
    {
        drawLocation(model, settings, player, actions);
        if (ImGui::CollapsingHeader("Sky, time and look", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawTimeAndLook(model, settings, actions);
        }
        if (ImGui::CollapsingHeader("Guided tours"))
        {
            drawTours(model, actions);
        }
        if (ImGui::CollapsingHeader("Weather and the year"))
        {
            drawWeather(model, settings);
        }
        if (ImGui::CollapsingHeader("This habitat"))
        {
            drawMetrics(model, settings, actions);
        }
        if (ImGui::CollapsingHeader("Renderer"))
        {
            drawRenderer(model);
        }
        if (ImGui::CollapsingHeader("Credits"))
        {
            drawCredits();
        }
        if (!model.status.empty())
        {
            ImGui::Separator();
            ui::textWrapped(model.status);
        }
        ImGui::Separator();
        drawHelp(model, settings);
    }
    ImGui::End();
    if (settings.showEditor)
    {
        drawEditor(model, editor, settings, actions);
    }
    if (settings.showGallery)
    {
        drawGallery(model, editor, settings, actions);
    }
    if (settings.showAlmanac)
    {
        drawAlmanac(model, settings);
    }
    drawSkyLabel(model.sky.label);
    return actions;
}

}  // namespace StarshipSimulator
