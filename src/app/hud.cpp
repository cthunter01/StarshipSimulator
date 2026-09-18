#include "hud.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <format>
#include <optional>
#include <string>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/sky_objects.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/player_controller.h"
#include "StarshipSimulator/core/physics/rotating_frame.h"
#include "StarshipSimulator/core/units.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/imgui_layer.h"

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

std::string describeSun(double openingAngleDeg)
{
    const double alpha = degreesToRadians(openingAngleDeg);
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

void drawThrowReport(const ThrowReport& report)
{
    ui::textWrapped(std::format(
        "Last throw: {:.1f} m in {:.2f} s. The spin moved the landing point {:.2f} m to the {} "
        "and {:.2f} m {} than on a planet with the same gravity.",
        report.range, report.flightTime, std::abs(report.lateral),
        report.lateral >= 0.0 ? "left" : "right", std::abs(report.along),
        report.along >= 0.0 ? "further" : "shorter"));
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

    std::string where = regionKindName(ground.region.kind);
    if (ground.region.index >= 0)
    {
        where += std::format(" {}", ground.region.index + 1);
    }
    ui::field("Where", std::format("{}, {:.2f} km along, {:.0f} deg around", where, eye.z / 1000.0,
                                   radiansToDegrees(HabitatGeometry::angleOf(eye))));
    ui::field("Height",
              std::format("{:.1f} m above ground, {:.0f} m from the axis",
                          std::max(0.0, ground.heightAboveGround - player.eyeHeight) + 0.0, r));
    ui::field("Gravity", std::format("{:.3f} g", gravity / units::kStandardGravity));
    ui::field("Air", std::format("{:.1f} kPa", pressure));
    ui::field("Coriolis", std::format("{:.3f} m/s^2 ({:.1f}% of your weight)", coriolis,
                                      100.0 * coriolis / std::max(gravity, 1e-6)));
    const char* mode = "airborne";
    if (you.locomotion() == Locomotion::Fly)
    {
        mode = "flying";
    }
    else if (you.grounded())
    {
        mode = "walking";
    }
    ui::field("Moving", std::format("{:.1f} m/s, {}", glm::length(you.velocity()), mode));
    if (you.locomotion() == Locomotion::Fly)
    {
        ui::field("Fly speed", std::format("{:.0f} m/s (mouse wheel)", player.flySpeed));
    }

    if (ImGui::Button(you.locomotion() == Locomotion::Walk ? "Fly (F)" : "Walk (F)"))
    {
        actions.toggleLocomotion = true;
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
    ui::textMuted(describeSun(static_cast<double>(settings.mirrorAngleDeg)));
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
    ImGui::SliderFloat("Field of view", &settings.fieldOfViewDeg, 2.0F, 100.0F, "%.0f deg",
                       ImGuiSliderFlags_Logarithmic);
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

void drawMetrics(const HudModel& model)
{
    const HabitatMetrics&     m    = *model.metrics;
    const OneillCylinderSpec& spec = model.geometry->spec();
    ui::field("Size", std::format("{:.1f} km across, {:.1f} km long", 2.0 * spec.radiusM / 1000.0,
                                  spec.lengthM / 1000.0));
    ui::field("Spin", std::format("{:.2f} rpm, one turn every {:.0f} s", m.rpm, m.periodS));
    ui::field("Floor", std::format("{:.2f} g, moving at {:.0f} m/s",
                                   m.floorGravity / units::kStandardGravity, m.rimSpeed));
    ui::field("Land", std::format("{:.0f} km^2 for {:.1f} million people", m.landAreaM2 / 1e6,
                                  m.population / 1e6));
    ui::field("Axis air", std::format("{:.0f}% of floor pressure, {:.0f} K colder",
                                      100.0 * m.axisPressureRatio, m.axisTemperatureDropK));
    ui::field("Walking",
              std::format("Coriolis {:.1f}% of your weight", 100.0 * m.coriolisWalkingRatio));
    ui::field("Hull", std::format("needs {:.3f} MJ/kg: {}", m.hoopSpecificStrength / 1e6,
                                  materialClassName(m.material)));
    ui::textMuted(buildableToday(m.material) ? "Buildable with materials we have today."
                                             : "Needs materials that do not exist yet.");
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
}

void drawEditorShape(OneillCylinderSpec& spec)
{
    sliderDouble("Radius (m)", spec.radiusM, 200.0, 10000.0, "%.0f");
    sliderDouble("Length (m)", spec.lengthM, 1000.0, 60000.0, "%.0f");
    sliderDouble("Gravity (g)", spec.surfaceGravityG, 0.1, 1.5, "%.2f");
    ImGui::SliderInt("Valleys", &spec.stripPairs, 1, 6);
    sliderDouble("Window share", spec.windowFraction, 0.2, 0.7, "%.2f");
    const HabitatMetrics preview = computeMetrics(spec);
    ui::textMuted(std::format("Spins at {:.2f} rpm; hull: {}", preview.rpm,
                              materialClassName(preview.material)));

    if (ImGui::CollapsingHeader("Endcaps"))
    {
        int antisunward = static_cast<int>(spec.antisunwardEndcap.shape);
        int sunward     = static_cast<int>(spec.sunwardEndcap.shape);
        ImGui::Combo("Anti-sunward end", &antisunward, "Flat wall\0Dome\0Mountain ramp\0");
        ImGui::Combo("Sunward end", &sunward, "Flat wall\0Dome\0Mountain ramp\0");
        spec.antisunwardEndcap.shape = static_cast<EndcapShape>(antisunward);
        spec.sunwardEndcap.shape     = static_cast<EndcapShape>(sunward);
        sliderDouble("Ramp slope (deg)", spec.antisunwardEndcap.rampSlopeDeg, 10.0, 40.0, "%.0f");
        spec.sunwardEndcap.rampSlopeDeg = spec.antisunwardEndcap.rampSlopeDeg;
    }
    if (ImGui::CollapsingHeader("Partner cylinder"))
    {
        ImGui::Checkbox("Counter-rotating partner", &spec.partner.enabled);
        double kilometres = spec.partner.separationM / 1000.0;
        sliderDouble("Distance (km)", kilometres, 10.0, 300.0, "%.0f");
        spec.partner.separationM = kilometres * 1000.0;
        ui::textMuted(std::format("At least {:.0f} km, so the mirrors clear each other",
                                  minimumPartnerSeparation(spec) / 1000.0));
    }
    if (ImGui::CollapsingHeader("Terrain and air"))
    {
        ImGui::InputScalar("Seed", ImGuiDataType_U64, &spec.terrain.seed);
        sliderDouble("Hills (m)", spec.terrain.hillHeightM, 0.0, 200.0, "%.0f");
        sliderDouble("Mountains (m)", spec.terrain.mountainHeightM, 0.0, 600.0, "%.0f");
        sliderDouble("Hill spacing (m)", spec.terrain.featureSizeM, 100.0, 3000.0, "%.0f");
        double kilopascals = spec.atmosphere.surfacePressurePa / 1000.0;
        sliderDouble("Air pressure (kPa)", kilopascals, 20.0, 110.0, "%.1f");
        spec.atmosphere.surfacePressurePa = kilopascals * 1000.0;
    }
}

void drawEditorFiles(EditorState& editor, HudActions& actions)
{
    ImGui::InputText("Name", editor.saveName.data(), editor.saveName.size());
    ImGui::SameLine();
    if (ImGui::Button("Save"))
    {
        actions.save = true;
    }
    if (editor.scenarios.empty())
    {
        return;
    }
    const auto label = [&](int i) {
        return editor.scenarios.at(static_cast<std::size_t>(i)).stem().string();
    };
    const int count = static_cast<int>(editor.scenarios.size());
    editor.selected = std::clamp(editor.selected, 0, count - 1);
    if (ImGui::BeginCombo("Scenario", label(editor.selected).c_str()))
    {
        for (int i = 0; i < count; ++i)
        {
            if (ImGui::Selectable(label(i).c_str(), i == editor.selected))
            {
                editor.selected = i;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button("Load"))
    {
        actions.load = editor.scenarios.at(static_cast<std::size_t>(editor.selected));
    }
}

void drawEditor(const HudModel& model, EditorState& editor, HudSettings& settings,
                HudActions& actions)
{
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetFontSize() * 30.0F, 0.0F), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Habitat editor", &settings.showEditor))
    {
        drawEditorShape(editor.draft);
        const auto problems = validate(editor.draft);
        for (const std::string& problem : problems)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.5F, 0.4F, 1.0F));
            ui::textWrapped(problem);
            ImGui::PopStyleColor();
        }
        ImGui::BeginDisabled(!problems.empty() || model.generating);
        if (ImGui::Button(model.generating ? "Generating..." : "Regenerate"))
        {
            actions.regenerate = true;
        }
        ImGui::EndDisabled();
        ImGui::Separator();
        drawEditorFiles(editor, actions);
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
        ui::textMuted("F walk/fly, G throw, C comfort, wheel fly speed");
        ui::textMuted("I identify, B binoculars, P pause time, comma/period slower/faster");
        ui::textMuted("Tab editor, F1 HUD, F12 screenshot");
    }
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
    ImGui::SetNextWindowPos(ImVec2(12.0F * scale, 12.0F * scale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(440.0F * scale, 0.0F), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(model.title.c_str()))
    {
        drawLocation(model, settings, player, actions);
        if (ImGui::CollapsingHeader("Sky, time and look", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawTimeAndLook(model, settings, actions);
        }
        if (ImGui::CollapsingHeader("This habitat"))
        {
            drawMetrics(model);
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
    drawSkyLabel(model.sky.label);
    return actions;
}

}  // namespace StarshipSimulator
