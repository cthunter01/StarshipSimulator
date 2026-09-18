#include "hud.h"

#include <imgui.h>

#include <cstddef>
#include <format>
#include <string>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/fly_controller.h"
#include "StarshipSimulator/core/math.h"
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

/// Floating labels over the waypoint beacons, projected with the same camera-relative transform
/// the renderer uses.
void drawWaypointLabels(const HudModel& model)
{
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    if (display.x <= 0.0F || display.y <= 0.0F)
    {
        return;
    }
    const Mat4d viewProjection = cameraRelativeViewProjection(
        model.camera, static_cast<double>(display.x) / static_cast<double>(display.y));
    ImDrawList* drawList = ImGui::GetBackgroundDrawList();
    for (const Waypoint& waypoint : model.waypoints)
    {
        const Vec3d relative = waypoint.beaconPosition - model.camera.position;
        const Vec4d clip     = viewProjection * Vec4d(relative, 1.0);
        if (clip.w <= 0.0)
        {
            continue;  // behind the camera
        }
        const double      x = (((clip.x / clip.w) * 0.5) + 0.5) * static_cast<double>(display.x);
        const double      y = (0.5 - ((clip.y / clip.w) * 0.5)) * static_cast<double>(display.y);
        const std::string label =
            std::format("{} ({:.0f} m)", waypoint.name, glm::length(relative));
        drawList->AddText(ImVec2(static_cast<float>(x), static_cast<float>(y)),
                          ImGui::GetColorU32(ImVec4(1.0F, 0.9F, 0.67F, 1.0F)), label.c_str());
    }
}

void drawStatus(const HudModel& model)
{
    const GpuInfo&       gpu        = *model.gpu;
    const FlyController& controller = *model.controller;
    const Vec3d&         position   = controller.eyePosition();

    ui::field("GPU", gpu.deviceName);
    ui::field("Driver", std::format("{} {}", gpu.driverName, gpu.driverVersion));
    ui::field("Present", std::format("{}, {}, MSAA {}x{}", gpu.presentMode, gpu.swapchainFormat,
                                     sampleCountValue(model.samples), gpu.debug ? ", debug" : ""));
    ui::field("Frame", std::format("{:.1f} fps, {:.2f} ms", model.fps, model.frameMs));
    ImGui::Separator();
    ui::field("x", std::format("{:.3f} m", position.x));
    ui::field("y", std::format("{:.3f} m", position.y));
    ui::field("z", std::format("{:.3f} m", position.z));
    ui::field("Speed", std::format("{:.2f} m/s", glm::length(controller.velocity())));
    ui::field("Mode", controller.locomotion() == Locomotion::Walk
                          ? std::string("walk")
                          : std::format("fly, {:.0f} m/s (wheel)", controller.settings.flySpeed));
}

}  // namespace

HudActions drawHud(const HudModel& model, HudSettings& settings)
{
    HudActions actions;
    drawWaypointLabels(model);

    const float scale = ImGui::GetFontSize() / 13.0F;
    ImGui::SetNextWindowPos(ImVec2(12.0F * scale, 12.0F * scale), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("StarshipSimulator", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        drawStatus(model);
        ImGui::Separator();

        ui::textMuted("Teleport (look for jitter on the small cubes ahead):");
        for (std::size_t i = 0; i < model.waypoints.size(); ++i)
        {
            if (i > 0)
            {
                ImGui::SameLine();
            }
            if (ImGui::Button(model.waypoints[i].name.c_str()))
            {
                actions.teleportTo = i;
            }
        }
        if (ImGui::Button(model.controller->locomotion() == Locomotion::Walk ? "Fly (F)"
                                                                             : "Walk (F)"))
        {
            actions.toggleLocomotion = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reload shaders (F5)"))
        {
            actions.reloadShaders = true;
        }
        ImGui::SliderFloat("Exposure", &settings.exposure, 0.1F, 8.0F, "%.2f",
                           ImGuiSliderFlags_Logarithmic);

        if (!model.status.empty())
        {
            ImGui::Separator();
            ui::textWrapped(model.status);
        }

        ImGui::Separator();
        ImGui::Checkbox("Show controls", &settings.showHelp);
        if (settings.showHelp)
        {
            ui::textMuted(model.mouseCaptured ? "Mouse captured: Esc to release"
                                              : "Click the view to look around");
            ui::textMuted("WASD move, Shift run/fast, Space jump/up, Ctrl down");
            ui::textMuted("F walk/fly, wheel fly speed, F1 HUD, F12 screenshot");
        }
    }
    ImGui::End();
    return actions;
}

}  // namespace StarshipSimulator
