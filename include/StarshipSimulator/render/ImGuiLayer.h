#pragma once

#include <string>

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_video.h>

struct ImDrawData;
struct ImGuiContext;

namespace StarshipSimulator
{

/// Owns the Dear ImGui context with its SDL3 platform and SDL_GPU renderer backends.
class ImGuiLayer
{
public:
    ImGuiLayer(SDL_Window* window, SDL_GPUDevice* device, SDL_GPUTextureFormat targetFormat);
    ~ImGuiLayer();

    ImGuiLayer(const ImGuiLayer&)            = delete;
    ImGuiLayer& operator=(const ImGuiLayer&) = delete;
    ImGuiLayer(ImGuiLayer&&)                 = delete;
    ImGuiLayer& operator=(ImGuiLayer&&)      = delete;

    void processEvent(const SDL_Event& event);
    void beginFrame();
    /// Ends the frame and returns what to draw (pass it to Renderer::renderFrame).
    [[nodiscard]] ImDrawData* endFrame();

    [[nodiscard]] bool wantsMouse() const;
    [[nodiscard]] bool wantsKeyboard() const;
    /// While the mouse drives the camera, ImGui must ignore it.
    void setMouseEnabled(bool enabled);

private:
    std::string   iniPath_;  // must outlive the ImGui context
    ImGuiContext* context_ = nullptr;
};

/// ImGui helpers that avoid printf-style formatting (our warnings forbid C varargs).
namespace ui
{
void text(const std::string& text);
void textMuted(const std::string& text);
void textWrapped(const std::string& text);
void textMutedWrapped(const std::string& text);
/// "label: value" on one line with the value aligned after the label column.
void field(const char* label, const std::string& value);
}  // namespace ui

}  // namespace StarshipSimulator
