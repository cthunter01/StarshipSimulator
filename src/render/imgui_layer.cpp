#include "StarshipSimulator/render/imgui_layer.h"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlgpu3.h>

#include <memory>
#include <stdexcept>
#include <string>

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_stdinc.h>
#include <SDL3/SDL_video.h>

namespace StarshipSimulator
{

namespace
{

struct SdlFree
{
    void operator()(char* memory) const noexcept { SDL_free(memory); }
};

ImGuiContext* createContext()
{
    IMGUI_CHECKVERSION();
    return ImGui::CreateContext();
}

std::string settingsPath()
{
    const std::unique_ptr<char, SdlFree> prefPath(SDL_GetPrefPath("", "StarshipSimulator"));
    return prefPath ? std::string(prefPath.get()) + "imgui.ini" : std::string("imgui.ini");
}

}  // namespace

ImGuiLayer::ImGuiLayer(SDL_Window* window, SDL_GPUDevice* device, SDL_GPUTextureFormat targetFormat)
  : iniPath_(settingsPath()), context_(createContext())
{
    ImGuiIO& io    = ImGui::GetIO();
    io.IniFilename = iniPath_.c_str();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    const float scale = SDL_GetWindowDisplayScale(window);
    if (scale > 0.0F)
    {
        style.ScaleAllSizes(scale);
        style.FontScaleDpi = scale;
    }
    style.WindowRounding = 6.0F;
    style.FrameRounding  = 4.0F;
    style.Colors[ImGuiCol_WindowBg].w =
        0.82F;  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    if (!ImGui_ImplSDL3_InitForSDLGPU(window))
    {
        ImGui::DestroyContext();
        throw std::runtime_error("ImGui SDL3 backend initialisation failed");
    }
    ImGui_ImplSDLGPU3_InitInfo info;
    info.Device            = device;
    info.ColorTargetFormat = targetFormat;
    info.MSAASamples       = SDL_GPU_SAMPLECOUNT_1;
    if (!ImGui_ImplSDLGPU3_Init(&info))
    {
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
        throw std::runtime_error("ImGui SDL_GPU backend initialisation failed");
    }
}

ImGuiLayer::~ImGuiLayer()
{
    ImGui::SetCurrentContext(context_);
    ImGui_ImplSDLGPU3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext(context_);
}

void ImGuiLayer::processEvent(const SDL_Event& event)
{
    ImGui::SetCurrentContext(context_);
    ImGui_ImplSDL3_ProcessEvent(&event);
}

void ImGuiLayer::beginFrame()
{
    ImGui::SetCurrentContext(context_);
    ImGui_ImplSDLGPU3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
}

ImDrawData* ImGuiLayer::endFrame()
{
    ImGui::SetCurrentContext(context_);
    ImGui::Render();
    return ImGui::GetDrawData();
}

bool ImGuiLayer::wantsMouse() const
{
    ImGui::SetCurrentContext(context_);
    return ImGui::GetIO().WantCaptureMouse;
}

bool ImGuiLayer::wantsKeyboard() const
{
    ImGui::SetCurrentContext(context_);
    return ImGui::GetIO().WantCaptureKeyboard;
}

void ImGuiLayer::setMouseEnabled(bool enabled)
{
    ImGui::SetCurrentContext(context_);
    ImGuiIO& io = ImGui::GetIO();
    if (enabled)
    {
        io.ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
    }
    else
    {
        io.ConfigFlags |= ImGuiConfigFlags_NoMouse;
    }
}

namespace ui
{

void text(const std::string& text)
{
    ImGui::TextUnformatted(text.c_str());
}

void textMuted(const std::string& text)
{
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextUnformatted(text.c_str());
    ImGui::PopStyleColor();
}

void textWrapped(const std::string& text)
{
    ImGui::PushTextWrapPos(0.0F);
    ImGui::TextUnformatted(text.c_str());
    ImGui::PopTextWrapPos();
}

void field(const char* label, const std::string& value)
{
    textMuted(label);
    ImGui::SameLine(ImGui::GetFontSize() * 7.0F);
    ImGui::TextUnformatted(value.c_str());
}

}  // namespace ui

}  // namespace StarshipSimulator
