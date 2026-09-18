#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/passes/marker_pass.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"

struct ImDrawData;

namespace StarshipSimulator
{

class GridPass;
class TonemapPass;

/// What to draw this frame.
struct SceneView
{
    Camera                  camera;
    std::span<const Marker> markers;
    float                   exposure = 1.0F;
};

struct FrameOptions
{
    ImDrawData*                          ui = nullptr;  // HUD drawn over the scene, may be null
    std::optional<std::filesystem::path> screenshot;    // save this frame as a PNG
    bool                                 screenshotIncludesUi = false;
};

struct FrameResult
{
    bool          presented = false;  // false while the window is minimized or hidden
    std::uint32_t width     = 0;      // swapchain size in pixels
    std::uint32_t height    = 0;
    std::optional<std::expected<std::filesystem::path, std::string>> screenshot;
};

/// Draws frames: the HDR scene (MSAA, reverse-Z), then tonemapping and the UI into the swapchain.
class Renderer
{
public:
    Renderer(GpuDevice& device, std::filesystem::path shaderDirectory);
    ~Renderer();

    Renderer(const Renderer&)            = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&)                 = delete;
    Renderer& operator=(Renderer&&)      = delete;

    FrameResult renderFrame(const SceneView& view, const FrameOptions& options);

    /// Rebuilds all pipelines from the shaders on disk. On failure the old pipelines stay in use.
    std::expected<void, std::string> reloadShaders();

    [[nodiscard]] const SceneFormats&          sceneFormats() const { return targets_.formats(); }
    [[nodiscard]] const std::filesystem::path& shaderDirectory() const
    {
        return shaders_.directory();
    }

private:
    struct Passes;

    [[nodiscard]] std::unique_ptr<Passes> createPasses() const;
    void drawScene(SDL_GPUCommandBuffer* commands, const SceneView& view, std::uint32_t width,
                   std::uint32_t height);
    void drawDisplay(SDL_GPUCommandBuffer* commands, SDL_GPUTexture* target, const SceneView& view,
                     ImDrawData* ui);
    [[nodiscard]] std::expected<std::filesystem::path, std::string> captureAndSubmit(
        SDL_GPUCommandBuffer* commands, const SceneView& view, const std::filesystem::path& path,
        ImDrawData* ui, std::uint32_t width, std::uint32_t height);

    GpuDevice*              device_;
    ShaderLibrary           shaders_;
    RenderTargets           targets_;
    std::unique_ptr<Passes> passes_;
};

}  // namespace StarshipSimulator
