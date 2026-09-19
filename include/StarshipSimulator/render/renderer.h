#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/assets/assets.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/gpu_landscape.h"
#include "StarshipSimulator/render/gpu_trees.h"
#include "StarshipSimulator/render/gpu_world.h"
#include "StarshipSimulator/render/passes/marker_pass.h"
#include "StarshipSimulator/render/passes/sky_passes.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"
#include "StarshipSimulator/render/shadow_map.h"
#include "StarshipSimulator/render/upload.h"

struct ImDrawData;

namespace StarshipSimulator
{

enum class BodyTextures : std::uint8_t
{
    Earth,
    Moon,
};

/// Earth or the Moon, and which images to draw it with.
struct BodyDraw
{
    gpu::BodyUniforms uniforms;
    BodyTextures      textures = BodyTextures::Earth;
};

/// What to draw this frame.
struct SceneView
{
    Camera                    camera;
    const GpuWorld*           world     = nullptr;  // the habitat; nothing but the sky if null
    GpuLandscape*             landscape = nullptr;  // its terrain (patches picked per frame)
    const GpuTrees*           trees     = nullptr;  // its trees
    gpu::ShadowUniforms       shadow;               // the trees' shadow map (params.x = 0: none)
    gpu::HabitatUniforms      habitat;
    gpu::SkyUniforms          sky;
    gpu::PlanetUniforms       planets;
    std::span<const BodyDraw> bodies;   // farthest first
    std::optional<Mat4d>      partner;  // the partner cylinder: its frame -> the habitat frame
    std::span<const Marker>   markers;
    float                     exposure         = 1.0F;
    float                     grade            = 1.0F;  // strength of the painterly colour grade
    double                    animationSeconds = 0.0;   // real time, for ripples and the like
};

/// The sky's images, decoded on the CPU. Missing ones keep their placeholders.
struct SkyImages
{
    std::optional<assets::HalfImage> milkyWay;
    std::optional<assets::Image8>    earthDay;
    std::optional<assets::Image8>    earthNight;
    std::optional<assets::Image8>    moon;
};

struct FrameOptions
{
    ImDrawData*                          ui = nullptr;  // HUD drawn over the scene, may be null
    std::optional<std::filesystem::path> screenshot;    // save this frame as a PNG
    bool                                 screenshotIncludesUi = false;
};

struct FrameResult
{
    bool          presented      = false;  // false while the window is minimized or hidden
    std::uint32_t width          = 0;      // swapchain size in pixels
    std::uint32_t height         = 0;
    std::uint32_t chunksDrawn    = 0;
    std::uint64_t trianglesDrawn = 0;
    std::optional<std::expected<std::filesystem::path, std::string>> screenshot;
};

/// Draws frames: the HDR scene (MSAA, reverse-Z), then tonemapping and the UI into the swapchain.
class Renderer
{
public:
    Renderer(GpuDevice& device, std::filesystem::path shaderDirectory, std::vector<GpuStar> stars);
    ~Renderer();

    Renderer(const Renderer&)            = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&)                 = delete;
    Renderer& operator=(Renderer&&)      = delete;

    FrameResult renderFrame(const SceneView& view, const FrameOptions& options);

    /// Rebuilds all pipelines from the shaders on disk. On failure the old pipelines stay in use.
    std::expected<void, std::string> reloadShaders();

    /// Replaces the stars (e.g. the placeholder field with the real catalog).
    void setStars(std::vector<GpuStar> stars);
    /// Uploads the sky's images. Throws std::runtime_error if the GPU cannot take them.
    void setSkyImages(const SkyImages& images);
    /// The outside of the habitat, drawn for the partner cylinder.
    void setHull(const CpuMesh& hull);

    [[nodiscard]] const SceneFormats&          sceneFormats() const { return targets_.formats(); }
    [[nodiscard]] const std::filesystem::path& shaderDirectory() const
    {
        return shaders_.directory();
    }

private:
    struct Passes;

    [[nodiscard]] std::unique_ptr<Passes> createPasses() const;
    void drawScene(SDL_GPUCommandBuffer* commands, const SceneView& view, std::uint32_t width,
                   std::uint32_t height, FrameResult& result);
    void drawShadows(SDL_GPUCommandBuffer* commands, const SceneView& view,
                     const gpu::FrameUniforms& frame);
    void drawDisplay(SDL_GPUCommandBuffer* commands, SDL_GPUTexture* target, const SceneView& view,
                     ImDrawData* ui);
    [[nodiscard]] std::expected<std::filesystem::path, std::string> captureAndSubmit(
        SDL_GPUCommandBuffer* commands, const SceneView& view, const std::filesystem::path& path,
        ImDrawData* ui, std::uint32_t width, std::uint32_t height);

    GpuDevice*              device_;
    std::vector<GpuStar>    stars_;
    ShaderLibrary           shaders_;
    RenderTargets           targets_;
    SkyTextures             skyTextures_;
    ShadowMap               shadowMap_;
    GpuMesh                 hull_;
    std::unique_ptr<Passes> passes_;
};

}  // namespace StarshipSimulator
