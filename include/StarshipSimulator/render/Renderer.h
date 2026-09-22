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
#include "StarshipSimulator/core/procgen/clouds.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/render/GpuBirds.h"
#include "StarshipSimulator/render/GpuDevice.h"
#include "StarshipSimulator/render/GpuLandscape.h"
#include "StarshipSimulator/render/GpuPeople.h"
#include "StarshipSimulator/render/GpuProps.h"
#include "StarshipSimulator/render/GpuSettlements.h"
#include "StarshipSimulator/render/GpuTransit.h"
#include "StarshipSimulator/render/GpuTrees.h"
#include "StarshipSimulator/render/GpuWorld.h"
#include "StarshipSimulator/render/RenderTargets.h"
#include "StarshipSimulator/render/ShaderLibrary.h"
#include "StarshipSimulator/render/ShadowMap.h"
#include "StarshipSimulator/render/passes/MarkerPass.h"
#include "StarshipSimulator/render/passes/habitat_passes.h"
#include "StarshipSimulator/render/passes/sky_passes.h"
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
    Camera                         camera;
    const GpuWorld*                world     = nullptr;  // the habitat; nothing but the sky if null
    GpuLandscape*                  landscape = nullptr;  // its terrain (patches picked per frame)
    const GpuTrees*                trees     = nullptr;  // its trees
    const GpuSettlements*          settlements = nullptr;  // its towns and farms
    GpuProps*                      props = nullptr;  // loose props (instances picked per frame)
    std::span<const PropPlacement> propPoses;        // where each prop is now
    GpuBirds*                      birds = nullptr;  // the flocks over the valleys
    std::span<const Bird>          birdPoses;
    GpuPeople*                     people = nullptr;  // the people about the towns and farms
    std::span<const Person>        peoplePoses;
    GpuTransit*                    transit = nullptr;  // the tramway
    std::span<const Tram>          trams;
    gpu::ShadowUniforms            shadow;  // the trees' shadow map (params.x = 0: none)
    gpu::HabitatUniforms           habitat;
    gpu::SkyUniforms               sky;
    gpu::PlanetUniforms            planets;
    std::span<const BodyDraw>      bodies;   // farthest first
    std::optional<Mat4d>           partner;  // the partner cylinder: its frame -> the habitat frame
    std::span<const Marker>        markers;
    float                          exposure = 1.0F;
    float                          grade    = 1.0F;  // strength of the painterly colour grade
    double                         animationSeconds = 0.0;  // real time, for ripples and the like
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
    // Photo mode: hold a long exposure, and render the saved picture larger than the window.
    bool          trails       = false;  // keep the brightest each pixel has been: star trails
    bool          trailsReset  = false;  // start the exposure again this frame
    std::uint32_t captureScale = 1;      // 2 or 3 renders the screenshot supersampled
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
    /// The habitat's cloud map (see makeCloudMap). Throws std::runtime_error on failure.
    void setCloudMap(const CloudMap& clouds);

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
    /// The inside of the habitat: terrain, buildings, props and trees.
    DrawStats   drawLand(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                         const SceneView& view, const HabitatFrame& habitatFrame);
    static void uploadPerFrameData(SDL_GPUCommandBuffer* commands, const SceneView& view,
                                   const Frustum& frustum);
    /// Adds this frame to the long exposure and returns what the display should show.
    SDL_GPUTexture* accumulate(SDL_GPUCommandBuffer* commands, const FrameOptions& options,
                               std::uint32_t width, std::uint32_t height);
    void            drawShadows(SDL_GPUCommandBuffer* commands, const SceneView& view,
                                const gpu::FrameUniforms& frame);
    void drawDisplay(SDL_GPUCommandBuffer* commands, SDL_GPUTexture* target, const SceneView& view,
                     ImDrawData* ui, SDL_GPUTexture* source);
    [[nodiscard]] std::expected<std::filesystem::path, std::string> captureAndSubmit(
        SDL_GPUCommandBuffer* commands, const SceneView& view, const std::filesystem::path& path,
        ImDrawData* ui, std::uint32_t width, std::uint32_t height, SDL_GPUTexture* source);

    GpuDevice*           device_;
    std::vector<GpuStar> stars_;
    ShaderLibrary        shaders_;
    RenderTargets        targets_;
    GpuTexture           trails_;  // the long exposure, when photo mode is holding one
    std::uint32_t        trailsWidth_  = 0;
    std::uint32_t        trailsHeight_ = 0;
    bool                 trailsClear_  = true;
    SkyTextures          skyTextures_;
    ShadowMap            shadowMap_;
    GpuMesh              hull_;
    GpuTexture           cloudMap_;  // the habitat's clouds, wrapped round it
    GpuSampler           cloudSampler_;
    // Bound for the landscape when there are no settlements (an empty ground-map atlas).
    std::unique_ptr<GpuSettlements> noSettlements_;
    std::unique_ptr<Passes>         passes_;
};

}  // namespace StarshipSimulator
