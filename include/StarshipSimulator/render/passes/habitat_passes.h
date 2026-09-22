#pragma once

#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/render/GpuBirds.h"
#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/GpuLandscape.h"
#include "StarshipSimulator/render/GpuSettlements.h"
#include "StarshipSimulator/render/GpuTrees.h"
#include "StarshipSimulator/render/GpuWorld.h"
#include "StarshipSimulator/render/RenderTargets.h"
#include "StarshipSimulator/render/ShaderLibrary.h"

// The passes that draw a habitat, in drawing order: stars, terrain, mirrors, then the window glass.
namespace StarshipSimulator
{

/// What every habitat pass needs to know about the frame.
struct HabitatFrame
{
    const gpu::FrameUniforms*   frame   = nullptr;
    const gpu::HabitatUniforms* habitat = nullptr;
    Mat4d                       viewProjection{1.0};  // camera-relative
    Vec3d                       camera{0.0};
    const Frustum*              frustum = nullptr;
    // The trees' shadow map, read by everything they shade.
    const gpu::ShadowUniforms* shadow        = nullptr;
    SDL_GPUTexture*            shadowMap     = nullptr;
    SDL_GPUSampler*            shadowSampler = nullptr;
    // The cloud map: the deck's shadows fall on everything below it.
    SDL_GPUTexture* cloudMap     = nullptr;
    SDL_GPUSampler* cloudSampler = nullptr;
};

struct DrawStats
{
    std::uint32_t chunks    = 0;
    std::uint64_t triangles = 0;
};

/// The stars, as seen through the windows: additive quads at infinity.
class StarPass
{
public:
    StarPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats,
             std::span<const GpuStar> stars);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
              const gpu::FrameUniforms& frame, const gpu::SkyUniforms& sky) const;

private:
    GpuGraphicsPipeline pipeline_;
    GpuBuffer           stars_;
    std::uint32_t       count_ = 0;
};

/// The land: the level-of-detail terrain over the whole habitat surface, with the towns' streets
/// and gardens from their ground maps.
class LandscapePass
{
public:
    LandscapePass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                   const GpuLandscape& landscape, const GpuSettlements& settlements,
                   const HabitatFrame& view) const;

private:
    GpuGraphicsPipeline pipeline_;
};

/// Trees: instanced meshes in two detail levels that cross-fade, fading out into the terrain's
/// painted canopy in the distance.
class TreePass
{
public:
    TreePass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuTrees& trees,
                   std::span<const TreeDraw> draws, const TreeRanges& ranges,
                   const HabitatFrame& view) const;

    /// Draws the trees' depth into a shadow map; `light` holds the light's view-projection.
    void drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuTrees& trees,
                    std::span<const TreeDraw> draws, const gpu::FrameUniforms& light) const;

private:
    GpuGraphicsPipeline pipeline_;
    GpuGraphicsPipeline shadow_;
};

/// The deck of cloud over the valleys, ray-marched: seen from below, from inside, and hanging
/// under the far side of the habitat overhead.
class CloudPass
{
public:
    CloudPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
              const HabitatFrame& view) const;

private:
    // The deck is drawn on a stand-in cylinder: from below it, on the faces where view rays enter
    // it; from inside it, on the faces where they leave.
    GpuGraphicsPipeline below_;
    GpuGraphicsPipeline inside_;
};

/// Flocks of birds over the valleys (shaders/bird.vert).
class BirdPass
{
public:
    BirdPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuBirds& birds,
                   const HabitatFrame& view) const;

private:
    GpuGraphicsPipeline pipeline_;
};

/// Rain falling past the camera, in layers of streaks (shaders/rain.frag).
class RainPass
{
public:
    RainPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
              const HabitatFrame& view) const;

private:
    GpuGraphicsPipeline pipeline_;
};

/// Rivers and lakes: the water surface over the bed, drawn twice like the glass: multiplying what
/// shows through it, then adding reflections and the light scattered in the water.
class WaterPass
{
public:
    WaterPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                   const GpuLandscape& landscape, const HabitatFrame& view) const;

private:
    GpuGraphicsPipeline transmit_;
    GpuGraphicsPipeline emit_;
};

/// Meshes inside the habitat that are not terrain: flat end walls and the hub caps.
class TerrainPass
{
public:
    TerrainPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuWorld& world,
                   const HabitatFrame& view) const;

private:
    GpuGraphicsPipeline pipeline_;
};

/// The external mirrors, generated in the vertex shader from the habitat uniforms.
class MirrorPass
{
public:
    MirrorPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    /// model: from the mirrors' habitat into our habitat frame (identity for our own mirrors).
    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const HabitatFrame& view,
              const Mat4d& model) const;

private:
    GpuGraphicsPipeline pipeline_;
};

/// The window strips: multiplies what lies beyond by the transmittance, then adds the air's glow.
class GlassPass
{
public:
    GlassPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuWorld& world,
                   const HabitatFrame& view) const;

private:
    GpuGraphicsPipeline transmit_;
    GpuGraphicsPipeline emit_;
};

}  // namespace StarshipSimulator
