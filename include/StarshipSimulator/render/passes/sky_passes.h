#pragma once

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/assets/assets.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/RenderTargets.h"
#include "StarshipSimulator/render/ShaderLibrary.h"
#include "StarshipSimulator/render/passes/habitat_passes.h"
#include "StarshipSimulator/render/upload.h"

// What lies outside the habitat, drawn before it: the Milky Way, the planets, Earth and the Moon,
// and the partner cylinder.
namespace StarshipSimulator
{

/// The sky's images on the GPU. Until real ones are set, placeholders stand in: a black Milky Way,
/// a blue-grey Earth and a grey Moon.
class SkyTextures
{
public:
    explicit SkyTextures(SDL_GPUDevice* device);

    void setMilkyWay(const assets::HalfImage& image);
    void setEarth(const assets::Image8& day, const assets::Image8& night);
    void setMoon(const assets::Image8& surface);

    [[nodiscard]] SDL_GPUTexture* milkyWay() const { return milkyWay_.get(); }
    [[nodiscard]] SDL_GPUTexture* earthDay() const { return earthDay_.get(); }
    [[nodiscard]] SDL_GPUTexture* earthNight() const { return earthNight_.get(); }
    [[nodiscard]] SDL_GPUTexture* moon() const { return moon_.get(); }
    [[nodiscard]] SDL_GPUTexture* black() const { return black_.get(); }
    [[nodiscard]] SDL_GPUSampler* sampler() const { return sampler_.get(); }

private:
    SDL_GPUDevice* device_;
    GpuTexture     milkyWay_;
    GpuTexture     earthDay_;
    GpuTexture     earthNight_;
    GpuTexture     moon_;
    GpuTexture     black_;
    GpuSampler     sampler_;
};

/// The Milky Way behind everything: a full-screen pass sampling NASA's all-sky map.
class MilkyWayPass
{
public:
    MilkyWayPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
              const gpu::FrameUniforms& frame, const gpu::SkyUniforms& sky,
              const SkyTextures& textures) const;

private:
    GpuGraphicsPipeline pipeline_;
};

/// The planets as points of light.
class PlanetPass
{
public:
    PlanetPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
              const gpu::FrameUniforms& frame, const gpu::SkyUniforms& sky,
              const gpu::PlanetUniforms& planets) const;

private:
    GpuGraphicsPipeline pipeline_;
};

/// Earth or the Moon: a lit, textured sphere at infinity.
class BodyPass
{
public:
    BodyPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
              const gpu::FrameUniforms& frame, const gpu::BodyUniforms& body,
              SDL_GPUTexture* dayMap, SDL_GPUTexture* nightMap, SDL_GPUSampler* sampler) const;

private:
    GpuGraphicsPipeline pipeline_;
};

/// A habitat seen from outside (the partner cylinder): its hull with the window strips.
class HullPass
{
public:
    HullPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    /// model: from the hull's own frame into the habitat frame (double precision).
    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuMesh& hull,
                   const HabitatFrame& view, const Mat4d& model) const;

private:
    GpuGraphicsPipeline pipeline_;
};

}  // namespace StarshipSimulator
