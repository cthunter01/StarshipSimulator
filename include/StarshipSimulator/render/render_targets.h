#pragma once

#include <cstdint>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/gpu_handles.h"

namespace StarshipSimulator
{

/// Formats and sample count of the scene render pass, needed to create compatible pipelines.
struct SceneFormats
{
    SDL_GPUTextureFormat color   = SDL_GPU_TEXTUREFORMAT_R16G16B16A16_FLOAT;
    SDL_GPUTextureFormat depth   = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    SDL_GPUSampleCount   samples = SDL_GPU_SAMPLECOUNT_1;
};

/// Picks the largest supported MSAA sample count up to the requested one for both scene formats.
[[nodiscard]] SceneFormats chooseSceneFormats(SDL_GPUDevice* device, SDL_GPUSampleCount wanted);

/// The HDR scene targets: linear RGBA16F colour (MSAA, resolved to a sampled texture) and
/// 32-bit float depth for reverse-Z. Recreated when the swapchain size changes.
class RenderTargets
{
public:
    RenderTargets(SDL_GPUDevice* device, const SceneFormats& formats);

    /// Makes sure the targets match the swapchain size. Throws if textures cannot be created.
    void resize(std::uint32_t width, std::uint32_t height);

    [[nodiscard]] SDL_GPUTexture* color() const { return color_.get(); }
    /// The single-sample colour to read after the scene pass (the resolve target, or color()).
    [[nodiscard]] SDL_GPUTexture*     resolved() const;
    [[nodiscard]] SDL_GPUTexture*     depth() const { return depth_.get(); }
    [[nodiscard]] bool                multisampled() const;
    [[nodiscard]] const SceneFormats& formats() const { return formats_; }
    [[nodiscard]] std::uint32_t       width() const { return width_; }
    [[nodiscard]] std::uint32_t       height() const { return height_; }

private:
    SDL_GPUDevice* device_;
    SceneFormats   formats_;
    GpuTexture     color_;
    GpuTexture     resolve_;
    GpuTexture     depth_;
    std::uint32_t  width_  = 0;
    std::uint32_t  height_ = 0;
};

}  // namespace StarshipSimulator
