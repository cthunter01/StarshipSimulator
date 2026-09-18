#include "StarshipSimulator/render/render_targets.h"

#include <array>
#include <cstdint>
#include <format>
#include <stdexcept>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/gpu_handles.h"

namespace StarshipSimulator
{

namespace
{

GpuTexture createTarget(SDL_GPUDevice* device, SDL_GPUTextureFormat format,
                        SDL_GPUTextureUsageFlags usage, SDL_GPUSampleCount samples,
                        std::uint32_t width, std::uint32_t height)
{
    const SDL_GPUTextureCreateInfo info{
        .type                 = SDL_GPU_TEXTURETYPE_2D,
        .format               = format,
        .usage                = usage,
        .width                = width,
        .height               = height,
        .layer_count_or_depth = 1,
        .num_levels           = 1,
        .sample_count         = samples,
        .props                = 0,
    };
    SDL_GPUTexture* texture = SDL_CreateGPUTexture(device, &info);
    if (texture == nullptr)
    {
        throw std::runtime_error(
            std::format("Cannot create a {}x{} render target: {}", width, height, SDL_GetError()));
    }
    return {device, texture};
}

}  // namespace

SceneFormats chooseSceneFormats(SDL_GPUDevice* device, SDL_GPUSampleCount wanted)
{
    SceneFormats                            formats;
    const std::array<SDL_GPUSampleCount, 4> candidates{
        SDL_GPU_SAMPLECOUNT_8, SDL_GPU_SAMPLECOUNT_4, SDL_GPU_SAMPLECOUNT_2, SDL_GPU_SAMPLECOUNT_1};
    for (const SDL_GPUSampleCount samples : candidates)
    {
        if (samples <= wanted &&
            SDL_GPUTextureSupportsSampleCount(device, formats.color, samples) &&
            SDL_GPUTextureSupportsSampleCount(device, formats.depth, samples))
        {
            formats.samples = samples;
            break;
        }
    }
    return formats;
}

RenderTargets::RenderTargets(SDL_GPUDevice* device, const SceneFormats& formats)
  : device_(device), formats_(formats)
{
}

bool RenderTargets::multisampled() const
{
    return formats_.samples != SDL_GPU_SAMPLECOUNT_1;
}

SDL_GPUTexture* RenderTargets::resolved() const
{
    return multisampled() ? resolve_.get() : color_.get();
}

void RenderTargets::resize(std::uint32_t width, std::uint32_t height)
{
    if (width == width_ && height == height_ && color_.valid())
    {
        return;
    }
    // Release first so peak memory stays at one set of targets.
    color_.reset();
    resolve_.reset();
    depth_.reset();

    if (multisampled())
    {
        color_   = createTarget(device_, formats_.color, SDL_GPU_TEXTUREUSAGE_COLOR_TARGET,
                                formats_.samples, width, height);
        resolve_ = createTarget(device_, formats_.color,
                                SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER,
                                SDL_GPU_SAMPLECOUNT_1, width, height);
    }
    else
    {
        color_ = createTarget(device_, formats_.color,
                              SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER,
                              SDL_GPU_SAMPLECOUNT_1, width, height);
    }
    depth_  = createTarget(device_, formats_.depth, SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET,
                           formats_.samples, width, height);
    width_  = width;
    height_ = height;
}

}  // namespace StarshipSimulator
