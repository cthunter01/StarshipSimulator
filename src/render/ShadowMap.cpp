#include "StarshipSimulator/render/ShadowMap.h"

#include <cstdint>
#include <format>
#include <stdexcept>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

ShadowMap::ShadowMap(SDL_GPUDevice* device, std::uint32_t resolution) : resolution_(resolution)
{
    const SDL_GPUTextureCreateInfo textureInfo{
        .type   = SDL_GPU_TEXTURETYPE_2D,
        .format = kFormat,
        .usage  = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER,
        .width  = resolution,
        .height = resolution,
        .layer_count_or_depth = 1,
        .num_levels           = 1,
        .sample_count         = SDL_GPU_SAMPLECOUNT_1,
    };
    const SDL_GPUSamplerCreateInfo samplerInfo{
        .min_filter     = SDL_GPU_FILTER_NEAREST,
        .mag_filter     = SDL_GPU_FILTER_NEAREST,
        .mipmap_mode    = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST,
        .address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
    };
    texture_ = GpuTexture(device, SDL_CreateGPUTexture(device, &textureInfo));
    sampler_ = GpuSampler(device, SDL_CreateGPUSampler(device, &samplerInfo));
    if (!texture_.valid() || !sampler_.valid())
    {
        throw std::runtime_error(std::format("Cannot create a shadow map: {}", SDL_GetError()));
    }
}

}  // namespace StarshipSimulator
