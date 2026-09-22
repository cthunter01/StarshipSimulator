#pragma once

#include <cstdint>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

inline constexpr std::uint32_t kShadowMapResolution = 2048;

/// A depth texture for one directional shadow map, and the sampler receivers read it with.
class ShadowMap
{
public:
    static constexpr SDL_GPUTextureFormat kFormat = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;

    ShadowMap(SDL_GPUDevice* device, std::uint32_t resolution);

    [[nodiscard]] SDL_GPUTexture* texture() const { return texture_.get(); }
    [[nodiscard]] SDL_GPUSampler* sampler() const { return sampler_.get(); }
    [[nodiscard]] std::uint32_t   resolution() const { return resolution_; }

private:
    std::uint32_t resolution_;
    GpuTexture    texture_;
    GpuSampler    sampler_;
};

}  // namespace StarshipSimulator
