#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

/// Pixels to upload into a 2D texture: tightly packed rows, top row first.
struct TextureData
{
    std::uint32_t              width  = 0;
    std::uint32_t              height = 0;
    SDL_GPUTextureFormat       format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
    std::span<const std::byte> pixels;
    bool                       mipmaps = true;  // generate the full mip chain on the GPU
};

/// Creates a sampled 2D texture and uploads the pixels (via its own command buffer, submitted
/// immediately; SDL_GPU orders submissions, so later frames see the data). Throws on failure.
[[nodiscard]] GpuTexture createTexture(SDL_GPUDevice* device, const TextureData& data);

/// A 1x1 texture of one colour, for data that is not loaded (yet).
[[nodiscard]] GpuTexture createSolidTexture(SDL_GPUDevice* device, SDL_GPUTextureFormat format,
                                            std::span<const std::byte> pixel);

/// Linear filtering with mipmaps and anisotropy; U repeats (longitude), V clamps (latitude).
[[nodiscard]] GpuSampler createMapSampler(SDL_GPUDevice* device);

}  // namespace StarshipSimulator
