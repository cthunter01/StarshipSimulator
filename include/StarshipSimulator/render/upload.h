#pragma once

#include <cstddef>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/gpu_handles.h"

namespace StarshipSimulator
{

/// Creates a GPU buffer and fills it with data (via a transfer buffer and its own copy pass,
/// submitted immediately). SDL_GPU orders submissions, so later frames see the data. Throws on
/// failure.
[[nodiscard]] GpuBuffer createBufferWithData(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage,
                                             std::span<const std::byte> data);

}  // namespace StarshipSimulator
