#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

/// Creates a GPU buffer and fills it with data (via a transfer buffer and its own copy pass,
/// submitted immediately). SDL_GPU orders submissions, so later frames see the data. Throws on
/// failure.
[[nodiscard]] GpuBuffer createBufferWithData(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage,
                                             std::span<const std::byte> data);

/// Creates a GPU buffer of the given size and lets fill() write its contents straight into the
/// mapped upload memory (no intermediate copy). Throws on failure.
[[nodiscard]] GpuBuffer createBufferFilledBy(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage,
                                             std::uint32_t                                    size,
                                             const std::function<void(std::span<std::byte>)>& fill);

/// A mesh in its own vertex and index buffers.
struct GpuMesh
{
    GpuBuffer     vertices;
    GpuBuffer     indices;
    std::uint32_t indexCount = 0;

    [[nodiscard]] bool valid() const { return indexCount > 0; }
};

/// Uploads a mesh (see createBufferWithData). Throws on failure.
[[nodiscard]] GpuMesh uploadMesh(SDL_GPUDevice* device, const CpuMesh& mesh);

}  // namespace StarshipSimulator
