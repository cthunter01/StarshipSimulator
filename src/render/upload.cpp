#include "StarshipSimulator/render/upload.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <limits>
#include <span>
#include <stdexcept>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/render/gpu_handles.h"

namespace StarshipSimulator
{

GpuBuffer createBufferFilledBy(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage,
                               std::uint32_t                                    size,
                               const std::function<void(std::span<std::byte>)>& fill)
{
    if (size == 0)
    {
        throw std::runtime_error("Cannot create an empty GPU buffer");
    }
    const SDL_GPUBufferCreateInfo         bufferInfo{.usage = usage, .size = size, .props = 0};
    GpuBuffer                             buffer(device, SDL_CreateGPUBuffer(device, &bufferInfo));
    const SDL_GPUTransferBufferCreateInfo transferInfo{
        .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, .size = size, .props = 0};
    const GpuTransferBuffer transfer(device, SDL_CreateGPUTransferBuffer(device, &transferInfo));
    if (!buffer.valid() || !transfer.valid())
    {
        throw std::runtime_error(
            std::format("Cannot create GPU buffers of {} bytes: {}", size, SDL_GetError()));
    }

    void* mapped = SDL_MapGPUTransferBuffer(device, transfer.get(), false);
    if (mapped == nullptr)
    {
        throw std::runtime_error(std::format("Cannot map a transfer buffer: {}", SDL_GetError()));
    }
    fill(std::span<std::byte>(static_cast<std::byte*>(mapped), size));
    SDL_UnmapGPUTransferBuffer(device, transfer.get());

    SDL_GPUCommandBuffer* commands = SDL_AcquireGPUCommandBuffer(device);
    if (commands == nullptr)
    {
        throw std::runtime_error(
            std::format("Cannot acquire a command buffer: {}", SDL_GetError()));
    }
    SDL_GPUCopyPass*                    copy = SDL_BeginGPUCopyPass(commands);
    const SDL_GPUTransferBufferLocation source{.transfer_buffer = transfer.get(), .offset = 0};
    const SDL_GPUBufferRegion destination{.buffer = buffer.get(), .offset = 0, .size = size};
    SDL_UploadToGPUBuffer(copy, &source, &destination, false);
    SDL_EndGPUCopyPass(copy);
    if (!SDL_SubmitGPUCommandBuffer(commands))
    {
        throw std::runtime_error(std::format("Cannot submit an upload: {}", SDL_GetError()));
    }
    // Releasing the transfer buffer now is fine: SDL keeps it alive until the copy has run.
    return buffer;
}

GpuBuffer createBufferWithData(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage,
                               std::span<const std::byte> data)
{
    if (data.empty() || data.size() > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error(std::format("Cannot upload a buffer of {} bytes", data.size()));
    }
    return createBufferFilledBy(
        device, usage, static_cast<std::uint32_t>(data.size()),
        [&](std::span<std::byte> target) { std::memcpy(target.data(), data.data(), data.size()); });
}

GpuMesh uploadMesh(SDL_GPUDevice* device, const CpuMesh& mesh)
{
    return {.vertices   = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                               std::as_bytes(std::span(mesh.vertices))),
            .indices    = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_INDEX,
                                               std::as_bytes(std::span(mesh.indices))),
            .indexCount = static_cast<std::uint32_t>(mesh.indices.size())};
}

}  // namespace StarshipSimulator
