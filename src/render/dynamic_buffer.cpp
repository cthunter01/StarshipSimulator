#include "StarshipSimulator/render/dynamic_buffer.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <span>
#include <stdexcept>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/gpu_handles.h"

namespace StarshipSimulator
{

DynamicBuffer::DynamicBuffer(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage,
                             std::uint32_t capacity)
  : device_(device), capacity_(capacity)
{
    const SDL_GPUBufferCreateInfo         bufferInfo{.usage = usage, .size = capacity, .props = 0};
    const SDL_GPUTransferBufferCreateInfo transferInfo{
        .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, .size = capacity, .props = 0};
    buffer_   = GpuBuffer(device, SDL_CreateGPUBuffer(device, &bufferInfo));
    transfer_ = GpuTransferBuffer(device, SDL_CreateGPUTransferBuffer(device, &transferInfo));
    if (!buffer_.valid() || !transfer_.valid())
    {
        throw std::runtime_error(std::format("Cannot create a dynamic buffer of {} bytes: {}",
                                             capacity, SDL_GetError()));
    }
}

std::uint32_t DynamicBuffer::upload(SDL_GPUCopyPass* copy, std::span<const std::byte> data)
{
    const auto size = static_cast<std::uint32_t>(std::min<std::size_t>(data.size(), capacity_));
    if (size == 0)
    {
        return 0;
    }
    void* mapped = SDL_MapGPUTransferBuffer(device_, transfer_.get(), true);
    if (mapped == nullptr)
    {
        throw std::runtime_error(std::format("Cannot map a transfer buffer: {}", SDL_GetError()));
    }
    std::memcpy(mapped, data.data(), size);
    SDL_UnmapGPUTransferBuffer(device_, transfer_.get());
    const SDL_GPUTransferBufferLocation source{.transfer_buffer = transfer_.get(), .offset = 0};
    const SDL_GPUBufferRegion destination{.buffer = buffer_.get(), .offset = 0, .size = size};
    SDL_UploadToGPUBuffer(copy, &source, &destination, true);
    return size;
}

}  // namespace StarshipSimulator
