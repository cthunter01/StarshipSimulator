#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

/// A GPU buffer rewritten every frame (instance lists and the like). Uploads cycle both the
/// transfer buffer and the GPU buffer, so frames still in flight keep their own data.
class DynamicBuffer
{
public:
    DynamicBuffer() = default;
    DynamicBuffer(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage, std::uint32_t capacity);

    /// Copies up to capacity() bytes into the buffer, in a copy pass before the render pass that
    /// reads it. Returns the number of bytes uploaded.
    std::uint32_t upload(SDL_GPUCopyPass* copy, std::span<const std::byte> data);

    [[nodiscard]] SDL_GPUBuffer* get() const { return buffer_.get(); }
    [[nodiscard]] std::uint32_t  capacity() const { return capacity_; }

private:
    SDL_GPUDevice*    device_   = nullptr;
    std::uint32_t     capacity_ = 0;
    GpuBuffer         buffer_;
    GpuTransferBuffer transfer_;
};

}  // namespace StarshipSimulator
