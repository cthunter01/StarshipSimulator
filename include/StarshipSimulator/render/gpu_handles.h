#pragma once

#include <utility>

#include <SDL3/SDL_gpu.h>

namespace StarshipSimulator
{

/// Owns one SDL_GPU object and releases it with the device that created it.
template <typename T, void (*Release)(SDL_GPUDevice*, T*)>
class GpuHandle
{
public:
    GpuHandle() = default;
    GpuHandle(SDL_GPUDevice* device, T* handle) noexcept : device_(device), handle_(handle) { }
    ~GpuHandle() { reset(); }

    GpuHandle(const GpuHandle&)            = delete;
    GpuHandle& operator=(const GpuHandle&) = delete;
    GpuHandle(GpuHandle&& other) noexcept
      : device_(std::exchange(other.device_, nullptr)),
        handle_(std::exchange(other.handle_, nullptr))
    {
    }
    GpuHandle& operator=(GpuHandle&& other) noexcept
    {
        if (this != &other)
        {
            reset();
            device_ = std::exchange(other.device_, nullptr);
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    void reset() noexcept
    {
        if (handle_ != nullptr)
        {
            Release(device_, handle_);
        }
        device_ = nullptr;
        handle_ = nullptr;
    }

    [[nodiscard]] T*   get() const noexcept { return handle_; }
    [[nodiscard]] bool valid() const noexcept { return handle_ != nullptr; }

private:
    SDL_GPUDevice* device_ = nullptr;
    T*             handle_ = nullptr;
};

using GpuBuffer           = GpuHandle<SDL_GPUBuffer, &SDL_ReleaseGPUBuffer>;
using GpuTransferBuffer   = GpuHandle<SDL_GPUTransferBuffer, &SDL_ReleaseGPUTransferBuffer>;
using GpuTexture          = GpuHandle<SDL_GPUTexture, &SDL_ReleaseGPUTexture>;
using GpuSampler          = GpuHandle<SDL_GPUSampler, &SDL_ReleaseGPUSampler>;
using GpuShader           = GpuHandle<SDL_GPUShader, &SDL_ReleaseGPUShader>;
using GpuGraphicsPipeline = GpuHandle<SDL_GPUGraphicsPipeline, &SDL_ReleaseGPUGraphicsPipeline>;
using GpuComputePipeline  = GpuHandle<SDL_GPUComputePipeline, &SDL_ReleaseGPUComputePipeline>;

}  // namespace StarshipSimulator
