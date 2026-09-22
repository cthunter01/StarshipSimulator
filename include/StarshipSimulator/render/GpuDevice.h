#pragma once

#include <memory>
#include <string>

#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_video.h>

namespace StarshipSimulator
{

struct GpuDeviceOptions
{
    bool debug = false;  // SDL_GPU debug mode + Vulkan validation layers (if installed)
    bool vsync = true;   // false: mailbox (or immediate) presentation
};

/// What the driver reports about the GPU in use, for the HUD and logs.
struct GpuInfo
{
    std::string backend;     // "vulkan"
    std::string deviceName;  // e.g. "NVIDIA RTX A1000 Laptop GPU"
    std::string driverName;
    std::string driverVersion;
    std::string driverInfo;
    std::string presentMode;
    std::string swapchainFormat;
    bool        debug = false;
};

/// The SDL_GPU device (Vulkan, SPIR-V shaders, discrete GPU preferred) bound to one window.
class GpuDevice
{
public:
    GpuDevice(SDL_Window* window, const GpuDeviceOptions& options);
    ~GpuDevice();

    GpuDevice(const GpuDevice&)            = delete;
    GpuDevice& operator=(const GpuDevice&) = delete;
    GpuDevice(GpuDevice&&)                 = delete;
    GpuDevice& operator=(GpuDevice&&)      = delete;

    [[nodiscard]] SDL_GPUDevice*       get() const { return device_.get(); }
    [[nodiscard]] SDL_Window*          window() const { return window_; }
    [[nodiscard]] SDL_GPUTextureFormat swapchainFormat() const;
    [[nodiscard]] const GpuInfo&       info() const { return info_; }

private:
    struct DeviceDeleter
    {
        void operator()(SDL_GPUDevice* device) const noexcept;
    };

    SDL_Window*                                   window_;
    std::unique_ptr<SDL_GPUDevice, DeviceDeleter> device_;
    GpuInfo                                       info_;
};

[[nodiscard]] const char* textureFormatName(SDL_GPUTextureFormat format);

}  // namespace StarshipSimulator
