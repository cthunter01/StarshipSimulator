#include "StarshipSimulator/render/gpu_device.h"

#include <stdexcept>
#include <string>
#include <string_view>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_properties.h>
#include <SDL3/SDL_video.h>

#include "StarshipSimulator/core/log.h"

namespace StarshipSimulator
{

namespace
{

std::string sdlError(std::string_view what)
{
    return std::string(what) + ": " + SDL_GetError();
}

std::string propertyString(SDL_PropertiesID properties, const char* name)
{
    const char* value = SDL_GetStringProperty(properties, name, nullptr);
    return value != nullptr ? std::string(value) : std::string("unknown");
}

const char* presentModeName(SDL_GPUPresentMode mode)
{
    switch (mode)
    {
        case SDL_GPU_PRESENTMODE_VSYNC:
            return "vsync";
        case SDL_GPU_PRESENTMODE_IMMEDIATE:
            return "immediate";
        case SDL_GPU_PRESENTMODE_MAILBOX:
            return "mailbox";
    }
    return "unknown";
}

/// Owns an SDL property set for the duration of device creation.
class Properties
{
public:
    Properties() : id_(SDL_CreateProperties())
    {
        if (id_ == 0)
        {
            throw std::runtime_error(sdlError("SDL_CreateProperties"));
        }
    }
    ~Properties() { SDL_DestroyProperties(id_); }
    Properties(const Properties&)            = delete;
    Properties& operator=(const Properties&) = delete;
    Properties(Properties&&)                 = delete;
    Properties& operator=(Properties&&)      = delete;

    [[nodiscard]] SDL_PropertiesID id() const { return id_; }

private:
    SDL_PropertiesID id_;
};

}  // namespace

void GpuDevice::DeviceDeleter::operator()(SDL_GPUDevice* device) const noexcept
{
    SDL_DestroyGPUDevice(device);
}

GpuDevice::GpuDevice(SDL_Window* window, const GpuDeviceOptions& options) : window_(window)
{
    {
        const Properties properties;
        SDL_SetBooleanProperty(properties.id(), SDL_PROP_GPU_DEVICE_CREATE_SHADERS_SPIRV_BOOLEAN,
                               true);
        SDL_SetStringProperty(properties.id(), SDL_PROP_GPU_DEVICE_CREATE_NAME_STRING, "vulkan");
        SDL_SetBooleanProperty(properties.id(), SDL_PROP_GPU_DEVICE_CREATE_PREFERLOWPOWER_BOOLEAN,
                               false);  // prefer the discrete GPU
        SDL_SetBooleanProperty(properties.id(), SDL_PROP_GPU_DEVICE_CREATE_DEBUGMODE_BOOLEAN,
                               options.debug);
        SDL_SetBooleanProperty(properties.id(), SDL_PROP_GPU_DEVICE_CREATE_VERBOSE_BOOLEAN, false);
        device_.reset(SDL_CreateGPUDeviceWithProperties(properties.id()));
    }
    if (!device_)
    {
        throw std::runtime_error(sdlError("Could not create a Vulkan GPU device"));
    }
    if (!SDL_ClaimWindowForGPUDevice(device_.get(), window_))
    {
        throw std::runtime_error(sdlError("SDL_ClaimWindowForGPUDevice"));
    }

    SDL_GPUPresentMode presentMode = SDL_GPU_PRESENTMODE_VSYNC;
    if (!options.vsync)
    {
        if (SDL_WindowSupportsGPUPresentMode(device_.get(), window_, SDL_GPU_PRESENTMODE_MAILBOX))
        {
            presentMode = SDL_GPU_PRESENTMODE_MAILBOX;
        }
        else if (SDL_WindowSupportsGPUPresentMode(device_.get(), window_,
                                                  SDL_GPU_PRESENTMODE_IMMEDIATE))
        {
            presentMode = SDL_GPU_PRESENTMODE_IMMEDIATE;
        }
    }
    // SDR: an 8-bit UNORM swapchain; tonemap.frag applies the sRGB encoding itself so ImGui's
    // colours (authored in sRGB) come out unchanged.
    if (!SDL_SetGPUSwapchainParameters(device_.get(), window_, SDL_GPU_SWAPCHAINCOMPOSITION_SDR,
                                       presentMode))
    {
        throw std::runtime_error(sdlError("SDL_SetGPUSwapchainParameters"));
    }

    const SDL_PropertiesID properties = SDL_GetGPUDeviceProperties(device_.get());
    const char*            backend    = SDL_GetGPUDeviceDriver(device_.get());
    info_.backend                     = backend != nullptr ? backend : "unknown";
    info_.deviceName                  = propertyString(properties, SDL_PROP_GPU_DEVICE_NAME_STRING);
    info_.driverName      = propertyString(properties, SDL_PROP_GPU_DEVICE_DRIVER_NAME_STRING);
    info_.driverVersion   = propertyString(properties, SDL_PROP_GPU_DEVICE_DRIVER_VERSION_STRING);
    info_.driverInfo      = propertyString(properties, SDL_PROP_GPU_DEVICE_DRIVER_INFO_STRING);
    info_.presentMode     = presentModeName(presentMode);
    info_.swapchainFormat = textureFormatName(swapchainFormat());
    info_.debug           = options.debug;

    log::info("GPU: {} ({}, driver {} {}), present mode {}, swapchain {}{}", info_.deviceName,
              info_.backend, info_.driverName, info_.driverVersion, info_.presentMode,
              info_.swapchainFormat, info_.debug ? ", debug mode" : "");
    if (info_.deviceName.contains("Intel") || info_.deviceName.contains("llvmpipe"))
    {
        log::warn(
            "Running on '{}', not a discrete GPU. To force NVIDIA: "
            "VK_LOADER_DRIVERS_SELECT='nvidia*' or prime-run",
            info_.deviceName);
    }
}

GpuDevice::~GpuDevice()
{
    if (device_)
    {
        SDL_WaitForGPUIdle(device_.get());
        SDL_ReleaseWindowFromGPUDevice(device_.get(), window_);
    }
}

SDL_GPUTextureFormat GpuDevice::swapchainFormat() const
{
    return SDL_GetGPUSwapchainTextureFormat(device_.get(), window_);
}

const char* textureFormatName(SDL_GPUTextureFormat format)
{
    switch (format)
    {
        case SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM:
            return "B8G8R8A8_UNORM";
        case SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM:
            return "R8G8B8A8_UNORM";
        case SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM_SRGB:
            return "B8G8R8A8_UNORM_SRGB";
        case SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM_SRGB:
            return "R8G8B8A8_UNORM_SRGB";
        case SDL_GPU_TEXTUREFORMAT_R10G10B10A2_UNORM:
            return "R10G10B10A2_UNORM";
        case SDL_GPU_TEXTUREFORMAT_R16G16B16A16_FLOAT:
            return "R16G16B16A16_FLOAT";
        case SDL_GPU_TEXTUREFORMAT_D32_FLOAT:
            return "D32_FLOAT";
        default:
            return "other";
    }
}

}  // namespace StarshipSimulator
