#include "StarshipSimulator/render/ShaderLibrary.h"

#include <cstddef>
#include <filesystem>
#include <format>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_iostream.h>
#include <SDL3/SDL_stdinc.h>

#include "StarshipSimulator/core/gpu_abi/metal_shader.h"
#include "StarshipSimulator/core/gpu_abi/spirv_reflect.h"
#include "StarshipSimulator/core/utf8_path.h"
#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

namespace
{

struct SdlFree
{
    void operator()(void* memory) const noexcept { SDL_free(memory); }
};

SDL_GPUShaderStage toSdlStage(gpu::ShaderStage stage, const std::string& name)
{
    switch (stage)
    {
        case gpu::ShaderStage::VERTEX:
            return SDL_GPU_SHADERSTAGE_VERTEX;
        case gpu::ShaderStage::FRAGMENT:
            return SDL_GPU_SHADERSTAGE_FRAGMENT;
        case gpu::ShaderStage::COMPUTE:
            break;
    }
    throw std::runtime_error(
        std::format("{}: compute shaders are loaded as compute pipelines, not shaders", name));
}

/// The shader format to hand SDL_GPU: SPIR-V where the device takes it (Vulkan), otherwise Metal
/// Shading Language (Metal), translated from the SPIR-V.
SDL_GPUShaderFormat shaderFormatFor(SDL_GPUDevice* device)
{
    const SDL_GPUShaderFormat formats = SDL_GetGPUShaderFormats(device);
    if ((formats & SDL_GPU_SHADERFORMAT_SPIRV) != 0)
    {
        return SDL_GPU_SHADERFORMAT_SPIRV;
    }
    if ((formats & SDL_GPU_SHADERFORMAT_MSL) != 0)
    {
        return SDL_GPU_SHADERFORMAT_MSL;
    }
    throw std::runtime_error("The GPU device takes neither SPIR-V nor Metal shaders");
}

}  // namespace

ShaderLibrary::ShaderLibrary(SDL_GPUDevice* device, std::filesystem::path directory)
  : device_(device), directory_(std::move(directory)), format_(shaderFormatFor(device))
{
}

GpuShader ShaderLibrary::load(std::string_view name) const
{
    const std::filesystem::path path     = directory_ / (std::string(name) + ".spv");
    const std::string           pathText = utf8String(path);

    std::size_t                          size = 0;
    const std::unique_ptr<void, SdlFree> data(SDL_LoadFile(pathText.c_str(), &size));
    if (!data)
    {
        throw std::runtime_error(
            std::format("Cannot read shader {}: {}", pathText, SDL_GetError()));
    }
    const std::span<const std::byte> bytes(static_cast<const std::byte*>(data.get()), size);

    auto words = gpu::spirvWordsFromBytes(bytes);
    if (!words)
    {
        throw std::runtime_error(std::format("{}: {}", pathText, words.error()));
    }
    auto reflection = gpu::reflectSpirv(*words);
    if (!reflection)
    {
        throw std::runtime_error(std::format("{}: {}", pathText, reflection.error()));
    }
    if (const auto problems = gpu::validateSdlGpuLayout(*reflection); !problems.empty())
    {
        std::string message = std::format("{} does not follow SDL_GPU's binding layout:", pathText);
        for (const std::string& problem : problems)
        {
            message += "\n  " + problem;
        }
        throw std::runtime_error(message);
    }

    // On Metal the SPIR-V is only the source of the MSL that SDL_GPU compiles.
    std::span<const Uint8> code(static_cast<const Uint8*>(data.get()), size);
    std::string            entryPoint = reflection->entryPoint;
    std::vector<Uint8>     msl;
    if (format_ == SDL_GPU_SHADERFORMAT_MSL)
    {
        const auto metal = gpu::translateToMetal(*words, *reflection);
        if (!metal)
        {
            throw std::runtime_error(std::format("{}: {}", pathText, metal.error()));
        }
        msl.assign(metal->source.begin(), metal->source.end());
        code       = msl;
        entryPoint = metal->entryPoint;
    }

    const gpu::ResourceCounts     counts = reflection->counts();
    const SDL_GPUShaderCreateInfo info{
        .code_size            = code.size(),
        .code                 = code.data(),
        .entrypoint           = entryPoint.c_str(),
        .format               = format_,
        .stage                = toSdlStage(reflection->stage, pathText),
        .num_samplers         = counts.samplers,
        .num_storage_textures = counts.readOnlyStorageTextures,
        .num_storage_buffers  = counts.readOnlyStorageBuffers,
        .num_uniform_buffers  = counts.uniformBuffers,
        .props                = 0,
    };
    SDL_GPUShader* shader = SDL_CreateGPUShader(device_, &info);
    if (shader == nullptr)
    {
        throw std::runtime_error(
            std::format("Cannot create shader {}: {}", pathText, SDL_GetError()));
    }
    return {device_, shader};
}

std::filesystem::path defaultShaderDirectory()
{
    const char* base = SDL_GetBasePath();
    return (base != nullptr ? pathFromUtf8(base) : std::filesystem::current_path()) / "shaders";
}

}  // namespace StarshipSimulator
