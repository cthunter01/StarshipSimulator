#include "StarshipSimulator/render/shader_library.h"

#include <cstddef>
#include <filesystem>
#include <format>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_iostream.h>
#include <SDL3/SDL_stdinc.h>

#include "StarshipSimulator/core/gpu_abi/spirv_reflect.h"
#include "StarshipSimulator/render/gpu_handles.h"

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
        case gpu::ShaderStage::Vertex:
            return SDL_GPU_SHADERSTAGE_VERTEX;
        case gpu::ShaderStage::Fragment:
            return SDL_GPU_SHADERSTAGE_FRAGMENT;
        case gpu::ShaderStage::Compute:
            break;
    }
    throw std::runtime_error(
        std::format("{}: compute shaders are loaded as compute pipelines, not shaders", name));
}

}  // namespace

ShaderLibrary::ShaderLibrary(SDL_GPUDevice* device, std::filesystem::path directory)
  : device_(device), directory_(std::move(directory))
{
}

GpuShader ShaderLibrary::load(std::string_view name) const
{
    const std::filesystem::path path     = directory_ / (std::string(name) + ".spv");
    const std::string           pathText = path.string();

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

    const gpu::ResourceCounts     counts = reflection->counts();
    const SDL_GPUShaderCreateInfo info{
        .code_size            = size,
        .code                 = static_cast<const Uint8*>(data.get()),
        .entrypoint           = reflection->entryPoint.c_str(),
        .format               = SDL_GPU_SHADERFORMAT_SPIRV,
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
    return (base != nullptr ? std::filesystem::path(base) : std::filesystem::current_path()) /
           "shaders";
}

}  // namespace StarshipSimulator
