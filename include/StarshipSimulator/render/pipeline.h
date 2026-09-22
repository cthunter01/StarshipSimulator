#pragma once

#include <cstdint>
#include <span>
#include <string_view>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/RenderTargets.h"

namespace StarshipSimulator
{

enum class BlendMode : std::uint8_t
{
    Opaque,
    Additive,  // dst + src
    Multiply,  // dst * src
    Alpha,     // premultiplied: src + dst * (1 - src.a)
    Lighten,   // max(src, dst): what a long exposure does to a moving point of light
};

enum class DepthMode : std::uint8_t
{
    None,       // no depth test or write
    TestWrite,  // reverse-Z: GREATER passes
    TestOnly,
    AlwaysWrite,
    ShadowWrite,  // shadow maps: ordinary depth, LESS passes
};

/// What differs between our graphics pipelines; everything else is shared.
struct PipelineDescription
{
    SDL_GPUShader* vertexShader   = nullptr;
    SDL_GPUShader* fragmentShader = nullptr;
    bool           meshVertices   = false;  // StarshipSimulator::Vertex input, else none
    // Any other vertex input (used when not empty, instead of meshVertices).
    std::span<const SDL_GPUVertexBufferDescription> vertexBuffers;
    std::span<const SDL_GPUVertexAttribute>         vertexAttributes;
    SDL_GPUCullMode                                 cull        = SDL_GPU_CULLMODE_NONE;
    DepthMode                                       depth       = DepthMode::None;
    BlendMode                                       blend       = BlendMode::Opaque;
    SDL_GPUTextureFormat                            colorFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;  // INVALID: no depth target
    SDL_GPUSampleCount   samples     = SDL_GPU_SAMPLECOUNT_1;
    float                depthBiasConstant = 0.0F;  // shadow maps: push depths away a little
    float                depthBiasSlope    = 0.0F;
};

/// A description for drawing into the HDR scene targets.
[[nodiscard]] PipelineDescription scenePipeline(const SceneFormats& formats);

/// Creates a graphics pipeline; throws std::runtime_error naming it on failure.
[[nodiscard]] GpuGraphicsPipeline createPipeline(SDL_GPUDevice*             device,
                                                 const PipelineDescription& description,
                                                 std::string_view           name);

}  // namespace StarshipSimulator
