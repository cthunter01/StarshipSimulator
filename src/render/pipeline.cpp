#include "StarshipSimulator/render/pipeline.h"

#include <array>
#include <cstddef>
#include <format>
#include <stdexcept>
#include <string_view>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_stdinc.h>

#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/render_targets.h"

namespace StarshipSimulator
{

namespace
{

SDL_GPUColorTargetBlendState blendState(BlendMode mode)
{
    const auto make = [](SDL_GPUBlendFactor source, SDL_GPUBlendFactor destination) {
        return SDL_GPUColorTargetBlendState{
            .src_color_blendfactor = source,
            .dst_color_blendfactor = destination,
            .color_blend_op        = SDL_GPU_BLENDOP_ADD,
            .src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO,
            .dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE,
            .alpha_blend_op        = SDL_GPU_BLENDOP_ADD,
            .enable_blend          = true,
        };
    };
    switch (mode)
    {
        case BlendMode::Opaque:
            return {};
        case BlendMode::Additive:
            return make(SDL_GPU_BLENDFACTOR_ONE, SDL_GPU_BLENDFACTOR_ONE);
        case BlendMode::Multiply:
            return make(SDL_GPU_BLENDFACTOR_ZERO, SDL_GPU_BLENDFACTOR_SRC_COLOR);
        case BlendMode::Alpha:
            return make(SDL_GPU_BLENDFACTOR_ONE, SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA);
    }
    return {};
}

SDL_GPUDepthStencilState depthState(DepthMode mode)
{
    switch (mode)
    {
        case DepthMode::None:
            return {};
        case DepthMode::TestWrite:
            return {.compare_op         = SDL_GPU_COMPAREOP_GREATER,
                    .enable_depth_test  = true,
                    .enable_depth_write = true};
        case DepthMode::TestOnly:
            return {.compare_op = SDL_GPU_COMPAREOP_GREATER, .enable_depth_test = true};
        case DepthMode::AlwaysWrite:
            return {.compare_op         = SDL_GPU_COMPAREOP_ALWAYS,
                    .enable_depth_test  = true,
                    .enable_depth_write = true};
        case DepthMode::ShadowWrite:
            return {.compare_op         = SDL_GPU_COMPAREOP_LESS,
                    .enable_depth_test  = true,
                    .enable_depth_write = true};
    }
    return {};
}

}  // namespace

PipelineDescription scenePipeline(const SceneFormats& formats)
{
    PipelineDescription description;
    description.colorFormat = formats.color;
    description.depthFormat = formats.depth;
    description.samples     = formats.samples;
    return description;
}

GpuGraphicsPipeline createPipeline(SDL_GPUDevice* device, const PipelineDescription& description,
                                   std::string_view name)
{
    const SDL_GPUVertexBufferDescription vertexBuffer{
        .slot               = 0,
        .pitch              = sizeof(Vertex),
        .input_rate         = SDL_GPU_VERTEXINPUTRATE_VERTEX,
        .instance_step_rate = 0,
    };
    const std::array<SDL_GPUVertexAttribute, 4> attributes{{
        {.location    = 0,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
         .offset      = offsetof(Vertex, position)},
        {.location    = 1,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
         .offset      = offsetof(Vertex, normal)},
        {.location    = 2,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2,
         .offset      = offsetof(Vertex, uv)},
        {.location    = 3,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_UINT,
         .offset      = offsetof(Vertex, material)},
    }};
    SDL_GPUVertexInputState                     vertexInput{};
    if (!description.vertexBuffers.empty())
    {
        vertexInput = {
            .vertex_buffer_descriptions = description.vertexBuffers.data(),
            .num_vertex_buffers         = static_cast<Uint32>(description.vertexBuffers.size()),
            .vertex_attributes          = description.vertexAttributes.data(),
            .num_vertex_attributes      = static_cast<Uint32>(description.vertexAttributes.size())};
    }
    else if (description.meshVertices)
    {
        vertexInput = {.vertex_buffer_descriptions = &vertexBuffer,
                       .num_vertex_buffers         = 1,
                       .vertex_attributes          = attributes.data(),
                       .num_vertex_attributes      = static_cast<Uint32>(attributes.size())};
    }

    const bool hasDepth = description.depthFormat != SDL_GPU_TEXTUREFORMAT_INVALID;
    const bool hasColor = description.colorFormat != SDL_GPU_TEXTUREFORMAT_INVALID;
    const bool hasBias =
        description.depthBiasConstant != 0.0F || description.depthBiasSlope != 0.0F;
    const SDL_GPUColorTargetDescription colorTarget{.format      = description.colorFormat,
                                                    .blend_state = blendState(description.blend)};
    const SDL_GPUGraphicsPipelineCreateInfo info{
        .vertex_shader      = description.vertexShader,
        .fragment_shader    = description.fragmentShader,
        .vertex_input_state = vertexInput,
        .primitive_type     = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
        .rasterizer_state   = {.fill_mode                  = SDL_GPU_FILLMODE_FILL,
                               .cull_mode                  = description.cull,
                               .front_face                 = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE,
                               .depth_bias_constant_factor = description.depthBiasConstant,
                               .depth_bias_slope_factor    = description.depthBiasSlope,
                               .enable_depth_bias          = hasBias,
                               .enable_depth_clip          = true},
        .multisample_state  = {.sample_count = description.samples},
        .depth_stencil_state =
            hasDepth ? depthState(description.depth) : SDL_GPUDepthStencilState{},
        .target_info = {.color_target_descriptions = hasColor ? &colorTarget : nullptr,
                        .num_color_targets         = hasColor ? 1U : 0U,
                        .depth_stencil_format      = description.depthFormat,
                        .has_depth_stencil_target  = hasDepth},
    };
    GpuGraphicsPipeline pipeline(device, SDL_CreateGPUGraphicsPipeline(device, &info));
    if (!pipeline.valid())
    {
        throw std::runtime_error(
            std::format("Cannot create the {} pipeline: {}", name, SDL_GetError()));
    }
    return pipeline;
}

}  // namespace StarshipSimulator
