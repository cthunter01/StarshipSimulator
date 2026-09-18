#include "StarshipSimulator/render/passes/marker_pass.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_stdinc.h>

#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

MarkerPass::MarkerPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                       const SceneFormats& formats)
{
    const GpuShader vertex   = shaders.load("mesh.vert");
    const GpuShader fragment = shaders.load("marker.frag");

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
    const SDL_GPUColorTargetDescription         colorTarget{.format = formats.color};
    const SDL_GPUGraphicsPipelineCreateInfo     info{
        .vertex_shader       = vertex.get(),
        .fragment_shader     = fragment.get(),
        .vertex_input_state  = {.vertex_buffer_descriptions = &vertexBuffer,
                                .num_vertex_buffers         = 1,
                                .vertex_attributes          = attributes.data(),
                                .num_vertex_attributes = static_cast<Uint32>(attributes.size())},
        .primitive_type      = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
        .rasterizer_state    = {.fill_mode         = SDL_GPU_FILLMODE_FILL,
                                .cull_mode         = SDL_GPU_CULLMODE_BACK,
                                .front_face        = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE,
                                .enable_depth_clip = true},
        .multisample_state   = {.sample_count = formats.samples},
        .depth_stencil_state = {.compare_op         = SDL_GPU_COMPAREOP_GREATER,  // reverse-Z
                                .enable_depth_test  = true,
                                .enable_depth_write = true},
        .target_info         = {.color_target_descriptions = &colorTarget,
                                .num_color_targets         = 1,
                                .depth_stencil_format      = formats.depth,
                                .has_depth_stencil_target  = true},
    };
    pipeline_ = GpuGraphicsPipeline(device, SDL_CreateGPUGraphicsPipeline(device, &info));
    if (!pipeline_.valid())
    {
        throw std::runtime_error(
            std::format("Cannot create the marker pipeline: {}", SDL_GetError()));
    }

    const CpuMesh box = makeBox(Vec3f(1.0F));
    vertices_         = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                             std::as_bytes(std::span(box.vertices)));
    indices_          = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_INDEX,
                                             std::as_bytes(std::span(box.indices)));
    indexCount_       = static_cast<std::uint32_t>(box.indices.size());
}

void MarkerPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                      const Mat4d& viewProjection, const Vec3d& cameraPosition,
                      std::span<const Marker> markers) const
{
    if (markers.empty())
    {
        return;
    }
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    const SDL_GPUBufferBinding vertexBinding{.buffer = vertices_.get(), .offset = 0};
    const SDL_GPUBufferBinding indexBinding{.buffer = indices_.get(), .offset = 0};
    SDL_BindGPUVertexBuffers(pass, 0, &vertexBinding, 1);
    SDL_BindGPUIndexBuffer(pass, &indexBinding, SDL_GPU_INDEXELEMENTSIZE_32BIT);

    for (const Marker& marker : markers)
    {
        // Camera-relative translation in double precision, then rounded to float once.
        const Mat4d scale = glm::scale(Mat4d(1.0), Vec3d(marker.halfExtents));
        const Mat4d model = glm::translate(Mat4d(1.0), marker.position - cameraPosition) * scale;
        const gpu::DrawUniforms draw{
            .modelViewProjection = Mat4f(viewProjection * model),
            .model               = Mat4f(scale),
        };
        const gpu::MaterialUniforms material{
            .color    = Vec4f(marker.color, 1.0F),
            .emission = Vec4f(marker.emission, 0.0F),
        };
        SDL_PushGPUVertexUniformData(commands, 0, &draw, sizeof(draw));
        SDL_PushGPUFragmentUniformData(commands, 0, &material, sizeof(material));
        SDL_DrawGPUIndexedPrimitives(pass, indexCount_, 1, 0, 0, 0);
    }
}

}  // namespace StarshipSimulator
