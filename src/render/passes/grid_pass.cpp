#include "StarshipSimulator/render/passes/grid_pass.h"

#include <format>
#include <stdexcept>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"

namespace StarshipSimulator
{

GridPass::GridPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats)
{
    const GpuShader vertex   = shaders.load("fullscreen.vert");
    const GpuShader fragment = shaders.load("grid.frag");

    const SDL_GPUColorTargetDescription     colorTarget{.format = formats.color};
    const SDL_GPUGraphicsPipelineCreateInfo info{
        .vertex_shader       = vertex.get(),
        .fragment_shader     = fragment.get(),
        .primitive_type      = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
        .rasterizer_state    = {.fill_mode         = SDL_GPU_FILLMODE_FILL,
                                .cull_mode         = SDL_GPU_CULLMODE_NONE,
                                .front_face        = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE,
                                .enable_depth_clip = true},
        .multisample_state   = {.sample_count = formats.samples},
        .depth_stencil_state = {.compare_op         = SDL_GPU_COMPAREOP_ALWAYS,
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
            std::format("Cannot create the grid pipeline: {}", SDL_GetError()));
    }
}

void GridPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                    const gpu::FrameUniforms& frame) const
{
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_PushGPUFragmentUniformData(commands, 0, &frame, sizeof(frame));
    SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
}

}  // namespace StarshipSimulator
