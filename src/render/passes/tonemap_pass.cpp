#include "StarshipSimulator/render/passes/tonemap_pass.h"

#include <format>
#include <stdexcept>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/shader_library.h"

namespace StarshipSimulator
{

namespace
{

bool isSrgbFormat(SDL_GPUTextureFormat format)
{
    return format == SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM_SRGB ||
           format == SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM_SRGB;
}

}  // namespace

TonemapPass::TonemapPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                         SDL_GPUTextureFormat targetFormat)
  : encodeSrgb_(!isSrgbFormat(targetFormat))
{
    const GpuShader vertex   = shaders.load("fullscreen.vert");
    const GpuShader fragment = shaders.load("tonemap.frag");

    const SDL_GPUColorTargetDescription     colorTarget{.format = targetFormat};
    const SDL_GPUGraphicsPipelineCreateInfo info{
        .vertex_shader    = vertex.get(),
        .fragment_shader  = fragment.get(),
        .primitive_type   = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
        .rasterizer_state = {.fill_mode         = SDL_GPU_FILLMODE_FILL,
                             .cull_mode         = SDL_GPU_CULLMODE_NONE,
                             .front_face        = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE,
                             .enable_depth_clip = true},
        .target_info      = {.color_target_descriptions = &colorTarget, .num_color_targets = 1},
    };
    pipeline_ = GpuGraphicsPipeline(device, SDL_CreateGPUGraphicsPipeline(device, &info));

    const SDL_GPUSamplerCreateInfo samplerInfo{
        .min_filter     = SDL_GPU_FILTER_NEAREST,
        .mag_filter     = SDL_GPU_FILTER_NEAREST,
        .mipmap_mode    = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST,
        .address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
    };
    sampler_ = GpuSampler(device, SDL_CreateGPUSampler(device, &samplerInfo));
    if (!pipeline_.valid() || !sampler_.valid())
    {
        throw std::runtime_error(
            std::format("Cannot create the tonemap pipeline: {}", SDL_GetError()));
    }
}

void TonemapPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                       SDL_GPUTexture* hdrScene, float exposure) const
{
    const gpu::TonemapUniforms uniforms{.params =
                                            Vec4f(exposure, encodeSrgb_ ? 1.0F : 0.0F, 0.0F, 0.0F)};
    const SDL_GPUTextureSamplerBinding binding{.texture = hdrScene, .sampler = sampler_.get()};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUFragmentSamplers(pass, 0, &binding, 1);
    SDL_PushGPUFragmentUniformData(commands, 0, &uniforms, sizeof(uniforms));
    SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
}

}  // namespace StarshipSimulator
