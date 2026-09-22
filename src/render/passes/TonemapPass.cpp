#include "StarshipSimulator/render/passes/TonemapPass.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>
#include <vector>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_stdinc.h>

#include "StarshipSimulator/core/gpu_abi/color_grade.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/ShaderLibrary.h"
#include "StarshipSimulator/render/texture.h"

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

    // The colour grade as a lookup table: its blue slices side by side in one 2D texture (the
    // shader blends between slices), sampled with linear filtering.
    constexpr std::uint32_t         kLutSize = 32;
    const std::vector<std::uint8_t> volume   = makeGradeLut(GradeSettings{}, kLutSize);
    std::vector<std::uint8_t>       strip(volume.size());
    for (std::uint32_t b = 0; b < kLutSize; ++b)
    {
        for (std::uint32_t g = 0; g < kLutSize; ++g)
        {
            const std::size_t from = ((static_cast<std::size_t>(b) * kLutSize) + g) * kLutSize * 4;
            const std::size_t to   = ((static_cast<std::size_t>(g) * kLutSize * kLutSize) +
                                      (static_cast<std::size_t>(b) * kLutSize)) *
                                     4;
            std::copy_n(volume.begin() + static_cast<std::ptrdiff_t>(from), kLutSize * 4,
                        strip.begin() + static_cast<std::ptrdiff_t>(to));
        }
    }
    gradeLut_ = createTexture(device, {.width   = kLutSize * kLutSize,
                                       .height  = kLutSize,
                                       .format  = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                                       .pixels  = std::as_bytes(std::span(strip)),
                                       .mipmaps = false});
    SDL_GPUSamplerCreateInfo lutInfo = samplerInfo;
    lutInfo.min_filter               = SDL_GPU_FILTER_LINEAR;
    lutInfo.mag_filter               = SDL_GPU_FILTER_LINEAR;
    lutSampler_                      = GpuSampler(device, SDL_CreateGPUSampler(device, &lutInfo));
    if (!pipeline_.valid() || !sampler_.valid() || !lutSampler_.valid())
    {
        throw std::runtime_error(
            std::format("Cannot create the tonemap pipeline: {}", SDL_GetError()));
    }
}

void TonemapPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                       SDL_GPUTexture* hdrScene, float exposure, float grade) const
{
    const gpu::TonemapUniforms uniforms{
        .params = Vec4f(exposure, encodeSrgb_ ? 1.0F : 0.0F, grade, 0.0F)};
    const std::array<SDL_GPUTextureSamplerBinding, 2> bindings{{
        {.texture = hdrScene, .sampler = sampler_.get()},
        {.texture = gradeLut_.get(), .sampler = lutSampler_.get()},
    }};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUFragmentSamplers(pass, 0, bindings.data(), static_cast<Uint32>(bindings.size()));
    SDL_PushGPUFragmentUniformData(commands, 0, &uniforms, sizeof(uniforms));
    SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
}

}  // namespace StarshipSimulator
