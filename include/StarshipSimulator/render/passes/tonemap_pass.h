#pragma once

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/shader_library.h"

namespace StarshipSimulator
{

/// Maps the linear HDR scene into a display target (the swapchain or a screenshot texture):
/// exposure, Khronos PBR Neutral tone mapping, and sRGB encoding for UNORM targets.
class TonemapPass
{
public:
    TonemapPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                SDL_GPUTextureFormat targetFormat);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, SDL_GPUTexture* hdrScene,
              float exposure) const;

private:
    GpuGraphicsPipeline pipeline_;
    GpuSampler          sampler_;
    bool                encodeSrgb_ = true;
};

}  // namespace StarshipSimulator
