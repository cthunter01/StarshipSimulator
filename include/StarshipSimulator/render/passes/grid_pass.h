#pragma once

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"

namespace StarshipSimulator
{

/// M0 test scene backdrop: an infinite metric grid on the plane z = 0 under a sky gradient.
/// Drawn first; it writes depth for every pixel (reverse-Z), so later geometry depth-tests against
/// it.
class GridPass
{
public:
    GridPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
              const gpu::FrameUniforms& frame) const;

private:
    GpuGraphicsPipeline pipeline_;
};

}  // namespace StarshipSimulator
