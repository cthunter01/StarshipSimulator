#pragma once

#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"

namespace StarshipSimulator
{

/// An axis-aligned box placed in the world, used for test and reference markers.
struct Marker
{
    Vec3d position{0.0};      // centre, world space (double precision)
    Vec3f halfExtents{0.5F};  // metres
    Vec3f color{0.8F};        // linear RGB
    Vec3f emission{0.0F};     // linear RGB, glows regardless of lighting
};

/// Draws markers as lit boxes. Each box's camera-relative transform is computed in double
/// precision.
class MarkerPass
{
public:
    MarkerPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const Mat4d& viewProjection,
              const Vec3d& cameraPosition, std::span<const Marker> markers) const;

private:
    GpuGraphicsPipeline pipeline_;
    GpuBuffer           vertices_;
    GpuBuffer           indices_;
    std::uint32_t       indexCount_ = 0;
};

}  // namespace StarshipSimulator
