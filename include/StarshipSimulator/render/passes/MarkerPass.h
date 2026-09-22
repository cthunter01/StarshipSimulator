#pragma once

#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/RenderTargets.h"
#include "StarshipSimulator/render/ShaderLibrary.h"

namespace StarshipSimulator
{

enum class MarkerShape : std::uint8_t
{
    BOX,
    SPHERE,
};

/// A simple shape placed in the world: thrown balls, trajectory dots, reference markers.
struct Marker
{
    Vec3d       position{0.0};      // centre, habitat frame (double precision)
    Vec3f       halfExtents{0.5F};  // metres (radius for spheres)
    Vec3f       color{0.8F};        // linear RGB
    Vec3f       emission{0.0F};     // linear RGB, glows regardless of lighting
    MarkerShape shape = MarkerShape::BOX;
};

/// Draws markers as lit shapes. Each one's camera-relative transform is computed in double
/// precision.
class MarkerPass
{
public:
    MarkerPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    void draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const Mat4d& viewProjection,
              const Vec3d& cameraPosition, std::span<const Marker> markers) const;

private:
    struct Range
    {
        std::uint32_t firstIndex   = 0;
        std::uint32_t indexCount   = 0;
        std::int32_t  vertexOffset = 0;
    };

    GpuGraphicsPipeline pipeline_;
    GpuBuffer           vertices_;
    GpuBuffer           indices_;
    Range               box_;
    Range               sphere_;
};

}  // namespace StarshipSimulator
