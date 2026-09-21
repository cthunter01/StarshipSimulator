#pragma once

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/gpu_landscape.h"
#include "StarshipSimulator/render/gpu_people.h"
#include "StarshipSimulator/render/gpu_props.h"
#include "StarshipSimulator/render/gpu_settlements.h"
#include "StarshipSimulator/render/gpu_transit.h"
#include "StarshipSimulator/render/passes/habitat_passes.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"

// The towns: their buildings, bridges and street furniture, and the props lying about.
namespace StarshipSimulator
{

/// Buildings, bridges and street furniture, lit like the land (hill and tree shadows).
class BuildingPass
{
public:
    BuildingPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                   const GpuSettlements& settlements, const GpuLandscape& landscape,
                   const HabitatFrame& view) const;

    /// Draws the buildings within `reach` of the camera into a shadow map.
    void drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                    const GpuSettlements& settlements, const Mat4d& lightFromCameraRelative,
                    const Frustum& lightFrustum, const Vec3d& camera, double reach) const;

private:
    GpuGraphicsPipeline pipeline_;
    GpuGraphicsPipeline shadow_;
};

/// The tramway: the track and the trams running on it.
class TransitPass
{
public:
    TransitPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                   const GpuTransit& transit, const GpuLandscape& landscape,
                   const HabitatFrame& view) const;

    void drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                    const GpuTransit& transit, const gpu::FrameUniforms& light) const;

private:
    GpuGraphicsPipeline pipeline_;
    GpuGraphicsPipeline shadow_;
};

/// The people of the towns and farms, instanced: one mesh, bent into a stride by the shader.
class PeoplePass
{
public:
    PeoplePass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuPeople& people,
                   const GpuLandscape& landscape, const HabitatFrame& view) const;

    /// Draws them into a shadow map; `light` holds the light's view-projection.
    void drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                    const GpuPeople& people, const gpu::FrameUniforms& light,
                    const gpu::HabitatUniforms& habitat) const;

private:
    GpuGraphicsPipeline pipeline_;
    GpuGraphicsPipeline shadow_;
};

/// Balls, crates, barrels, bales and cafe furniture, instanced where the physics has them.
class PropPass
{
public:
    PropPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats);

    DrawStats draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuProps& props,
                   const GpuLandscape& landscape, const HabitatFrame& view) const;

    /// Draws the props into a shadow map; `light` holds the light's view-projection.
    void drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuProps& props,
                    const gpu::FrameUniforms& light) const;

private:
    GpuGraphicsPipeline pipeline_;
    GpuGraphicsPipeline shadow_;
};

}  // namespace StarshipSimulator
