#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/render/DynamicBuffer.h"
#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

/// One prop as the GPU draws it (32 bytes); must match the instance inputs of shaders/prop.vert.
struct PropInstance
{
    Vec3f         position{0.0F};                    // relative to the camera
    std::uint32_t tint = 0;                          // 0..255
    Vec4f         rotation{0.0F, 0.0F, 0.0F, 1.0F};  // quaternion (x, y, z, w)
};
static_assert(sizeof(PropInstance) == 32);

/// One instanced draw: every visible prop of a kind.
struct PropDraw
{
    std::uint32_t firstInstance = 0;
    std::uint32_t instanceCount = 0;
    std::uint32_t firstIndex    = 0;
    std::uint32_t indexCount    = 0;
    std::int32_t  vertexOffset  = 0;
};

/// The props' meshes, and every frame the props near the camera as instances.
class GpuProps
{
public:
    explicit GpuProps(SDL_GPUDevice* device);

    /// Uploads the props within `range` of the camera (habitat frame); call in a copy pass
    /// before the render passes that draw them.
    void prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, std::span<const PropPlacement> props,
                 double range);

    [[nodiscard]] std::span<const PropDraw> draws() const { return draws_; }
    [[nodiscard]] SDL_GPUBuffer*            vertices() const { return vertices_.get(); }
    [[nodiscard]] SDL_GPUBuffer*            indices() const { return indices_.get(); }
    [[nodiscard]] SDL_GPUBuffer*            instances() const { return instances_.get(); }

private:
    struct MeshRange
    {
        std::uint32_t firstIndex   = 0;
        std::uint32_t indexCount   = 0;
        std::int32_t  vertexOffset = 0;
    };

    GpuBuffer                                             vertices_;
    GpuBuffer                                             indices_;
    std::array<MeshRange, kPropKindCount>                 meshes_{};
    DynamicBuffer                                         instances_;
    std::vector<PropDraw>                                 draws_;
    std::array<std::vector<PropInstance>, kPropKindCount> byKind_;
    std::vector<PropInstance>                             upload_;
};

}  // namespace StarshipSimulator
