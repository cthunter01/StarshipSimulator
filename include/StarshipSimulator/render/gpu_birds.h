#pragma once

#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/birds.h"
#include "StarshipSimulator/render/dynamic_buffer.h"

namespace StarshipSimulator
{

/// One bird as the GPU draws it (32 bytes); must match the instance inputs of shaders/bird.vert.
struct BirdInstance
{
    Vec3f position{0.0F};  // relative to the camera
    float wingspanM = 0.9F;
    Vec3f forward{0.0F, 0.0F, 1.0F};
    float wingBeat = 0.0F;  // -1 wings down, +1 wings up
};
static_assert(sizeof(BirdInstance) == 32);

/// The birds near the camera, uploaded fresh every frame. They have no mesh: the six vertices of a
/// bird are worked out in the vertex shader.
class GpuBirds
{
public:
    explicit GpuBirds(SDL_GPUDevice* device);

    /// Uploads the birds; call in a copy pass before the pass that draws them.
    void prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, std::span<const Bird> birds);

    [[nodiscard]] SDL_GPUBuffer* instances() const { return instances_.get(); }
    [[nodiscard]] std::uint32_t  count() const { return count_; }

private:
    DynamicBuffer             instances_;
    std::uint32_t             count_ = 0;
    std::vector<BirdInstance> staging_;
};

}  // namespace StarshipSimulator
