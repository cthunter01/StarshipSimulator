#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/people.h"
#include "StarshipSimulator/render/dynamic_buffer.h"
#include "StarshipSimulator/render/gpu_handles.h"

namespace StarshipSimulator
{

/// One person as the GPU draws them (40 bytes); must match the instance inputs of
/// shaders/person.vert.
struct PersonInstance
{
    Vec3f position{0.0F};  // their feet, relative to the camera
    Vec3f forward{0.0F, 0.0F, 1.0F};
    float gait  = 0.0F;  // 0..1 through a stride
    float speed = 0.0F;  // m/s
    float scale = 1.0F;  // their height over the mesh's 1.75 m
    // clothes | skin << 8 | activity << 16
    std::uint32_t look = 0;
};
static_assert(sizeof(PersonInstance) == 40);

/// The one person mesh, and the people near the camera uploaded fresh every frame.
class GpuPeople
{
public:
    explicit GpuPeople(SDL_GPUDevice* device);

    /// Uploads the people; call in a copy pass before the pass that draws them.
    void prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, std::span<const Person> people);

    [[nodiscard]] SDL_GPUBuffer* vertices() const { return vertices_.get(); }
    [[nodiscard]] SDL_GPUBuffer* indices() const { return indices_.get(); }
    [[nodiscard]] SDL_GPUBuffer* instances() const { return instances_.get(); }
    [[nodiscard]] std::uint32_t  indexCount() const { return indexCount_; }
    [[nodiscard]] std::uint32_t  count() const { return count_; }

private:
    GpuBuffer                   vertices_;
    GpuBuffer                   indices_;
    std::uint32_t               indexCount_ = 0;
    DynamicBuffer               instances_;
    std::uint32_t               count_ = 0;
    std::vector<PersonInstance> staging_;
};

}  // namespace StarshipSimulator
