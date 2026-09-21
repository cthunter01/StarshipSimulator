#include "StarshipSimulator/render/gpu_people.h"

#include <cstdint>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/people.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

namespace
{
constexpr std::uint32_t kMaxPeople = 1024;
}

GpuPeople::GpuPeople(SDL_GPUDevice* device)
  : instances_(device, SDL_GPU_BUFFERUSAGE_VERTEX, kMaxPeople * sizeof(PersonInstance))
{
    const CpuMesh mesh = buildPersonMesh();
    vertices_          = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                              std::as_bytes(std::span(mesh.vertices)));
    indices_           = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_INDEX,
                                              std::as_bytes(std::span(mesh.indices)));
    indexCount_        = static_cast<std::uint32_t>(mesh.indices.size());
    staging_.reserve(kMaxPeople);
}

void GpuPeople::prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, std::span<const Person> people)
{
    staging_.clear();
    for (const Person& person : people)
    {
        if (staging_.size() >= kMaxPeople)
        {
            break;
        }
        staging_.push_back(
            PersonInstance{.position = Vec3f(person.position - camera),
                           .forward  = Vec3f(person.forward),
                           .gait     = static_cast<float>(person.gait),
                           .speed    = static_cast<float>(person.speedMS),
                           .scale    = static_cast<float>(person.heightM / kPersonHeightM),
                           .look     = static_cast<std::uint32_t>(person.clothes) |
                                       (static_cast<std::uint32_t>(person.skin) << 8U) |
                                       (static_cast<std::uint32_t>(person.doing) << 16U)});
    }
    count_ = static_cast<std::uint32_t>(staging_.size());
    if (count_ > 0)
    {
        instances_.upload(copy, std::as_bytes(std::span(staging_)));
    }
}

}  // namespace StarshipSimulator
