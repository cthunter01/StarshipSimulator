#include "StarshipSimulator/render/gpu_birds.h"

#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/birds.h"

namespace StarshipSimulator
{

namespace
{
constexpr std::uint32_t kMaxBirds = 2048;
}

GpuBirds::GpuBirds(SDL_GPUDevice* device)
  : instances_(device, SDL_GPU_BUFFERUSAGE_VERTEX, kMaxBirds * sizeof(BirdInstance))
{
    staging_.reserve(kMaxBirds);
}

void GpuBirds::prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, std::span<const Bird> birds)
{
    staging_.clear();
    for (const Bird& bird : birds)
    {
        if (staging_.size() >= kMaxBirds)
        {
            break;
        }
        staging_.push_back(BirdInstance{.position  = Vec3f(bird.position - camera),
                                        .wingspanM = static_cast<float>(bird.wingspanM),
                                        .forward   = Vec3f(bird.forward),
                                        .wingBeat  = static_cast<float>(bird.wingBeat)});
    }
    count_ = static_cast<std::uint32_t>(staging_.size());
    if (count_ > 0)
    {
        instances_.upload(copy, std::as_bytes(std::span(staging_)));
    }
}

}  // namespace StarshipSimulator
