#include "StarshipSimulator/render/GpuProps.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/render/DynamicBuffer.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

namespace
{

constexpr std::uint32_t kMaxPropInstances = 8192;

}  // namespace

GpuProps::GpuProps(SDL_GPUDevice* device)
  : instances_(device, SDL_GPU_BUFFERUSAGE_VERTEX,
               static_cast<std::uint32_t>(kMaxPropInstances * sizeof(PropInstance)))
{
    std::vector<Vertex>        vertices;
    std::vector<std::uint32_t> indices;
    for (std::size_t k = 0; k < kPropKindCount; ++k)
    {
        const CpuMesh mesh = makePropMesh(static_cast<PropKind>(k));
        meshes_.at(k)      = {.firstIndex   = static_cast<std::uint32_t>(indices.size()),
                              .indexCount   = static_cast<std::uint32_t>(mesh.indices.size()),
                              .vertexOffset = static_cast<std::int32_t>(vertices.size())};
        vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        indices.insert(indices.end(), mesh.indices.begin(), mesh.indices.end());
    }
    vertices_ = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                     std::as_bytes(std::span(vertices)));
    indices_ =
        createBufferWithData(device, SDL_GPU_BUFFERUSAGE_INDEX, std::as_bytes(std::span(indices)));
}

void GpuProps::prepare(SDL_GPUCopyPass* copy, const Vec3d& camera,
                       std::span<const PropPlacement> props, double range)
{
    for (auto& list : byKind_)
    {
        list.clear();
    }
    const double rangeSquared = range * range;
    for (const PropPlacement& prop : props)
    {
        const Vec3d relative = prop.position - camera;
        if (glm::dot(relative, relative) > rangeSquared)
        {
            continue;
        }
        const Quatd q = prop.orientation;
        byKind_.at(static_cast<std::size_t>(prop.kind))
            .push_back({.position = Vec3f(relative),
                        .tint     = static_cast<std::uint32_t>(prop.tint * 255.0F),
                        .rotation = Vec4f(Vec4d(q.x, q.y, q.z, q.w))});
    }
    upload_.clear();
    draws_.clear();
    for (std::size_t k = 0; k < kPropKindCount; ++k)
    {
        const auto& list  = byKind_.at(k);
        const auto  first = static_cast<std::uint32_t>(upload_.size());
        const auto  count = static_cast<std::uint32_t>(
            std::min<std::size_t>(list.size(), kMaxPropInstances - upload_.size()));
        if (count == 0)
        {
            continue;
        }
        upload_.insert(upload_.end(), list.begin(), list.begin() + count);
        const MeshRange& mesh = meshes_.at(k);
        draws_.push_back({.firstInstance = first,
                          .instanceCount = count,
                          .firstIndex    = mesh.firstIndex,
                          .indexCount    = mesh.indexCount,
                          .vertexOffset  = mesh.vertexOffset});
    }
    instances_.upload(copy, std::as_bytes(std::span(upload_)));
}

}  // namespace StarshipSimulator
