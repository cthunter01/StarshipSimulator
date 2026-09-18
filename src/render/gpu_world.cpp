#include "StarshipSimulator/render/gpu_world.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <span>
#include <stdexcept>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/frustum.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

namespace
{

std::uint32_t checkedSize(std::size_t bytes)
{
    if (bytes == 0 || bytes > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error("habitat mesh too large (or empty) for one GPU buffer");
    }
    return static_cast<std::uint32_t>(bytes);
}

}  // namespace

GpuWorld::GpuWorld(SDL_GPUDevice* device, const HabitatMeshes& meshes)
{
    std::size_t vertexCount = 0;
    std::size_t indexCount  = 0;
    for (const MeshChunk& chunk : meshes.chunks)
    {
        chunks_.push_back({.kind         = chunk.kind,
                           .origin       = chunk.origin,
                           .boundsMin    = chunk.boundsMin,
                           .boundsMax    = chunk.boundsMax,
                           .firstIndex   = static_cast<std::uint32_t>(indexCount),
                           .indexCount   = static_cast<std::uint32_t>(chunk.mesh.indices.size()),
                           .vertexOffset = static_cast<std::int32_t>(vertexCount)});
        vertexCount += chunk.mesh.vertices.size();
        indexCount += chunk.mesh.indices.size();
    }
    triangleCount_ = indexCount / 3;

    vertices_ = createBufferFilledBy(
        device, SDL_GPU_BUFFERUSAGE_VERTEX, checkedSize(vertexCount * sizeof(Vertex)),
        [&](std::span<std::byte> target) {
            std::size_t offset = 0;
            for (const MeshChunk& chunk : meshes.chunks)
            {
                const auto bytes = std::as_bytes(std::span(chunk.mesh.vertices));
                std::memcpy(target.subspan(offset).data(), bytes.data(), bytes.size());
                offset += bytes.size();
            }
        });
    indices_ = createBufferFilledBy(
        device, SDL_GPU_BUFFERUSAGE_INDEX, checkedSize(indexCount * sizeof(std::uint32_t)),
        [&](std::span<std::byte> target) {
            std::size_t offset = 0;
            for (const MeshChunk& chunk : meshes.chunks)
            {
                const auto bytes = std::as_bytes(std::span(chunk.mesh.indices));
                std::memcpy(target.subspan(offset).data(), bytes.data(), bytes.size());
                offset += bytes.size();
            }
        });
}

void GpuWorld::forEachVisible(ChunkKind kind, const Frustum& frustum, const Vec3d& camera,
                              const std::function<void(const ChunkDraw&)>& draw) const
{
    for (const ChunkDraw& chunk : chunks_)
    {
        const Vec3d relative = chunk.origin - camera;
        if (chunk.kind == kind && frustum.intersects(relative + Vec3d(chunk.boundsMin),
                                                     relative + Vec3d(chunk.boundsMax)))
        {
            draw(chunk);
        }
    }
}

}  // namespace StarshipSimulator
