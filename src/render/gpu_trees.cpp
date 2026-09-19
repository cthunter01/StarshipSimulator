#include "StarshipSimulator/render/gpu_trees.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/frustum.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/trees.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

namespace
{

double distanceToBox(const Vec3d& p, const Vec3d& boxMin, const Vec3d& boxMax)
{
    return glm::length(glm::max(glm::max(boxMin - p, p - boxMax), Vec3d(0.0)));
}

double farthestInBox(const Vec3d& p, const Vec3d& boxMin, const Vec3d& boxMax)
{
    return glm::length(glm::max(glm::abs(boxMin - p), glm::abs(boxMax - p)));
}

}  // namespace

GpuTrees::GpuTrees(SDL_GPUDevice* device, const TreeLayer& layer)
  : tiles_(layer.tiles), treeCount_(layer.instances.size())
{
    std::vector<Vertex>        vertices;
    std::vector<std::uint32_t> indices;
    for (std::size_t s = 0; s < kTreeSpeciesCount; ++s)
    {
        for (const bool detailed : {false, true})
        {
            const CpuMesh mesh = makeTreeMesh(static_cast<TreeSpecies>(s), detailed);
            meshes_.at(s).at(detailed ? 1 : 0) = {
                .firstIndex   = static_cast<std::uint32_t>(indices.size()),
                .indexCount   = static_cast<std::uint32_t>(mesh.indices.size()),
                .vertexOffset = static_cast<std::int32_t>(vertices.size())};
            vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
            indices.insert(indices.end(), mesh.indices.begin(), mesh.indices.end());
        }
    }
    vertices_ = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                     std::as_bytes(std::span(vertices)));
    indices_ =
        createBufferWithData(device, SDL_GPU_BUFFERUSAGE_INDEX, std::as_bytes(std::span(indices)));
    if (!layer.instances.empty())
    {
        instances_ = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                          std::as_bytes(std::span(layer.instances)));
    }
}

std::vector<TreeDraw> GpuTrees::select(const Vec3d& camera, const Frustum& frustum,
                                       const TreeRanges& ranges) const
{
    std::vector<TreeDraw> draws;
    if (!instances_.valid())
    {
        return draws;
    }
    for (const TreeTile& tile : tiles_)
    {
        const Vec3d  boxMin  = tile.origin + Vec3d(tile.boundsMin);
        const Vec3d  boxMax  = tile.origin + Vec3d(tile.boundsMax);
        const double nearest = distanceToBox(camera, boxMin, boxMax);
        if (nearest > ranges.farEnd || !frustum.intersects(boxMin - camera, boxMax - camera))
        {
            continue;
        }
        const double  farthest = farthestInBox(camera, boxMin, boxMax);
        const Vec3f   origin(tile.origin - camera);
        std::uint32_t first = tile.first;
        for (std::size_t s = 0; s < kTreeSpeciesCount; ++s)
        {
            const std::uint32_t count = tile.counts.at(s);
            for (const bool detailed : {true, false})
            {
                // Detailed near, simple beyond, both in the band where they cross-fade.
                const bool wanted = detailed ? nearest < ranges.detailEnd
                                             : farthest > ranges.detailEnd - ranges.blend;
                if (count == 0 || !wanted)
                {
                    continue;
                }
                const MeshRange& mesh = meshes_.at(s).at(detailed ? 1 : 0);
                draws.push_back({.origin        = origin,
                                 .firstInstance = first,
                                 .instanceCount = count,
                                 .firstIndex    = mesh.firstIndex,
                                 .indexCount    = mesh.indexCount,
                                 .vertexOffset  = mesh.vertexOffset,
                                 .detailed      = detailed});
            }
            first += count;
        }
    }
    return draws;
}

std::vector<TreeDraw> GpuTrees::selectNear(const Vec3d& camera, double radius) const
{
    std::vector<TreeDraw> draws;
    if (!instances_.valid())
    {
        return draws;
    }
    for (const TreeTile& tile : tiles_)
    {
        const Vec3d boxMin = tile.origin + Vec3d(tile.boundsMin);
        const Vec3d boxMax = tile.origin + Vec3d(tile.boundsMax);
        if (distanceToBox(camera, boxMin, boxMax) > radius)
        {
            continue;
        }
        std::uint32_t first = tile.first;
        for (std::size_t s = 0; s < kTreeSpeciesCount; ++s)
        {
            const std::uint32_t count = tile.counts.at(s);
            if (count > 0)
            {
                const MeshRange& mesh = meshes_.at(s).at(0);
                draws.push_back({.origin        = Vec3f(tile.origin - camera),
                                 .firstInstance = first,
                                 .instanceCount = count,
                                 .firstIndex    = mesh.firstIndex,
                                 .indexCount    = mesh.indexCount,
                                 .vertexOffset  = mesh.vertexOffset,
                                 .detailed      = false});
            }
            first += count;
        }
    }
    return draws;
}

}  // namespace StarshipSimulator
