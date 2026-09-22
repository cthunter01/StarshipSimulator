#include "StarshipSimulator/render/GpuTransit.h"

#include <cstdint>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

namespace
{

constexpr std::uint32_t kMaxTransitInstances = 512;

/// The rotation that stands a mesh (y up, facing +z) on the floor facing a direction.
Vec4f facing(const Vec3d& position, const Vec3d& forward)
{
    const Vec3d up    = HabitatGeometry::localUp(position);
    const Vec3d ahead = glm::normalize(forward - (up * glm::dot(forward, up)));
    // Right-handed, or quat_cast would give a reflection and the mesh would turn inside out.
    const Vec3d side = glm::cross(up, ahead);
    const Mat3d frame(side, up, ahead);
    const Quatd turn = glm::normalize(glm::quat_cast(frame));
    return {static_cast<float>(turn.x), static_cast<float>(turn.y), static_cast<float>(turn.z),
            static_cast<float>(turn.w)};
}

}  // namespace

GpuTransit::GpuTransit(SDL_GPUDevice* device, const std::vector<TrackChunk>& track)
  : instances_(device, SDL_GPU_BUFFERUSAGE_VERTEX, kMaxTransitInstances * sizeof(TransitInstance))
{
    std::vector<Vertex>        vertices;
    std::vector<std::uint32_t> indices;
    const auto                 append = [&](const CpuMesh& mesh) {
        const Chunk range{.firstIndex   = static_cast<std::uint32_t>(indices.size()),
                          .indexCount   = static_cast<std::uint32_t>(mesh.indices.size()),
                          .vertexOffset = static_cast<std::int32_t>(vertices.size())};
        vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        indices.insert(indices.end(), mesh.indices.begin(), mesh.indices.end());
        return range;
    };
    chunks_.reserve(track.size());
    for (const TrackChunk& piece : track)
    {
        Chunk chunk  = append(piece.mesh);
        chunk.origin = piece.origin;
        chunk.centre = piece.origin + piece.centre;
        chunk.radius = piece.radius;
        chunks_.push_back(chunk);
    }
    tram_ = append(buildTramMesh());
    if (!vertices.empty())
    {
        vertices_ = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                         std::as_bytes(std::span(vertices)));
        indices_  = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_INDEX,
                                         std::as_bytes(std::span(indices)));
    }
    staging_.reserve(kMaxTransitInstances);
}

void GpuTransit::prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, const Frustum& frustum,
                         std::span<const Tram> trams, double range)
{
    staging_.clear();
    draws_.clear();
    for (const Chunk& chunk : chunks_)
    {
        if (staging_.size() >= kMaxTransitInstances)
        {
            break;
        }
        const Vec3d relative = chunk.centre - camera;
        if (glm::length(relative) > range + chunk.radius ||
            !frustum.intersects(relative - Vec3d(chunk.radius), relative + Vec3d(chunk.radius)))
        {
            continue;
        }
        draws_.push_back({.firstInstance = static_cast<std::uint32_t>(staging_.size()),
                          .instanceCount = 1,
                          .firstIndex    = chunk.firstIndex,
                          .indexCount    = chunk.indexCount,
                          .vertexOffset  = chunk.vertexOffset});
        staging_.push_back({.position = Vec3f(chunk.origin - camera)});
    }

    const auto    firstTram = static_cast<std::uint32_t>(staging_.size());
    std::uint32_t drawn     = 0;
    for (const Tram& tram : trams)
    {
        if (staging_.size() >= kMaxTransitInstances ||
            glm::distance(tram.position, camera) > range + 40.0)
        {
            continue;
        }
        staging_.push_back({.position = Vec3f(tram.position - camera),
                            .tint     = static_cast<std::uint32_t>(tram.line % 4),
                            .rotation = facing(tram.position, tram.forward)});
        ++drawn;
    }
    if (drawn > 0)
    {
        draws_.push_back({.firstInstance = firstTram,
                          .instanceCount = drawn,
                          .firstIndex    = tram_.firstIndex,
                          .indexCount    = tram_.indexCount,
                          .vertexOffset  = tram_.vertexOffset});
    }
    if (!staging_.empty())
    {
        instances_.upload(copy, std::as_bytes(std::span(staging_)));
    }
}

}  // namespace StarshipSimulator
