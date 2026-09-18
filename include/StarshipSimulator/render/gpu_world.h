#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/frustum.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/render/gpu_handles.h"

namespace StarshipSimulator
{

/// Where one mesh chunk lives in the shared GPU buffers.
struct ChunkDraw
{
    ChunkKind     kind = ChunkKind::Terrain;
    Vec3d         origin{0.0};
    Vec3f         boundsMin{0.0F};
    Vec3f         boundsMax{0.0F};
    std::uint32_t firstIndex   = 0;
    std::uint32_t indexCount   = 0;
    std::int32_t  vertexOffset = 0;
};

/// The habitat's meshes on the GPU: all chunks packed into one vertex and one index buffer.
class GpuWorld
{
public:
    /// Uploads the meshes. Throws std::runtime_error if the GPU buffers cannot be created.
    GpuWorld(SDL_GPUDevice* device, const HabitatMeshes& meshes);

    [[nodiscard]] SDL_GPUBuffer*                vertices() const { return vertices_.get(); }
    [[nodiscard]] SDL_GPUBuffer*                indices() const { return indices_.get(); }
    [[nodiscard]] const std::vector<ChunkDraw>& chunks() const { return chunks_; }
    [[nodiscard]] std::size_t                   triangleCount() const { return triangleCount_; }

    /// Calls draw for each chunk of the kind whose bounds intersect the camera-relative frustum.
    void forEachVisible(ChunkKind kind, const Frustum& frustum, const Vec3d& camera,
                        const std::function<void(const ChunkDraw&)>& draw) const;

private:
    GpuBuffer              vertices_;
    GpuBuffer              indices_;
    std::vector<ChunkDraw> chunks_;
    std::size_t            triangleCount_ = 0;
};

}  // namespace StarshipSimulator
