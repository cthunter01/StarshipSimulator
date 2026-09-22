#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/trees.h"
#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

/// One instanced draw of trees: a run of one tile's instances of one species at one detail level.
struct TreeDraw
{
    Vec3f         origin{0.0F};  // the tile's origin, relative to the camera
    std::uint32_t firstInstance = 0;
    std::uint32_t instanceCount = 0;
    std::uint32_t firstIndex    = 0;
    std::uint32_t indexCount    = 0;
    std::int32_t  vertexOffset  = 0;
    bool          detailed      = false;
};

/// Distances (m) at which trees change detail and fade out; the terrain's painted canopy takes
/// over beyond.
struct TreeRanges
{
    double detailEnd = 320.0;   // detailed meshes out to here
    double blend     = 60.0;    // over which the detail levels cross-fade
    double farEnd    = 1800.0;  // no instanced trees beyond
    double fade      = 300.0;   // over which they fade out
};

/// All the habitat's trees on the GPU: their instances (by tile) and the species' meshes.
class GpuTrees
{
public:
    GpuTrees(SDL_GPUDevice* device, const TreeLayer& layer);

    /// The draws for a camera (habitat frame) and camera-relative frustum.
    [[nodiscard]] std::vector<TreeDraw> select(const Vec3d& camera, const Frustum& frustum,
                                               const TreeRanges& ranges) const;

    /// Simple-mesh draws of every tree within `radius` of a point (for shadow maps).
    [[nodiscard]] std::vector<TreeDraw> selectNear(const Vec3d& camera, double radius) const;

    [[nodiscard]] SDL_GPUBuffer* vertices() const { return vertices_.get(); }
    [[nodiscard]] SDL_GPUBuffer* indices() const { return indices_.get(); }
    [[nodiscard]] SDL_GPUBuffer* instances() const { return instances_.get(); }
    [[nodiscard]] std::size_t    treeCount() const { return treeCount_; }
    /// GPU memory used by the trees, in bytes (approximate).
    [[nodiscard]] std::size_t memoryBytes() const { return treeCount_ * sizeof(TreeInstance); }

private:
    struct MeshRange
    {
        std::uint32_t firstIndex   = 0;
        std::uint32_t indexCount   = 0;
        std::int32_t  vertexOffset = 0;
    };

    std::vector<TreeTile>                                   tiles_;
    std::array<std::array<MeshRange, 2>, kTreeSpeciesCount> meshes_{};  // [species][detailed]
    GpuBuffer                                               vertices_;
    GpuBuffer                                               indices_;
    GpuBuffer                                               instances_;
    std::size_t                                             treeCount_ = 0;
};

}  // namespace StarshipSimulator
