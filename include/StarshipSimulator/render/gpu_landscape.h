#pragma once

#include <cstddef>
#include <cstdint>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/frustum.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/terrain_lod.h"
#include "StarshipSimulator/render/dynamic_buffer.h"
#include "StarshipSimulator/render/gpu_handles.h"

namespace StarshipSimulator
{

/// The habitat's terrain on the GPU: height, land-cover and profile textures, one grid mesh that
/// every level-of-detail patch reuses, and the patches picked for the current camera.
class GpuLandscape
{
public:
    /// Uploads the grid. Throws std::runtime_error if the GPU cannot take it.
    GpuLandscape(SDL_GPUDevice* device, const TerrainGrid& grid, TerrainLod lod);

    /// Picks the patches for a camera (habitat frame) and camera-relative frustum and uploads
    /// them. Call in a copy pass before the render pass that draws the terrain.
    void prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, const Frustum& frustum);

    [[nodiscard]] const gpu::LandscapeUniforms& uniforms() const { return uniforms_; }
    [[nodiscard]] std::uint32_t                 patchCount() const { return patchCount_; }
    [[nodiscard]] std::uint32_t  patchTriangles() const { return gridIndexCount_ / 3; }
    [[nodiscard]] SDL_GPUBuffer* patches() const { return patches_.get(); }
    /// The patches that hold some water, for drawing the water surface.
    [[nodiscard]] std::uint32_t   waterPatchCount() const { return waterPatchCount_; }
    [[nodiscard]] SDL_GPUBuffer*  waterPatches() const { return waterPatches_.get(); }
    [[nodiscard]] SDL_GPUBuffer*  gridVertices() const { return gridVertices_.get(); }
    [[nodiscard]] SDL_GPUBuffer*  gridIndices() const { return gridIndices_.get(); }
    [[nodiscard]] std::uint32_t   gridIndexCount() const { return gridIndexCount_; }
    [[nodiscard]] SDL_GPUTexture* heights() const { return heights_.get(); }
    [[nodiscard]] SDL_GPUTexture* cover() const { return cover_.get(); }
    [[nodiscard]] SDL_GPUTexture* profile() const { return profile_.get(); }
    [[nodiscard]] SDL_GPUTexture* arcByZ() const { return arcByZ_.get(); }
    [[nodiscard]] SDL_GPUSampler* surfaceSampler() const { return surfaceSampler_.get(); }
    [[nodiscard]] SDL_GPUSampler* profileSampler() const { return profileSampler_.get(); }
    /// GPU memory used by the terrain's textures and buffers, in bytes (approximate).
    [[nodiscard]] std::size_t memoryBytes() const { return memoryBytes_; }

private:
    TerrainLod             lod_;
    gpu::LandscapeUniforms uniforms_;
    GpuTexture             heights_;
    GpuTexture             cover_;
    GpuTexture             profile_;
    GpuTexture             arcByZ_;
    GpuSampler             surfaceSampler_;  // wraps around the axis, clamps along it
    GpuSampler             profileSampler_;  // clamps
    GpuBuffer              gridVertices_;
    GpuBuffer              gridIndices_;
    std::uint32_t          gridIndexCount_ = 0;
    DynamicBuffer          patches_;
    std::uint32_t          patchCount_ = 0;
    DynamicBuffer          waterPatches_;
    std::uint32_t          waterPatchCount_ = 0;
    std::size_t            memoryBytes_     = 0;
};

}  // namespace StarshipSimulator
