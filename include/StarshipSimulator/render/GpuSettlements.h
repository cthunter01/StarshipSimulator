#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/buildings.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

/// Where one settlement's mesh lives in the shared buffers.
struct SettlementDraw
{
    Vec3d         origin{0.0};
    Vec3f         boundsMin{0.0F};
    Vec3f         boundsMax{0.0F};
    std::uint32_t firstIndex   = 0;
    std::uint32_t indexCount   = 0;
    std::int32_t  vertexOffset = 0;
};

/// The towns and farms on the GPU: their buildings, bridges and furniture in one vertex and index
/// buffer, and the towns' ground maps (an atlas and where each map lies) for the landscape shader.
/// With no settlements it still provides an (empty) atlas, so the landscape shader always has one.
class GpuSettlements
{
public:
    GpuSettlements(SDL_GPUDevice* device, std::span<const SettlementMesh> meshes,
                   const Settlements& settlements);

    /// Calls draw for each settlement whose bounds intersect the camera-relative frustum and lie
    /// within `range` of the camera.
    void forEachVisible(const Frustum& frustum, const Vec3d& camera, double range,
                        const std::function<void(const SettlementDraw&)>& draw) const;

    [[nodiscard]] SDL_GPUBuffer*  vertices() const { return vertices_.get(); }
    [[nodiscard]] SDL_GPUBuffer*  indices() const { return indices_.get(); }
    [[nodiscard]] bool            empty() const { return draws_.empty(); }
    [[nodiscard]] SDL_GPUTexture* groundAtlas() const { return atlas_.get(); }
    [[nodiscard]] SDL_GPUSampler* groundSampler() const { return sampler_.get(); }
    [[nodiscard]] SDL_GPUBuffer*  groundMaps() const { return records_.get(); }
    [[nodiscard]] std::size_t     buildingCount() const { return buildings_; }
    /// GPU memory used, in bytes (approximate).
    [[nodiscard]] std::size_t memoryBytes() const { return memoryBytes_; }

private:
    std::vector<SettlementDraw> draws_;
    GpuBuffer                   vertices_;
    GpuBuffer                   indices_;
    GpuTexture                  atlas_;
    GpuSampler                  sampler_;
    GpuBuffer                   records_;
    std::size_t                 buildings_   = 0;
    std::size_t                 memoryBytes_ = 0;
};

}  // namespace StarshipSimulator
