#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/render/DynamicBuffer.h"
#include "StarshipSimulator/render/GpuHandle.h"

namespace StarshipSimulator
{

/// One track chunk or tram as the GPU draws it (32 bytes); must match shaders/transit.vert.
struct TransitInstance
{
    Vec3f         position{0.0F};                    // relative to the camera
    std::uint32_t tint = 0;                          // which livery
    Vec4f         rotation{0.0F, 0.0F, 0.0F, 1.0F};  // quaternion (x, y, z, w)
};
static_assert(sizeof(TransitInstance) == 32);

/// One instanced draw: a stretch of track, or all the trams.
struct TransitDraw
{
    std::uint32_t firstInstance = 0;
    std::uint32_t instanceCount = 0;
    std::uint32_t firstIndex    = 0;
    std::uint32_t indexCount    = 0;
    std::int32_t  vertexOffset  = 0;
};

/// The tramway: the track's chunks and the tram car's (and a lift cabin's) mesh, uploaded once,
/// with the chunks in view and the trams on the line picked out fresh every frame.
class GpuTransit
{
public:
    GpuTransit(SDL_GPUDevice* device, const std::vector<TrackChunk>& track);

    /// Picks the track in view and places the trams; call in a copy pass before drawing.
    void prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, const Frustum& frustum,
                 std::span<const Tram> trams, double range);

    [[nodiscard]] std::span<const TransitDraw> draws() const { return draws_; }
    [[nodiscard]] SDL_GPUBuffer*               vertices() const { return vertices_.get(); }
    [[nodiscard]] SDL_GPUBuffer*               indices() const { return indices_.get(); }
    [[nodiscard]] SDL_GPUBuffer*               instances() const { return instances_.get(); }
    [[nodiscard]] std::size_t                  chunkCount() const { return chunks_.size(); }

private:
    struct Chunk
    {
        Vec3d         origin{0.0};
        Vec3d         centre{0.0};  // of its bounding sphere, in the habitat frame
        double        radius       = 0.0;
        std::uint32_t firstIndex   = 0;
        std::uint32_t indexCount   = 0;
        std::int32_t  vertexOffset = 0;
    };

    GpuBuffer                    vertices_;
    GpuBuffer                    indices_;
    std::vector<Chunk>           chunks_;
    Chunk                        tram_;
    Chunk                        lift_;
    DynamicBuffer                instances_;
    std::vector<TransitInstance> staging_;
    std::vector<TransitDraw>     draws_;
};

}  // namespace StarshipSimulator
