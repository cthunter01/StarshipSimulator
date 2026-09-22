#include "StarshipSimulator/render/GpuSettlements.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <span>
#include <stdexcept>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/gpu_abi/ground_atlas.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/buildings.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/texture.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

GpuSettlements::GpuSettlements(SDL_GPUDevice* device, std::span<const SettlementMesh> meshes,
                               const Settlements& settlements)
  : buildings_(settlements.buildings.size())
{
    std::size_t vertexCount = 0;
    std::size_t indexCount  = 0;
    for (const SettlementMesh& m : meshes)
    {
        draws_.push_back({.origin       = m.origin,
                          .boundsMin    = m.boundsMin,
                          .boundsMax    = m.boundsMax,
                          .firstIndex   = static_cast<std::uint32_t>(indexCount),
                          .indexCount   = static_cast<std::uint32_t>(m.mesh.indices.size()),
                          .vertexOffset = static_cast<std::int32_t>(vertexCount)});
        vertexCount += m.mesh.vertices.size();
        indexCount += m.mesh.indices.size();
    }
    if (indexCount > 0)
    {
        vertices_ = createBufferFilledBy(
            device, SDL_GPU_BUFFERUSAGE_VERTEX,
            static_cast<std::uint32_t>(vertexCount * sizeof(Vertex)),
            [&](std::span<std::byte> target) {
                std::size_t offset = 0;
                for (const SettlementMesh& m : meshes)
                {
                    const auto bytes = std::as_bytes(std::span(m.mesh.vertices));
                    std::memcpy(target.subspan(offset).data(), bytes.data(), bytes.size());
                    offset += bytes.size();
                }
            });
        indices_ = createBufferFilledBy(
            device, SDL_GPU_BUFFERUSAGE_INDEX,
            static_cast<std::uint32_t>(indexCount * sizeof(std::uint32_t)),
            [&](std::span<std::byte> target) {
                std::size_t offset = 0;
                for (const SettlementMesh& m : meshes)
                {
                    const auto bytes = std::as_bytes(std::span(m.mesh.indices));
                    std::memcpy(target.subspan(offset).data(), bytes.data(), bytes.size());
                    offset += bytes.size();
                }
            });
    }

    const gpu::GroundAtlas atlas = gpu::packGroundAtlas(settlements);
    atlas_   = createTexture(device, {.width   = atlas.width,
                                      .height  = atlas.height,
                                      .format  = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                                      .pixels  = std::as_bytes(std::span(atlas.pixels)),
                                      .mipmaps = true});
    records_ = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ,
                                    std::as_bytes(std::span(atlas.records)));
    const SDL_GPUSamplerCreateInfo info{
        .min_filter     = SDL_GPU_FILTER_LINEAR,
        .mag_filter     = SDL_GPU_FILTER_LINEAR,
        .mipmap_mode    = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR,
        .address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .max_lod        = 1000.0F,
    };
    sampler_ = GpuSampler(device, SDL_CreateGPUSampler(device, &info));
    if (!sampler_.valid())
    {
        throw std::runtime_error(std::format("Cannot create a sampler: {}", SDL_GetError()));
    }
    memoryBytes_ = (vertexCount * sizeof(Vertex)) + (indexCount * sizeof(std::uint32_t)) +
                   (atlas.pixels.size() * 4 / 3) + (atlas.records.size() * sizeof(Vec4f));
}

void GpuSettlements::forEachVisible(const Frustum& frustum, const Vec3d& camera, double range,
                                    const std::function<void(const SettlementDraw&)>& draw) const
{
    for (const SettlementDraw& d : draws_)
    {
        const Vec3d low     = d.origin + Vec3d(d.boundsMin) - camera;
        const Vec3d high    = d.origin + Vec3d(d.boundsMax) - camera;
        const Vec3d nearest = glm::clamp(Vec3d(0.0), low, high);
        if (glm::length(nearest) <= range && frustum.intersects(low, high))
        {
            draw(d);
        }
    }
}

}  // namespace StarshipSimulator
