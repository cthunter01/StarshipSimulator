#include "StarshipSimulator/render/gpu_landscape.h"

#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/frustum.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/terrain_lod.h"
#include "StarshipSimulator/render/dynamic_buffer.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/texture.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

namespace
{

constexpr std::uint32_t kMaxPatches = 32768;

GpuSampler createSampler(SDL_GPUDevice* device, SDL_GPUSamplerAddressMode aroundMode)
{
    const SDL_GPUSamplerCreateInfo info{
        .min_filter     = SDL_GPU_FILTER_LINEAR,
        .mag_filter     = SDL_GPU_FILTER_LINEAR,
        .mipmap_mode    = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR,
        .address_mode_u = aroundMode,
        .address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
        .max_lod        = 1000.0F,
    };
    GpuSampler sampler(device, SDL_CreateGPUSampler(device, &info));
    if (!sampler.valid())
    {
        throw std::runtime_error(std::format("Cannot create a sampler: {}", SDL_GetError()));
    }
    return sampler;
}

/// About how much GPU memory the terrain takes (mipmaps add a third to their base level).
std::size_t memoryFor(const TerrainGrid& grid)
{
    const std::size_t mipmapped = (grid.heights.size() * sizeof(std::uint16_t)) + grid.cover.size();
    return (mipmapped * 4 / 3) + (grid.profile.size() * sizeof(Vec4f)) +
           (grid.arcByZ.size() * sizeof(float)) + (std::size_t{2} * kMaxPatches * sizeof(Vec4f));
}

}  // namespace

GpuLandscape::GpuLandscape(SDL_GPUDevice* device, const TerrainGrid& grid, TerrainLod lod)
  : lod_(std::move(lod)),
    uniforms_(gpu::makeLandscapeUniforms(grid, lod_.morphs())),
    memoryBytes_(memoryFor(grid))
{
    const TerrainGridLayout& layout = grid.layout;
    heights_ = createTexture(device, {.width   = layout.columns,
                                      .height  = layout.rows(),
                                      .format  = SDL_GPU_TEXTUREFORMAT_R16_UNORM,
                                      .pixels  = std::as_bytes(std::span(grid.heights)),
                                      .mipmaps = true});
    cover_   = createTexture(device, {.width   = grid.coverColumns,
                                      .height  = grid.coverRows,
                                      .format  = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                                      .pixels  = std::as_bytes(std::span(grid.cover)),
                                      .mipmaps = true});
    profile_ = createTexture(device, {.width   = layout.rows(),
                                      .height  = 1,
                                      .format  = SDL_GPU_TEXTUREFORMAT_R32G32B32A32_FLOAT,
                                      .pixels  = std::as_bytes(std::span(grid.profile)),
                                      .mipmaps = false});
    arcByZ_  = createTexture(device, {.width   = static_cast<std::uint32_t>(grid.arcByZ.size()),
                                      .height  = 1,
                                      .format  = SDL_GPU_TEXTUREFORMAT_R32_FLOAT,
                                      .pixels  = std::as_bytes(std::span(grid.arcByZ)),
                                      .mipmaps = false});

    surfaceSampler_ = createSampler(device, SDL_GPU_SAMPLERADDRESSMODE_REPEAT);
    profileSampler_ = createSampler(device, SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE);

    // One patch mesh: a grid of quads x quads, triangles facing the axis (counter-clockwise seen
    // from inside the habitat).
    const std::uint32_t quads = layout.nodeQuads;
    std::vector<Vec2f>  vertices;
    for (std::uint32_t row = 0; row <= quads; ++row)
    {
        for (std::uint32_t column = 0; column <= quads; ++column)
        {
            vertices.emplace_back(static_cast<float>(column), static_cast<float>(row));
        }
    }
    std::vector<std::uint32_t> indices;
    for (std::uint32_t row = 0; row < quads; ++row)
    {
        for (std::uint32_t column = 0; column < quads; ++column)
        {
            const std::uint32_t a = (row * (quads + 1)) + column;  // (column, row)
            const std::uint32_t b = a + 1;                         // next column
            const std::uint32_t c = a + quads + 1;                 // next row
            const std::uint32_t d = c + 1;
            for (const std::uint32_t i : {a, c, b, b, c, d})
            {
                indices.push_back(i);
            }
        }
    }
    gridVertices_ = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                         std::as_bytes(std::span(vertices)));
    gridIndices_ =
        createBufferWithData(device, SDL_GPU_BUFFERUSAGE_INDEX, std::as_bytes(std::span(indices)));
    gridIndexCount_ = static_cast<std::uint32_t>(indices.size());
    patches_        = DynamicBuffer(device, SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ,
                                    kMaxPatches * sizeof(Vec4f));
    waterPatches_   = DynamicBuffer(device, SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ,
                                    kMaxPatches * sizeof(Vec4f));
}

void GpuLandscape::prepare(SDL_GPUCopyPass* copy, const Vec3d& camera, const Frustum& frustum)
{
    const std::vector<TerrainPatch> selected = lod_.select(camera, frustum);
    std::vector<Vec4f>              all;
    std::vector<Vec4f>              water;
    all.reserve(selected.size());
    for (const TerrainPatch& patch : selected)
    {
        const Vec4f data(static_cast<float>(patch.column), static_cast<float>(patch.row),
                         static_cast<float>(patch.level), 0.0F);
        all.push_back(data);
        if (patch.water)
        {
            water.push_back(data);
        }
    }
    patchCount_      = patches_.upload(copy, std::as_bytes(std::span(all))) / sizeof(Vec4f);
    waterPatchCount_ = waterPatches_.upload(copy, std::as_bytes(std::span(water))) / sizeof(Vec4f);
}

}  // namespace StarshipSimulator
