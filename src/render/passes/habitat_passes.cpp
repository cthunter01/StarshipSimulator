#include "StarshipSimulator/render/passes/habitat_passes.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_stdinc.h>

#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/core/procgen/trees.h"
#include "StarshipSimulator/render/gpu_birds.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/gpu_landscape.h"
#include "StarshipSimulator/render/gpu_settlements.h"
#include "StarshipSimulator/render/gpu_trees.h"
#include "StarshipSimulator/render/gpu_world.h"
#include "StarshipSimulator/render/pipeline.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"
#include "StarshipSimulator/render/shadow_map.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

namespace
{

constexpr std::uint32_t kVerticesPerQuad = 6;

struct alignas(16) GlassUniforms
{
    Vec4f mode{0.0F};  // x: 0 = transmittance pass, 1 = emission pass
};

void bindWorld(SDL_GPURenderPass* pass, const GpuWorld& world)
{
    const SDL_GPUBufferBinding vertices{.buffer = world.vertices(), .offset = 0};
    const SDL_GPUBufferBinding indices{.buffer = world.indices(), .offset = 0};
    SDL_BindGPUVertexBuffers(pass, 0, &vertices, 1);
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
}

/// Draws the visible chunks of one kind, pushing each chunk's camera-relative transform.
DrawStats drawChunks(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const GpuWorld& world,
                     const HabitatFrame& view, ChunkKind kind)
{
    DrawStats stats;
    world.forEachVisible(kind, *view.frustum, view.camera, [&](const ChunkDraw& chunk) {
        const Mat4d             model = glm::translate(Mat4d(1.0), chunk.origin - view.camera);
        const gpu::DrawUniforms draw{.modelViewProjection = Mat4f(view.viewProjection * model),
                                     .model               = Mat4f(model)};
        SDL_PushGPUVertexUniformData(commands, 0, &draw, sizeof(draw));
        SDL_DrawGPUIndexedPrimitives(pass, chunk.indexCount, 1, chunk.firstIndex,
                                     chunk.vertexOffset, 0);
        ++stats.chunks;
        stats.triangles += chunk.indexCount / 3;
    });
    return stats;
}

constexpr std::uint32_t kCloudShellSegments = 128;  // SHELL_SEGMENTS in clouds_shell.glsl

void pushHabitatFragmentUniforms(SDL_GPUCommandBuffer* commands, const HabitatFrame& view)
{
    SDL_PushGPUFragmentUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    SDL_PushGPUFragmentUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
}

}  // namespace

// ---- Stars -------------------------------------------------------------------------------------

StarPass::StarPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats,
                   std::span<const GpuStar> stars)
  : count_(static_cast<std::uint32_t>(stars.size()))
{
    const GpuShader     vertex      = shaders.load("stars.vert");
    const GpuShader     fragment    = shaders.load("stars.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.blend               = BlendMode::Additive;
    pipeline_                       = createPipeline(device, description, "star");
    if (!stars.empty())
    {
        stars_ = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ,
                                      std::as_bytes(stars));
    }
}

void StarPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                    const gpu::FrameUniforms& frame, const gpu::SkyUniforms& sky) const
{
    if (count_ == 0)
    {
        return;
    }
    // SDL takes SDL_GPUBuffer* const*, so the pointee cannot be const.
    SDL_GPUBuffer* const buffer = stars_.get();  // NOLINT(misc-const-correctness)
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexStorageBuffers(pass, 0, &buffer, 1);
    SDL_PushGPUVertexUniformData(commands, 0, &frame, sizeof(frame));
    SDL_PushGPUVertexUniformData(commands, 1, &sky, sizeof(sky));
    SDL_DrawGPUPrimitives(pass, count_ * kVerticesPerQuad, 1, 0, 0);
}

// ---- Landscape ---------------------------------------------------------------------------------

LandscapePass::LandscapePass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                             const SceneFormats& formats)
{
    const GpuShader                      vertex   = shaders.load("landscape.vert");
    const GpuShader                      fragment = shaders.load("landscape.frag");
    const SDL_GPUVertexBufferDescription grid{
        .slot = 0, .pitch = sizeof(Vec2f), .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX};
    const SDL_GPUVertexAttribute position{
        .location = 0, .buffer_slot = 0, .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, .offset = 0};
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.vertexBuffers       = std::span(&grid, 1);
    description.vertexAttributes    = std::span(&position, 1);
    description.cull                = SDL_GPU_CULLMODE_BACK;
    description.depth               = DepthMode::TestWrite;
    pipeline_                       = createPipeline(device, description, "landscape");
}

DrawStats LandscapePass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                              const GpuLandscape& landscape, const GpuSettlements& settlements,
                              const HabitatFrame& view) const
{
    const std::uint32_t patches = landscape.patchCount();
    if (patches == 0)
    {
        return {};
    }
    const std::array<SDL_GPUTextureSamplerBinding, 7> samplers{{
        {.texture = landscape.heights(), .sampler = landscape.surfaceSampler()},
        {.texture = landscape.profile(), .sampler = landscape.profileSampler()},
        {.texture = landscape.cover(), .sampler = landscape.surfaceSampler()},
        {.texture = landscape.arcByZ(), .sampler = landscape.profileSampler()},
        {.texture = view.shadowMap, .sampler = view.shadowSampler},
        {.texture = settlements.groundAtlas(), .sampler = settlements.groundSampler()},
        {.texture = view.cloudMap, .sampler = view.cloudSampler},
    }};
    // SDL takes SDL_GPUBuffer* const*, so the pointee cannot be const.
    SDL_GPUBuffer* const townMaps = settlements.groundMaps();  // NOLINT(misc-const-correctness)
    // SDL takes SDL_GPUBuffer* const*, so the pointee cannot be const.
    SDL_GPUBuffer* const       patchBuffer = landscape.patches();  // NOLINT(misc-const-correctness)
    const SDL_GPUBufferBinding vertices{.buffer = landscape.gridVertices(), .offset = 0};
    const SDL_GPUBufferBinding indices{.buffer = landscape.gridIndices(), .offset = 0};
    const gpu::LandscapeUniforms& uniforms = landscape.uniforms();

    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexBuffers(pass, 0, &vertices, 1);
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_BindGPUVertexSamplers(pass, 0, samplers.data(), 2);
    SDL_BindGPUVertexStorageBuffers(pass, 0, &patchBuffer, 1);
    SDL_BindGPUFragmentSamplers(pass, 0, samplers.data(), static_cast<Uint32>(samplers.size()));
    SDL_BindGPUFragmentStorageBuffers(pass, 0, &townMaps, 1);
    SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    SDL_PushGPUVertexUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
    SDL_PushGPUVertexUniformData(commands, 2, &uniforms, sizeof(uniforms));
    pushHabitatFragmentUniforms(commands, view);
    SDL_PushGPUFragmentUniformData(commands, 2, &uniforms, sizeof(uniforms));
    SDL_PushGPUFragmentUniformData(commands, 3, view.shadow, sizeof(gpu::ShadowUniforms));
    SDL_DrawGPUIndexedPrimitives(pass, landscape.gridIndexCount(), patches, 0, 0, 0);
    return {.chunks    = patches,
            .triangles = static_cast<std::uint64_t>(patches) * landscape.patchTriangles()};
}

// ---- Trees -------------------------------------------------------------------------------------

TreePass::TreePass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats)
{
    const GpuShader                                     vertex   = shaders.load("tree.vert");
    const GpuShader                                     fragment = shaders.load("tree.frag");
    const std::array<SDL_GPUVertexBufferDescription, 2> buffers{{
        {.slot = 0, .pitch = sizeof(Vertex), .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX},
        {.slot = 1, .pitch = sizeof(TreeInstance), .input_rate = SDL_GPU_VERTEXINPUTRATE_INSTANCE},
    }};
    const std::array<SDL_GPUVertexAttribute, 6>         attributes{{
        {.location    = 0,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
         .offset      = offsetof(Vertex, position)},
        {.location    = 1,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
         .offset      = offsetof(Vertex, normal)},
        {.location    = 2,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2,
         .offset      = offsetof(Vertex, uv)},
        {.location    = 3,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_UINT,
         .offset      = offsetof(Vertex, material)},
        {.location    = 4,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
         .offset      = offsetof(TreeInstance, position)},
        {.location    = 5,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_UINT,
         .offset      = offsetof(TreeInstance, packed)},
    }};
    PipelineDescription                                 description = scenePipeline(formats);
    description.vertexShader                                        = vertex.get();
    description.fragmentShader                                      = fragment.get();
    description.vertexBuffers                                       = buffers;
    description.vertexAttributes                                    = attributes;
    description.cull                                                = SDL_GPU_CULLMODE_BACK;
    description.depth                                               = DepthMode::TestWrite;
    pipeline_ = createPipeline(device, description, "tree");

    const GpuShader depthOnly     = shaders.load("shadow.frag");
    description.fragmentShader    = depthOnly.get();
    description.colorFormat       = SDL_GPU_TEXTUREFORMAT_INVALID;
    description.depthFormat       = ShadowMap::kFormat;
    description.samples           = SDL_GPU_SAMPLECOUNT_1;
    description.cull              = SDL_GPU_CULLMODE_NONE;
    description.depth             = DepthMode::ShadowWrite;
    description.depthBiasConstant = 2.0F;
    description.depthBiasSlope    = 2.0F;
    shadow_                       = createPipeline(device, description, "tree shadow");
}

DrawStats TreePass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                         const GpuTrees& trees, std::span<const TreeDraw> draws,
                         const TreeRanges& ranges, const HabitatFrame& view) const
{
    if (draws.empty())
    {
        return {};
    }
    const std::array<SDL_GPUBufferBinding, 2> vertexBuffers{{
        {.buffer = trees.vertices(), .offset = 0},
        {.buffer = trees.instances(), .offset = 0},
    }};
    const SDL_GPUBufferBinding                indices{.buffer = trees.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexBuffers(pass, 0, vertexBuffers.data(),
                             static_cast<Uint32>(vertexBuffers.size()));
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    SDL_PushGPUVertexUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
    pushHabitatFragmentUniforms(commands, view);
    SDL_PushGPUFragmentUniformData(commands, 2, view.shadow, sizeof(gpu::ShadowUniforms));
    const std::array<SDL_GPUTextureSamplerBinding, 2> samplers{{
        {.texture = view.shadowMap, .sampler = view.shadowSampler},
        {.texture = view.cloudMap, .sampler = view.cloudSampler},
    }};
    SDL_BindGPUFragmentSamplers(pass, 0, samplers.data(), static_cast<Uint32>(samplers.size()));

    DrawStats stats;
    for (const TreeDraw& d : draws)
    {
        const gpu::TreeDrawUniforms uniforms{
            .origin = Vec4f(d.origin, 0.0F),
            .lod =
                Vec4f(Vec4d(d.detailed ? 1.0 : 0.0, ranges.detailEnd, ranges.blend, ranges.farEnd)),
            .fade = Vec4f(static_cast<float>(ranges.fade), 0.0F, 0.0F, 0.0F)};
        SDL_PushGPUVertexUniformData(commands, 2, &uniforms, sizeof(uniforms));
        SDL_DrawGPUIndexedPrimitives(pass, d.indexCount, d.instanceCount, d.firstIndex,
                                     d.vertexOffset, d.firstInstance);
        stats.chunks += 1;
        stats.triangles += static_cast<std::uint64_t>(d.indexCount / 3) * d.instanceCount;
    }
    return stats;
}

void TreePass::drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                          const GpuTrees& trees, std::span<const TreeDraw> draws,
                          const gpu::FrameUniforms& light) const
{
    if (draws.empty())
    {
        return;
    }
    const std::array<SDL_GPUBufferBinding, 2> vertexBuffers{{
        {.buffer = trees.vertices(), .offset = 0},
        {.buffer = trees.instances(), .offset = 0},
    }};
    const SDL_GPUBufferBinding                indices{.buffer = trees.indices(), .offset = 0};
    const gpu::HabitatUniforms                unused;
    SDL_BindGPUGraphicsPipeline(pass, shadow_.get());
    SDL_BindGPUVertexBuffers(pass, 0, vertexBuffers.data(),
                             static_cast<Uint32>(vertexBuffers.size()));
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, &light, sizeof(light));
    SDL_PushGPUVertexUniformData(commands, 1, &unused, sizeof(unused));
    for (const TreeDraw& d : draws)
    {
        // Every tree casts, at full size: no fading in the shadow map.
        const gpu::TreeDrawUniforms uniforms{.origin = Vec4f(d.origin, 0.0F),
                                             .lod    = Vec4f(0.0F, -1.0F, 1.0F, 1.0e9F),
                                             .fade   = Vec4f(1.0F, 0.0F, 0.0F, 0.0F)};
        SDL_PushGPUVertexUniformData(commands, 2, &uniforms, sizeof(uniforms));
        SDL_DrawGPUIndexedPrimitives(pass, d.indexCount, d.instanceCount, d.firstIndex,
                                     d.vertexOffset, d.firstInstance);
    }
}

// ---- Clouds ------------------------------------------------------------------------------------

CloudPass::CloudPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                     const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("cloud_shell.vert");
    const GpuShader     fragment    = shaders.load("clouds.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.depth               = DepthMode::TestOnly;
    description.blend               = BlendMode::Alpha;
    description.cull                = SDL_GPU_CULLMODE_BACK;  // the near faces, where rays enter
    below_                          = createPipeline(device, description, "cloud (below)");
    description.cull                = SDL_GPU_CULLMODE_FRONT;  // the far faces, where they leave
    inside_                         = createPipeline(device, description, "cloud (inside)");
}

BirdPass::BirdPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats)
{
    const GpuShader vertex   = shaders.load("bird.vert");
    const GpuShader fragment = shaders.load("bird.frag");
    // One instance per bird; the six vertices come from gl_VertexIndex.
    const std::array<SDL_GPUVertexBufferDescription, 1> buffers{{
        {.slot               = 0,
         .pitch              = sizeof(BirdInstance),
         .input_rate         = SDL_GPU_VERTEXINPUTRATE_INSTANCE,
         .instance_step_rate = 0},
    }};
    const std::array<SDL_GPUVertexAttribute, 4>         attributes{{
        {.location    = 0,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
         .offset      = 0},
        {.location    = 1,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT,
         .offset      = offsetof(BirdInstance, wingspanM)},
        {.location    = 2,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
         .offset      = offsetof(BirdInstance, forward)},
        {.location    = 3,
         .buffer_slot = 0,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT,
         .offset      = offsetof(BirdInstance, wingBeat)},
    }};
    PipelineDescription                                 description = scenePipeline(formats);
    description.vertexShader                                        = vertex.get();
    description.fragmentShader                                      = fragment.get();
    description.vertexBuffers                                       = buffers;
    description.vertexAttributes                                    = attributes;
    description.depth                                               = DepthMode::TestWrite;
    pipeline_ = createPipeline(device, description, "bird");
}

DrawStats BirdPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                         const GpuBirds& birds, const HabitatFrame& view) const
{
    DrawStats stats;
    if (birds.count() == 0)
    {
        return stats;
    }
    const SDL_GPUBufferBinding instances{.buffer = birds.instances(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexBuffers(pass, 0, &instances, 1);
    SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    pushHabitatFragmentUniforms(commands, view);
    SDL_DrawGPUPrimitives(pass, 6, birds.count(), 0, 0);
    stats.chunks    = 1;
    stats.triangles = 2ULL * birds.count();
    return stats;
}

RainPass::RainPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("fullscreen.vert");
    const GpuShader     fragment    = shaders.load("rain.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.depth               = DepthMode::TestOnly;  // the shader writes its own depth
    description.blend               = BlendMode::Alpha;
    pipeline_                       = createPipeline(device, description, "rain");
}

void RainPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                    const HabitatFrame& view) const
{
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    pushHabitatFragmentUniforms(commands, view);
    SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
}

void CloudPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                     const HabitatFrame& view) const
{
    const Vec4f& deck = view.habitat->cloud;
    if (deck.z <= 0.001F)
    {
        return;  // a clear sky
    }
    // Must match shellRadius() in shaders/include/clouds_shell.glsl.
    const double shell =
        static_cast<double>(deck.y) + (0.1 * static_cast<double>(deck.y - deck.x)) + 5.0;
    const bool                         inside = std::hypot(view.camera.x, view.camera.y) < shell;
    const SDL_GPUTextureSamplerBinding clouds{.texture = view.cloudMap,
                                              .sampler = view.cloudSampler};
    SDL_BindGPUGraphicsPipeline(pass, inside ? inside_.get() : below_.get());
    SDL_BindGPUFragmentSamplers(pass, 0, &clouds, 1);
    SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    SDL_PushGPUVertexUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
    pushHabitatFragmentUniforms(commands, view);
    // The side (six vertices a segment) and the two ends (three each): see cloud_shell.vert.
    SDL_DrawGPUPrimitives(pass, 12 * kCloudShellSegments, 1, 0, 0);
}

// ---- Water -------------------------------------------------------------------------------------

WaterPass::WaterPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                     const SceneFormats& formats)
{
    const GpuShader                      vertex   = shaders.load("landscape.vert");
    const GpuShader                      fragment = shaders.load("water.frag");
    const SDL_GPUVertexBufferDescription grid{
        .slot = 0, .pitch = sizeof(Vec2f), .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX};
    const SDL_GPUVertexAttribute position{
        .location = 0, .buffer_slot = 0, .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, .offset = 0};
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.vertexBuffers       = std::span(&grid, 1);
    description.vertexAttributes    = std::span(&position, 1);
    description.cull                = SDL_GPU_CULLMODE_BACK;
    description.depth               = DepthMode::TestOnly;
    description.blend               = BlendMode::Multiply;
    transmit_                       = createPipeline(device, description, "water transmittance");
    description.blend               = BlendMode::Additive;
    emit_                           = createPipeline(device, description, "water surface");
}

DrawStats WaterPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                          const GpuLandscape& landscape, const HabitatFrame& view) const
{
    const std::uint32_t patches = landscape.waterPatchCount();
    if (patches == 0)
    {
        return {};
    }
    const std::array<SDL_GPUTextureSamplerBinding, 2> samplers{{
        {.texture = landscape.heights(), .sampler = landscape.surfaceSampler()},
        {.texture = landscape.profile(), .sampler = landscape.profileSampler()},
    }};
    // SDL takes SDL_GPUBuffer* const*, so the pointee cannot be const.
    SDL_GPUBuffer* const patchBuffer = landscape.waterPatches();  // NOLINT(misc-const-correctness)
    const SDL_GPUBufferBinding vertices{.buffer = landscape.gridVertices(), .offset = 0};
    const SDL_GPUBufferBinding indices{.buffer = landscape.gridIndices(), .offset = 0};
    gpu::LandscapeUniforms     uniforms = landscape.uniforms();
    for (const bool emission : {false, true})
    {
        uniforms.mode = Vec4f(1.0F, emission ? 1.0F : 0.0F, 0.0F, 0.0F);
        SDL_BindGPUGraphicsPipeline(pass, emission ? emit_.get() : transmit_.get());
        SDL_BindGPUVertexBuffers(pass, 0, &vertices, 1);
        SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
        SDL_BindGPUVertexSamplers(pass, 0, samplers.data(), static_cast<Uint32>(samplers.size()));
        SDL_BindGPUVertexStorageBuffers(pass, 0, &patchBuffer, 1);
        SDL_BindGPUFragmentSamplers(pass, 0, samplers.data(), static_cast<Uint32>(samplers.size()));
        SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
        SDL_PushGPUVertexUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
        SDL_PushGPUVertexUniformData(commands, 2, &uniforms, sizeof(uniforms));
        pushHabitatFragmentUniforms(commands, view);
        SDL_PushGPUFragmentUniformData(commands, 2, &uniforms, sizeof(uniforms));
        SDL_DrawGPUIndexedPrimitives(pass, landscape.gridIndexCount(), patches, 0, 0, 0);
    }
    return {.chunks    = patches,
            .triangles = static_cast<std::uint64_t>(patches) * landscape.patchTriangles()};
}

// ---- Terrain -----------------------------------------------------------------------------------

TerrainPass::TerrainPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                         const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("mesh.vert");
    const GpuShader     fragment    = shaders.load("habitat.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.meshVertices        = true;
    description.cull                = SDL_GPU_CULLMODE_BACK;
    description.depth               = DepthMode::TestWrite;
    pipeline_                       = createPipeline(device, description, "terrain");
}

DrawStats TerrainPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                            const GpuWorld& world, const HabitatFrame& view) const
{
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    bindWorld(pass, world);
    pushHabitatFragmentUniforms(commands, view);
    return drawChunks(commands, pass, world, view, ChunkKind::Terrain);
}

// ---- Mirrors -----------------------------------------------------------------------------------

MirrorPass::MirrorPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                       const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("mirror.vert");
    const GpuShader     fragment    = shaders.load("mirror.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.depth               = DepthMode::TestWrite;
    pipeline_                       = createPipeline(device, description, "mirror");
}

void MirrorPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                      const HabitatFrame& view, const Mat4d& model) const
{
    const gpu::PlacementUniforms placement{
        .model = Mat4f(glm::translate(Mat4d(1.0), -view.camera) * model)};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    SDL_PushGPUVertexUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
    SDL_PushGPUVertexUniformData(commands, 2, &placement, sizeof(placement));
    pushHabitatFragmentUniforms(commands, view);
    SDL_DrawGPUPrimitives(pass, static_cast<std::uint32_t>(gpu::kMaxSunBeams) * kVerticesPerQuad, 1,
                          0, 0);
}

// ---- Glass -------------------------------------------------------------------------------------

GlassPass::GlassPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                     const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("mesh.vert");
    const GpuShader     fragment    = shaders.load("glass.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.meshVertices        = true;
    description.depth               = DepthMode::TestOnly;
    description.blend               = BlendMode::Multiply;
    transmit_                       = createPipeline(device, description, "glass transmittance");
    description.blend               = BlendMode::Additive;
    emit_                           = createPipeline(device, description, "glass emission");
}

DrawStats GlassPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                          const GpuWorld& world, const HabitatFrame& view) const
{
    DrawStats stats;
    for (const bool emission : {false, true})
    {
        const GlassUniforms mode{.mode = Vec4f(emission ? 1.0F : 0.0F, 0.0F, 0.0F, 0.0F)};
        SDL_BindGPUGraphicsPipeline(pass, emission ? emit_.get() : transmit_.get());
        bindWorld(pass, world);
        pushHabitatFragmentUniforms(commands, view);
        SDL_PushGPUFragmentUniformData(commands, 2, &mode, sizeof(mode));
        stats = drawChunks(commands, pass, world, view, ChunkKind::Glass);
    }
    return stats;
}

}  // namespace StarshipSimulator
