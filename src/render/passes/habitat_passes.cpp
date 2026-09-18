#include "StarshipSimulator/render/passes/habitat_passes.h"

#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/gpu_world.h"
#include "StarshipSimulator/render/pipeline.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shader_library.h"
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
