#include "StarshipSimulator/render/passes/town_passes.h"

#include <array>
#include <cstddef>
#include <cstdint>

#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_stdinc.h>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/GpuLandscape.h"
#include "StarshipSimulator/render/GpuPeople.h"
#include "StarshipSimulator/render/GpuProps.h"
#include "StarshipSimulator/render/GpuSettlements.h"
#include "StarshipSimulator/render/GpuTransit.h"
#include "StarshipSimulator/render/RenderTargets.h"
#include "StarshipSimulator/render/ShaderLibrary.h"
#include "StarshipSimulator/render/ShadowMap.h"
#include "StarshipSimulator/render/passes/habitat_passes.h"
#include "StarshipSimulator/render/pipeline.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kBuildingRange = 14000.0;  // m: the far side of Island Three is 8 km away

/// The shadow-map variant of a scene pipeline: depth only, into the shadow map's format.
PipelineDescription shadowVariant(PipelineDescription description, SDL_GPUShader* depthOnly)
{
    description.fragmentShader    = depthOnly;
    description.colorFormat       = SDL_GPU_TEXTUREFORMAT_INVALID;
    description.depthFormat       = ShadowMap::kFormat;
    description.samples           = SDL_GPU_SAMPLECOUNT_1;
    description.cull              = SDL_GPU_CULLMODE_NONE;
    description.depth             = DepthMode::ShadowWrite;
    description.depthBiasConstant = 2.0F;
    description.depthBiasSlope    = 2.0F;
    return description;
}

/// Binds the textures a lit surface reads (hills for their shadows, and the shadow map) and
/// pushes the fragment uniforms: frame, habitat, landscape, shadow.
void bindLighting(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                  const GpuLandscape& landscape, const HabitatFrame& view)
{
    const std::array<SDL_GPUTextureSamplerBinding, 5> samplers{{
        {.texture = landscape.heights(), .sampler = landscape.surfaceSampler()},
        {.texture = landscape.profile(), .sampler = landscape.profileSampler()},
        {.texture = landscape.arcByZ(), .sampler = landscape.profileSampler()},
        {.texture = view.shadowMap, .sampler = view.shadowSampler},
        {.texture = view.cloudMap, .sampler = view.cloudSampler},
    }};
    SDL_BindGPUFragmentSamplers(pass, 0, samplers.data(), static_cast<Uint32>(samplers.size()));
    SDL_PushGPUFragmentUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    SDL_PushGPUFragmentUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
    SDL_PushGPUFragmentUniformData(commands, 2, &landscape.uniforms(),
                                   sizeof(gpu::LandscapeUniforms));
    SDL_PushGPUFragmentUniformData(commands, 3, view.shadow, sizeof(gpu::ShadowUniforms));
}

}  // namespace

// ---- Buildings --------------------------------------------------------------------------------

BuildingPass::BuildingPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                           const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("mesh.vert");
    const GpuShader     fragment    = shaders.load("building.frag");
    const GpuShader     depthOnly   = shaders.load("shadow.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.meshVertices        = true;
    description.cull                = SDL_GPU_CULLMODE_BACK;
    description.depth               = DepthMode::TestWrite;
    pipeline_                       = createPipeline(device, description, "building");
    shadow_ =
        createPipeline(device, shadowVariant(description, depthOnly.get()), "building shadow");
}

DrawStats BuildingPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                             const GpuSettlements& settlements, const GpuLandscape& landscape,
                             const HabitatFrame& view) const
{
    DrawStats stats;
    if (settlements.empty())
    {
        return stats;
    }
    const SDL_GPUBufferBinding vertices{.buffer = settlements.vertices(), .offset = 0};
    const SDL_GPUBufferBinding indices{.buffer = settlements.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexBuffers(pass, 0, &vertices, 1);
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    bindLighting(commands, pass, landscape, view);
    settlements.forEachVisible(
        *view.frustum, view.camera, kBuildingRange, [&](const SettlementDraw& draw) {
            const Mat4d             model = glm::translate(Mat4d(1.0), draw.origin - view.camera);
            const gpu::DrawUniforms uniforms{
                .modelViewProjection = Mat4f(view.viewProjection * model), .model = Mat4f(model)};
            SDL_PushGPUVertexUniformData(commands, 0, &uniforms, sizeof(uniforms));
            SDL_DrawGPUIndexedPrimitives(pass, draw.indexCount, 1, draw.firstIndex,
                                         draw.vertexOffset, 0);
            ++stats.chunks;
            stats.triangles += draw.indexCount / 3;
        });
    return stats;
}

void BuildingPass::drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                              const GpuSettlements& settlements,
                              const Mat4d& lightFromCameraRelative, const Frustum& lightFrustum,
                              const Vec3d& camera, double reach) const
{
    if (settlements.empty())
    {
        return;
    }
    const SDL_GPUBufferBinding vertices{.buffer = settlements.vertices(), .offset = 0};
    const SDL_GPUBufferBinding indices{.buffer = settlements.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, shadow_.get());
    SDL_BindGPUVertexBuffers(pass, 0, &vertices, 1);
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    settlements.forEachVisible(lightFrustum, camera, reach, [&](const SettlementDraw& draw) {
        const Mat4d             model = glm::translate(Mat4d(1.0), draw.origin - camera);
        const gpu::DrawUniforms uniforms{
            .modelViewProjection = Mat4f(lightFromCameraRelative * model), .model = Mat4f(model)};
        SDL_PushGPUVertexUniformData(commands, 0, &uniforms, sizeof(uniforms));
        SDL_DrawGPUIndexedPrimitives(pass, draw.indexCount, 1, draw.firstIndex, draw.vertexOffset,
                                     0);
    });
}

// ---- Props ------------------------------------------------------------------------------------

TransitPass::TransitPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                         const SceneFormats& formats)
{
    const GpuShader                                     vertex    = shaders.load("transit.vert");
    const GpuShader                                     fragment  = shaders.load("transit.frag");
    const GpuShader                                     depthOnly = shaders.load("shadow.frag");
    const std::array<SDL_GPUVertexBufferDescription, 2> buffers{{
        {.slot = 0, .pitch = sizeof(Vertex), .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX},
        {.slot       = 1,
         .pitch      = sizeof(TransitInstance),
         .input_rate = SDL_GPU_VERTEXINPUTRATE_INSTANCE},
    }};
    const std::array<SDL_GPUVertexAttribute, 7>         attributes{{
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
         .offset      = offsetof(TransitInstance, position)},
        {.location    = 5,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_UINT,
         .offset      = offsetof(TransitInstance, tint)},
        {.location    = 6,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,
         .offset      = offsetof(TransitInstance, rotation)},
    }};
    PipelineDescription                                 description = scenePipeline(formats);
    description.vertexShader                                        = vertex.get();
    description.fragmentShader                                      = fragment.get();
    description.vertexBuffers                                       = buffers;
    description.vertexAttributes                                    = attributes;
    description.cull                                                = SDL_GPU_CULLMODE_BACK;
    description.depth                                               = DepthMode::TestWrite;
    pipeline_ = createPipeline(device, description, "transit");
    shadow_ = createPipeline(device, shadowVariant(description, depthOnly.get()), "transit shadow");
}

DrawStats TransitPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                            const GpuTransit& transit, const GpuLandscape& landscape,
                            const HabitatFrame& view) const
{
    DrawStats stats;
    if (transit.draws().empty())
    {
        return stats;
    }
    const std::array<SDL_GPUBufferBinding, 2> vertexBuffers{{
        {.buffer = transit.vertices(), .offset = 0},
        {.buffer = transit.instances(), .offset = 0},
    }};
    const SDL_GPUBufferBinding                indices{.buffer = transit.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexBuffers(pass, 0, vertexBuffers.data(),
                             static_cast<Uint32>(vertexBuffers.size()));
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    bindLighting(commands, pass, landscape, view);
    for (const TransitDraw& d : transit.draws())
    {
        SDL_DrawGPUIndexedPrimitives(pass, d.indexCount, d.instanceCount, d.firstIndex,
                                     d.vertexOffset, d.firstInstance);
        ++stats.chunks;
        stats.triangles += static_cast<std::uint64_t>(d.indexCount / 3) * d.instanceCount;
    }
    return stats;
}

void TransitPass::drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                             const GpuTransit& transit, const gpu::FrameUniforms& light) const
{
    if (transit.draws().empty())
    {
        return;
    }
    const std::array<SDL_GPUBufferBinding, 2> vertexBuffers{{
        {.buffer = transit.vertices(), .offset = 0},
        {.buffer = transit.instances(), .offset = 0},
    }};
    const SDL_GPUBufferBinding                indices{.buffer = transit.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, shadow_.get());
    SDL_BindGPUVertexBuffers(pass, 0, vertexBuffers.data(),
                             static_cast<Uint32>(vertexBuffers.size()));
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, &light, sizeof(light));
    for (const TransitDraw& d : transit.draws())
    {
        SDL_DrawGPUIndexedPrimitives(pass, d.indexCount, d.instanceCount, d.firstIndex,
                                     d.vertexOffset, d.firstInstance);
    }
}

PeoplePass::PeoplePass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                       const SceneFormats& formats)
{
    const GpuShader                                     vertex    = shaders.load("person.vert");
    const GpuShader                                     fragment  = shaders.load("person.frag");
    const GpuShader                                     depthOnly = shaders.load("shadow.frag");
    const std::array<SDL_GPUVertexBufferDescription, 2> buffers{{
        {.slot = 0, .pitch = sizeof(Vertex), .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX},
        {.slot       = 1,
         .pitch      = sizeof(PersonInstance),
         .input_rate = SDL_GPU_VERTEXINPUTRATE_INSTANCE},
    }};
    const std::array<SDL_GPUVertexAttribute, 10>        attributes{{
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
         .offset      = offsetof(PersonInstance, position)},
        {.location    = 5,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
         .offset      = offsetof(PersonInstance, forward)},
        {.location    = 6,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT,
         .offset      = offsetof(PersonInstance, gait)},
        {.location    = 7,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT,
         .offset      = offsetof(PersonInstance, speed)},
        {.location    = 8,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT,
         .offset      = offsetof(PersonInstance, scale)},
        {.location    = 9,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_UINT,
         .offset      = offsetof(PersonInstance, look)},
    }};
    PipelineDescription                                 description = scenePipeline(formats);
    description.vertexShader                                        = vertex.get();
    description.fragmentShader                                      = fragment.get();
    description.vertexBuffers                                       = buffers;
    description.vertexAttributes                                    = attributes;
    description.cull                                                = SDL_GPU_CULLMODE_BACK;
    description.depth                                               = DepthMode::TestWrite;
    pipeline_ = createPipeline(device, description, "person");
    shadow_ = createPipeline(device, shadowVariant(description, depthOnly.get()), "person shadow");
}

DrawStats PeoplePass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                           const GpuPeople& people, const GpuLandscape& landscape,
                           const HabitatFrame& view) const
{
    DrawStats stats;
    if (people.count() == 0)
    {
        return stats;
    }
    const std::array<SDL_GPUBufferBinding, 2> vertexBuffers{{
        {.buffer = people.vertices(), .offset = 0},
        {.buffer = people.instances(), .offset = 0},
    }};
    const SDL_GPUBufferBinding                indices{.buffer = people.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexBuffers(pass, 0, vertexBuffers.data(),
                             static_cast<Uint32>(vertexBuffers.size()));
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    SDL_PushGPUVertexUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
    bindLighting(commands, pass, landscape, view);
    SDL_DrawGPUIndexedPrimitives(pass, people.indexCount(), people.count(), 0, 0, 0);
    stats.chunks    = 1;
    stats.triangles = static_cast<std::uint64_t>(people.indexCount() / 3) * people.count();
    return stats;
}

void PeoplePass::drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                            const GpuPeople& people, const gpu::FrameUniforms& light,
                            const gpu::HabitatUniforms& habitat) const
{
    if (people.count() == 0)
    {
        return;
    }
    const std::array<SDL_GPUBufferBinding, 2> vertexBuffers{{
        {.buffer = people.vertices(), .offset = 0},
        {.buffer = people.instances(), .offset = 0},
    }};
    const SDL_GPUBufferBinding                indices{.buffer = people.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, shadow_.get());
    SDL_BindGPUVertexBuffers(pass, 0, vertexBuffers.data(),
                             static_cast<Uint32>(vertexBuffers.size()));
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, &light, sizeof(light));
    SDL_PushGPUVertexUniformData(commands, 1, &habitat, sizeof(habitat));
    SDL_DrawGPUIndexedPrimitives(pass, people.indexCount(), people.count(), 0, 0, 0);
}

PropPass::PropPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats)
{
    const GpuShader                                     vertex    = shaders.load("prop.vert");
    const GpuShader                                     fragment  = shaders.load("prop.frag");
    const GpuShader                                     depthOnly = shaders.load("shadow.frag");
    const std::array<SDL_GPUVertexBufferDescription, 2> buffers{{
        {.slot = 0, .pitch = sizeof(Vertex), .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX},
        {.slot = 1, .pitch = sizeof(PropInstance), .input_rate = SDL_GPU_VERTEXINPUTRATE_INSTANCE},
    }};
    const std::array<SDL_GPUVertexAttribute, 7>         attributes{{
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
         .offset      = offsetof(PropInstance, position)},
        {.location    = 5,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_UINT,
         .offset      = offsetof(PropInstance, tint)},
        {.location    = 6,
         .buffer_slot = 1,
         .format      = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,
         .offset      = offsetof(PropInstance, rotation)},
    }};
    PipelineDescription                                 description = scenePipeline(formats);
    description.vertexShader                                        = vertex.get();
    description.fragmentShader                                      = fragment.get();
    description.vertexBuffers                                       = buffers;
    description.vertexAttributes                                    = attributes;
    description.cull                                                = SDL_GPU_CULLMODE_BACK;
    description.depth                                               = DepthMode::TestWrite;
    pipeline_ = createPipeline(device, description, "prop");
    shadow_   = createPipeline(device, shadowVariant(description, depthOnly.get()), "prop shadow");
}

DrawStats PropPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                         const GpuProps& props, const GpuLandscape& landscape,
                         const HabitatFrame& view) const
{
    DrawStats stats;
    if (props.draws().empty())
    {
        return stats;
    }
    const std::array<SDL_GPUBufferBinding, 2> vertexBuffers{{
        {.buffer = props.vertices(), .offset = 0},
        {.buffer = props.instances(), .offset = 0},
    }};
    const SDL_GPUBufferBinding                indices{.buffer = props.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexBuffers(pass, 0, vertexBuffers.data(),
                             static_cast<Uint32>(vertexBuffers.size()));
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    bindLighting(commands, pass, landscape, view);
    for (const PropDraw& d : props.draws())
    {
        SDL_DrawGPUIndexedPrimitives(pass, d.indexCount, d.instanceCount, d.firstIndex,
                                     d.vertexOffset, d.firstInstance);
        ++stats.chunks;
        stats.triangles += static_cast<std::uint64_t>(d.indexCount / 3) * d.instanceCount;
    }
    return stats;
}

void PropPass::drawShadow(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                          const GpuProps& props, const gpu::FrameUniforms& light) const
{
    if (props.draws().empty())
    {
        return;
    }
    const std::array<SDL_GPUBufferBinding, 2> vertexBuffers{{
        {.buffer = props.vertices(), .offset = 0},
        {.buffer = props.instances(), .offset = 0},
    }};
    const SDL_GPUBufferBinding                indices{.buffer = props.indices(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, shadow_.get());
    SDL_BindGPUVertexBuffers(pass, 0, vertexBuffers.data(),
                             static_cast<Uint32>(vertexBuffers.size()));
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, &light, sizeof(light));
    for (const PropDraw& d : props.draws())
    {
        SDL_DrawGPUIndexedPrimitives(pass, d.indexCount, d.instanceCount, d.firstIndex,
                                     d.vertexOffset, d.firstInstance);
    }
}

}  // namespace StarshipSimulator
