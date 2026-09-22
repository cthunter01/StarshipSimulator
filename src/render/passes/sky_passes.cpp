#include "StarshipSimulator/render/passes/sky_passes.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/assets/assets.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/RenderTargets.h"
#include "StarshipSimulator/render/ShaderLibrary.h"
#include "StarshipSimulator/render/passes/habitat_passes.h"
#include "StarshipSimulator/render/pipeline.h"
#include "StarshipSimulator/render/texture.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

namespace
{

constexpr std::uint32_t kVerticesPerQuad = 6;

GpuTexture solidColor(SDL_GPUDevice* device, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    const std::array<std::byte, 4> pixel{std::byte{r}, std::byte{g}, std::byte{b}, std::byte{255}};
    return createSolidTexture(device, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM_SRGB, pixel);
}

GpuTexture colorTexture(SDL_GPUDevice* device, const assets::Image8& image)
{
    return createTexture(device, {.width   = static_cast<std::uint32_t>(image.width),
                                  .height  = static_cast<std::uint32_t>(image.height),
                                  .format  = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM_SRGB,
                                  .pixels  = std::as_bytes(std::span(image.rgba)),
                                  .mipmaps = true});
}

}  // namespace

// ---- Textures ----------------------------------------------------------------------------------

SkyTextures::SkyTextures(SDL_GPUDevice* device)
  : device_(device),
    milkyWay_(solidColor(device, 0, 0, 0)),
    earthDay_(solidColor(device, 90, 110, 150)),
    earthNight_(solidColor(device, 0, 0, 0)),
    moon_(solidColor(device, 150, 145, 140)),
    black_(solidColor(device, 0, 0, 0)),
    sampler_(createMapSampler(device))
{
}

void SkyTextures::setMilkyWay(const assets::HalfImage& image)
{
    milkyWay_ = createTexture(device_, {.width   = static_cast<std::uint32_t>(image.width),
                                        .height  = static_cast<std::uint32_t>(image.height),
                                        .format  = SDL_GPU_TEXTUREFORMAT_R16G16B16A16_FLOAT,
                                        .pixels  = std::as_bytes(std::span(image.rgba)),
                                        .mipmaps = true});
}

void SkyTextures::setEarth(const assets::Image8& day, const assets::Image8& night)
{
    earthDay_   = colorTexture(device_, day);
    earthNight_ = colorTexture(device_, night);
}

void SkyTextures::setMoon(const assets::Image8& surface)
{
    moon_ = colorTexture(device_, surface);
}

// ---- Milky Way ---------------------------------------------------------------------------------

MilkyWayPass::MilkyWayPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                           const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("fullscreen.vert");
    const GpuShader     fragment    = shaders.load("milkyway.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    pipeline_                       = createPipeline(device, description, "milky way");
}

void MilkyWayPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                        const gpu::FrameUniforms& frame, const gpu::SkyUniforms& sky,
                        const SkyTextures& textures) const
{
    const SDL_GPUTextureSamplerBinding binding{.texture = textures.milkyWay(),
                                               .sampler = textures.sampler()};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUFragmentSamplers(pass, 0, &binding, 1);
    SDL_PushGPUFragmentUniformData(commands, 0, &frame, sizeof(frame));
    SDL_PushGPUFragmentUniformData(commands, 1, &sky, sizeof(sky));
    SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
}

// ---- Planets -----------------------------------------------------------------------------------

PlanetPass::PlanetPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                       const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("planets.vert");
    const GpuShader     fragment    = shaders.load("stars.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.blend               = BlendMode::Additive;
    pipeline_                       = createPipeline(device, description, "planet");
}

void PlanetPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                      const gpu::FrameUniforms& frame, const gpu::SkyUniforms& sky,
                      const gpu::PlanetUniforms& planets) const
{
    if (planets.count.x < 0.5F)
    {
        return;
    }
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_PushGPUVertexUniformData(commands, 0, &frame, sizeof(frame));
    SDL_PushGPUVertexUniformData(commands, 1, &sky, sizeof(sky));
    SDL_PushGPUVertexUniformData(commands, 2, &planets, sizeof(planets));
    SDL_DrawGPUPrimitives(pass, static_cast<std::uint32_t>(gpu::kMaxPlanets) * kVerticesPerQuad, 1,
                          0, 0);
}

// ---- Earth and Moon ----------------------------------------------------------------------------

BodyPass::BodyPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("body.vert");
    const GpuShader     fragment    = shaders.load("body.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.blend               = BlendMode::Alpha;
    pipeline_                       = createPipeline(device, description, "body");
}

void BodyPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                    const gpu::FrameUniforms& frame, const gpu::BodyUniforms& body,
                    SDL_GPUTexture* dayMap, SDL_GPUTexture* nightMap, SDL_GPUSampler* sampler) const
{
    const std::array<SDL_GPUTextureSamplerBinding, 2> bindings{{
        {.texture = dayMap, .sampler = sampler},
        {.texture = nightMap, .sampler = sampler},
    }};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUFragmentSamplers(pass, 0, bindings.data(), bindings.size());
    SDL_PushGPUVertexUniformData(commands, 0, &frame, sizeof(frame));
    SDL_PushGPUVertexUniformData(commands, 1, &body, sizeof(body));
    SDL_PushGPUFragmentUniformData(commands, 0, &frame, sizeof(frame));
    SDL_PushGPUFragmentUniformData(commands, 1, &body, sizeof(body));
    SDL_DrawGPUPrimitives(pass, kVerticesPerQuad, 1, 0, 0);
}

// ---- Partner hull ------------------------------------------------------------------------------

HullPass::HullPass(SDL_GPUDevice* device, const ShaderLibrary& shaders, const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("mesh.vert");
    const GpuShader     fragment    = shaders.load("hull.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.meshVertices        = true;
    description.cull                = SDL_GPU_CULLMODE_BACK;
    description.depth               = DepthMode::TestWrite;
    pipeline_                       = createPipeline(device, description, "hull");
}

DrawStats HullPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                         const GpuMesh& hull, const HabitatFrame& view, const Mat4d& model) const
{
    if (!hull.valid())
    {
        return {};
    }
    const Mat4d                relative = glm::translate(Mat4d(1.0), -view.camera) * model;
    const gpu::DrawUniforms    draw{.modelViewProjection = Mat4f(view.viewProjection * relative),
                                    .model               = Mat4f(relative)};
    const SDL_GPUBufferBinding vertices{.buffer = hull.vertices.get(), .offset = 0};
    const SDL_GPUBufferBinding indices{.buffer = hull.indices.get(), .offset = 0};
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    SDL_BindGPUVertexBuffers(pass, 0, &vertices, 1);
    SDL_BindGPUIndexBuffer(pass, &indices, SDL_GPU_INDEXELEMENTSIZE_32BIT);
    SDL_PushGPUVertexUniformData(commands, 0, &draw, sizeof(draw));
    SDL_PushGPUFragmentUniformData(commands, 0, view.frame, sizeof(gpu::FrameUniforms));
    SDL_PushGPUFragmentUniformData(commands, 1, view.habitat, sizeof(gpu::HabitatUniforms));
    SDL_DrawGPUIndexedPrimitives(pass, hull.indexCount, 1, 0, 0, 0);
    return {.chunks = 1, .triangles = hull.indexCount / 3};
}

}  // namespace StarshipSimulator
