#include "StarshipSimulator/render/passes/MarkerPass.h"

#include <cstdint>
#include <span>
#include <vector>

#include <SDL3/SDL_gpu.h>

#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/render/GpuHandle.h"
#include "StarshipSimulator/render/RenderTargets.h"
#include "StarshipSimulator/render/ShaderLibrary.h"
#include "StarshipSimulator/render/pipeline.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

MarkerPass::MarkerPass(SDL_GPUDevice* device, const ShaderLibrary& shaders,
                       const SceneFormats& formats)
{
    const GpuShader     vertex      = shaders.load("mesh.vert");
    const GpuShader     fragment    = shaders.load("marker.frag");
    PipelineDescription description = scenePipeline(formats);
    description.vertexShader        = vertex.get();
    description.fragmentShader      = fragment.get();
    description.meshVertices        = true;
    description.cull                = SDL_GPU_CULLMODE_BACK;
    description.depth               = DepthMode::TEST_WRITE;
    pipeline_                       = createPipeline(device, description, "marker");

    // Both shapes in one pair of buffers.
    const CpuMesh              box    = makeBox(Vec3f(1.0F));
    const CpuMesh              sphere = makeSphere(1.0F, 24);
    std::vector<Vertex>        vertices(box.vertices);
    std::vector<std::uint32_t> indices(box.indices);
    vertices.insert(vertices.end(), sphere.vertices.begin(), sphere.vertices.end());
    indices.insert(indices.end(), sphere.indices.begin(), sphere.indices.end());
    box_      = {.firstIndex   = 0,
                 .indexCount   = static_cast<std::uint32_t>(box.indices.size()),
                 .vertexOffset = 0};
    sphere_   = {.firstIndex   = static_cast<std::uint32_t>(box.indices.size()),
                 .indexCount   = static_cast<std::uint32_t>(sphere.indices.size()),
                 .vertexOffset = static_cast<std::int32_t>(box.vertices.size())};
    vertices_ = createBufferWithData(device, SDL_GPU_BUFFERUSAGE_VERTEX,
                                     std::as_bytes(std::span(vertices)));
    indices_ =
        createBufferWithData(device, SDL_GPU_BUFFERUSAGE_INDEX, std::as_bytes(std::span(indices)));
}

void MarkerPass::draw(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                      const Mat4d& viewProjection, const Vec3d& cameraPosition,
                      std::span<const Marker> markers) const
{
    if (markers.empty())
    {
        return;
    }
    SDL_BindGPUGraphicsPipeline(pass, pipeline_.get());
    const SDL_GPUBufferBinding vertexBinding{.buffer = vertices_.get(), .offset = 0};
    const SDL_GPUBufferBinding indexBinding{.buffer = indices_.get(), .offset = 0};
    SDL_BindGPUVertexBuffers(pass, 0, &vertexBinding, 1);
    SDL_BindGPUIndexBuffer(pass, &indexBinding, SDL_GPU_INDEXELEMENTSIZE_32BIT);

    for (const Marker& marker : markers)
    {
        // Camera-relative translation in double precision, then rounded to float once.
        const Mat4d model = glm::translate(Mat4d(1.0), marker.position - cameraPosition) *
                            glm::scale(Mat4d(1.0), Vec3d(marker.halfExtents));
        const gpu::DrawUniforms draw{
            .modelViewProjection = Mat4f(viewProjection * model),
            .model               = Mat4f(model),
        };
        const gpu::MaterialUniforms material{
            .color    = Vec4f(marker.color, 1.0F),
            .emission = Vec4f(marker.emission, 0.0F),
        };
        const Range& range = marker.shape == MarkerShape::SPHERE ? sphere_ : box_;
        SDL_PushGPUVertexUniformData(commands, 0, &draw, sizeof(draw));
        SDL_PushGPUFragmentUniformData(commands, 0, &material, sizeof(material));
        SDL_DrawGPUIndexedPrimitives(pass, range.indexCount, 1, range.firstIndex,
                                     range.vertexOffset, 0);
    }
}

}  // namespace StarshipSimulator
