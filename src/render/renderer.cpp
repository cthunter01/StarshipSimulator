#include "StarshipSimulator/render/renderer.h"

#include <imgui.h>
#include <imgui_impl_sdlgpu3.h>

#include <array>
#include <cstdint>
#include <exception>
#include <expected>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_pixels.h>
#include <SDL3/SDL_surface.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/frustum.h"
#include "StarshipSimulator/core/gpu_abi/uniforms.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/buildings.h"
#include "StarshipSimulator/core/procgen/clouds.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/render/gpu_device.h"
#include "StarshipSimulator/render/gpu_handles.h"
#include "StarshipSimulator/render/gpu_props.h"
#include "StarshipSimulator/render/gpu_settlements.h"
#include "StarshipSimulator/render/gpu_trees.h"
#include "StarshipSimulator/render/passes/habitat_passes.h"
#include "StarshipSimulator/render/passes/marker_pass.h"
#include "StarshipSimulator/render/passes/sky_passes.h"
#include "StarshipSimulator/render/passes/tonemap_pass.h"
#include "StarshipSimulator/render/passes/town_passes.h"
#include "StarshipSimulator/render/render_targets.h"
#include "StarshipSimulator/render/shadow_map.h"
#include "StarshipSimulator/render/texture.h"
#include "StarshipSimulator/render/upload.h"

namespace StarshipSimulator
{

struct Renderer::Passes
{
    MilkyWayPass  milkyWay;
    StarPass      stars;
    PlanetPass    planets;
    BodyPass      bodies;
    HullPass      hull;
    LandscapePass landscape;
    CloudPass     clouds;
    BirdPass      birds;
    RainPass      rain;
    WaterPass     water;
    TreePass      trees;
    BuildingPass  buildings;
    PropPass      props;
    PeoplePass    people;
    TransitPass   transit;
    TerrainPass   terrain;
    MirrorPass    mirrors;
    GlassPass     glass;
    MarkerPass    markers;
    TonemapPass   tonemap;
};

namespace
{

constexpr std::uint32_t kBytesPerPixel = 4;

constexpr std::array<std::uint8_t, 2> kClearSky{0, 0};

/// Linear filtering with mipmaps, repeating both ways: the cloud map wraps round the habitat and
/// along it.
GpuSampler createWrappingSampler(SDL_GPUDevice* device)
{
    const SDL_GPUSamplerCreateInfo info{
        .min_filter     = SDL_GPU_FILTER_LINEAR,
        .mag_filter     = SDL_GPU_FILTER_LINEAR,
        .mipmap_mode    = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR,
        .address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_REPEAT,
        .address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_REPEAT,
        .address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT,
        .max_lod        = 1000.0F,
    };
    GpuSampler sampler(device, SDL_CreateGPUSampler(device, &info));
    if (!sampler.valid())
    {
        throw std::runtime_error(std::format("Cannot create a sampler: {}", SDL_GetError()));
    }
    return sampler;
}
constexpr double kPropRange    = 400.0;   // m: props farther away are too small to see
constexpr double kTransitRange = 2500.0;  // the track runs the length of the valley

std::optional<SDL_PixelFormat> pixelFormatOf(SDL_GPUTextureFormat format)
{
    switch (format)
    {
        case SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM:
        case SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM_SRGB:
            return SDL_PIXELFORMAT_BGRA32;
        case SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM:
        case SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM_SRGB:
            return SDL_PIXELFORMAT_RGBA32;
        default:
            return std::nullopt;
    }
}

std::expected<std::filesystem::path, std::string> savePng(const std::filesystem::path& path,
                                                          SDL_PixelFormat format, void* pixels,
                                                          std::uint32_t width, std::uint32_t height)
{
    std::error_code error;
    if (path.has_parent_path())
    {
        std::filesystem::create_directories(path.parent_path(), error);
    }
    SDL_Surface* surface =
        SDL_CreateSurfaceFrom(static_cast<int>(width), static_cast<int>(height), format, pixels,
                              static_cast<int>(width * kBytesPerPixel));
    if (surface == nullptr)
    {
        return std::unexpected(std::format("Cannot wrap the screenshot: {}", SDL_GetError()));
    }
    const bool saved = SDL_SavePNG(surface, path.string().c_str());
    SDL_DestroySurface(surface);
    if (!saved)
    {
        return std::unexpected(std::format("Cannot save {}: {}", path.string(), SDL_GetError()));
    }
    return path;
}

}  // namespace

Renderer::Renderer(GpuDevice& device, std::filesystem::path shaderDirectory,
                   std::vector<GpuStar> stars)
  : device_(&device),
    stars_(std::move(stars)),
    shaders_(device.get(), std::move(shaderDirectory)),
    targets_(device.get(), chooseSceneFormats(device.get(), SDL_GPU_SAMPLECOUNT_4)),
    skyTextures_(device.get()),
    shadowMap_(device.get(), kShadowMapResolution),
    // Until a habitat is generated: a clear sky.
    cloudMap_(createSolidTexture(device.get(), SDL_GPU_TEXTUREFORMAT_R8G8_UNORM,
                                 std::as_bytes(std::span(kClearSky)))),
    cloudSampler_(createWrappingSampler(device.get())),
    noSettlements_(std::make_unique<GpuSettlements>(device.get(), std::span<const SettlementMesh>(),
                                                    Settlements{})),
    passes_(createPasses())
{
}

Renderer::~Renderer()
{
    SDL_WaitForGPUIdle(device_->get());
}

std::unique_ptr<Renderer::Passes> Renderer::createPasses() const
{
    SDL_GPUDevice*      device  = device_->get();
    const SceneFormats& formats = targets_.formats();
    return std::make_unique<Passes>(Passes{
        .milkyWay  = MilkyWayPass(device, shaders_, formats),
        .stars     = StarPass(device, shaders_, formats, stars_),
        .planets   = PlanetPass(device, shaders_, formats),
        .bodies    = BodyPass(device, shaders_, formats),
        .hull      = HullPass(device, shaders_, formats),
        .landscape = LandscapePass(device, shaders_, formats),
        .clouds    = CloudPass(device, shaders_, formats),
        .birds     = BirdPass(device, shaders_, formats),
        .rain      = RainPass(device, shaders_, formats),
        .water     = WaterPass(device, shaders_, formats),
        .trees     = TreePass(device, shaders_, formats),
        .buildings = BuildingPass(device, shaders_, formats),
        .props     = PropPass(device, shaders_, formats),
        .people    = PeoplePass(device, shaders_, formats),
        .transit   = TransitPass(device, shaders_, formats),
        .terrain   = TerrainPass(device, shaders_, formats),
        .mirrors   = MirrorPass(device, shaders_, formats),
        .glass     = GlassPass(device, shaders_, formats),
        .markers   = MarkerPass(device, shaders_, formats),
        .tonemap   = TonemapPass(device, shaders_, device_->swapchainFormat()),
    });
}

std::expected<void, std::string> Renderer::reloadShaders()
{
    try
    {
        auto passes = createPasses();
        SDL_WaitForGPUIdle(device_->get());
        passes_ = std::move(passes);
        return {};
    }
    catch (const std::exception& e)
    {
        return std::unexpected(std::string(e.what()));
    }
}

void Renderer::setStars(std::vector<GpuStar> stars)
{
    stars_         = std::move(stars);
    passes_->stars = StarPass(device_->get(), shaders_, targets_.formats(), stars_);
}

void Renderer::setSkyImages(const SkyImages& images)
{
    if (images.milkyWay)
    {
        skyTextures_.setMilkyWay(*images.milkyWay);
    }
    if (images.earthDay && images.earthNight)
    {
        skyTextures_.setEarth(*images.earthDay, *images.earthNight);
    }
    if (images.moon)
    {
        skyTextures_.setMoon(*images.moon);
    }
}

void Renderer::setHull(const CpuMesh& hull)
{
    hull_ = uploadMesh(device_->get(), hull);
}

void Renderer::setCloudMap(const CloudMap& clouds)
{
    if (clouds.width == 0 || clouds.height == 0)
    {
        return;
    }
    cloudMap_ = createTexture(device_->get(), {.width   = clouds.width,
                                               .height  = clouds.height,
                                               .format  = SDL_GPU_TEXTUREFORMAT_R8G8_UNORM,
                                               .pixels  = std::as_bytes(std::span(clouds.texels)),
                                               .mipmaps = true});
}

FrameResult Renderer::renderFrame(const SceneView& view, const FrameOptions& options)
{
    SDL_GPUCommandBuffer* commands = SDL_AcquireGPUCommandBuffer(device_->get());
    if (commands == nullptr)
    {
        throw std::runtime_error(
            std::format("Cannot acquire a command buffer: {}", SDL_GetError()));
    }
    SDL_GPUTexture* swapchain = nullptr;
    std::uint32_t   width     = 0;
    std::uint32_t   height    = 0;
    if (!SDL_WaitAndAcquireGPUSwapchainTexture(commands, device_->window(), &swapchain, &width,
                                               &height))
    {
        SDL_CancelGPUCommandBuffer(commands);
        throw std::runtime_error(std::format("Cannot acquire the swapchain: {}", SDL_GetError()));
    }

    FrameResult result{.width = width, .height = height};
    if (swapchain == nullptr || width == 0 || height == 0)
    {
        // Minimized or occluded: nothing to draw, but the command buffer must still be submitted.
        SDL_SubmitGPUCommandBuffer(commands);
        return result;
    }

    targets_.resize(width, height);
    if (options.ui != nullptr)
    {
        ImGui_ImplSDLGPU3_PrepareDrawData(options.ui, commands);  // copy pass: before render passes
    }
    drawScene(commands, view, width, height, result);
    drawDisplay(commands, swapchain, view, options.ui);
    result.presented = true;

    if (options.screenshot)
    {
        result.screenshot =
            captureAndSubmit(commands, view, *options.screenshot,
                             options.screenshotIncludesUi ? options.ui : nullptr, width, height);
    }
    else if (!SDL_SubmitGPUCommandBuffer(commands))
    {
        throw std::runtime_error(std::format("Cannot submit the frame: {}", SDL_GetError()));
    }
    return result;
}

void Renderer::drawScene(SDL_GPUCommandBuffer* commands, const SceneView& view, std::uint32_t width,
                         std::uint32_t height, FrameResult& result)
{
    const gpu::FrameUniforms frame =
        gpu::makeFrameUniforms(view.camera, width, height, view.animationSeconds);
    const Mat4d viewProjection = cameraRelativeViewProjection(
        view.camera, static_cast<double>(width) / static_cast<double>(height));
    const Frustum      frustum(viewProjection);
    const HabitatFrame habitatFrame{.frame          = &frame,
                                    .habitat        = &view.habitat,
                                    .viewProjection = viewProjection,
                                    .camera         = view.camera.position,
                                    .frustum        = &frustum,
                                    .shadow         = &view.shadow,
                                    .shadowMap      = shadowMap_.texture(),
                                    .shadowSampler  = shadowMap_.sampler(),
                                    .cloudMap       = cloudMap_.get(),
                                    .cloudSampler   = cloudSampler_.get()};

    const bool                   msaa = targets_.multisampled();
    const SDL_GPUColorTargetInfo color{
        .texture               = targets_.color(),
        .clear_color           = {.r = 0.0F, .g = 0.0F, .b = 0.0F, .a = 1.0F},
        .load_op               = SDL_GPU_LOADOP_CLEAR,
        .store_op              = msaa ? SDL_GPU_STOREOP_RESOLVE : SDL_GPU_STOREOP_STORE,
        .resolve_texture       = msaa ? targets_.resolved() : nullptr,
        .cycle                 = true,
        .cycle_resolve_texture = msaa,
    };
    const SDL_GPUDepthStencilTargetInfo depth{
        .texture          = targets_.depth(),
        .clear_depth      = 0.0F,  // reverse-Z: 0 is infinitely far
        .load_op          = SDL_GPU_LOADOP_CLEAR,
        .store_op         = SDL_GPU_STOREOP_DONT_CARE,
        .stencil_load_op  = SDL_GPU_LOADOP_DONT_CARE,
        .stencil_store_op = SDL_GPU_STOREOP_DONT_CARE,
        .cycle            = true,
    };
    uploadPerFrameData(commands, view, frustum);
    drawShadows(commands, view, frame);
    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(commands, &color, 1, &depth);

    // Outside, from far to near: the sky at infinity, then the partner cylinder.
    passes_->milkyWay.draw(commands, pass, frame, view.sky, skyTextures_);
    passes_->stars.draw(commands, pass, frame, view.sky);
    passes_->planets.draw(commands, pass, frame, view.sky, view.planets);
    for (const BodyDraw& body : view.bodies)
    {
        const bool earth = body.textures == BodyTextures::Earth;
        passes_->bodies.draw(commands, pass, frame, body.uniforms,
                             earth ? skyTextures_.earthDay() : skyTextures_.moon(),
                             earth ? skyTextures_.earthNight() : skyTextures_.black(),
                             skyTextures_.sampler());
    }
    DrawStats partner;
    if (view.partner && view.world != nullptr)
    {
        partner = passes_->hull.draw(commands, pass, hull_, habitatFrame, *view.partner);
        passes_->mirrors.draw(commands, pass, habitatFrame, *view.partner);
    }

    // Inside: the land, our mirrors seen through the windows, birds, markers, water, the window
    // glass, then the clouds and rain in front of it all.
    if (view.world != nullptr)
    {
        const DrawStats terrain = drawLand(commands, pass, view, habitatFrame);
        passes_->mirrors.draw(commands, pass, habitatFrame, Mat4d(1.0));
        if (view.birds != nullptr)
        {
            passes_->birds.draw(commands, pass, *view.birds, habitatFrame);
        }
        passes_->markers.draw(commands, pass, viewProjection, view.camera.position, view.markers);
        if (view.landscape != nullptr)
        {
            passes_->water.draw(commands, pass, *view.landscape, habitatFrame);
        }
        const DrawStats glass = passes_->glass.draw(commands, pass, *view.world, habitatFrame);
        // The clouds and the rain are inside the glass, in front of everything they cover.
        passes_->clouds.draw(commands, pass, habitatFrame);
        passes_->rain.draw(commands, pass, habitatFrame);
        result.chunksDrawn    = terrain.chunks + glass.chunks + partner.chunks;
        result.trianglesDrawn = terrain.triangles + glass.triangles + partner.triangles;
    }
    else
    {
        passes_->markers.draw(commands, pass, viewProjection, view.camera.position, view.markers);
    }
    SDL_EndGPURenderPass(pass);
}

DrawStats Renderer::drawLand(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass,
                             const SceneView& view, const HabitatFrame& habitatFrame)
{
    DrawStats  stats = passes_->terrain.draw(commands, pass, *view.world, habitatFrame);
    const auto add   = [&](const DrawStats& more) {
        stats.chunks += more.chunks;
        stats.triangles += more.triangles;
    };
    const GpuSettlements& settlements =
        view.settlements != nullptr ? *view.settlements : *noSettlements_;
    if (view.landscape != nullptr)
    {
        add(passes_->landscape.draw(commands, pass, *view.landscape, settlements, habitatFrame));
        add(passes_->buildings.draw(commands, pass, settlements, *view.landscape, habitatFrame));
        if (view.props != nullptr)
        {
            add(passes_->props.draw(commands, pass, *view.props, *view.landscape, habitatFrame));
        }
        if (view.people != nullptr)
        {
            add(passes_->people.draw(commands, pass, *view.people, *view.landscape, habitatFrame));
        }
        if (view.transit != nullptr)
        {
            add(passes_->transit.draw(commands, pass, *view.transit, *view.landscape,
                                      habitatFrame));
        }
    }
    if (view.trees != nullptr)
    {
        const TreeRanges            ranges;
        const std::vector<TreeDraw> draws =
            view.trees->select(view.camera.position, *habitatFrame.frustum, ranges);
        add(passes_->trees.draw(commands, pass, *view.trees, draws, ranges, habitatFrame));
    }
    return stats;
}

/// Everything that changes every frame: which terrain patches are in view, and where the props,
/// the birds, the people and the trams are now. All of it in one copy pass before the drawing.
void Renderer::uploadPerFrameData(SDL_GPUCommandBuffer* commands, const SceneView& view,
                                  const Frustum& frustum)
{
    if (view.landscape == nullptr && view.props == nullptr && view.birds == nullptr &&
        view.people == nullptr && view.transit == nullptr)
    {
        return;
    }
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(commands);
    if (view.landscape != nullptr)
    {
        view.landscape->prepare(copy, view.camera.position, frustum);
    }
    if (view.props != nullptr)
    {
        view.props->prepare(copy, view.camera.position, view.propPoses, kPropRange);
    }
    if (view.birds != nullptr)
    {
        view.birds->prepare(copy, view.camera.position, view.birdPoses);
    }
    if (view.people != nullptr)
    {
        view.people->prepare(copy, view.camera.position, view.peoplePoses);
    }
    if (view.transit != nullptr)
    {
        view.transit->prepare(copy, view.camera.position, frustum, view.trams, kTransitRange);
    }
    SDL_EndGPUCopyPass(copy);
}

void Renderer::drawShadows(SDL_GPUCommandBuffer* commands, const SceneView& view,
                           const gpu::FrameUniforms& frame)
{
    // Always cleared, so receivers can sample it even when no trees cast shadows.
    const SDL_GPUDepthStencilTargetInfo target{
        .texture          = shadowMap_.texture(),
        .clear_depth      = 1.0F,
        .load_op          = SDL_GPU_LOADOP_CLEAR,
        .store_op         = SDL_GPU_STOREOP_STORE,
        .stencil_load_op  = SDL_GPU_LOADOP_DONT_CARE,
        .stencil_store_op = SDL_GPU_STOREOP_DONT_CARE,
        .cycle            = true,
    };
    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(commands, nullptr, 0, &target);
    if (view.shadow.params.x > 0.5F)
    {
        gpu::FrameUniforms light = frame;
        light.viewProjection     = view.shadow.lightFromCameraRelative;
        // Things just outside the box can still throw shadows into it.
        const double reach = static_cast<double>(view.shadow.params.z) * kShadowMapResolution;
        if (view.trees != nullptr)
        {
            passes_->trees.drawShadow(commands, pass, *view.trees,
                                      view.trees->selectNear(view.camera.position, reach), light);
        }
        if (view.settlements != nullptr)
        {
            const Mat4d   lightMatrix(view.shadow.lightFromCameraRelative);
            const Frustum lightFrustum(lightMatrix);
            passes_->buildings.drawShadow(commands, pass, *view.settlements, lightMatrix,
                                          lightFrustum, view.camera.position, reach);
        }
        if (view.props != nullptr)
        {
            passes_->props.drawShadow(commands, pass, *view.props, light);
        }
        if (view.people != nullptr)
        {
            passes_->people.drawShadow(commands, pass, *view.people, light, view.habitat);
        }
        if (view.transit != nullptr)
        {
            passes_->transit.drawShadow(commands, pass, *view.transit, light);
        }
    }
    SDL_EndGPURenderPass(pass);
}

void Renderer::drawDisplay(SDL_GPUCommandBuffer* commands, SDL_GPUTexture* target,
                           const SceneView& view, ImDrawData* ui)
{
    const SDL_GPUColorTargetInfo color{
        .texture  = target,
        .load_op  = SDL_GPU_LOADOP_DONT_CARE,  // the tonemap pass covers every pixel
        .store_op = SDL_GPU_STOREOP_STORE,
    };
    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(commands, &color, 1, nullptr);
    passes_->tonemap.draw(commands, pass, targets_.resolved(), view.exposure, view.grade);
    if (ui != nullptr)
    {
        ImGui_ImplSDLGPU3_RenderDrawData(ui, commands, pass);
    }
    SDL_EndGPURenderPass(pass);
}

std::expected<std::filesystem::path, std::string> Renderer::captureAndSubmit(
    SDL_GPUCommandBuffer* commands, const SceneView& view, const std::filesystem::path& path,
    ImDrawData* ui, std::uint32_t width, std::uint32_t height)
{
    SDL_GPUDevice*                 device      = device_->get();
    const SDL_GPUTextureFormat     format      = device_->swapchainFormat();
    const auto                     pixelFormat = pixelFormatOf(format);
    const std::uint32_t            byteCount   = width * height * kBytesPerPixel;
    const SDL_GPUTextureCreateInfo textureInfo{
        .type                 = SDL_GPU_TEXTURETYPE_2D,
        .format               = format,
        .usage                = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET,
        .width                = width,
        .height               = height,
        .layer_count_or_depth = 1,
        .num_levels           = 1,
        .sample_count         = SDL_GPU_SAMPLECOUNT_1,
        .props                = 0,
    };
    const SDL_GPUTransferBufferCreateInfo transferInfo{
        .usage = SDL_GPU_TRANSFERBUFFERUSAGE_DOWNLOAD, .size = byteCount, .props = 0};
    const GpuTexture        capture(device, SDL_CreateGPUTexture(device, &textureInfo));
    const GpuTransferBuffer download(device, SDL_CreateGPUTransferBuffer(device, &transferInfo));
    if (!pixelFormat || !capture.valid() || !download.valid())
    {
        SDL_SubmitGPUCommandBuffer(commands);
        return std::unexpected(pixelFormat ? std::format("Cannot capture: {}", SDL_GetError())
                                           : std::string("Cannot capture this swapchain format"));
    }

    // Draw the frame again into a texture we can read back (never blit the swapchain).
    drawDisplay(commands, capture.get(), view, ui);
    SDL_GPUCopyPass*           copy = SDL_BeginGPUCopyPass(commands);
    const SDL_GPUTextureRegion region{.texture = capture.get(), .w = width, .h = height, .d = 1};
    const SDL_GPUTextureTransferInfo destination{.transfer_buffer = download.get(), .offset = 0};
    SDL_DownloadFromGPUTexture(copy, &region, &destination);
    SDL_EndGPUCopyPass(copy);

    SDL_GPUFence* fence = SDL_SubmitGPUCommandBufferAndAcquireFence(commands);
    if (fence == nullptr)
    {
        return std::unexpected(std::format("Cannot submit the capture: {}", SDL_GetError()));
    }
    SDL_WaitForGPUFences(device, true, &fence, 1);
    SDL_ReleaseGPUFence(device, fence);

    void* pixels = SDL_MapGPUTransferBuffer(device, download.get(), false);
    if (pixels == nullptr)
    {
        return std::unexpected(std::format("Cannot read the capture: {}", SDL_GetError()));
    }
    auto saved = savePng(path, *pixelFormat, pixels, width, height);
    SDL_UnmapGPUTransferBuffer(device, download.get());
    return saved;
}

}  // namespace StarshipSimulator
