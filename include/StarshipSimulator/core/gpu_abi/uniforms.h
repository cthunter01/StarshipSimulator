#pragma once

#include <cstddef>
#include <cstdint>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/math.h"

// C++ mirrors of the std140 uniform blocks in shaders/include/*.glsl. Only vec4 and mat4 members,
// so C++ and GLSL layouts agree without padding rules. Keep both sides in sync.
namespace StarshipSimulator::gpu
{

/// Per-frame camera data (shaders/include/frame.glsl, block "Frame").
struct alignas(16) FrameUniforms
{
    Mat4f viewProjection{1.0F};  // projection * rotation-only view; positions are camera-relative
    Mat4f inverseViewProjection{1.0F};  // for reconstructing view rays from NDC
    Vec4f gridOrigin{
        0.0F};  // xy: camera xy modulo 10 km, z: camera height above z = 0, w: near plane
    Vec4f viewport{0.0F};                        // xy: size in pixels, zw: 1 / size
    Vec4f sunDirection{0.0F, 0.0F, 1.0F, 0.0F};  // xyz: direction toward the sun (world), w: unused
};
static_assert(sizeof(FrameUniforms) == 176);
static_assert(offsetof(FrameUniforms, gridOrigin) == 128);

/// Per-draw data for simple meshes (shaders/mesh.vert "Draw" and shaders/marker.frag "Material").
struct alignas(16) DrawUniforms
{
    Mat4f modelViewProjection{1.0F};  // model matrix includes the camera-relative translation
    Mat4f model{1.0F};                // rotation/scale only, for normals
};
static_assert(sizeof(DrawUniforms) == 128);

struct alignas(16) MaterialUniforms
{
    Vec4f color{1.0F};     // linear RGB, a: unused
    Vec4f emission{0.0F};  // linear RGB added after lighting
};
static_assert(sizeof(MaterialUniforms) == 32);

/// Tonemapping (shaders/tonemap.frag "Tonemap").
struct alignas(16) TonemapUniforms
{
    Vec4f params{1.0F, 1.0F, 0.0F, 0.0F};  // x: exposure, y: 1 = encode sRGB (UNORM target)
};
static_assert(sizeof(TonemapUniforms) == 16);

/// Grid period used for FrameUniforms::gridOrigin; the grid shader's largest line spacing divides
/// it.
inline constexpr double kGridPeriodMeters = 10000.0;

/// Builds the per-frame uniforms for a camera and a render target size in pixels.
[[nodiscard]] FrameUniforms makeFrameUniforms(const Camera& camera, std::uint32_t width,
                                              std::uint32_t height);

}  // namespace StarshipSimulator::gpu
