#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"

// C++ mirrors of the std140 uniform blocks in shaders/include/*.glsl. Only vec4 and mat4 members
// (and arrays of them), so C++ and GLSL layouts agree without padding rules. Keep both sides in
// sync.
namespace StarshipSimulator::gpu
{

/// Per-frame camera data (shaders/include/frame.glsl, block "Frame").
struct alignas(16) FrameUniforms
{
    Mat4f viewProjection{1.0F};  // projection * rotation-only view; positions are camera-relative
    Mat4f inverseViewProjection{1.0F};  // for reconstructing view rays from NDC
    Vec4f cameraPosition{0.0F};         // xyz: camera in the habitat frame, w: near plane (m)
    Vec4f viewport{0.0F};               // xy: size in pixels, zw: 1 / size
};
static_assert(sizeof(FrameUniforms) == 160);
static_assert(offsetof(FrameUniforms, cameraPosition) == 128);

inline constexpr std::size_t kMaxSunBeams = 6;

/// The habitat's shape, lighting and air (shaders/include/habitat.glsl, block "Habitat").
struct alignas(16) HabitatUniforms
{
    Vec4f shape{0.0F};   // x: hull radius, y: floor z min, z: floor z max, w: window half-angle
    Vec4f strips{0.0F};  // x: strip angle, y: strip count, z: habitat z min, w: habitat z max
    std::array<Vec4f, kMaxSunBeams> beams{};  // xyz: toward the sun image (window i), w: intensity
    Vec4f                           sunColor{0.0F};  // rgb: full sunlight (linear, scene units)
    Vec4f ambientUp{0.0F};                           // rgb: light from the sunlit far side overhead
    Vec4f ambientDown{0.0F};                         // rgb: light bounced up from the ground
    Vec4f atmosphere{
        0.0F};           // x: density falloff k (1/m^2), y: pressure / 1 atm, z: haze, w: daylight
    Vec4f mirror{0.0F};  // x: opening angle, y: mirror length, z: half width, w: hinge z
    Vec4f sun{0.0F};     // xyz: direction to the Sun, w: its angular radius (rad)
};
static_assert(sizeof(HabitatUniforms) == 16 * (2 + kMaxSunBeams + 6));
static_assert(offsetof(HabitatUniforms, sunColor) == 16 * (2 + kMaxSunBeams));

/// The starry sky (shaders/stars.vert, block "Sky").
struct alignas(16) SkyUniforms
{
    Mat4f habitatFromInertial{1.0F};  // rotates the fixed stars into the spinning habitat frame
    Vec4f params{1.0F, 0.0F, 0.0F, 0.0F};  // x: brightness
};
static_assert(sizeof(SkyUniforms) == 80);

/// Per-draw data for meshes (shaders/mesh.vert, block "Draw").
struct alignas(16) DrawUniforms
{
    Mat4f modelViewProjection{1.0F};  // model includes the camera-relative translation
    Mat4f model{1.0F};                // camera-relative model matrix
};
static_assert(sizeof(DrawUniforms) == 128);

/// Marker colours (shaders/marker.frag, block "Material").
struct alignas(16) MaterialUniforms
{
    Vec4f color{1.0F};     // linear RGB, a: unused
    Vec4f emission{0.0F};  // linear RGB added after lighting
};
static_assert(sizeof(MaterialUniforms) == 32);

/// Tonemapping (shaders/tonemap.frag, block "Tonemap").
struct alignas(16) TonemapUniforms
{
    Vec4f params{1.0F, 1.0F, 0.0F, 0.0F};  // x: exposure, y: 1 = encode sRGB (UNORM target)
};
static_assert(sizeof(TonemapUniforms) == 16);

/// Builds the per-frame uniforms for a camera and a render target size in pixels.
[[nodiscard]] FrameUniforms makeFrameUniforms(const Camera& camera, std::uint32_t width,
                                              std::uint32_t height);

/// Artistic lighting controls on top of the physical model.
struct LightingSettings
{
    double sunIntensity = 3.2;  // scene units for full sunlight on a surface facing it
    double ambient      = 1.0;  // scales light from the far side and ground
    double haze         = 1.0;  // scales aerial perspective
};

/// Builds the habitat uniforms for a mirror opening angle (radians).
[[nodiscard]] HabitatUniforms makeHabitatUniforms(const HabitatGeometry&  geometry,
                                                  double                  openingAngle,
                                                  const LightingSettings& lighting);

/// Stars: the habitat has turned by spinPhase radians since t = 0.
[[nodiscard]] SkyUniforms makeSkyUniforms(double spinPhase, double brightness);

}  // namespace StarshipSimulator::gpu
