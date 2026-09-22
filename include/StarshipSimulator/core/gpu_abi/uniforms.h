#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/weather.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/TerrainLod.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

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
    Vec4f time{0.0F};  // x: seconds for animation (ripples, leaves), wraps every hour
};
static_assert(sizeof(FrameUniforms) == 176);
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
    // x: radius of the cloud deck's top (nearest the axis), y: its base, z: cover, w: how much
    // light the deck lets through
    Vec4f cloud{0.0F};
    // x: rain, y: mist, z: how wet the ground is, w: how far the clouds have drifted along the
    // axis (m; they also turn, see cloudTurn)
    Vec4f weather{0.0F};
    // x: fresh green, y: autumn gold, z: spring blossom, w: how far the clouds have turned (rad)
    Vec4f season{1.0F, 0.0F, 0.0F, 0.0F};
};
static_assert(sizeof(HabitatUniforms) == 16 * (2 + kMaxSunBeams + 9));
static_assert(offsetof(HabitatUniforms, sunColor) == 16 * (2 + kMaxSunBeams));

/// The starry sky (shaders/include/sky.glsl, block "Sky").
struct alignas(16) SkyUniforms
{
    Mat4f habitatFromInertial{1.0F};  // rotates fixed (EQJ) directions into the spinning habitat
    Vec4f params{1.0F, 0.0F, 0.0F, 0.0F};  // x: star brightness, y: Milky Way brightness
};
static_assert(sizeof(SkyUniforms) == 80);

/// Earth or the Moon, drawn as a lit sphere (shaders/body.vert and body.frag, block "Body").
struct alignas(16) BodyUniforms
{
    Vec4f direction{0.0F, 0.0F, 1.0F, 0.0F};  // xyz: toward the body (habitat frame), w: radius
                                              // (angular, radians)
    Vec4f towardSun{0.0F, 0.0F, 1.0F, 0.0F};  // xyz: from the body toward the Sun (habitat frame)
    Vec4f sunlight{0.0F};  // rgb: sunlight falling on the body (scene units), a: albedo scale
    Vec4f params{0.0F};    // x: 1 = has an atmosphere, y: night lights, z: ambient (earthshine)
    Mat4f bodyFromHabitat{1.0F};  // habitat directions -> body-fixed (x: prime meridian, z: north)
};
static_assert(sizeof(BodyUniforms) == 128);

inline constexpr std::size_t kMaxPlanets = 8;

/// The planets as points of light (shaders/planets.vert, block "Planets"): GpuStar pairs.
struct alignas(16) PlanetUniforms
{
    std::array<Vec4f, 2 * kMaxPlanets> stars{};      // per planet: direction + size, colour
    Vec4f                              count{0.0F};  // x: number of planets
};
static_assert(sizeof(PlanetUniforms) == 16 * ((2 * kMaxPlanets) + 1));

/// Where a habitat's exterior (mirrors, hull) is drawn (shaders/mirror.vert, block "Placement").
struct alignas(16) PlacementUniforms
{
    Mat4f model{1.0F};  // habitat frame -> camera-relative
};
static_assert(sizeof(PlacementUniforms) == 64);

inline constexpr std::size_t kMaxTerrainLevels = 12;

/// The level-of-detail terrain (shaders/include/landscape.glsl, block "Landscape").
struct alignas(16) LandscapeUniforms
{
    // x: columns, y: rows (along the profile), z: metres per row, w: 2 pi / columns
    Vec4f grid{0.0F};
    // x: height at texel value 0, y: at 1 (the UNORM range), z: water level, w: floor radius
    Vec4f heights{0.0F};
    // Per level: x: morph start (m), y: morph end, z: 1 / (end - start)
    std::array<Vec4f, kMaxTerrainLevels> morph{};
    Vec4f mode{0.0F};    // x: 0 = ground, 1 = water surface; y: water pass (0 dims, 1 adds)
    Vec4f extent{0.0F};  // x, y: z range of the profile (for the arc-by-z table)
};
static_assert(sizeof(LandscapeUniforms) == 16 * (4 + kMaxTerrainLevels));

/// The trees' shadow map (shaders/include/shadow.glsl, block "Shadow"): an orthographic view along
/// one mirror's beam, centred on the camera.
struct alignas(16) ShadowUniforms
{
    Mat4f lightFromCameraRelative{1.0F};  // camera-relative position -> shadow clip space (0..1 z)
    Vec4f params{0.0F};  // x: 1 = in use, y: the window whose beam it is for, z: texel (m), w: bias
};
static_assert(sizeof(ShadowUniforms) == 80);

/// Per-draw data for instanced trees (shaders/tree.vert, block "TreeDraw").
struct alignas(16) TreeDrawUniforms
{
    Vec4f origin{0.0F};  // xyz: the tile's origin relative to the camera
    Vec4f lod{0.0F};     // x: 1 = detailed mesh, y: detail end (m), z: blend (m), w: far end (m)
    Vec4f fade{0.0F};    // x: fade-out length (m)
};
static_assert(sizeof(TreeDrawUniforms) == 48);

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
    Vec4f params{1.0F, 1.0F, 0.0F, 0.0F};  // x: exposure, y: 1 = encode sRGB, z: grade strength
};
static_assert(sizeof(TonemapUniforms) == 16);

/// Builds the per-frame uniforms for a camera and a render target size in pixels.
[[nodiscard]] FrameUniforms makeFrameUniforms(const Camera& camera, std::uint32_t width,
                                              std::uint32_t height, double animationSeconds = 0.0);

/// Artistic lighting controls on top of the physical model.
struct LightingSettings
{
    double sunIntensity = 3.2;  // scene units for full sunlight on a surface facing it
    double ambient      = 1.0;  // scales light from the far side and ground
    double haze         = 1.0;  // scales aerial perspective
};

/// Where the clouds are and how far they have drifted, for the habitat uniforms.
struct CloudSettings
{
    double baseM   = 420.0;  // the deck, above the floor
    double topM    = 820.0;
    double driftM  = 0.0;  // how far they have blown along the axis
    double turnRad = 0.0;  // and around the habitat
};

/// How much of the mirrors' light reaches the ground under this much cloud (1 clear, 0.25 under a
/// solid deck). The same factor dims the beams, the air and the eye's adaptation.
[[nodiscard]] double cloudShade(double cloudCover);

/// Builds the habitat uniforms for a mirror opening angle (radians), in this weather.
[[nodiscard]] HabitatUniforms makeHabitatUniforms(const HabitatGeometry&  geometry,
                                                  double                  openingAngle,
                                                  const LightingSettings& lighting,
                                                  const Weather&          weather,
                                                  const CloudSettings&    clouds);

/// The sky seen from the habitat: habitatFromInertial rotates EQJ directions into the habitat frame
/// (see astro::habitatFromEqj).
[[nodiscard]] SkyUniforms makeSkyUniforms(const Mat3d& habitatFromInertial, double starBrightness,
                                          double milkyWayBrightness);

/// The terrain grid and its level-of-detail morph distances.
[[nodiscard]] LandscapeUniforms makeLandscapeUniforms(const TerrainGrid&               grid,
                                                      const std::vector<TerrainMorph>& morphs);

/// A shadow map box of 2 * halfExtentM across, centred on the camera and looking along the beam
/// toward the sun image; snapped to whole texels in the habitat frame so shadows do not crawl as
/// the camera moves.
[[nodiscard]] ShadowUniforms makeShadowUniforms(const Vec3d& cameraPosition, const Vec3d& towardSun,
                                                int window, double halfExtentM,
                                                std::uint32_t resolution);

/// Earth or the Moon as seen from the habitat. Sunlight is in the same scene units as the light
/// in the habitat (LightingSettings::sunIntensity).
[[nodiscard]] BodyUniforms makeBodyUniforms(const astro::VisibleBody& body,
                                            const Mat3d&              habitatFromInertial,
                                            const LightingSettings&   lighting);

/// The planets as points: brightness from their magnitudes.
[[nodiscard]] PlanetUniforms makePlanetUniforms(const astro::SkyState& sky);

}  // namespace StarshipSimulator::gpu
