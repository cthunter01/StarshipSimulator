#pragma once

#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

// The people of the habitat: walking the streets of the towns, standing about the square, sitting
// on the benches, working the farmyards. Like the birds and the weather, none of it is simulated or
// remembered: where everyone is follows from the clock, so the same habitat at the same moment is
// always the same, and nothing has to be saved.
namespace StarshipSimulator
{

enum class Activity : std::uint8_t
{
    WALKING,
    STANDING,
    SITTING,
};

/// One person as they are drawn.
struct Person
{
    Vec3d        position{0.0};           // between the feet, on the ground
    Vec3d        forward{0.0, 0.0, 1.0};  // unit, along the ground
    double       gait    = 0.0;           // 0..1 through a stride
    double       speedMS = 0.0;           // 0 when standing or sitting
    Activity     doing   = Activity::STANDING;
    double       heightM = 1.75;
    std::uint8_t clothes = 0;  // which of kPersonClothes
    std::uint8_t skin    = 0;  // which of kPersonSkins
};

inline constexpr std::uint8_t kPersonClothes = 12;
inline constexpr std::uint8_t kPersonSkins   = 6;

/// How many people are about, and how far off they are still drawn.
struct CrowdSettings
{
    double rangeM     = 280.0;  // people nearer than this to the camera are drawn
    double perStreetM = 40.0;   // one walker per this much street
    double busy       = 1.0;    // 0..1: how many are out (fewer at night and in the rain)
    int    maxPeople  = 500;
};

/// The people within `settings.rangeM` of a point, at a moment. `seconds` is wall-clock time (they
/// walk in real time, like the spin), and `seed` is the habitat's terrain seed.
[[nodiscard]] std::vector<Person> peopleNear(const Settlements& settlements,
                                             const TerrainGrid& grid, const Vec3d& camera,
                                             double seconds, std::uint64_t seed,
                                             const CrowdSettings& settings = {});

/// Vertex materials of the person mesh: each is a part of the body, bent about its own joint by
/// shaders/person.vert.
namespace person_part
{
inline constexpr std::uint32_t kHead   = 0;
inline constexpr std::uint32_t kHair   = 1;
inline constexpr std::uint32_t kTorso  = 2;  // and the hips
inline constexpr std::uint32_t kArmL   = 3;
inline constexpr std::uint32_t kArmR   = 4;
inline constexpr std::uint32_t kThighL = 5;
inline constexpr std::uint32_t kThighR = 6;
inline constexpr std::uint32_t kShinL  = 7;
inline constexpr std::uint32_t kShinR  = 8;
}  // namespace person_part

/// Where the joints are in the mesh's own frame (metres, y up), for the shader.
inline constexpr double kPersonHeightM   = 1.75;  // the mesh is built this tall
inline constexpr double kPersonHipM      = 0.92;
inline constexpr double kPersonKneeM     = 0.48;
inline constexpr double kPersonShoulderM = 1.42;

/// One person, standing straight, 1.75 m tall, feet at the origin, facing +z.
[[nodiscard]] CpuMesh buildPersonMesh();

}  // namespace StarshipSimulator
