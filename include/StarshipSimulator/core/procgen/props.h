#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"

// Loose things that can be pushed, kicked and thrown: balls, crates, barrels, hay bales, cafe
// chairs and tables. Each has a mesh and a collision shape in its own frame: y up, the origin on
// the ground under it when it stands upright.
namespace StarshipSimulator
{

enum class PropKind : std::uint8_t
{
    Ball,
    Crate,
    Barrel,
    HayBale,
    Chair,
    Table,
};
inline constexpr std::size_t kPropKindCount = 6;

[[nodiscard]] const char* propKindName(PropKind kind);

/// One convex piece of a prop's collision shape.
struct PropPart
{
    enum class Shape : std::uint8_t
    {
        Box,       // halfExtents
        Sphere,    // halfExtents.x: radius
        Cylinder,  // along y; halfExtents.x: radius, halfExtents.y: half height
    };
    Shape shape = Shape::Box;
    Vec3f centre{0.0F};
    Vec3f halfExtents{0.5F};
};

struct PropInfo
{
    std::vector<PropPart> parts;
    float                 massKg         = 1.0F;
    float                 friction       = 0.6F;
    float                 restitution    = 0.2F;
    float                 angularDamping = 0.2F;  // 1/s: rolling resistance
    float                 buoyancy       = 1.0F;  // above 1 floats
};

[[nodiscard]] const PropInfo& propInfo(PropKind kind);

/// Vertex materials of prop meshes (Vertex::material).
namespace propMaterial
{
inline constexpr std::uint32_t kWood     = 0;  // crate planks
inline constexpr std::uint32_t kStaves   = 1;  // barrel
inline constexpr std::uint32_t kMetal    = 2;  // hoops, chair and table frames
inline constexpr std::uint32_t kBall     = 3;  // coloured panels
inline constexpr std::uint32_t kStraw    = 4;
inline constexpr std::uint32_t kTableTop = 5;
}  // namespace propMaterial

[[nodiscard]] CpuMesh makePropMesh(PropKind kind);

/// A prop where the world begins: standing on the ground somewhere.
struct PropPlacement
{
    PropKind kind = PropKind::Crate;
    Vec3d    position{0.0};                    // habitat frame: the prop's origin
    Quatd    orientation{1.0, 0.0, 0.0, 0.0};  // its axes in the habitat frame
    float    tint = 0.0F;                      // 0..1: picks a colour variant
};

}  // namespace StarshipSimulator
