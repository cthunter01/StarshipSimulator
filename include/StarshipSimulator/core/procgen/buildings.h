#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"

// The meshes and collision shapes of the settlements' buildings, bridges and street furniture.
namespace StarshipSimulator
{

/// Vertex materials of settlement meshes: the low 4 bits of Vertex::material.
namespace buildingMaterial
{
inline constexpr std::uint32_t kWall = 0;  // plaster; windows, doors and shutters in the shader
inline constexpr std::uint32_t kRoofTiles = 1;
inline constexpr std::uint32_t kFlatRoof  = 2;
inline constexpr std::uint32_t kStone     = 3;
inline constexpr std::uint32_t kWood      = 4;
inline constexpr std::uint32_t kMetal     = 5;
inline constexpr std::uint32_t kLampGlass = 6;  // glows at night
inline constexpr std::uint32_t kAwning    = 7;
inline constexpr std::uint32_t kWater     = 8;
inline constexpr std::uint32_t kSoffit    = 9;  // under the eaves
inline constexpr std::uint32_t kFoliage   = 10;
}  // namespace buildingMaterial

/// Wall styles (how the shader draws the facade), bits 16-17 of Vertex::material.
enum class FacadeStyle : std::uint8_t
{
    House = 0,  // 3 m storeys, shuttered windows
    Tower = 1,  // belfry openings at the top
    Barn  = 2,  // planks and a big door
    Hall  = 3,  // tall storeys and tall windows
};

/// What the building shader needs to draw a surface, packed into Vertex::material:
/// bits 0-3 surface, 4-7 colour, 8-13 windows along a wall, 14 door, 15 shop front, 16-17 style,
/// 18-20 shutter colour (0: none), 21-31 a per-wall random seed.
struct Facade
{
    std::uint32_t surface = buildingMaterial::kWall;
    std::uint32_t colour  = 0;
    std::uint32_t windows = 0;
    bool          door    = false;
    bool          shop    = false;
    FacadeStyle   style   = FacadeStyle::House;
    std::uint32_t shutter = 0;
    std::uint32_t seed    = 0;
};

[[nodiscard]] std::uint32_t packFacade(const Facade& facade);
[[nodiscard]] Facade        unpackFacade(std::uint32_t material);

/// Wall height (m) of one storey in each style.
[[nodiscard]] double storeyHeight(FacadeStyle style);

/// Window spacing along walls (m): a wall's uv.x counts window bays, [0, windows) being windows.
inline constexpr double kWindowBayM = 3.2;

/// One settlement's buildings, bridges and furniture as one mesh, relative to `origin`.
struct SettlementMesh
{
    std::size_t settlement = 0;
    Vec3d       origin{0.0};
    CpuMesh     mesh;
    Vec3f       boundsMin{0.0F};
    Vec3f       boundsMax{0.0F};
};

[[nodiscard]] std::vector<SettlementMesh> buildSettlementMeshes(const Settlements& settlements);

/// Collision shapes for everything the settlements built.
[[nodiscard]] StaticColliders settlementColliders(const Settlements& settlements);

}  // namespace StarshipSimulator
