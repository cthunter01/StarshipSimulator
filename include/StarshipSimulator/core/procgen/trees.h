#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

// Trees: a few procedural species in the rounded, painterly shapes of the 1970s habitat paintings,
// planted over the habitat's woods, river banks and fields.
namespace StarshipSimulator
{

enum class TreeSpecies : std::uint8_t
{
    BROADLEAF,  // oak-like: a clump of rounded crowns
    CONIFER,    // pine-like: stacked cones
    POPLAR,     // tall, narrow: along rivers
};
inline constexpr std::size_t kTreeSpeciesCount = 3;

/// Vertex materials of tree meshes (Vertex::material).
namespace tree_material
{
inline constexpr std::uint32_t kBark   = 0;
inline constexpr std::uint32_t kLeaves = 1;
}  // namespace tree_material

/// A tree mesh one unit tall, trunk base at the origin, growing along +Y.
[[nodiscard]] CpuMesh makeTreeMesh(TreeSpecies species, bool detailed);

/// One tree as the GPU draws it: 16 bytes. Position relative to its tile's origin (m); the packed
/// word holds the height (dm, 12 bits), turn (8 bits), species (4 bits) and a tint (8 bits).
struct TreeInstance
{
    Vec3f         position{0.0F};
    std::uint32_t packed = 0;
};
static_assert(sizeof(TreeInstance) == 16);

[[nodiscard]] std::uint32_t packTree(double heightM, double turn, TreeSpecies species, double tint);

/// Trees over a square of the terrain grid, in contiguous runs by species.
struct TreeTile
{
    Vec3d         origin{0.0};  // habitat frame
    Vec3f         boundsMin{0.0F};
    Vec3f         boundsMax{0.0F};
    std::uint32_t first = 0;  // index of the first instance in TreeLayer::instances
    std::array<std::uint32_t, kTreeSpeciesCount> counts{};
};

struct TreeLayer
{
    std::vector<TreeTile>     tiles;
    std::vector<TreeInstance> instances;
};

/// A place kept free of trees (where visitors start, later towns and roads).
struct Clearing
{
    Vec3d  centre{0.0};  // habitat frame
    double radiusM = 20.0;
};

struct TreeSettings
{
    double                spacingM = 7.0;  // between trees in dense woods
    unsigned              threads  = 0;    // 0: all cores
    std::vector<Clearing> clearings;
    // Where no wild trees grow (towns, farmyards): called with (z, theta), from several threads.
    std::function<bool(double, double)> keepOff;
};

/// Plants trees on the terrain grid (woods from its land cover, heights from its height field).
/// Deterministic for a given habitat.
[[nodiscard]] TreeLayer plantTrees(const HabitatGeometry& geometry, const TerrainGrid& grid,
                                   const TreeSettings& settings);

}  // namespace StarshipSimulator
