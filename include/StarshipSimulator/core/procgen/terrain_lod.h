#pragma once

#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/frustum.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

namespace StarshipSimulator
{

/// One square of terrain to draw: the same nodeQuads x nodeQuads grid mesh placed at a grid cell
/// and scaled to the level's size (nodeQuads << level cells).
struct TerrainPatch
{
    std::uint32_t column = 0;
    std::uint32_t row    = 0;
    std::uint32_t level  = 0;
    bool          water  = false;  // some of it lies under water
};

/// Distances (m) over which a level's vertices morph into the next coarser level's grid.
struct TerrainMorph
{
    double start = 0.0;
    double end   = 0.0;
};

/// Continuous distance-dependent level of detail (after Strugar's CDLOD, 2009) over the terrain
/// grid: a quadtree of square patches, finer near the camera, whose vertices morph smoothly into
/// the coarser grid before a level changes, so nothing pops.
class TerrainLod
{
public:
    /// baseRangeM: distance out to which the finest level is used; each level doubles it.
    TerrainLod(const TerrainGrid& grid, const HabitatGeometry& geometry, double baseRangeM);

    /// The patches to draw for a camera (habitat frame) and a camera-relative frustum.
    [[nodiscard]] std::vector<TerrainPatch> select(const Vec3d&   camera,
                                                   const Frustum& frustum) const;

    [[nodiscard]] const TerrainGridLayout&         layout() const { return layout_; }
    [[nodiscard]] const std::vector<TerrainMorph>& morphs() const { return morphs_; }

private:
    struct Node
    {
        Vec3d boxMin{0.0};
        Vec3d boxMax{0.0};
        bool  hidden = false;  // entirely window glass: nothing to draw
        bool  water  = false;  // some ground below the water level
    };

    [[nodiscard]] const Node& node(std::uint32_t level, std::uint32_t x, std::uint32_t y) const;
    [[nodiscard]] bool        select(std::uint32_t level, std::uint32_t x, std::uint32_t y,
                                     const Vec3d& camera, const Frustum& frustum,
                                     std::vector<TerrainPatch>& out) const;

    TerrainGridLayout              layout_;
    std::vector<double>            ranges_;  // per level
    std::vector<TerrainMorph>      morphs_;  // per level
    std::vector<std::vector<Node>> nodes_;   // per level: columns x rows of nodes, row-major
    std::vector<std::uint32_t>     nodesAcross_;
};

}  // namespace StarshipSimulator
