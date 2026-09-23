#include "StarshipSimulator/core/procgen/TerrainLod.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "StarshipSimulator/core/Frustum.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kMorphFraction = 0.35;  // the outer part of each level's band where it morphs
constexpr double kUnlimited     = 1.0e30;

double distanceSquaredToBox(const Vec3d& p, const Vec3d& boxMin, const Vec3d& boxMax)
{
    const Vec3d outside = glm::max(glm::max(boxMin - p, p - boxMax), Vec3d(0.0));
    return glm::dot(outside, outside);
}

/// Grows a box to hold an arc of radius r from angle a0 to a1 (a1 > a0, less than a full turn).
void addArc(Vec3d& boxMin, Vec3d& boxMax, double r, double a0, double a1)
{
    const auto add = [&](double angle) {
        const Vec3d p(r * std::cos(angle), r * std::sin(angle), boxMin.z);
        boxMin.x = std::min(boxMin.x, p.x);
        boxMin.y = std::min(boxMin.y, p.y);
        boxMax.x = std::max(boxMax.x, p.x);
        boxMax.y = std::max(boxMax.y, p.y);
    };
    add(a0);
    add(a1);
    for (auto k = static_cast<int>(std::ceil(a0 / (kPi / 2.0))); k * (kPi / 2.0) < a1; ++k)
    {
        add(k * (kPi / 2.0));
    }
}

struct LeafBounds
{
    Vec3d boxMin{0.0};
    Vec3d boxMax{0.0};
    bool  hidden = false;
    bool  water  = false;
};

/// A finest-level node's bounds, from the patch's samples and the part of the profile it spans.
LeafBounds leafBounds(const TerrainGrid& grid, const HabitatGeometry& geometry, std::uint32_t x,
                      std::uint32_t y)
{
    const TerrainGridLayout& layout = grid.layout;
    const std::uint32_t      quads  = layout.nodeQuads;
    double                   hMin   = std::numeric_limits<double>::max();
    double                   hMax   = std::numeric_limits<double>::lowest();
    Vec2d                    zRange(hMin, hMax);
    Vec2d                    rRange(hMin, hMax);
    for (std::uint32_t row = y * quads; row <= (y + 1) * quads; ++row)
    {
        const Vec4d profile(grid.profile[row]);
        zRange = Vec2d(std::min(zRange.x, profile.x), std::max(zRange.y, profile.x));
        rRange = Vec2d(std::min(rRange.x, profile.y), std::max(rRange.y, profile.y));
        for (std::uint32_t column = x * quads; column <= (x + 1) * quads; ++column)
        {
            const double h = grid.height(column, row);
            hMin           = std::min(hMin, h);
            hMax           = std::max(hMax, h);
        }
    }
    LeafBounds   leaf;
    const double a0 = layout.theta(x * quads);
    const double a1 = layout.theta((x + 1) * quads);
    leaf.boxMin     = Vec3d(std::numeric_limits<double>::max());
    leaf.boxMax     = Vec3d(std::numeric_limits<double>::lowest());
    addArc(leaf.boxMin, leaf.boxMax, std::max(0.0, rRange.x - hMax), a0, a1);
    addArc(leaf.boxMin, leaf.boxMax, std::max(0.0, rRange.y - hMin), a0, a1);
    leaf.boxMin.z = zRange.x;
    leaf.boxMax.z = zRange.y;
    leaf.water    = hMin < kWaterLevelM;
    // Entirely window glass: within the floor, and both edges inside one window strip.
    const double strip   = geometry.stripAngle();
    const double nearest = std::round(0.5 * (a0 + a1) / strip) * strip;
    const double half    = geometry.windowHalfAngle();
    leaf.hidden = zRange.x >= geometry.floorZMin() && zRange.y <= geometry.floorZMax() &&
                  std::abs(a0 - nearest) <= half && std::abs(a1 - nearest) <= half &&
                  geometry.kind() == HabitatKind::ONEILL_CYLINDER;  // the others: no strips
    return leaf;
}

}  // namespace

TerrainLod::TerrainLod(const TerrainGrid& grid, const HabitatGeometry& geometry, double baseRangeM)
  : layout_(grid.layout)
{
    const std::uint32_t levels = layout_.rootLevel + 1;
    for (std::uint32_t level = 0; level < levels; ++level)
    {
        const bool   top   = level == layout_.rootLevel;
        const double range = top ? kUnlimited : baseRangeM * static_cast<double>(1U << level);
        const double inner = level == 0 ? 0.0 : ranges_.back();
        ranges_.push_back(range);
        morphs_.push_back(
            top ? TerrainMorph{.start = kUnlimited, .end = kUnlimited}
                : TerrainMorph{.start = range - (kMorphFraction * (range - inner)), .end = range});
    }

    // Finest level: bounds from the patch's samples and the profile it spans.
    const std::uint32_t quads  = layout_.nodeQuads;
    const std::uint32_t across = layout_.columns / quads;
    const std::uint32_t along  = layout_.cells / quads;
    nodes_.resize(levels);
    nodesAcross_.resize(levels);
    nodes_[0].resize(static_cast<std::size_t>(across) * along);
    nodesAcross_[0] = across;
    for (std::uint32_t y = 0; y < along; ++y)
    {
        for (std::uint32_t x = 0; x < across; ++x)
        {
            const LeafBounds leaf = leafBounds(grid, geometry, x, y);
            Node&            node = nodes_[0][(static_cast<std::size_t>(y) * across) + x];
            node.boxMin           = leaf.boxMin;
            node.boxMax           = leaf.boxMax;
            node.hidden           = leaf.hidden;
            node.water            = leaf.water;
        }
    }

    // Coarser levels: unions of their four children.
    for (std::uint32_t level = 1; level < levels; ++level)
    {
        const std::uint32_t childAcross = nodesAcross_[level - 1];
        const std::uint32_t childAlong =
            static_cast<std::uint32_t>(nodes_[level - 1].size()) / childAcross;
        nodesAcross_[level] = childAcross / 2;
        nodes_[level].resize(static_cast<std::size_t>(childAcross / 2) * (childAlong / 2));
        for (std::uint32_t y = 0; y < childAlong / 2; ++y)
        {
            for (std::uint32_t x = 0; x < childAcross / 2; ++x)
            {
                Node& parent = nodes_[level][(static_cast<std::size_t>(y) * (childAcross / 2)) + x];
                parent.boxMin = Vec3d(std::numeric_limits<double>::max());
                parent.boxMax = Vec3d(std::numeric_limits<double>::lowest());
                parent.hidden = true;
                for (std::uint32_t k = 0; k < 4; ++k)
                {
                    const Node& child = node(level - 1, (2 * x) + (k % 2), (2 * y) + (k / 2));
                    parent.boxMin     = glm::min(parent.boxMin, child.boxMin);
                    parent.boxMax     = glm::max(parent.boxMax, child.boxMax);
                    parent.hidden     = parent.hidden && child.hidden;
                    parent.water      = parent.water || child.water;
                }
            }
        }
    }
}

const TerrainLod::Node& TerrainLod::node(std::uint32_t level, std::uint32_t x,
                                         std::uint32_t y) const
{
    return nodes_[level][(static_cast<std::size_t>(y) * nodesAcross_[level]) + x];
}

bool TerrainLod::select(std::uint32_t level, std::uint32_t x, std::uint32_t y, const Vec3d& camera,
                        const Frustum& frustum, std::vector<TerrainPatch>& out) const
{
    const Node&  n     = node(level, x, y);
    const double range = ranges_[level];
    if (distanceSquaredToBox(camera, n.boxMin, n.boxMax) > range * range)
    {
        return false;  // beyond this level's reach: the parent draws it
    }
    if (n.hidden || !frustum.intersects(n.boxMin - camera, n.boxMax - camera))
    {
        return true;  // handled: nothing to draw
    }
    const std::uint32_t cells = layout_.nodeCells(level);
    const double        finer = level > 0 ? ranges_[level - 1] : 0.0;
    if (level == 0 || distanceSquaredToBox(camera, n.boxMin, n.boxMax) > finer * finer)
    {
        out.push_back({.column = x * cells, .row = y * cells, .level = level, .water = n.water});
        return true;
    }
    for (std::uint32_t k = 0; k < 4; ++k)
    {
        const std::uint32_t cx = (2 * x) + (k % 2);
        const std::uint32_t cy = (2 * y) + (k / 2);
        if (!select(level - 1, cx, cy, camera, frustum, out))
        {
            // This quarter lies wholly beyond the finer range: its vertices morph all the way to
            // this level's grid.
            const Node& child = node(level - 1, cx, cy);
            if (!child.hidden && frustum.intersects(child.boxMin - camera, child.boxMax - camera))
            {
                const std::uint32_t childCells = layout_.nodeCells(level - 1);
                out.push_back({.column = cx * childCells,
                               .row    = cy * childCells,
                               .level  = level - 1,
                               .water  = child.water});
            }
        }
    }
    return true;
}

std::vector<TerrainPatch> TerrainLod::select(const Vec3d& camera, const Frustum& frustum) const
{
    std::vector<TerrainPatch> patches;
    const std::uint32_t       top        = layout_.rootLevel;
    const std::uint32_t       across     = nodesAcross_[top];
    const auto                rootsAlong = static_cast<std::uint32_t>(nodes_[top].size()) / across;
    for (std::uint32_t y = 0; y < rootsAlong; ++y)
    {
        for (std::uint32_t x = 0; x < across; ++x)
        {
            // The top level reaches everywhere, so every root is handled here.
            static_cast<void>(select(top, x, y, camera, frustum, patches));
        }
    }
    return patches;
}

}  // namespace StarshipSimulator
