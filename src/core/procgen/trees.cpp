#include "StarshipSimulator/core/procgen/trees.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/SimplexNoise.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

namespace StarshipSimulator
{

namespace
{

constexpr std::uint32_t kTileCells      = 64;  // trees are grouped in tiles of 64 x 64 grid cells
constexpr double        kSolitaryChance = 0.012;  // a lone tree on open land, per candidate spot
constexpr double        kSteepest       = 0.7;    // no trees on slopes steeper than this (tan)
constexpr double        kCrownMargin    = 30.0;   // tile bounds grow by this for crowns (m)

// ---- Meshes -------------------------------------------------------------------------------------

void addVertex(CpuMesh& mesh, const Vec3d& position, const Vec3d& normal, std::uint32_t material)
{
    mesh.vertices.push_back({.position = Vec3f(position),
                             .normal   = Vec3f(glm::normalize(normal)),
                             .uv       = Vec2f(0.0F),
                             .material = material});
}

/// A tapering prism from y0 to y1 (open at both ends), faces outward.
void addTrunk(CpuMesh& mesh, double r0, double r1, double y0, double y1, int sides)
{
    const auto base  = static_cast<std::uint32_t>(mesh.vertices.size());
    const auto count = static_cast<std::uint32_t>(sides);
    for (int i = 0; i < sides; ++i)
    {
        const double a = 2.0 * kPi * i / sides;
        const Vec3d  outward(std::cos(a), 0.0, std::sin(a));
        addVertex(mesh, Vec3d(r0 * outward.x, y0, r0 * outward.z), outward, tree_material::kBark);
        addVertex(mesh, Vec3d(r1 * outward.x, y1, r1 * outward.z), outward, tree_material::kBark);
    }
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const std::uint32_t a = base + (2 * i);
        const std::uint32_t b = base + (2 * ((i + 1) % count));
        // (bottom i, top i, bottom i + 1) winds counter-clockwise seen from outside.
        for (const std::uint32_t k : {a, a + 1, b, b, a + 1, b + 1})
        {
            mesh.indices.push_back(k);
        }
    }
}

/// A cone (with its base disc) from y0 up to the apex at y1.
void addCone(CpuMesh& mesh, double radius, double y0, double y1, int sides)
{
    const auto   base  = static_cast<std::uint32_t>(mesh.vertices.size());
    const auto   count = static_cast<std::uint32_t>(sides);
    const double slope = radius / (y1 - y0);
    for (int i = 0; i < sides; ++i)
    {
        const double a = 2.0 * kPi * (i + 0.5) / sides;
        const Vec3d  outward(std::cos(a), slope, std::sin(a));
        const double a0 = 2.0 * kPi * i / sides;
        const double a1 = 2.0 * kPi * (i + 1) / sides;
        addVertex(mesh, Vec3d(radius * std::cos(a0), y0, radius * std::sin(a0)),
                  Vec3d(std::cos(a0), slope, std::sin(a0)), tree_material::kLeaves);
        addVertex(mesh, Vec3d(0.0, y1, 0.0), outward, tree_material::kLeaves);
        addVertex(mesh, Vec3d(radius * std::cos(a1), y0, radius * std::sin(a1)),
                  Vec3d(std::cos(a1), slope, std::sin(a1)), tree_material::kLeaves);
    }
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const std::uint32_t v = base + (3 * i);
        for (const std::uint32_t k : {v, v + 1, v + 2})
        {
            mesh.indices.push_back(k);
        }
    }
    // The underside, facing down (its own vertices: the rim's normals point out and up).
    const auto centre = static_cast<std::uint32_t>(mesh.vertices.size());
    addVertex(mesh, Vec3d(0.0, y0, 0.0), Vec3d(0.0, -1.0, 0.0), tree_material::kLeaves);
    for (int i = 0; i < sides; ++i)
    {
        const double a = 2.0 * kPi * i / sides;
        addVertex(mesh, Vec3d(radius * std::cos(a), y0, radius * std::sin(a)),
                  Vec3d(0.0, -1.0, 0.0), tree_material::kLeaves);
    }
    for (std::uint32_t i = 0; i < count; ++i)
    {
        mesh.indices.push_back(centre);
        mesh.indices.push_back(centre + 1 + i);
        mesh.indices.push_back(centre + 1 + ((i + 1) % count));
    }
}

/// A lumpy ellipsoid of foliage: an icosphere (subdivided `levels` times), its radius wobbling.
void addBlob(CpuMesh& mesh, const Vec3d& centre, const Vec3d& radii, int levels,
             const SimplexNoise& noise, double lumpiness)
{
    const double       t = std::numbers::phi;
    std::vector<Vec3d> points{{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0},
                              {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t},
                              {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    std::vector<std::array<std::uint32_t, 3>> faces{
        {0, 11, 5},  {0, 5, 1},  {0, 1, 7},  {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
        {11, 10, 2}, {10, 7, 6}, {7, 1, 8},  {3, 9, 4},  {3, 4, 2},   {3, 2, 6}, {3, 6, 8},
        {3, 8, 9},   {4, 9, 5},  {2, 4, 11}, {6, 2, 10}, {8, 6, 7},   {9, 8, 1}};
    for (Vec3d& p : points)
    {
        p = glm::normalize(p);
    }
    for (int level = 0; level < levels; ++level)
    {
        std::vector<std::array<std::uint32_t, 3>>            finer;
        std::vector<std::pair<std::uint64_t, std::uint32_t>> midpoints;
        const auto midpoint = [&](std::uint32_t a, std::uint32_t b) {
            const std::uint64_t key = (std::uint64_t{std::min(a, b)} << 32U) | std::max(a, b);
            for (const auto& [k, index] : midpoints)
            {
                if (k == key)
                {
                    return index;
                }
            }
            points.push_back(glm::normalize(points[a] + points[b]));
            const auto index = static_cast<std::uint32_t>(points.size() - 1);
            midpoints.emplace_back(key, index);
            return index;
        };
        for (const auto& [a, b, c] : faces)
        {
            const std::uint32_t ab = midpoint(a, b);
            const std::uint32_t bc = midpoint(b, c);
            const std::uint32_t ca = midpoint(c, a);
            finer.push_back({a, ab, ca});
            finer.push_back({b, bc, ab});
            finer.push_back({c, ca, bc});
            finer.push_back({ab, bc, ca});
        }
        faces = std::move(finer);
    }
    const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
    for (const Vec3d& direction : points)
    {
        const double wobble = 1.0 + (lumpiness * noise.sample((direction * 1.7) + centre * 3.0));
        const Vec3d  p      = centre + (direction * radii * wobble);
        // Soft foliage shading: normals of the smooth ellipsoid, not of the lumps.
        addVertex(mesh, p, direction / radii, tree_material::kLeaves);
    }
    for (const auto& [a, b, c] : faces)
    {
        mesh.indices.push_back(base + a);  // the icosahedron's faces wind counter-clockwise outside
        mesh.indices.push_back(base + b);
        mesh.indices.push_back(base + c);
    }
}

// ---- Planting ----------------------------------------------------------------------------------

/// Land cover at fractional grid coordinates: (woods, wetness), bilinear.
Vec2d coverAt(const TerrainGrid& grid, double column, double row)
{
    const double c  = column / 2.0;
    const double r  = std::clamp(row / 2.0, 0.0, static_cast<double>(grid.coverRows - 1));
    const auto   c0 = static_cast<std::int64_t>(std::floor(c));
    const auto   r0 = static_cast<std::int64_t>(std::floor(r));
    const double fc = c - static_cast<double>(c0);
    const double fr = r - static_cast<double>(r0);
    const auto   at = [&](std::int64_t cc, std::int64_t rr) {
        const auto columns = static_cast<std::int64_t>(grid.coverColumns);
        const auto x       = static_cast<std::size_t>(((cc % columns) + columns) % columns);
        const auto y       = static_cast<std::size_t>(
            std::clamp<std::int64_t>(rr, 0, static_cast<std::int64_t>(grid.coverRows) - 1));
        const std::size_t i = ((y * grid.coverColumns) + x) * 4;
        return Vec2d(grid.cover[i], grid.cover[i + 1]) / 255.0;
    };
    return glm::mix(glm::mix(at(c0, r0), at(c0 + 1, r0), fc),
                    glm::mix(at(c0, r0 + 1), at(c0 + 1, r0 + 1), fc), fr);
}

struct PlantedTile
{
    TreeTile                                                 tile;
    std::array<std::vector<TreeInstance>, kTreeSpeciesCount> bySpecies;
};

/// The species (if any) that grows at a spot on the grid, where it stands (habitat frame) and how
/// tall trees there grow.
struct Growth
{
    TreeSpecies species = TreeSpecies::BROADLEAF;
    Vec3d       position{0.0};
};

/// Heights in metres, by species: shortest and tallest.
constexpr std::array<Vec2d, kTreeSpeciesCount> kHeights{
    {Vec2d(11.0, 20.0), Vec2d(14.0, 26.0), Vec2d(16.0, 24.0)}};

std::optional<Growth> growthAt(const HabitatGeometry& geometry, const TerrainGrid& grid,
                               const SimplexNoise& stands, double column, double row, double pick)
{
    const TerrainGridLayout& layout = grid.layout;
    const Vec4d              at(grid.profile[static_cast<std::size_t>(row)]);
    const double             z     = at.x;
    const double             theta = layout.theta(column);
    const bool               floor = z >= geometry.floorZMin() && z <= geometry.floorZMax();
    const Vec2d              cover = coverAt(grid, column, row);

    // Woods (pine stands among the broadleaves), poplars along the water, a lone tree in the open.
    double      chance   = glm::smoothstep(0.2, 0.75, cover.x);
    const bool  conifers = floor ? stands.sample(Vec3d(column / 180.0, row / 180.0, 0.5)) > 0.35
                                 : pick < 0.75 * chance;  // mostly pines up the mountains
    TreeSpecies species  = conifers ? TreeSpecies::CONIFER : TreeSpecies::BROADLEAF;
    if (floor && cover.y > 0.6)
    {
        const double shore = geometry.landscape().shoreDistance(z, theta, 60.0);
        if (shore > 6.0 && shore < 22.0)
        {
            chance  = 0.35;
            species = TreeSpecies::POPLAR;
        }
    }
    chance = std::max(chance, floor ? kSolitaryChance : 0.0);
    if (pick >= chance || geometry.regionAt(z, theta).kind == RegionKind::WINDOW)
    {
        return std::nullopt;
    }

    // Dry, not too steep ground.
    const double h = grid.heightAt(column, row);
    if (h < grid.waterLevelAtRow(static_cast<std::uint32_t>(row)) + 0.3)
    {
        return std::nullopt;
    }
    const double around = (grid.heightAt(column + 1.0, row) - grid.heightAt(column - 1.0, row)) /
                          (2.0 * layout.cellArcM * at.y / layout.radiusM);
    const double along  = (grid.heightAt(column, row + 1.0) - grid.heightAt(column, row - 1.0)) /
                          (2.0 * layout.cellU);
    if (std::hypot(around, along) > kSteepest)
    {
        return std::nullopt;
    }
    const double groundR = at.y - h;
    return Growth{.species  = species,
                  .position = Vec3d(groundR * std::cos(theta), groundR * std::sin(theta), z)};
}

PlantedTile plantTile(const HabitatGeometry& geometry, const TerrainGrid& grid,
                      const TreeSettings& settings, std::uint32_t tileX, std::uint32_t tileY,
                      const SimplexNoise& stands)
{
    const TerrainGridLayout& layout      = grid.layout;
    const std::uint32_t      tilesAcross = layout.columns / kTileCells;
    const auto               cells       = static_cast<double>(layout.cells);
    SplitMix64 random(hashSeed(geometry.spec().terrain.seed ^ 0x7EE5ULL,
                               (static_cast<std::uint64_t>(tileY) * tilesAcross) + tileX));

    // The tile's origin: its centre, on the profile surface.
    PlantedTile planted;
    const Vec4d centre(
        grid.profile[static_cast<std::size_t>(std::min((tileY + 0.5) * kTileCells, cells))]);
    const double centreTheta = layout.theta((tileX + 0.5) * kTileCells);
    planted.tile.origin =
        Vec3d(centre.y * std::cos(centreTheta), centre.y * std::sin(centreTheta), centre.x);
    Vec3d lowest(std::numeric_limits<double>::max());
    Vec3d highest(std::numeric_limits<double>::lowest());

    // Candidate spots on a jittered grid, spaced evenly in metres whatever the radius.
    const double rowStep  = settings.spacingM / layout.cellU;
    const double firstRow = tileY * kTileCells;
    const double lastRow  = std::min((tileY + 1.0) * kTileCells, cells);
    const auto   rows     = static_cast<int>(std::ceil((lastRow - firstRow) / rowStep));
    for (int i = 0; i < rows; ++i)
    {
        const double row    = firstRow + (i * rowStep);
        const auto   radius = static_cast<double>(grid.profile[static_cast<std::size_t>(row)].y);
        if (radius < 1.0)
        {
            continue;
        }
        const double columnStep = settings.spacingM * layout.columns / (2.0 * kPi * radius);
        const auto   columns    = static_cast<int>(std::ceil(kTileCells / columnStep));
        for (int j = 0; j < columns; ++j)
        {
            const double column = (tileX * kTileCells) + (j * columnStep);
            const double c      = column + (random.uniform(-0.4, 0.4) * columnStep);
            const double r    = std::clamp(row + (random.uniform(-0.4, 0.4) * rowStep), 0.0, cells);
            const auto   grow = growthAt(geometry, grid, stands, c, r, random.uniform());
            if (!grow)
            {
                continue;
            }
            const auto   index  = static_cast<std::size_t>(grow->species);
            const Vec2d  range  = kHeights.at(index);
            const double height = random.uniform(range.x, range.y);
            const bool   cleared =
                std::ranges::any_of(settings.clearings, [&](const Clearing& clearing) {
                    return glm::distance(clearing.centre, grow->position) < clearing.radiusM;
                });
            if (cleared ||
                (settings.keepOff &&
                 settings.keepOff(grow->position.z, HabitatGeometry::angleOf(grow->position))))
            {
                continue;
            }
            const Vec3d local = grow->position - planted.tile.origin;
            lowest            = glm::min(lowest, local);
            highest           = glm::max(highest, local);
            // One draw to a line: C++ does not say which argument of a call is worked out first,
            // so two draws in one call would come out in a different order under another compiler
            // and the same habitat file would grow a different wood.
            const double turn = random.uniform();
            const double tint = random.uniform();
            planted.bySpecies.at(index).push_back(
                {.position = Vec3f(local), .packed = packTree(height, turn, grow->species, tint)});
        }
    }
    planted.tile.boundsMin = Vec3f(lowest - Vec3d(kCrownMargin));
    planted.tile.boundsMax = Vec3f(highest + Vec3d(kCrownMargin));
    return planted;
}

}  // namespace

std::uint32_t packTree(double heightM, double turn, TreeSpecies species, double tint)
{
    const auto height = static_cast<std::uint32_t>(std::clamp(heightM * 10.0, 0.0, 4095.0));
    const auto angle  = static_cast<std::uint32_t>(std::clamp(turn, 0.0, 1.0) * 255.0);
    const auto shade  = static_cast<std::uint32_t>(std::clamp(tint, 0.0, 1.0) * 255.0);
    return height | (angle << 12U) | (static_cast<std::uint32_t>(species) << 20U) | (shade << 24U);
}

CpuMesh makeTreeMesh(TreeSpecies species, bool detailed)
{
    CpuMesh            mesh;
    const SimplexNoise noise(hashSeed(1977, static_cast<std::uint64_t>(species)));
    switch (species)
    {
        case TreeSpecies::BROADLEAF:
            addTrunk(mesh, 0.035, 0.02, 0.0, 0.5, detailed ? 6 : 3);
            if (detailed)
            {
                addBlob(mesh, Vec3d(0.0, 0.66, 0.0), Vec3d(0.27, 0.24, 0.27), 1, noise, 0.12);
                for (int i = 0; i < 4; ++i)
                {
                    const double a = (2.0 * kPi * i / 4.0) + 0.4;
                    const double y = 0.56 + (0.06 * (i % 2));
                    addBlob(mesh, Vec3d(0.17 * std::cos(a), y, 0.17 * std::sin(a)),
                            Vec3d(0.19, 0.17, 0.19), 1, noise, 0.15);
                }
            }
            else
            {
                addBlob(mesh, Vec3d(0.0, 0.62, 0.0), Vec3d(0.36, 0.3, 0.36), 0, noise, 0.0);
            }
            break;
        case TreeSpecies::CONIFER:
            addTrunk(mesh, 0.03, 0.01, 0.0, 0.3, detailed ? 6 : 3);
            if (detailed)
            {
                for (int k = 0; k < 4; ++k)
                {
                    const double y0 = 0.16 + (0.19 * k);
                    addCone(mesh, 0.25 * (1.0 - (0.2 * k)), y0, std::min(1.0, y0 + 0.36), 8);
                }
            }
            else
            {
                addCone(mesh, 0.24, 0.14, 1.0, 6);
            }
            break;
        case TreeSpecies::POPLAR:
            addTrunk(mesh, 0.03, 0.015, 0.0, 0.35, detailed ? 6 : 3);
            addBlob(mesh, Vec3d(0.0, 0.58, 0.0), Vec3d(0.12, 0.42, 0.12), detailed ? 1 : 0, noise,
                    detailed ? 0.1 : 0.0);
            break;
    }
    return mesh;
}

TreeLayer plantTrees(const HabitatGeometry& geometry, const TerrainGrid& grid,
                     const TreeSettings& settings)
{
    const std::uint32_t      across = grid.layout.columns / kTileCells;
    const std::uint32_t      along  = (grid.layout.cells + kTileCells - 1) / kTileCells;
    const SimplexNoise       stands(hashSeed(geometry.spec().terrain.seed, 6));
    std::vector<PlantedTile> planted(static_cast<std::size_t>(across) * along);

    std::atomic<std::uint32_t> next{0};
    const auto                 work = [&]() {
        for (std::uint32_t i = next++; i < planted.size(); i = next++)
        {
            planted[i] = plantTile(geometry, grid, settings, i % across, i / across, stands);
        }
    };
    {
        const unsigned            threads = settings.threads > 0
                                                ? settings.threads
                                                : std::max(1U, std::thread::hardware_concurrency());
        std::vector<std::jthread> workers;
        for (unsigned t = 1; t < threads; ++t)
        {
            workers.emplace_back(work);
        }
        work();
    }

    TreeLayer layer;
    for (PlantedTile& tile : planted)
    {
        tile.tile.first   = static_cast<std::uint32_t>(layer.instances.size());
        std::size_t total = 0;
        for (std::size_t s = 0; s < kTreeSpeciesCount; ++s)
        {
            tile.tile.counts.at(s) = static_cast<std::uint32_t>(tile.bySpecies.at(s).size());
            layer.instances.insert(layer.instances.end(), tile.bySpecies.at(s).begin(),
                                   tile.bySpecies.at(s).end());
            total += tile.bySpecies.at(s).size();
        }
        if (total > 0)
        {
            layer.tiles.push_back(tile.tile);
        }
    }
    return layer;
}

}  // namespace StarshipSimulator
