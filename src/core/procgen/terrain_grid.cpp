#include "StarshipSimulator/core/procgen/terrain_grid.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <thread>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/landscape.h"
#include "StarshipSimulator/core/habitat/meridian_profile.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr std::uint32_t kNodeQuads     = 32;
constexpr std::uint32_t kMaxRootLevel  = 8;
constexpr std::size_t   kArcByZSamples = 4096;
constexpr double        kWetlandM      = 220.0;  // how far from water the meadows are lush
constexpr double        kShapingReachM = 250.0;  // water shapes the land out to this distance

/// Runs work(i) for i in [0, count) on `threads` threads (0: all cores).
void parallelFor(std::uint32_t count, unsigned threads,
                 const std::function<void(std::uint32_t)>& work)
{
    std::atomic<std::uint32_t> next{0};
    const auto                 run = [&]() {
        for (std::uint32_t i = next++; i < count; i = next++)
        {
            work(i);
        }
    };
    const unsigned workers =
        threads > 0 ? threads : std::max(1U, std::thread::hardware_concurrency());
    std::vector<std::jthread> pool;
    for (unsigned t = 1; t < workers; ++t)
    {
        pool.emplace_back(run);
    }
    run();
}

/// Catmull-Rom interpolation between b (t = 0) and c (t = 1).
double catmullRom(double a, double b, double c, double d, double t)
{
    return b + (0.5 * t *
                (c - a +
                 (t * ((2.0 * a) - (5.0 * b) + (4.0 * c) - d + (t * ((3.0 * (b - c)) + d - a))))));
}

std::uint8_t toByte(double value)
{
    return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0, 1.0) * 255.0));
}

}  // namespace

double TerrainGridLayout::theta(double column) const
{
    return 2.0 * kPi * column / static_cast<double>(columns);
}

TerrainGridLayout makeTerrainLayout(const HabitatGeometry& geometry, double targetCellM)
{
    const double cell          = std::clamp(targetCellM, 0.5, 6.0);
    const double circumference = 2.0 * kPi * geometry.radius();
    const double length        = geometry.profile().length();
    const double around        = circumference / cell;
    const double along         = length / cell;

    // The largest quadtree root that fits at least three times along both directions.
    const double  smaller = std::min(around, along) / 3.0;
    std::uint32_t level   = 0;
    while (level < kMaxRootLevel && static_cast<double>(kNodeQuads << (level + 1)) <= smaller)
    {
        ++level;
    }
    TerrainGridLayout layout;
    layout.nodeQuads = kNodeQuads;
    layout.rootLevel = level;
    layout.radiusM   = geometry.radius();
    const auto root  = static_cast<double>(layout.rootCells());
    layout.columns   = static_cast<std::uint32_t>(std::max(1.0, std::round(around / root)) * root);
    layout.cells     = static_cast<std::uint32_t>(std::max(1.0, std::round(along / root)) * root);
    layout.cellU     = length / layout.cells;
    layout.cellArcM  = circumference / layout.columns;
    return layout;
}

double TerrainGrid::height(std::uint32_t column, std::uint32_t row) const
{
    const std::uint16_t value =
        heights[(static_cast<std::size_t>(row) * layout.columns) + (column % layout.columns)];
    return static_cast<double>(heightMin) +
           (static_cast<double>(value) / 65535.0 * static_cast<double>(heightRange));
}

TerrainGrid sampleTerrain(const HabitatGeometry& geometry, double targetCellM, unsigned threads)
{
    TerrainGrid            grid;
    TerrainGridLayout&     layout  = grid.layout;
    const MeridianProfile& profile = geometry.profile();
    const TerrainSpec&     terrain = geometry.spec().terrain;
    layout                         = makeTerrainLayout(geometry, targetCellM);
    const std::uint32_t rows       = layout.rows();

    // Heights are quantized over the range the terrain model can reach.
    grid.heightMin   = static_cast<float>(kWaterLevelM - kWaterDepthM - 0.5);
    grid.heightRange = static_cast<float>(terrain.hillHeightM + terrain.mountainHeightM + 1.0 -
                                          static_cast<double>(grid.heightMin));
    grid.profile.resize(rows);
    for (std::uint32_t row = 0; row < rows; ++row)
    {
        const double u    = layout.arc(row);
        const Vec2d  zr   = profile.pointAt(u);
        const Vec2d  n    = profile.inwardNormalAt(u);
        grid.profile[row] = Vec4f(Vec4d(zr.x, zr.y, n.x, n.y));
    }

    // The hills and mountains are smooth: sample their (slow) noise at half resolution and
    // interpolate. Water, which has sharp banks, shapes the full-resolution grid.
    const std::uint32_t coarseColumns = layout.columns / 2;
    const std::uint32_t coarseRows    = (layout.cells / 2) + 1;
    std::vector<float>  natural(static_cast<std::size_t>(coarseRows) * coarseColumns);
    parallelFor(coarseRows, threads, [&](std::uint32_t row) {
        const double z = profile.pointAt(layout.arc(2.0 * row)).x;
        for (std::uint32_t column = 0; column < coarseColumns; ++column)
        {
            natural[(static_cast<std::size_t>(row) * coarseColumns) + column] =
                static_cast<float>(geometry.naturalHeight(z, layout.theta(2.0 * column)));
        }
    });
    const auto coarse = [&](std::int64_t column, std::int64_t row) {
        const auto c = static_cast<std::size_t>((column + coarseColumns) % coarseColumns);
        const auto r = static_cast<std::size_t>(std::clamp<std::int64_t>(row, 0, coarseRows - 1));
        return static_cast<double>(natural[(r * coarseColumns) + c]);
    };

    grid.heights.resize(static_cast<std::size_t>(rows) * layout.columns);
    const auto       low       = static_cast<double>(grid.heightMin);
    const auto       range     = static_cast<double>(grid.heightRange);
    const Landscape& landscape = geometry.landscape();
    parallelFor(rows, threads, [&](std::uint32_t row) {
        const auto                     z = static_cast<double>(grid.profile[row].x);
        const std::span<std::uint16_t> out(
            &grid.heights[static_cast<std::size_t>(row) * layout.columns], layout.columns);
        const auto   r0 = static_cast<std::int64_t>(row / 2);
        const double fr = (row % 2) * 0.5;
        for (std::uint32_t column = 0; column < layout.columns; ++column)
        {
            const auto   c0 = static_cast<std::int64_t>(column / 2);
            const double fc = (column % 2) * 0.5;
            double       h  = 0.0;
            if (fc == 0.0 && fr == 0.0)
            {
                h = coarse(c0, r0);
            }
            else
            {
                std::array<double, 4> rowValues{};
                for (std::int64_t k = 0; k < 4; ++k)
                {
                    rowValues.at(static_cast<std::size_t>(k)) =
                        catmullRom(coarse(c0 - 1, r0 + k - 1), coarse(c0, r0 + k - 1),
                                   coarse(c0 + 1, r0 + k - 1), coarse(c0 + 2, r0 + k - 1), fc);
                }
                h = std::max(
                    0.0, catmullRom(rowValues[0], rowValues[1], rowValues[2], rowValues[3], fr));
            }
            const double theta = layout.theta(column);
            h = Landscape::shapeNearWater(h, landscape.shoreDistance(z, theta, kShapingReachM));
            out[column] = static_cast<std::uint16_t>(
                std::lround(std::clamp((h - low) / range, 0.0, 1.0) * 65535.0));
        }
    });

    // Land cover at half resolution.
    grid.coverColumns = layout.columns / 2;
    grid.coverRows    = (layout.cells / 2) + 1;
    grid.cover.resize(static_cast<std::size_t>(grid.coverRows) * grid.coverColumns * 4);
    parallelFor(grid.coverRows, threads, [&](std::uint32_t row) {
        const double                  z = profile.pointAt(layout.arc(2.0 * row)).x;
        const std::span<std::uint8_t> out(
            &grid.cover[static_cast<std::size_t>(row) * grid.coverColumns * 4],
            static_cast<std::size_t>(grid.coverColumns) * 4);
        for (std::uint32_t column = 0; column < grid.coverColumns; ++column)
        {
            const double theta    = layout.theta(2.0 * column);
            const double shore    = landscape.shoreDistance(z, theta, kWetlandM);
            out[(column * 4) + 0] = toByte(geometry.forestDensity(z, theta));
            out[(column * 4) + 1] = toByte(1.0 - glm::smoothstep(0.0, kWetlandM, shore));
            out[(column * 4) + 2] = 0;
            out[(column * 4) + 3] = 255;
        }
    });

    // u(z), for rays crossing the habitat: z increases strictly along the profile.
    grid.zMin = static_cast<float>(profile.zMin());
    grid.zMax = static_cast<float>(profile.zMax());
    grid.arcByZ.resize(kArcByZSamples);
    for (std::size_t i = 0; i < kArcByZSamples; ++i)
    {
        const double z = std::lerp(profile.zMin(), profile.zMax(),
                                   static_cast<double>(i) / (kArcByZSamples - 1));
        grid.arcByZ[i] = static_cast<float>(profile.arcAt(z));
    }
    return grid;
}

}  // namespace StarshipSimulator
