#pragma once

#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// How the habitat's surface is gridded for the GPU: columns around the axis (theta, wrapping) and
/// rows along the meridian profile (arc length u), from the anti-sunward hub to the sunward one.
/// Both counts are whole multiples of the level-of-detail quadtree's root size.
struct TerrainGridLayout
{
    std::uint32_t columns   = 0;    // cells around; column c is at theta = 2 pi c / columns
    std::uint32_t cells     = 0;    // cells along the profile; row j (0..cells) is at u = j * cellU
    double        cellU     = 1.0;  // metres along the profile
    double        cellArcM  = 1.0;  // metres around, at the floor radius
    std::uint32_t nodeQuads = 32;   // quads along each side of a level-of-detail patch
    std::uint32_t rootLevel = 0;    // quadtree levels 0 (finest) .. rootLevel
    double        radiusM   = 0.0;  // floor radius

    [[nodiscard]] std::uint32_t rows() const { return cells + 1; }
    [[nodiscard]] std::uint32_t rootCells() const { return nodeQuads << rootLevel; }
    [[nodiscard]] std::uint32_t nodeCells(std::uint32_t level) const { return nodeQuads << level; }
    [[nodiscard]] double        theta(double column) const;
    [[nodiscard]] double        arc(double row) const { return row * cellU; }
    [[nodiscard]] double        profileLength() const { return cells * cellU; }
};

/// Chooses the grid for a habitat: cells of about targetCellM (clamped to 0.5..6 m).
[[nodiscard]] TerrainGridLayout makeTerrainLayout(const HabitatGeometry& geometry,
                                                  double                 targetCellM);

/// The terrain sampled on the grid, ready for textures.
struct TerrainGrid
{
    TerrainGridLayout layout;
    float             heightMin   = 0.0F;  // height = heightMin + value / 65535 * heightRange
    float             heightRange = 1.0F;
    std::vector<std::uint16_t> heights;  // rows x columns, row-major

    // Land cover at half resolution (coverRows x coverColumns, RGBA8): woods, wetness (meadows
    // beside water), and two spare channels.
    std::uint32_t             coverColumns = 0;
    std::uint32_t             coverRows    = 0;
    std::vector<std::uint8_t> cover;

    std::vector<Vec4f> profile;  // per row: z, radius, inward normal (z, radius components)
    // The water's surface as a radius from the axis where the floor is not level (a sphere); 0
    // where the water lies at kWaterLevelM everywhere.
    double waterDatumRadius = 0.0;
    // u as a function of z, sampled evenly over [zMin, zMax] (for rays that cross the habitat).
    std::vector<float> arcByZ;
    float              zMin = 0.0F;
    float              zMax = 0.0F;

    /// Decoded height at a grid point (metres, toward the axis).
    [[nodiscard]] double height(std::uint32_t column, std::uint32_t row) const;
    /// Moves a grid point's ground, for earthworks (it is clamped to the range the grid can hold).
    void setHeight(std::uint32_t column, std::uint32_t row, double metres);
    /// Height at fractional grid coordinates, interpolated the way the GPU draws the terrain.
    [[nodiscard]] double heightAt(double column, double row) const;
    /// Fractional grid coordinates (column, row) of the surface point at (z, theta).
    [[nodiscard]] Vec2d cellAt(double z, double theta) const;
    /// Height of the ground at (z, theta), as drawn.
    [[nodiscard]] double groundHeight(double z, double theta) const;
    /// The water's surface on a row, measured like the heights (kWaterLevelM on a level floor).
    [[nodiscard]] double waterLevelAtRow(std::uint32_t row) const;
};

/// Samples heights (and land cover) over the whole surface on all cores. Deterministic.
[[nodiscard]] TerrainGrid sampleTerrain(const HabitatGeometry& geometry, double targetCellM,
                                        unsigned threads = 0);

}  // namespace StarshipSimulator
