#pragma once

#include <cstddef>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/star_field.h"

// The real stars, from the HYG database (v4.4, CC BY-SA 4.0, https://codeberg.org/astronexus/hyg).
namespace StarshipSimulator::astro
{

struct CatalogStar
{
    Vec3d       direction{1.0, 0.0, 0.0};  // unit, EQJ (J2000 equator and equinox)
    double      magnitude       = 0.0;     // apparent visual magnitude from Earth
    double      colorIndex      = 0.65;    // B-V
    double      distanceParsecs = 0.0;     // 0 when unknown
    int         hip             = 0;       // Hipparcos number, 0 if none
    std::string properName;                // "Betelgeuse"; empty for most stars
    std::string designation;               // "Alpha Orionis", "61 Cygni" or "HIP 12345"
    std::string constellation;             // IAU abbreviation, e.g. "Ori"
};

struct StarCatalog
{
    std::vector<CatalogStar> stars;  // brightest first

    /// The star someone pointing along `direction` most likely means: close to the direction, and
    /// bright rather than faint. Only stars within `maxAngle` radians are considered.
    [[nodiscard]] std::optional<std::size_t> identify(const Vec3d& direction,
                                                      double       maxAngle) const;

    /// The star with this proper name or designation ("Vega", "alpha lyrae"; any letter case).
    [[nodiscard]] const CatalogStar* find(std::string_view name) const;
};

/// How well a point of light `angle` radians from the crosshair matches what someone is pointing
/// at: lower is better, and below 1 counts as a match. Bright stars are easier targets.
[[nodiscard]] double pointingScore(double angle, double magnitude);

/// Parses the HYG CSV (header row first), keeping stars at or brighter than `magnitudeLimit`.
[[nodiscard]] std::expected<StarCatalog, std::string> parseHygCatalog(std::string_view csv,
                                                                      double magnitudeLimit);
/// Loads a HYG .csv or .csv.gz file.
[[nodiscard]] std::expected<StarCatalog, std::string> loadHygCatalog(
    const std::filesystem::path& path, double magnitudeLimit);

/// "Betelgeuse", else the designation.
[[nodiscard]] const std::string& displayName(const CatalogStar& star);

/// Effective temperature from the B-V colour index (Ballesteros 2012).
[[nodiscard]] double colorIndexToKelvin(double colorIndex);

/// Point sprites for the renderer: brightness from magnitude, colour from temperature.
[[nodiscard]] std::vector<GpuStar> toGpuStars(const StarCatalog& catalog);
[[nodiscard]] GpuStar              gpuStar(const Vec3d& direction, double magnitude, double kelvin);

/// "Orion" and "Orionis" for "Ori"; empty if unknown.
[[nodiscard]] std::string_view constellationName(std::string_view abbreviation);
[[nodiscard]] std::string_view constellationGenitive(std::string_view abbreviation);

}  // namespace StarshipSimulator::astro
