#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "StarshipSimulator/core/astro/star_catalog.h"
#include "StarshipSimulator/render/Renderer.h"

namespace StarshipSimulator
{

/// The sky's data files, decoded (on a worker thread; uploading happens on the main thread).
struct SkyData
{
    std::optional<astro::StarCatalog> catalog;
    SkyImages                         images;
    std::vector<std::string>          problems;  // files that were missing or unreadable
    double                            seconds = 0.0;
};

/// Where the sky data is: bin/data/sky next to the executable (installed builds), else the
/// download cache the build was configured with.
[[nodiscard]] std::filesystem::path findSkyDataDirectory(
    const std::filesystem::path& dataDirectory);

/// Loads the star catalog (down to the given magnitude) and the sky's images.
[[nodiscard]] SkyData loadSkyData(const std::filesystem::path& directory, double magnitudeLimit);

}  // namespace StarshipSimulator
