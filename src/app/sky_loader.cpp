#include "sky_loader.h"

#include <chrono>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <format>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/assets/assets.h"
#include "StarshipSimulator/core/astro/star_catalog.h"
#include "StarshipSimulator/core/utf8_path.h"
#include "StarshipSimulator/render/Renderer.h"

namespace StarshipSimulator
{

namespace
{

constexpr const char* kCatalogFile    = "hyg_v44.csv.gz";
constexpr const char* kMilkyWayFile   = "milkyway_2020_4k.exr";
constexpr const char* kEarthDayFile   = "earth_day_2048.jpg";
constexpr const char* kEarthNightFile = "earth_night_3600.jpg";
constexpr const char* kMoonFile       = "moon_1k.jpg";

std::expected<assets::Image8, std::string> loadImage(const std::filesystem::path& path)
{
    return assets::readFile(path).and_then(
        [](const std::vector<std::byte>& bytes) { return assets::decodeImage(bytes); });
}

/// Stores a loaded file, or notes why it could not be loaded.
template <typename T>
void keep(std::expected<T, std::string> loaded, std::optional<T>& into,
          const std::filesystem::path& path, std::vector<std::string>& problems)
{
    if (loaded)
    {
        into = std::move(*loaded);
    }
    else
    {
        problems.push_back(std::format("{}: {}", utf8String(path.filename()), loaded.error()));
    }
}

}  // namespace

std::filesystem::path findSkyDataDirectory(const std::filesystem::path& dataDirectory)
{
    std::error_code error;
    auto            installed = dataDirectory / "sky";
    if (std::filesystem::exists(installed / kCatalogFile, error))
    {
        return installed;
    }
#ifdef STARSHIPSIMULATOR_SKY_DATA_DIR
    return pathFromUtf8(STARSHIPSIMULATOR_SKY_DATA_DIR);
#else
    return installed;
#endif
}

SkyData loadSkyData(const std::filesystem::path& directory, double magnitudeLimit)
{
    const auto start = std::chrono::steady_clock::now();
    SkyData    data;
    keep(astro::loadHygCatalog(directory / kCatalogFile, magnitudeLimit), data.catalog,
         directory / kCatalogFile, data.problems);
    keep(
        assets::readFile(directory / kMilkyWayFile)
            .and_then([](const std::vector<std::byte>& bytes) { return assets::decodeExr(bytes); }),
        data.images.milkyWay, directory / kMilkyWayFile, data.problems);
    keep(loadImage(directory / kEarthDayFile), data.images.earthDay, directory / kEarthDayFile,
         data.problems);
    keep(loadImage(directory / kEarthNightFile), data.images.earthNight,
         directory / kEarthNightFile, data.problems);
    keep(loadImage(directory / kMoonFile), data.images.moon, directory / kMoonFile, data.problems);
    data.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return data;
}

}  // namespace StarshipSimulator
