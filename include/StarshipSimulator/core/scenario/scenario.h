#pragma once

#include <expected>
#include <filesystem>
#include <string>
#include <string_view>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"

namespace StarshipSimulator
{

/// Where the visitor starts: on the floor of a valley, at an axial position, facing a heading
/// (0 = toward the sunward end, positive turning left).
struct StartSpec
{
    int    valley     = 1;
    double zM         = 0.0;
    double headingDeg = 0.0;
};

/// Where in the solar system the habitat is, and when the visit starts.
struct SkySpec
{
    astro::Location location       = astro::Location::EarthMoonL5;
    astro::SimTime  start          = defaultStartTime();
    double          utcOffsetHours = 0.0;  // the habitat's local clock (drives the day schedule)

    // 2045-06-15 09:00 UTC: morning, with Earth well clear of the axis in the sky.
    [[nodiscard]] static astro::SimTime defaultStartTime();
};

/// A shareable habitat file: everything needed to regenerate the same world.
struct Scenario
{
    static constexpr int kFormatVersion    = 1;  // file layout
    static constexpr int kGeneratorVersion = 2;  // terrain generation (2: rivers, lakes, woods)

    int                formatVersion    = kFormatVersion;
    int                generatorVersion = kGeneratorVersion;
    std::string        title            = "Untitled habitat";
    std::string        description;
    OneillCylinderSpec habitat;
    StartSpec          start;
    SkySpec            sky;
    DayScheduleSpec    day;
};

struct ScenarioError
{
    std::string message;
    int         line = 0;  // 1-based line in the file, 0 when not tied to a line

    [[nodiscard]] std::string describe() const;
};

/// Parses a scenario from TOML. Unknown keys, wrong types and invalid habitats are errors.
[[nodiscard]] std::expected<Scenario, ScenarioError> parseScenario(std::string_view toml);

/// Writes a scenario as commented TOML that parseScenario reads back identically.
[[nodiscard]] std::string serializeScenario(const Scenario& scenario);

[[nodiscard]] std::expected<Scenario, ScenarioError> loadScenario(
    const std::filesystem::path& path);
[[nodiscard]] std::expected<void, std::string> saveScenario(const Scenario&              scenario,
                                                            const std::filesystem::path& path);

}  // namespace StarshipSimulator
