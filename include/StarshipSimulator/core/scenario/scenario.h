#pragma once

#include <expected>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/weather.h"

namespace StarshipSimulator
{

/// Where the visitor starts: on a band of land (an O'Neill cylinder's valley; the only band
/// elsewhere), `alongM` along it and `acrossM` across it from its middle line, facing a heading.
/// Along an O'Neill cylinder's valley is z itself, and heading 0 faces its sunward end.
struct StartSpec
{
    int    band       = 1;
    double alongM     = 0.0;
    double acrossM    = 0.0;
    double headingDeg = 0.0;
};

/// Where in the solar system the habitat is, and when the visit starts.
struct SkySpec
{
    astro::Location location       = astro::Location::EARTH_MOON_L5;
    astro::SimTime  start          = defaultStartTime();
    double          utcOffsetHours = 0.0;  // the habitat's local clock (drives the day schedule)

    // 2045-06-15 09:00 UTC: morning, with Earth well clear of the axis in the sky.
    [[nodiscard]] static astro::SimTime defaultStartTime();
};

/// A shareable habitat file: everything needed to regenerate the same world.
struct Scenario
{
    static constexpr int kFormatVersion = 1;  // file layout
    // World generation: 2 added rivers, lakes and woods; 3 towns and farms; 4 the tramway's
    // earthworks, and fixing draws that came out in a different order under another compiler;
    // 5 Kalpana One, whose land runs round the axis. (M8's habitat kinds leave the O'Neill
    // cylinder's world exactly as version 4 made it.)
    static constexpr int kGeneratorVersion = 5;

    int             formatVersion    = kFormatVersion;
    int             generatorVersion = kGeneratorVersion;
    std::string     title            = "Untitled habitat";
    std::string     description;
    HabitatSpec     habitat;
    StartSpec       start;
    SkySpec         sky;
    DayScheduleSpec day;
    ClimateSpec     climate;
};

struct ScenarioError
{
    std::string message;
    int         line = 0;  // 1-based line in the file, 0 when not tied to a line

    [[nodiscard]] std::string describe() const;
};

/// A habitat in one line, for a list of them: how big it is, how hard it pulls, how much land it
/// holds and what its hull would have to be made of.
[[nodiscard]] std::string describeHabitat(const HabitatSpec& spec);

/// Everything that stops a scenario being built, as human-readable messages (empty when it is
/// ready to open). The habitat, its day, its climate and where the visit starts are all checked.
[[nodiscard]] std::vector<std::string> validateScenario(const Scenario& scenario);

/// Parses a scenario from TOML. Unknown keys, wrong types and invalid habitats are errors.
[[nodiscard]] std::expected<Scenario, ScenarioError> parseScenario(std::string_view toml);

/// Writes a scenario as commented TOML that parseScenario reads back identically.
[[nodiscard]] std::string serializeScenario(const Scenario& scenario);

[[nodiscard]] std::expected<Scenario, ScenarioError> loadScenario(
    const std::filesystem::path& path);
[[nodiscard]] std::expected<void, std::string> saveScenario(const Scenario&              scenario,
                                                            const std::filesystem::path& path);

}  // namespace StarshipSimulator
