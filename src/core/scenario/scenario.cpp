#include "StarshipSimulator/core/scenario/scenario.h"

#include <cmath>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include <toml++/toml.hpp>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"

namespace StarshipSimulator
{

namespace
{

int lineOf(const toml::node& node)
{
    return static_cast<int>(node.source().begin.line);
}

/// "earth_moon_l4, earth_moon_l5, ..."
std::string locationKeyList()
{
    std::string keys;
    for (const astro::Location location : astro::allLocations())
    {
        keys += std::format("{}{}", keys.empty() ? "" : ", ", astro::locationKey(location));
    }
    return keys;
}

/// Reads typed values from one TOML table, remembering the first error.
class TableReader
{
public:
    TableReader(const toml::table& table, std::string name, std::optional<ScenarioError>& error)
      : table_(&table), name_(std::move(name)), error_(&error)
    {
    }

    /// Fails on keys that are not in the allowed list (catches typos).
    void allowOnly(std::initializer_list<std::string_view> keys)
    {
        for (const auto& [key, node] : *table_)
        {
            bool known = false;
            for (const std::string_view allowed : keys)
            {
                known = known || key.str() == allowed;
            }
            if (!known)
            {
                fail(std::format("unknown key '{}' in [{}]", key.str(), name_), lineOf(node));
            }
        }
    }

    void read(std::string_view key, double& value)
    {
        if (const toml::node* node = table_->get(key))
        {
            if (const auto number = node->value<double>();
                number && (node->is_floating_point() || node->is_integer()))
            {
                value = *number;
            }
            else
            {
                fail(std::format("'{}' in [{}] must be a number", key, name_), lineOf(*node));
            }
        }
    }

    void read(std::string_view key, bool& value)
    {
        if (const toml::node* node = table_->get(key))
        {
            if (const auto flag = node->value_exact<bool>())
            {
                value = *flag;
            }
            else
            {
                fail(std::format("'{}' in [{}] must be true or false", key, name_), lineOf(*node));
            }
        }
    }

    void read(std::string_view key, int& value)
    {
        if (const toml::node* node = table_->get(key))
        {
            if (const auto number = node->value_exact<std::int64_t>())
            {
                value = static_cast<int>(*number);
            }
            else
            {
                fail(std::format("'{}' in [{}] must be a whole number", key, name_), lineOf(*node));
            }
        }
    }

    void read(std::string_view key, std::uint64_t& value)
    {
        if (const toml::node* node = table_->get(key))
        {
            if (const auto number = node->value_exact<std::int64_t>(); number && *number >= 0)
            {
                value = static_cast<std::uint64_t>(*number);
            }
            else
            {
                fail(std::format("'{}' in [{}] must be a non-negative whole number", key, name_),
                     lineOf(*node));
            }
        }
    }

    void read(std::string_view key, std::string& value)
    {
        if (const toml::node* node = table_->get(key))
        {
            if (const auto text = node->value_exact<std::string>())
            {
                value = *text;
            }
            else
            {
                fail(std::format("'{}' in [{}] must be a string", key, name_), lineOf(*node));
            }
        }
    }

    void read(std::string_view key, EndcapShape& value)
    {
        const toml::node* node = table_->get(key);
        if (node == nullptr)
        {
            return;
        }
        std::string text;
        read(key, text);
        for (const EndcapShape shape :
             {EndcapShape::Flat, EndcapShape::Hemisphere, EndcapShape::ConicalRamp})
        {
            if (text == endcapShapeName(shape))
            {
                value = shape;
                return;
            }
        }
        fail(std::format("'{}' in [{}] must be flat, hemisphere or conical_ramp", key, name_),
             lineOf(*node));
    }

    void read(std::string_view key, astro::Location& value)
    {
        const toml::node* node = table_->get(key);
        if (node == nullptr)
        {
            return;
        }
        std::string text;
        read(key, text);
        if (const auto location = astro::locationFromKey(text))
        {
            value = *location;
            return;
        }
        fail(std::format("'{}' in [{}] must be one of {}", key, name_, locationKeyList()),
             lineOf(*node));
    }

    void read(std::string_view key, astro::SimTime& value)
    {
        const toml::node* node = table_->get(key);
        if (node == nullptr)
        {
            return;
        }
        std::string text;
        read(key, text);
        if (const auto time = astro::parseIsoTime(text))
        {
            value = *time;
        }
        else
        {
            fail(std::format("'{}' in [{}]: {}", key, name_, time.error()), lineOf(*node));
        }
    }

    /// A sub-table, if present (an error if the key exists but is not a table).
    [[nodiscard]] const toml::table* table(std::string_view key)
    {
        const toml::node* node = table_->get(key);
        if (node == nullptr)
        {
            return nullptr;
        }
        if (!node->is_table())
        {
            fail(std::format("'{}' in [{}] must be a table", key, name_), lineOf(*node));
            return nullptr;
        }
        return node->as_table();
    }

    [[nodiscard]] int line() const { return lineOf(*table_); }

private:
    void fail(std::string message, int line)
    {
        if (!*error_)
        {
            *error_ = ScenarioError{.message = std::move(message), .line = line};
        }
    }

    const toml::table*            table_;
    std::string                   name_;
    std::optional<ScenarioError>* error_;
};

void readEndcap(TableReader& parent, std::string_view key, const std::string& name,
                EndcapSpec& endcap, std::optional<ScenarioError>& error)
{
    const toml::table* table = parent.table(key);
    if (table == nullptr)
    {
        return;
    }
    TableReader reader(*table, name, error);
    reader.allowOnly(
        {"shape", "ramp_slope_deg", "ramp_top_radius_fraction", "upper_slope_deg", "hub_radius_m"});
    reader.read("shape", endcap.shape);
    reader.read("ramp_slope_deg", endcap.rampSlopeDeg);
    reader.read("ramp_top_radius_fraction", endcap.rampTopRadiusFraction);
    reader.read("upper_slope_deg", endcap.upperSlopeDeg);
    reader.read("hub_radius_m", endcap.hubRadiusM);
}

void readHabitat(const toml::table& table, OneillCylinderSpec& spec,
                 std::optional<ScenarioError>& error)
{
    TableReader habitat(table, "habitat", error);
    habitat.allowOnly({"type", "radius_m", "length_m", "surface_gravity_g", "strip_pairs",
                       "window_fraction", "population_density_per_km2", "sunward_endcap",
                       "antisunward_endcap", "mirrors", "partner", "atmosphere", "terrain"});
    std::string type = "oneill_cylinder";
    habitat.read("type", type);
    if (type != "oneill_cylinder" && !error)
    {
        error = ScenarioError{
            .message =
                std::format("habitat type '{}' is not supported yet (only oneill_cylinder)", type),
            .line = habitat.line()};
    }
    habitat.read("radius_m", spec.radiusM);
    habitat.read("length_m", spec.lengthM);
    habitat.read("surface_gravity_g", spec.surfaceGravityG);
    habitat.read("strip_pairs", spec.stripPairs);
    habitat.read("window_fraction", spec.windowFraction);
    habitat.read("population_density_per_km2", spec.populationDensityPerKm2);
    readEndcap(habitat, "sunward_endcap", "habitat.sunward_endcap", spec.sunwardEndcap, error);
    readEndcap(habitat, "antisunward_endcap", "habitat.antisunward_endcap", spec.antisunwardEndcap,
               error);

    if (const toml::table* mirrors = habitat.table("mirrors"))
    {
        TableReader reader(*mirrors, "habitat.mirrors", error);
        reader.allowOnly({"opening_angle_deg", "reflectivity"});
        reader.read("opening_angle_deg", spec.mirrors.openingAngleDeg);
        reader.read("reflectivity", spec.mirrors.reflectivity);
    }
    if (const toml::table* partner = habitat.table("partner"))
    {
        TableReader reader(*partner, "habitat.partner", error);
        reader.allowOnly({"enabled", "separation_m"});
        reader.read("enabled", spec.partner.enabled);
        reader.read("separation_m", spec.partner.separationM);
    }
    if (const toml::table* atmosphere = habitat.table("atmosphere"))
    {
        TableReader reader(*atmosphere, "habitat.atmosphere", error);
        reader.allowOnly({"surface_pressure_kpa", "temperature_k"});
        double kilopascals = spec.atmosphere.surfacePressurePa / 1000.0;
        reader.read("surface_pressure_kpa", kilopascals);
        spec.atmosphere.surfacePressurePa = kilopascals * 1000.0;
        reader.read("temperature_k", spec.atmosphere.temperatureK);
    }
    if (const toml::table* terrain = habitat.table("terrain"))
    {
        TableReader reader(*terrain, "habitat.terrain", error);
        reader.allowOnly({"seed", "hill_height_m", "mountain_height_m", "feature_size_m",
                          "river_width_m", "lakes_per_valley", "lake_radius_m", "forest_cover"});
        reader.read("seed", spec.terrain.seed);
        reader.read("hill_height_m", spec.terrain.hillHeightM);
        reader.read("mountain_height_m", spec.terrain.mountainHeightM);
        reader.read("feature_size_m", spec.terrain.featureSizeM);
        reader.read("river_width_m", spec.terrain.riverWidthM);
        reader.read("lakes_per_valley", spec.terrain.lakesPerValley);
        reader.read("lake_radius_m", spec.terrain.lakeRadiusM);
        reader.read("forest_cover", spec.terrain.forestCover);
    }
}

/// A TOML float that reads back exactly (always with a decimal point or exponent).
std::string number(double value)
{
    std::string text = std::format("{}", value);
    if (text.find_first_of(".eEn") == std::string::npos)
    {
        text += ".0";
    }
    return text;
}

std::string tomlString(std::string_view text)
{
    std::string out = "\"";
    for (const char c : text)
    {
        switch (c)
        {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            default:
                out += c;
        }
    }
    return out + "\"";
}

void readSkyAndDay(TableReader& top, Scenario& scenario, std::optional<ScenarioError>& error)
{
    if (const toml::table* sky = top.table("sky"))
    {
        TableReader reader(*sky, "sky", error);
        reader.allowOnly({"location", "start", "utc_offset_hours"});
        reader.read("location", scenario.sky.location);
        reader.read("start", scenario.sky.start);
        reader.read("utc_offset_hours", scenario.sky.utcOffsetHours);
    }
    if (const toml::table* day = top.table("day"))
    {
        TableReader reader(*day, "day", error);
        reader.allowOnly(
            {"enabled", "day_length_hours", "sunrise_hour", "noon_angle_deg", "night_angle_deg"});
        reader.read("enabled", scenario.day.enabled);
        reader.read("day_length_hours", scenario.day.dayLengthHours);
        reader.read("sunrise_hour", scenario.day.sunriseHour);
        reader.read("noon_angle_deg", scenario.day.noonAngleDeg);
        reader.read("night_angle_deg", scenario.day.nightAngleDeg);
    }
}

/// Problems with the sky and day settings (the habitat has its own validate()).
std::optional<std::string> validateSkyAndDay(const Scenario& scenario)
{
    const DayScheduleSpec& day = scenario.day;
    if (std::abs(scenario.sky.utcOffsetHours) > 14.0)
    {
        return "utc_offset_hours must be between -14 and 14";
    }
    if (day.dayLengthHours < 1.0 || day.dayLengthHours > 23.0 || day.sunriseHour < 0.0 ||
        day.sunriseHour >= 24.0)
    {
        return "the day must last 1..23 hours and start at an hour of 0..24";
    }
    if (day.noonAngleDeg < 20.0 || day.noonAngleDeg > 85.0 || day.nightAngleDeg < 90.0 ||
        day.nightAngleDeg > 150.0)
    {
        return "the noon mirror angle must be 20..85 degrees and the night angle 90..150";
    }
    return std::nullopt;
}

std::string boolean(bool value)
{
    return value ? "true" : "false";
}

void writeEndcap(std::string& out, std::string_view table, const EndcapSpec& endcap)
{
    out += std::format("\n[habitat.{}]\nshape = {}\n", table,
                       tomlString(endcapShapeName(endcap.shape)));
    if (endcap.shape == EndcapShape::ConicalRamp)
    {
        out += std::format(
            "ramp_slope_deg = {}\nramp_top_radius_fraction = {}\nupper_slope_deg = {}\n"
            "hub_radius_m = {}\n",
            number(endcap.rampSlopeDeg), number(endcap.rampTopRadiusFraction),
            number(endcap.upperSlopeDeg), number(endcap.hubRadiusM));
    }
}

}  // namespace

astro::SimTime SkySpec::defaultStartTime()
{
    return astro::fromCalendar(
        {.year = 2045, .month = 6, .day = 15, .hour = 9, .minute = 0, .second = 0.0});
}

std::string ScenarioError::describe() const
{
    return line > 0 ? std::format("line {}: {}", line, message) : message;
}

std::expected<Scenario, ScenarioError> parseScenario(std::string_view toml)
{
    toml::table root;
    try
    {
        root = toml::parse(toml);
    }
    catch (const toml::parse_error& e)
    {
        return std::unexpected(ScenarioError{.message = std::string(e.description()),
                                             .line    = static_cast<int>(e.source().begin.line)});
    }

    Scenario                     scenario;
    std::optional<ScenarioError> error;
    TableReader                  top(root, "top level", error);
    top.allowOnly({"format_version", "generator_version", "title", "description", "habitat",
                   "start", "sky", "day"});
    top.read("format_version", scenario.formatVersion);
    top.read("generator_version", scenario.generatorVersion);
    top.read("title", scenario.title);
    top.read("description", scenario.description);
    if (!error && scenario.formatVersion > Scenario::kFormatVersion)
    {
        return std::unexpected(ScenarioError{
            .message = std::format("file format {} is newer than this program supports ({})",
                                   scenario.formatVersion, Scenario::kFormatVersion),
            .line    = 0});
    }
    if (const toml::table* habitat = top.table("habitat"))
    {
        readHabitat(*habitat, scenario.habitat, error);
    }
    if (const toml::table* start = top.table("start"))
    {
        TableReader reader(*start, "start", error);
        reader.allowOnly({"valley", "z_m", "heading_deg"});
        reader.read("valley", scenario.start.valley);
        reader.read("z_m", scenario.start.zM);
        reader.read("heading_deg", scenario.start.headingDeg);
    }
    readSkyAndDay(top, scenario, error);
    if (error)
    {
        return std::unexpected(*error);
    }
    if (const auto problem = validateSkyAndDay(scenario))
    {
        return std::unexpected(ScenarioError{.message = *problem, .line = 0});
    }
    if (const auto problems = validate(scenario.habitat); !problems.empty())
    {
        const toml::node* habitat = root.get("habitat");
        return std::unexpected(ScenarioError{.message = problems.front(),
                                             .line    = habitat != nullptr ? lineOf(*habitat) : 0});
    }
    return scenario;
}

std::string serializeScenario(const Scenario& scenario)
{
    const OneillCylinderSpec& spec = scenario.habitat;
    std::string               out;
    out += "# StarshipSimulator habitat scenario\n";
    out +=
        std::format("format_version = {}\ngenerator_version = {}\ntitle = {}\n",
                    scenario.formatVersion, scenario.generatorVersion, tomlString(scenario.title));
    if (!scenario.description.empty())
    {
        out += std::format("description = {}\n", tomlString(scenario.description));
    }
    out += "\n# Spin axis +Z points at the Sun. Gravity comes from spin: omega^2 * radius.\n";
    out += "[habitat]\ntype = \"oneill_cylinder\"\n";
    out += std::format(
        "radius_m = {}\nlength_m = {}\nsurface_gravity_g = {}\nstrip_pairs = {}\n"
        "window_fraction = {}\npopulation_density_per_km2 = {}\n",
        number(spec.radiusM), number(spec.lengthM), number(spec.surfaceGravityG), spec.stripPairs,
        number(spec.windowFraction), number(spec.populationDensityPerKm2));
    writeEndcap(out, "sunward_endcap", spec.sunwardEndcap);
    writeEndcap(out, "antisunward_endcap", spec.antisunwardEndcap);
    out += "\n# 45 degrees puts the sun overhead; 90 is sunset.\n";
    out += std::format("[habitat.mirrors]\nopening_angle_deg = {}\nreflectivity = {}\n",
                       number(spec.mirrors.openingAngleDeg), number(spec.mirrors.reflectivity));
    out += "\n# The counter-rotating partner cylinder, alongside (axis to axis).\n";
    out += std::format("[habitat.partner]\nenabled = {}\nseparation_m = {}\n",
                       boolean(spec.partner.enabled), number(spec.partner.separationM));
    out += std::format("\n[habitat.atmosphere]\nsurface_pressure_kpa = {}\ntemperature_k = {}\n",
                       number(spec.atmosphere.surfacePressurePa / 1000.0),
                       number(spec.atmosphere.temperatureK));
    const TerrainSpec& terrain = spec.terrain;
    out += std::format(
        "\n[habitat.terrain]\nseed = {}\nhill_height_m = {}\nmountain_height_m = {}\n"
        "feature_size_m = {}\nriver_width_m = {}\nlakes_per_valley = {}\nlake_radius_m = {}\n"
        "forest_cover = {}\n",
        terrain.seed, number(terrain.hillHeightM), number(terrain.mountainHeightM),
        number(terrain.featureSizeM), number(terrain.riverWidthM), terrain.lakesPerValley,
        number(terrain.lakeRadiusM), number(terrain.forestCover));
    out +=
        std::format("\n[start]\nvalley = {}\nz_m = {}\nheading_deg = {}\n", scenario.start.valley,
                    number(scenario.start.zM), number(scenario.start.headingDeg));
    out += std::format("\n# Where the habitat is and when the visit begins (UTC).\n# Locations: {}",
                       locationKeyList());
    out += std::format("\n[sky]\nlocation = {}\nstart = {}\nutc_offset_hours = {}\n",
                       tomlString(astro::locationKey(scenario.sky.location)),
                       tomlString(astro::formatIsoTime(scenario.sky.start)),
                       number(scenario.sky.utcOffsetHours));
    const DayScheduleSpec& day = scenario.day;
    out += "\n# The mirrors' daily swing (local time): 90 degrees at sunrise and sunset.\n";
    out += std::format(
        "[day]\nenabled = {}\nday_length_hours = {}\nsunrise_hour = {}\nnoon_angle_deg = {}\n"
        "night_angle_deg = {}\n",
        boolean(day.enabled), number(day.dayLengthHours), number(day.sunriseHour),
        number(day.noonAngleDeg), number(day.nightAngleDeg));
    return out;
}

std::expected<Scenario, ScenarioError> loadScenario(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        return std::unexpected(
            ScenarioError{.message = std::format("cannot open {}", path.string()), .line = 0});
    }
    const std::string text((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
    auto              scenario = parseScenario(text);
    if (!scenario)
    {
        scenario.error().message =
            std::format("{}: {}", path.filename().string(), scenario.error().message);
    }
    return scenario;
}

std::expected<void, std::string> saveScenario(const Scenario&              scenario,
                                              const std::filesystem::path& path)
{
    std::error_code ignored;
    if (path.has_parent_path())
    {
        std::filesystem::create_directories(path.parent_path(), ignored);
    }
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file)
    {
        return std::unexpected(std::format("cannot write {}", path.string()));
    }
    file << serializeScenario(scenario);
    if (!file)
    {
        return std::unexpected(std::format("failed writing {}", path.string()));
    }
    return {};
}

}  // namespace StarshipSimulator
