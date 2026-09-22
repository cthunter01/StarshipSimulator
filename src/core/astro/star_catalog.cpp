#include "StarshipSimulator/core/astro/star_catalog.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/assets/assets.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/parse_number.h"
#include "StarshipSimulator/core/procgen/star_field.h"
#include "StarshipSimulator/core/utf8_path.h"

namespace StarshipSimulator::astro
{

namespace
{

constexpr double kUnknownDistanceParsecs = 100000.0;  // HYG's placeholder for "no parallax"

struct Constellation
{
    std::string_view abbreviation;
    std::string_view name;
    std::string_view genitive;
};

constexpr std::array<Constellation, 88> kConstellations{{
    {.abbreviation = "And", .name = "Andromeda", .genitive = "Andromedae"},
    {.abbreviation = "Ant", .name = "Antlia", .genitive = "Antliae"},
    {.abbreviation = "Aps", .name = "Apus", .genitive = "Apodis"},
    {.abbreviation = "Aqr", .name = "Aquarius", .genitive = "Aquarii"},
    {.abbreviation = "Aql", .name = "Aquila", .genitive = "Aquilae"},
    {.abbreviation = "Ara", .name = "Ara", .genitive = "Arae"},
    {.abbreviation = "Ari", .name = "Aries", .genitive = "Arietis"},
    {.abbreviation = "Aur", .name = "Auriga", .genitive = "Aurigae"},
    {.abbreviation = "Boo", .name = "Bootes", .genitive = "Bootis"},
    {.abbreviation = "Cae", .name = "Caelum", .genitive = "Caeli"},
    {.abbreviation = "Cam", .name = "Camelopardalis", .genitive = "Camelopardalis"},
    {.abbreviation = "Cnc", .name = "Cancer", .genitive = "Cancri"},
    {.abbreviation = "CVn", .name = "Canes Venatici", .genitive = "Canum Venaticorum"},
    {.abbreviation = "CMa", .name = "Canis Major", .genitive = "Canis Majoris"},
    {.abbreviation = "CMi", .name = "Canis Minor", .genitive = "Canis Minoris"},
    {.abbreviation = "Cap", .name = "Capricornus", .genitive = "Capricorni"},
    {.abbreviation = "Car", .name = "Carina", .genitive = "Carinae"},
    {.abbreviation = "Cas", .name = "Cassiopeia", .genitive = "Cassiopeiae"},
    {.abbreviation = "Cen", .name = "Centaurus", .genitive = "Centauri"},
    {.abbreviation = "Cep", .name = "Cepheus", .genitive = "Cephei"},
    {.abbreviation = "Cet", .name = "Cetus", .genitive = "Ceti"},
    {.abbreviation = "Cha", .name = "Chamaeleon", .genitive = "Chamaeleontis"},
    {.abbreviation = "Cir", .name = "Circinus", .genitive = "Circini"},
    {.abbreviation = "Col", .name = "Columba", .genitive = "Columbae"},
    {.abbreviation = "Com", .name = "Coma Berenices", .genitive = "Comae Berenices"},
    {.abbreviation = "CrA", .name = "Corona Australis", .genitive = "Coronae Australis"},
    {.abbreviation = "CrB", .name = "Corona Borealis", .genitive = "Coronae Borealis"},
    {.abbreviation = "Crv", .name = "Corvus", .genitive = "Corvi"},
    {.abbreviation = "Crt", .name = "Crater", .genitive = "Crateris"},
    {.abbreviation = "Cru", .name = "Crux", .genitive = "Crucis"},
    {.abbreviation = "Cyg", .name = "Cygnus", .genitive = "Cygni"},
    {.abbreviation = "Del", .name = "Delphinus", .genitive = "Delphini"},
    {.abbreviation = "Dor", .name = "Dorado", .genitive = "Doradus"},
    {.abbreviation = "Dra", .name = "Draco", .genitive = "Draconis"},
    {.abbreviation = "Equ", .name = "Equuleus", .genitive = "Equulei"},
    {.abbreviation = "Eri", .name = "Eridanus", .genitive = "Eridani"},
    {.abbreviation = "For", .name = "Fornax", .genitive = "Fornacis"},
    {.abbreviation = "Gem", .name = "Gemini", .genitive = "Geminorum"},
    {.abbreviation = "Gru", .name = "Grus", .genitive = "Gruis"},
    {.abbreviation = "Her", .name = "Hercules", .genitive = "Herculis"},
    {.abbreviation = "Hor", .name = "Horologium", .genitive = "Horologii"},
    {.abbreviation = "Hya", .name = "Hydra", .genitive = "Hydrae"},
    {.abbreviation = "Hyi", .name = "Hydrus", .genitive = "Hydri"},
    {.abbreviation = "Ind", .name = "Indus", .genitive = "Indi"},
    {.abbreviation = "Lac", .name = "Lacerta", .genitive = "Lacertae"},
    {.abbreviation = "Leo", .name = "Leo", .genitive = "Leonis"},
    {.abbreviation = "LMi", .name = "Leo Minor", .genitive = "Leonis Minoris"},
    {.abbreviation = "Lep", .name = "Lepus", .genitive = "Leporis"},
    {.abbreviation = "Lib", .name = "Libra", .genitive = "Librae"},
    {.abbreviation = "Lup", .name = "Lupus", .genitive = "Lupi"},
    {.abbreviation = "Lyn", .name = "Lynx", .genitive = "Lyncis"},
    {.abbreviation = "Lyr", .name = "Lyra", .genitive = "Lyrae"},
    {.abbreviation = "Men", .name = "Mensa", .genitive = "Mensae"},
    {.abbreviation = "Mic", .name = "Microscopium", .genitive = "Microscopii"},
    {.abbreviation = "Mon", .name = "Monoceros", .genitive = "Monocerotis"},
    {.abbreviation = "Mus", .name = "Musca", .genitive = "Muscae"},
    {.abbreviation = "Nor", .name = "Norma", .genitive = "Normae"},
    {.abbreviation = "Oct", .name = "Octans", .genitive = "Octantis"},
    {.abbreviation = "Oph", .name = "Ophiuchus", .genitive = "Ophiuchi"},
    {.abbreviation = "Ori", .name = "Orion", .genitive = "Orionis"},
    {.abbreviation = "Pav", .name = "Pavo", .genitive = "Pavonis"},
    {.abbreviation = "Peg", .name = "Pegasus", .genitive = "Pegasi"},
    {.abbreviation = "Per", .name = "Perseus", .genitive = "Persei"},
    {.abbreviation = "Phe", .name = "Phoenix", .genitive = "Phoenicis"},
    {.abbreviation = "Pic", .name = "Pictor", .genitive = "Pictoris"},
    {.abbreviation = "Psc", .name = "Pisces", .genitive = "Piscium"},
    {.abbreviation = "PsA", .name = "Piscis Austrinus", .genitive = "Piscis Austrini"},
    {.abbreviation = "Pup", .name = "Puppis", .genitive = "Puppis"},
    {.abbreviation = "Pyx", .name = "Pyxis", .genitive = "Pyxidis"},
    {.abbreviation = "Ret", .name = "Reticulum", .genitive = "Reticuli"},
    {.abbreviation = "Sge", .name = "Sagitta", .genitive = "Sagittae"},
    {.abbreviation = "Sgr", .name = "Sagittarius", .genitive = "Sagittarii"},
    {.abbreviation = "Sco", .name = "Scorpius", .genitive = "Scorpii"},
    {.abbreviation = "Scl", .name = "Sculptor", .genitive = "Sculptoris"},
    {.abbreviation = "Sct", .name = "Scutum", .genitive = "Scuti"},
    {.abbreviation = "Ser", .name = "Serpens", .genitive = "Serpentis"},
    {.abbreviation = "Sex", .name = "Sextans", .genitive = "Sextantis"},
    {.abbreviation = "Tau", .name = "Taurus", .genitive = "Tauri"},
    {.abbreviation = "Tel", .name = "Telescopium", .genitive = "Telescopii"},
    {.abbreviation = "Tri", .name = "Triangulum", .genitive = "Trianguli"},
    {.abbreviation = "TrA", .name = "Triangulum Australe", .genitive = "Trianguli Australis"},
    {.abbreviation = "Tuc", .name = "Tucana", .genitive = "Tucanae"},
    {.abbreviation = "UMa", .name = "Ursa Major", .genitive = "Ursae Majoris"},
    {.abbreviation = "UMi", .name = "Ursa Minor", .genitive = "Ursae Minoris"},
    {.abbreviation = "Vel", .name = "Vela", .genitive = "Velorum"},
    {.abbreviation = "Vir", .name = "Virgo", .genitive = "Virginis"},
    {.abbreviation = "Vol", .name = "Volans", .genitive = "Volantis"},
    {.abbreviation = "Vul", .name = "Vulpecula", .genitive = "Vulpeculae"},
}};

const Constellation* findConstellation(std::string_view abbreviation)
{
    const auto* found =
        std::ranges::find(kConstellations, abbreviation, &Constellation::abbreviation);
    return found == kConstellations.end() ? nullptr : found;
}

struct GreekLetter
{
    std::string_view code;  // as in HYG's "bayer" column
    std::string_view name;
};

constexpr std::array<GreekLetter, 24> kGreek{{
    {.code = "Alp", .name = "Alpha"},   {.code = "Bet", .name = "Beta"},
    {.code = "Gam", .name = "Gamma"},   {.code = "Del", .name = "Delta"},
    {.code = "Eps", .name = "Epsilon"}, {.code = "Zet", .name = "Zeta"},
    {.code = "Eta", .name = "Eta"},     {.code = "The", .name = "Theta"},
    {.code = "Iot", .name = "Iota"},    {.code = "Kap", .name = "Kappa"},
    {.code = "Lam", .name = "Lambda"},  {.code = "Mu", .name = "Mu"},
    {.code = "Nu", .name = "Nu"},       {.code = "Xi", .name = "Xi"},
    {.code = "Omi", .name = "Omicron"}, {.code = "Pi", .name = "Pi"},
    {.code = "Rho", .name = "Rho"},     {.code = "Sig", .name = "Sigma"},
    {.code = "Tau", .name = "Tau"},     {.code = "Ups", .name = "Upsilon"},
    {.code = "Phi", .name = "Phi"},     {.code = "Chi", .name = "Chi"},
    {.code = "Psi", .name = "Psi"},     {.code = "Ome", .name = "Omega"},
}};

/// "Alp" -> "Alpha", "Pi-1" -> "Pi1".
std::string bayerLetter(std::string_view code)
{
    const std::size_t dash   = code.find('-');
    const auto        letter = code.substr(0, dash);
    const auto  index = dash == std::string_view::npos ? std::string_view{} : code.substr(dash + 1);
    const auto* greek = std::ranges::find(kGreek, letter, &GreekLetter::code);
    std::string name(greek == kGreek.end() ? letter : greek->name);
    name.append(index);
    return name;
}

std::string designationOf(std::string_view bayer, std::string_view flamsteed,
                          std::string_view constellation, int hip, std::string_view hd,
                          std::string_view id)
{
    const Constellation* con = findConstellation(constellation);
    if (con != nullptr && !bayer.empty())
    {
        return std::format("{} {}", bayerLetter(bayer), con->genitive);
    }
    if (con != nullptr && !flamsteed.empty())
    {
        return std::format("{} {}", flamsteed, con->genitive);
    }
    if (hip > 0)
    {
        return std::format("HIP {}", hip);
    }
    if (!hd.empty())
    {
        return std::format("HD {}", hd);
    }
    return std::format("HYG {}", id);
}

/// Splits one CSV record into fields (RFC 4180 quoting; no embedded newlines in HYG).
void splitCsv(std::string_view line, std::vector<std::string_view>& fields)
{
    fields.clear();
    std::size_t at = 0;
    while (at <= line.size())
    {
        if (at < line.size() && line[at] == '"')
        {
            const std::size_t close = line.find('"', at + 1);
            const std::size_t end   = close == std::string_view::npos ? line.size() : close;
            fields.push_back(line.substr(at + 1, end - at - 1));
            at = line.find(',', end);
        }
        else
        {
            const std::size_t comma = line.find(',', at);
            fields.push_back(
                line.substr(at, comma == std::string_view::npos ? line.size() - at : comma - at));
            at = comma;
        }
        if (at == std::string_view::npos)
        {
            break;
        }
        ++at;
    }
}

// The HYG columns we read, as indices into kColumnNames.
constexpr std::size_t kId          = 0;
constexpr std::size_t kHip         = 1;
constexpr std::size_t kHd          = 2;
constexpr std::size_t kProper      = 3;
constexpr std::size_t kRa          = 4;
constexpr std::size_t kDec         = 5;
constexpr std::size_t kDist        = 6;
constexpr std::size_t kMag         = 7;
constexpr std::size_t kCi          = 8;
constexpr std::size_t kBayer       = 9;
constexpr std::size_t kFlam        = 10;
constexpr std::size_t kCon         = 11;
constexpr std::size_t kColumnCount = 12;

constexpr std::array<std::string_view, kColumnCount> kColumnNames{
    "id", "hip", "hd", "proper", "ra", "dec", "dist", "mag", "ci", "bayer", "flam", "con"};

/// Builds a star from one record, or nothing if it is too faint (or is the Sun).
std::optional<CatalogStar> readStar(const std::vector<std::string_view>&         fields,
                                    const std::array<std::size_t, kColumnCount>& columns,
                                    double                                       magnitudeLimit)
{
    const auto field     = [&](std::size_t column) { return fields[columns.at(column)]; };
    const auto magnitude = parseDouble(field(kMag));
    const auto ra        = parseDouble(field(kRa));
    const auto dec       = parseDouble(field(kDec));
    const auto distance  = parseDouble(field(kDist));
    if (!magnitude || !ra || !dec || *magnitude > magnitudeLimit || (distance && *distance <= 0.0))
    {
        return std::nullopt;  // distance 0 is the Sun
    }
    const double alpha = *ra * (kPi / 12.0);
    const double delta = degreesToRadians(*dec);
    const int    hip   = parseInt(field(kHip)).value_or(0);

    CatalogStar star;
    star.direction  = Vec3d(std::cos(delta) * std::cos(alpha), std::cos(delta) * std::sin(alpha),
                            std::sin(delta));
    star.magnitude  = *magnitude;
    star.colorIndex = parseDouble(field(kCi)).value_or(0.65);
    star.distanceParsecs = distance && *distance < kUnknownDistanceParsecs ? *distance : 0.0;
    star.hip             = hip;
    star.properName      = std::string(field(kProper));
    star.constellation   = std::string(field(kCon));
    star.designation =
        designationOf(field(kBayer), field(kFlam), field(kCon), hip, field(kHd), field(kId));
    return star;
}

}  // namespace

double pointingScore(double angle, double magnitude)
{
    // Each star "catches" the crosshair within a radius that grows with brightness; the star with
    // the crosshair deepest inside its radius wins. So a faint star right under the crosshair beats
    // a bright one nearby, but a bright star wins over a faint one at a similar distance.
    const double catchRadius = degreesToRadians(0.4 + (0.15 * std::max(0.0, 7.0 - magnitude)));
    return angle / catchRadius;
}

std::optional<std::size_t> StarCatalog::identify(const Vec3d& direction, double maxAngle) const
{
    const Vec3d                d      = glm::normalize(direction);
    const double               cosMax = std::cos(maxAngle);
    std::optional<std::size_t> best;
    double                     bestScore = 0.0;
    for (std::size_t i = 0; i < stars.size(); ++i)
    {
        const double c = glm::dot(stars[i].direction, d);
        if (c < cosMax)
        {
            continue;
        }
        const double score = pointingScore(std::acos(std::min(c, 1.0)), stars[i].magnitude);
        if (!best || score < bestScore)
        {
            best      = i;
            bestScore = score;
        }
    }
    return best;
}

const CatalogStar* StarCatalog::find(std::string_view name) const
{
    if (name.empty())
    {
        return nullptr;
    }
    const auto lower = [](char c) {
        return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
    };
    const auto same = [&](std::string_view a) {
        return std::ranges::equal(a, name, {}, lower, lower);
    };
    const auto found = std::ranges::find_if(stars, [&](const CatalogStar& star) {
        return same(star.properName) || same(star.designation);
    });
    return found == stars.end() ? nullptr : &*found;
}

std::expected<StarCatalog, std::string> parseHygCatalog(std::string_view csv, double magnitudeLimit)
{
    std::vector<std::string_view> fields;
    std::size_t                   lineStart = 0;
    const auto                    nextLine  = [&]() {
        const std::size_t end  = csv.find('\n', lineStart);
        std::string_view  line = csv.substr(
            lineStart, end == std::string_view::npos ? std::string_view::npos : end - lineStart);
        lineStart = end == std::string_view::npos ? csv.size() : end + 1;
        if (!line.empty() && line.back() == '\r')
        {
            line.remove_suffix(1);
        }
        return line;
    };

    splitCsv(nextLine(), fields);
    std::array<std::size_t, kColumnCount> columns{};
    for (std::size_t c = 0; c < kColumnCount; ++c)
    {
        const auto found = std::ranges::find(fields, kColumnNames.at(c));
        if (found == fields.end())
        {
            return std::unexpected(
                std::format("star catalog: missing column '{}' in the header", kColumnNames.at(c)));
        }
        columns.at(c) = static_cast<std::size_t>(found - fields.begin());
    }
    const std::size_t needed = *std::ranges::max_element(columns) + 1;

    StarCatalog catalog;
    int         lineNumber = 1;
    while (lineStart < csv.size())
    {
        ++lineNumber;
        const std::string_view line = nextLine();
        if (line.empty())
        {
            continue;
        }
        splitCsv(line, fields);
        if (fields.size() < needed)
        {
            return std::unexpected(std::format("star catalog: line {} has {} fields, expected {}",
                                               lineNumber, fields.size(), needed));
        }
        if (auto star = readStar(fields, columns, magnitudeLimit))
        {
            catalog.stars.push_back(std::move(*star));
        }
    }
    std::ranges::stable_sort(catalog.stars, {}, &CatalogStar::magnitude);
    return catalog;
}

std::expected<StarCatalog, std::string> loadHygCatalog(const std::filesystem::path& path,
                                                       double                       magnitudeLimit)
{
    auto bytes = assets::readFile(path);
    if (!bytes)
    {
        return std::unexpected(bytes.error());
    }
    if (path.extension() == ".gz")
    {
        bytes = assets::gunzip(*bytes);
        if (!bytes)
        {
            return std::unexpected(std::format("{}: {}", utf8String(path), bytes.error()));
        }
    }
    const std::string_view text(
        reinterpret_cast<const char*>(bytes->data()),  // NOLINT(*-reinterpret-cast)
        bytes->size());
    return parseHygCatalog(text, magnitudeLimit);
}

const std::string& displayName(const CatalogStar& star)
{
    return star.properName.empty() ? star.designation : star.properName;
}

double colorIndexToKelvin(double colorIndex)
{
    const double bv = std::clamp(colorIndex, -0.4, 2.0);
    return 4600.0 * ((1.0 / ((0.92 * bv) + 1.7)) + (1.0 / ((0.92 * bv) + 0.62)));
}

GpuStar gpuStar(const Vec3d& direction, double magnitude, double kelvin)
{
    const double flux  = std::pow(10.0, -0.4 * magnitude);  // relative to magnitude 0
    const double size  = 1.4 + (0.9 * std::clamp(2.0 - (0.4 * magnitude), 0.0, 3.0));
    const Vec3f  color = blackbodyColor(kelvin);
    return {.direction = Vec4f(Vec3f(glm::normalize(direction)), static_cast<float>(size)),
            .color     = Vec4f(color * static_cast<float>(flux), 0.0F)};
}

std::vector<GpuStar> toGpuStars(const StarCatalog& catalog)
{
    std::vector<GpuStar> stars;
    stars.reserve(catalog.stars.size());
    for (const CatalogStar& star : catalog.stars)
    {
        stars.push_back(
            gpuStar(star.direction, star.magnitude, colorIndexToKelvin(star.colorIndex)));
    }
    return stars;
}

std::string_view constellationName(std::string_view abbreviation)
{
    const Constellation* con = findConstellation(abbreviation);
    return con == nullptr ? std::string_view{} : con->name;
}

std::string_view constellationGenitive(std::string_view abbreviation)
{
    const Constellation* con = findConstellation(abbreviation);
    return con == nullptr ? std::string_view{} : con->genitive;
}

}  // namespace StarshipSimulator::astro
