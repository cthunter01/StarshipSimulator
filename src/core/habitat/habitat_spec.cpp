#include "StarshipSimulator/core/habitat/habitat_spec.h"

#include <cmath>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

double endcapDepthInside(const EndcapSpec& endcap, double radiusM)
{
    if (endcap.shape != EndcapShape::CONICAL_RAMP)
    {
        return 0.0;
    }
    const double topRadius = endcap.rampTopRadiusFraction * radiusM;
    const double lower = (radiusM - topRadius) / std::tan(degreesToRadians(endcap.rampSlopeDeg));
    const double upper =
        (topRadius - endcap.hubRadiusM) / std::tan(degreesToRadians(endcap.upperSlopeDeg));
    return lower + upper;
}

double minimumPartnerSeparation(const HabitatSpec& spec)
{
    return 2.0 * (spec.radiusM + spec.lengthM);
}

const char* endcapShapeName(EndcapShape shape)
{
    switch (shape)
    {
        case EndcapShape::FLAT:
            return "flat";
        case EndcapShape::HEMISPHERE:
            return "hemisphere";
        case EndcapShape::CONICAL_RAMP:
            return "conical_ramp";
    }
    return "flat";
}

namespace
{

void validateEndcap(const EndcapSpec& endcap, const char* which, double radiusM,
                    std::vector<std::string>& problems)
{
    if (endcap.shape != EndcapShape::CONICAL_RAMP)
    {
        return;
    }
    if (endcap.rampSlopeDeg < 5.0 || endcap.rampSlopeDeg > 60.0 || endcap.upperSlopeDeg < 5.0 ||
        endcap.upperSlopeDeg > 70.0)
    {
        problems.push_back(
            std::format("{} endcap: ramp slopes must be 5..60 and 5..70 degrees", which));
    }
    if (endcap.rampTopRadiusFraction <= 0.05 || endcap.rampTopRadiusFraction >= 0.95)
    {
        problems.push_back(
            std::format("{} endcap: ramp top radius must be 5%..95% of the radius", which));
    }
    if (endcap.hubRadiusM < 5.0 || endcap.hubRadiusM >= endcap.rampTopRadiusFraction * radiusM)
    {
        problems.push_back(std::format(
            "{} endcap: hub radius must be at least 5 m and below the ramp top", which));
    }
}

/// The terrain, water, woods and settlements.
void validateLand(const HabitatSpec& spec, std::vector<std::string>& problems)
{
    if (spec.terrain.featureSizeM < 50.0 || spec.terrain.hillHeightM < 0.0 ||
        spec.terrain.mountainHeightM < 0.0)
    {
        problems.emplace_back(
            "terrain needs a feature size of at least 50 m and non-negative heights");
    }
    const TerrainSpec& terrain = spec.terrain;
    if (terrain.riverWidthM < 0.0 || terrain.riverWidthM > 300.0 || terrain.lakesPerValley < 0 ||
        terrain.lakesPerValley > 8 || terrain.lakeRadiusM < 10.0 || terrain.lakeRadiusM > 3000.0)
    {
        problems.emplace_back(
            "water needs a river width of 0..300 m, 0..8 lakes per valley and lakes of 10..3000 m");
    }
    if (terrain.forestCover < 0.0 || terrain.forestCover > 0.9)
    {
        problems.emplace_back("forest cover must be between 0 and 0.9");
    }
    const SettlementSpec& settlements = spec.settlements;
    if (settlements.townsPerValley < 0 || settlements.townsPerValley > 12 ||
        settlements.townRadiusM < 40.0 || settlements.townRadiusM > 800.0 ||
        settlements.farmsPerValley < 0 || settlements.farmsPerValley > 60)
    {
        problems.emplace_back(
            "settlements need 0..12 towns per valley of 40..800 m and 0..60 farms per valley");
    }
}

/// Spin gravity on the floor.
void validateSpin(const HabitatSpec& spec, std::vector<std::string>& problems)
{
    if (spec.surfaceGravityG < 0.05 || spec.surfaceGravityG > 2.0)
    {
        problems.emplace_back("surface gravity must be between 0.05 g and 2 g");
    }
}

/// The mirrors and the air, common to every kind.
void validateAirAndLight(const HabitatSpec& spec, std::vector<std::string>& problems)
{
    if (spec.mirrors.reflectivity < 0.0 || spec.mirrors.reflectivity > 1.0)
    {
        problems.emplace_back("mirror reflectivity must be between 0 and 1");
    }
    if (spec.atmosphere.surfacePressurePa < 1000.0 || spec.atmosphere.temperatureK < 150.0 ||
        spec.atmosphere.temperatureK > 400.0)
    {
        problems.emplace_back("atmosphere needs at least 1 kPa and a temperature of 150..400 K");
    }
}

/// An O'Neill cylinder: the rules Island Three was built to.
void validateOneill(const HabitatSpec& spec, std::vector<std::string>& problems)
{
    if (spec.radiusM < 100.0 || spec.radiusM > 20000.0)
    {
        problems.emplace_back("radius must be between 100 m and 20 km");
    }
    if (spec.lengthM < 500.0 || spec.lengthM > 200000.0)
    {
        problems.emplace_back("length must be between 500 m and 200 km");
    }
    validateSpin(spec, problems);
    if (spec.stripPairs < 1 || spec.stripPairs > 6)
    {
        problems.emplace_back("strip pairs must be between 1 and 6");
    }
    if (spec.windowFraction < 0.1 || spec.windowFraction > 0.8)
    {
        problems.emplace_back("window fraction must be between 0.1 and 0.8");
    }
    validateAirAndLight(spec, problems);
    validateLand(spec, problems);
    if (spec.partner.enabled && spec.partner.separationM < minimumPartnerSeparation(spec))
    {
        problems.push_back(std::format(
            "the partner cylinder must be at least {:.0f} km away (axis to axis) so the mirrors "
            "clear each other",
            minimumPartnerSeparation(spec) / 1000.0));
    }
    validateEndcap(spec.sunwardEndcap, "sunward", spec.radiusM, problems);
    validateEndcap(spec.antisunwardEndcap, "anti-sunward", spec.radiusM, problems);
    const double endcaps = endcapDepthInside(spec.sunwardEndcap, spec.radiusM) +
                           endcapDepthInside(spec.antisunwardEndcap, spec.radiusM);
    if (problems.empty() && spec.lengthM - endcaps < 0.2 * spec.lengthM)
    {
        problems.push_back(std::format(
            "the endcap ramps ({:.0f} m) leave too little flat floor in a {:.0f} m cylinder",
            endcaps, spec.lengthM));
    }
}

void validateKalpana(const HabitatSpec& spec, std::vector<std::string>& problems)
{
    if (spec.radiusM < 100.0 || spec.radiusM > 20000.0)
    {
        problems.emplace_back("radius must be between 100 m and 20 km");
    }
    if (spec.lengthM < 100.0 || spec.lengthM > 20000.0)
    {
        problems.emplace_back("length must be between 100 m and 20 km");
    }
    validateSpin(spec, problems);
    validateAirAndLight(spec, problems);
    validateLand(spec, problems);
}

void validateTorus(const HabitatSpec& spec, std::vector<std::string>& problems)
{
    const TorusSpec& torus = spec.torus;
    if (spec.radiusM < 200.0 || spec.radiusM > 20000.0)
    {
        problems.emplace_back("the floor must be between 200 m and 20 km from the axis");
    }
    validateSpin(spec, problems);
    if (torus.tubeRadiusM < 20.0 || torus.tubeRadiusM > 600.0 ||
        torus.tubeRadiusM > 0.35 * spec.radiusM)
    {
        problems.push_back(std::format(
            "the tube's radius must be between 20 and 600 m, and no more than {:.0f} m on a wheel "
            "this size",
            0.35 * spec.radiusM));
    }
    if (torus.hubRadiusM < 5.0 || torus.hubRadiusM > 0.25 * spec.radiusM)
    {
        problems.emplace_back(
            "the hub's radius must be at least 5 m and at most a quarter of the "
            "floor's");
    }
    if (torus.spokes < 0 || torus.spokes > 12 || torus.spokeRadiusM < 1.0 ||
        torus.spokeRadiusM > torus.tubeRadiusM)
    {
        problems.emplace_back(
            "a torus has 0 to 12 spokes, each at least 1 m and at most a tube "
            "wide");
    }
    if (torus.landHalfAngleDeg < 5.0 || torus.landHalfAngleDeg > 60.0 ||
        torus.ceilingWindowShare < 0.0 || torus.ceilingWindowShare > 0.9)
    {
        problems.emplace_back(
            "the land must reach 5 to 60 degrees up the tube's sides, and the "
            "windows take 0 to 90% of its circumference");
    }
    if (torus.sections < 0 || torus.sections > 24)
    {
        problems.emplace_back("a torus has 0 to 24 sections of town and farmland");
    }
    validateAirAndLight(spec, problems);
    validateLand(spec, problems);
}

void validateSphere(const HabitatSpec& spec, std::vector<std::string>& problems)
{
    const SphereSpec& sphere = spec.sphere;
    if (spec.radiusM < 100.0 || spec.radiusM > 20000.0)
    {
        problems.emplace_back("radius must be between 100 m and 20 km");
    }
    validateSpin(spec, problems);
    if (sphere.landLatitudeDeg < 5.0 || sphere.landLatitudeDeg > 60.0 ||
        sphere.windowLatitudeDeg < sphere.landLatitudeDeg + 5.0 || sphere.windowLatitudeDeg > 85.0)
    {
        problems.emplace_back(
            "the land must reach 5 to 60 degrees of latitude, and the polar "
            "windows begin at least 5 degrees beyond it and before 85");
    }
    validateAirAndLight(spec, problems);
    validateLand(spec, problems);
}

void validateRing(const HabitatSpec& spec, std::vector<std::string>& problems)
{
    if (spec.radiusM < 100000.0 || spec.radiusM > 3.0e9)
    {
        problems.emplace_back("a ring's radius must be between 100 km and 3 million km");
    }
    if (spec.lengthM < 10000.0 || spec.lengthM > spec.radiusM)
    {
        problems.emplace_back("a ring must be at least 10 km wide and no wider than its radius");
    }
    if (spec.ring.wallHeightM < 1000.0 || spec.ring.wallHeightM > 0.5 * spec.radiusM)
    {
        problems.emplace_back("a ring's walls must be between 1 km and half its radius high");
    }
    validateSpin(spec, problems);
    validateAirAndLight(spec, problems);
    validateLand(spec, problems);
}

}  // namespace

std::vector<std::string> validate(const HabitatSpec& spec)
{
    std::vector<std::string> problems;
    switch (spec.kind)
    {
        case HabitatKind::ONEILL_CYLINDER:
            validateOneill(spec, problems);
            break;
        case HabitatKind::KALPANA_CYLINDER:
            validateKalpana(spec, problems);
            break;
        case HabitatKind::STANFORD_TORUS:
            validateTorus(spec, problems);
            break;
        case HabitatKind::BERNAL_SPHERE:
            validateSphere(spec, problems);
            break;
        case HabitatKind::BISHOP_RING:
            validateRing(spec, problems);
            break;
    }
    return problems;
}

bool habitatKindBuilt(HabitatKind kind)
{
    // Every kind can be described and saved; the worlds inside them arrive one M8 step at a time.
    return kind == HabitatKind::ONEILL_CYLINDER || kind == HabitatKind::KALPANA_CYLINDER ||
           kind == HabitatKind::BERNAL_SPHERE;
}

bool axisPointsAtSun(HabitatKind kind)
{
    return kind != HabitatKind::KALPANA_CYLINDER;
}

HabitatSpec normalizedForKind(const HabitatSpec& spec)
{
    HabitatSpec normal = spec;
    if (spec.kind == HabitatKind::KALPANA_CYLINDER)
    {
        normal.sunwardEndcap     = makeEndcap(EndcapShape::FLAT);
        normal.antisunwardEndcap = makeEndcap(EndcapShape::FLAT);
    }
    if (spec.kind != HabitatKind::ONEILL_CYLINDER)
    {
        // Only O'Neill's cylinders fly in counter-rotating pairs; the other kinds' files do not
        // mention a partner, and the default would otherwise give them one.
        normal.partner.enabled = false;
    }
    return normal;
}

const char* habitatKindKey(HabitatKind kind)
{
    switch (kind)
    {
        case HabitatKind::ONEILL_CYLINDER:
            return "oneill_cylinder";
        case HabitatKind::KALPANA_CYLINDER:
            return "kalpana_cylinder";
        case HabitatKind::STANFORD_TORUS:
            return "stanford_torus";
        case HabitatKind::BERNAL_SPHERE:
            return "bernal_sphere";
        case HabitatKind::BISHOP_RING:
            return "bishop_ring";
    }
    return "oneill_cylinder";
}

const char* habitatKindName(HabitatKind kind)
{
    switch (kind)
    {
        case HabitatKind::ONEILL_CYLINDER:
            return "O'Neill cylinder";
        case HabitatKind::KALPANA_CYLINDER:
            return "Kalpana cylinder";
        case HabitatKind::STANFORD_TORUS:
            return "Stanford torus";
        case HabitatKind::BERNAL_SPHERE:
            return "Bernal sphere";
        case HabitatKind::BISHOP_RING:
            return "Bishop ring";
    }
    return "habitat";
}

std::vector<HabitatKind> allHabitatKinds()
{
    return {HabitatKind::ONEILL_CYLINDER, HabitatKind::KALPANA_CYLINDER,
            HabitatKind::STANFORD_TORUS, HabitatKind::BERNAL_SPHERE, HabitatKind::BISHOP_RING};
}

std::optional<HabitatKind> habitatKindFromKey(std::string_view key)
{
    for (const HabitatKind kind : allHabitatKinds())
    {
        if (key == habitatKindKey(kind))
        {
            return kind;
        }
    }
    return std::nullopt;
}

DaylightKind daylightKindFor(HabitatKind kind)
{
    switch (kind)
    {
        case HabitatKind::ONEILL_CYLINDER:
            return DaylightKind::MIRROR_STRIPS;
        case HabitatKind::KALPANA_CYLINDER:
            return DaylightKind::END_CAPS;
        case HabitatKind::STANFORD_TORUS:
            return DaylightKind::OVERHEAD_MIRROR;
        case HabitatKind::BERNAL_SPHERE:
            return DaylightKind::POLAR_WINDOWS;
        case HabitatKind::BISHOP_RING:
            return DaylightKind::OPEN_SKY;
    }
    return DaylightKind::MIRROR_STRIPS;
}

int bandCount(const HabitatSpec& spec)
{
    return spec.kind == HabitatKind::ONEILL_CYLINDER ? spec.stripPairs : 1;
}

double headroomM(const HabitatSpec& spec)
{
    switch (spec.kind)
    {
        case HabitatKind::STANFORD_TORUS:
            return 2.0 * spec.torus.tubeRadiusM;
        case HabitatKind::BISHOP_RING:
            return spec.ring.wallHeightM;
        case HabitatKind::ONEILL_CYLINDER:
        case HabitatKind::KALPANA_CYLINDER:
        case HabitatKind::BERNAL_SPHERE:
            break;
    }
    return spec.radiusM;
}

}  // namespace StarshipSimulator
