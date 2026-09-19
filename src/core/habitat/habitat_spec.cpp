#include "StarshipSimulator/core/habitat/habitat_spec.h"

#include <cmath>
#include <format>
#include <string>
#include <vector>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

double endcapDepthInside(const EndcapSpec& endcap, double radiusM)
{
    if (endcap.shape != EndcapShape::ConicalRamp)
    {
        return 0.0;
    }
    const double topRadius = endcap.rampTopRadiusFraction * radiusM;
    const double lower = (radiusM - topRadius) / std::tan(degreesToRadians(endcap.rampSlopeDeg));
    const double upper =
        (topRadius - endcap.hubRadiusM) / std::tan(degreesToRadians(endcap.upperSlopeDeg));
    return lower + upper;
}

double minimumPartnerSeparation(const OneillCylinderSpec& spec)
{
    return 2.0 * (spec.radiusM + spec.lengthM);
}

const char* endcapShapeName(EndcapShape shape)
{
    switch (shape)
    {
        case EndcapShape::Flat:
            return "flat";
        case EndcapShape::Hemisphere:
            return "hemisphere";
        case EndcapShape::ConicalRamp:
            return "conical_ramp";
    }
    return "flat";
}

namespace
{

void validateEndcap(const EndcapSpec& endcap, const char* which, double radiusM,
                    std::vector<std::string>& problems)
{
    if (endcap.shape != EndcapShape::ConicalRamp)
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

}  // namespace

std::vector<std::string> validate(const OneillCylinderSpec& spec)
{
    std::vector<std::string> problems;
    if (spec.radiusM < 100.0 || spec.radiusM > 20000.0)
    {
        problems.emplace_back("radius must be between 100 m and 20 km");
    }
    if (spec.lengthM < 500.0 || spec.lengthM > 200000.0)
    {
        problems.emplace_back("length must be between 500 m and 200 km");
    }
    if (spec.surfaceGravityG < 0.05 || spec.surfaceGravityG > 2.0)
    {
        problems.emplace_back("surface gravity must be between 0.05 g and 2 g");
    }
    if (spec.stripPairs < 1 || spec.stripPairs > 6)
    {
        problems.emplace_back("strip pairs must be between 1 and 6");
    }
    if (spec.windowFraction < 0.1 || spec.windowFraction > 0.8)
    {
        problems.emplace_back("window fraction must be between 0.1 and 0.8");
    }
    if (spec.mirrors.reflectivity < 0.0 || spec.mirrors.reflectivity > 1.0)
    {
        problems.emplace_back("mirror reflectivity must be between 0 and 1");
    }
    if (spec.atmosphere.surfacePressurePa < 1000.0 || spec.atmosphere.temperatureK < 150.0 ||
        spec.atmosphere.temperatureK > 400.0)
    {
        problems.emplace_back("atmosphere needs at least 1 kPa and a temperature of 150..400 K");
    }
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
    return problems;
}

}  // namespace StarshipSimulator
