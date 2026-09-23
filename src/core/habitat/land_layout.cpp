#include "StarshipSimulator/core/habitat/land_layout.h"

#include <cmath>
#include <memory>
#include <vector>

#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

Vec2d LandBand::toPlan(double z, double theta) const
{
    const double around = std::remainder(theta - centreTheta, 2.0 * kPi) * radiusM;
    if (axis == BandAxis::ALONG_Z)
    {
        return {around, z};
    }
    // Plan x runs toward -z along the profile, so that x, y and up keep the ALONG_Z handedness.
    return {centreU - profile->arcAt(z), around};
}

SurfaceSpot LandBand::toSurface(const Vec2d& plan) const
{
    if (axis == BandAxis::ALONG_Z)
    {
        return {.z = plan.y, .theta = centreTheta + (plan.x / radiusM)};
    }
    return {.z = profile->pointAt(centreU - plan.x).x, .theta = centreTheta + (plan.y / radiusM)};
}

Vec3d LandBand::alongDirection(double theta) const
{
    if (axis == BandAxis::ALONG_Z)
    {
        return {0.0, 0.0, 1.0};
    }
    return {-std::sin(theta), std::cos(theta), 0.0};
}

std::vector<LandBand> planLandBands(const HabitatSpec&                            spec,
                                    const std::shared_ptr<const MeridianProfile>& floor,
                                    double floorZMin, double floorZMax)
{
    std::vector<LandBand> bands;
    if (spec.kind == HabitatKind::ONEILL_CYLINDER)
    {
        // A valley between each pair of window strips, the length of the floor.
        const double strip = 2.0 * kPi / static_cast<double>(spec.stripPairs);
        for (int i = 0; i < spec.stripPairs; ++i)
        {
            LandBand& band   = bands.emplace_back();
            band.index       = i;
            band.axis        = BandAxis::ALONG_Z;
            band.centreTheta = std::fmod((static_cast<double>(i) + 0.5) * strip, 2.0 * kPi);
            band.radiusM     = spec.radiusM;
            band.alongMinM   = floorZMin;
            band.alongMaxM   = floorZMax;
            band.halfWidthM  = 0.5 * (1.0 - spec.windowFraction) * strip * spec.radiusM;
            band.profile     = floor;
        }
        return bands;
    }
    // Every other kind has one band going all the way round, across the floor's z range.
    LandBand& band  = bands.emplace_back();
    band.axis       = BandAxis::AROUND;
    band.centreU    = 0.5 * (floor->arcAt(floorZMin) + floor->arcAt(floorZMax));
    band.radiusM    = floor->radiusAt(floor->pointAt(band.centreU).x).value_or(spec.radiusM);
    band.alongMinM  = -kPi * band.radiusM;
    band.alongMaxM  = kPi * band.radiusM;
    band.halfWidthM = 0.5 * (floor->arcAt(floorZMax) - floor->arcAt(floorZMin));
    band.wraps      = true;
    band.profile    = floor;
    return bands;
}

}  // namespace StarshipSimulator
