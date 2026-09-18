#include "StarshipSimulator/core/habitat/metrics.h"

#include <cmath>

#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/units.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kWalkingSpeed = 1.4;  // m/s
constexpr double kEyeHeight    = 1.7;  // m

}  // namespace

double spinRate(double radiusM, double gravityG)
{
    return std::sqrt(gravityG * units::kStandardGravity / radiusM);
}

double pressureRatioAt(double omega, double radiusM, double r, double temperatureK)
{
    // Hydrostatic balance in the centrifugal potential: dp/dr = rho * omega^2 * r, with p = rho R
    // T.
    return std::exp(-(omega * omega * ((radiusM * radiusM) - (r * r))) /
                    (2.0 * units::kSpecificGasConstantAir * temperatureK));
}

MaterialClass materialClassFor(double specificStrength)
{
    // Rough working specific strengths (J/kg) with a safety margin.
    if (specificStrength < 0.1e6)
    {
        return MaterialClass::Steel;
    }
    if (specificStrength < 0.5e6)
    {
        return MaterialClass::HighStrength;
    }
    if (specificStrength < 3.0e6)
    {
        return MaterialClass::CarbonFibre;
    }
    if (specificStrength < 40.0e6)
    {
        return MaterialClass::FutureMaterials;
    }
    return MaterialClass::BeyondKnown;
}

const char* materialClassName(MaterialClass material)
{
    switch (material)
    {
        case MaterialClass::Steel:
            return "structural steel";
        case MaterialClass::HighStrength:
            return "high-strength steel or titanium";
        case MaterialClass::CarbonFibre:
            return "carbon fibre composites";
        case MaterialClass::FutureMaterials:
            return "carbon nanotube class (future materials)";
        case MaterialClass::BeyondKnown:
            return "beyond any known material";
    }
    return "unknown";
}

bool buildableToday(MaterialClass material)
{
    return material == MaterialClass::Steel || material == MaterialClass::HighStrength ||
           material == MaterialClass::CarbonFibre;
}

HabitatMetrics computeMetrics(const OneillCylinderSpec& spec)
{
    HabitatMetrics metrics;
    const double   radius        = spec.radiusM;
    metrics.omega                = spinRate(radius, spec.surfaceGravityG);
    metrics.periodS              = 2.0 * kPi / metrics.omega;
    metrics.rpm                  = units::kSecondsPerMinute / metrics.periodS;
    metrics.rimSpeed             = metrics.omega * radius;
    metrics.floorGravity         = metrics.omega * metrics.omega * radius;
    metrics.headToFootGradient   = kEyeHeight / radius;
    metrics.coriolisWalkingRatio = 2.0 * metrics.omega * kWalkingSpeed / metrics.floorGravity;
    metrics.axisPressureRatio =
        pressureRatioAt(metrics.omega, radius, 0.0, spec.atmosphere.temperatureK);
    metrics.axisTemperatureDropK =
        (metrics.rimSpeed * metrics.rimSpeed) / (2.0 * units::kSpecificHeatAir);

    const double wall            = 2.0 * kPi * radius * spec.lengthM;
    metrics.landAreaM2           = wall * (1.0 - spec.windowFraction);
    metrics.windowAreaM2         = wall * spec.windowFraction;
    metrics.volumeM3             = kPi * radius * radius * spec.lengthM;
    metrics.population           = spec.populationDensityPerKm2 * metrics.landAreaM2 * 1e-6;
    metrics.hoopSpecificStrength = metrics.rimSpeed * metrics.rimSpeed;  // sigma/rho = v^2
    metrics.material             = materialClassFor(metrics.hoopSpecificStrength);
    return metrics;
}

}  // namespace StarshipSimulator
