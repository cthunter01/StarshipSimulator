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

double airDensityAt(const AtmosphereSpec& atmosphere, double omega, double radiusM, double r)
{
    // Ideal gas at the floor, thinning toward the axis with the pressure.
    const double floorDensity =
        atmosphere.surfacePressurePa / (units::kSpecificGasConstantAir * atmosphere.temperatureK);
    return floorDensity * pressureRatioAt(omega, radiusM, r, atmosphere.temperatureK);
}

MaterialClass materialClassFor(double specificStrength)
{
    // Rough working specific strengths (J/kg) with a safety margin.
    if (specificStrength < 0.1e6)
    {
        return MaterialClass::STEEL;
    }
    if (specificStrength < 0.5e6)
    {
        return MaterialClass::HIGH_STRENGTH;
    }
    if (specificStrength < 3.0e6)
    {
        return MaterialClass::CARBON_FIBRE;
    }
    if (specificStrength < 40.0e6)
    {
        return MaterialClass::FUTURE_MATERIALS;
    }
    return MaterialClass::BEYOND_KNOWN;
}

const char* materialClassName(MaterialClass material)
{
    switch (material)
    {
        case MaterialClass::STEEL:
            return "structural steel";
        case MaterialClass::HIGH_STRENGTH:
            return "high-strength steel or titanium";
        case MaterialClass::CARBON_FIBRE:
            return "carbon fibre composites";
        case MaterialClass::FUTURE_MATERIALS:
            return "carbon nanotube class (future materials)";
        case MaterialClass::BEYOND_KNOWN:
            return "beyond any known material";
    }
    return "unknown";
}

bool buildableToday(MaterialClass material)
{
    return material == MaterialClass::STEEL || material == MaterialClass::HIGH_STRENGTH ||
           material == MaterialClass::CARBON_FIBRE;
}

namespace
{

/// Land, windows and the volume of air, which depend on the habitat's shape.
struct Areas
{
    double land   = 0.0;
    double window = 0.0;
    double volume = 0.0;
};

Areas areasOf(const HabitatSpec& spec)
{
    const double radius = spec.radiusM;
    switch (spec.kind)
    {
        case HabitatKind::ONEILL_CYLINDER:
        {
            const double wall = 2.0 * kPi * radius * spec.lengthM;
            return {.land   = wall * (1.0 - spec.windowFraction),
                    .window = wall * spec.windowFraction,
                    .volume = kPi * radius * radius * spec.lengthM};
        }
        case HabitatKind::KALPANA_CYLINDER:
            // The whole hull is land; the light comes in through the two ends.
            return {.land   = 2.0 * kPi * radius * spec.lengthM,
                    .window = 2.0 * kPi * radius * radius,
                    .volume = kPi * radius * radius * spec.lengthM};
        case HabitatKind::STANFORD_TORUS:
        {
            // The tube's circle at angle phi from its lowest point lies at R_c + a cos(phi) from
            // the axis; a band of it d(phi) wide has area 2 pi (R_c + a cos(phi)) a d(phi).
            const double a      = spec.torus.tubeRadiusM;
            const double centre = radius - a;
            const double land   = degreesToRadians(spec.torus.landHalfAngleDeg);
            const double window = kPi * spec.torus.ceilingWindowShare;  // half-angle, about pi
            return {.land   = 2.0 * kPi * a * 2.0 * ((centre * land) + (a * std::sin(land))),
                    .window = 2.0 * kPi * a * 2.0 * ((centre * window) - (a * std::sin(window))),
                    .volume = 2.0 * kPi * kPi * centre * a * a};
        }
        case HabitatKind::BERNAL_SPHERE:
        {
            // A zone of a sphere between two latitudes has area 2 pi R^2 (sin b - sin a).
            const double land   = degreesToRadians(spec.sphere.landLatitudeDeg);
            const double window = degreesToRadians(spec.sphere.windowLatitudeDeg);
            return {.land   = 2.0 * kPi * radius * radius * 2.0 * std::sin(land),
                    .window = 2.0 * 2.0 * kPi * radius * radius * (1.0 - std::sin(window)),
                    .volume = 4.0 / 3.0 * kPi * radius * radius * radius};
        }
        case HabitatKind::BISHOP_RING:
            return {.land   = 2.0 * kPi * radius * spec.lengthM,
                    .window = 0.0,
                    .volume = 2.0 * kPi * radius * spec.lengthM * spec.ring.wallHeightM};
    }
    return {};
}

}  // namespace

HabitatMetrics computeMetrics(const HabitatSpec& spec)
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

    const Areas areas            = areasOf(spec);
    metrics.landAreaM2           = areas.land;
    metrics.windowAreaM2         = areas.window;
    metrics.volumeM3             = areas.volume;
    metrics.population           = spec.populationDensityPerKm2 * metrics.landAreaM2 * 1e-6;
    metrics.hoopSpecificStrength = metrics.rimSpeed * metrics.rimSpeed;  // sigma/rho = v^2
    metrics.material             = materialClassFor(metrics.hoopSpecificStrength);
    return metrics;
}

}  // namespace StarshipSimulator
