#pragma once

#include <cstdint>

#include "StarshipSimulator/core/habitat/habitat_spec.h"

namespace StarshipSimulator
{

/// What it would take to build the hull: the material class able to hold the spinning structure.
enum class MaterialClass : std::uint8_t
{
    STEEL,             // ordinary structural steel suffices
    HIGH_STRENGTH,     // high-strength steel or titanium alloys
    CARBON_FIBRE,      // carbon fibre / aramid composites (exist today, costly)
    FUTURE_MATERIALS,  // carbon nanotube or graphene class (not yet buildable at scale)
    BEYOND_KNOWN,      // stronger than any known material
};

/// Derived numbers for the almanac, the HUD and the editor.
struct HabitatMetrics
{
    double omega                = 0.0;  // spin rate, rad/s
    double periodS              = 0.0;  // one rotation, seconds
    double rpm                  = 0.0;
    double rimSpeed             = 0.0;  // floor speed in the inertial frame, m/s
    double floorGravity         = 0.0;  // m/s^2
    double headToFootGradient   = 0.0;  // relative gravity change over 1.7 m of height
    double coriolisWalkingRatio = 0.0;  // Coriolis acceleration at 1.4 m/s over floor gravity
    double axisPressureRatio    = 0.0;  // air pressure at the axis relative to the floor
    double axisTemperatureDropK = 0.0;  // for a well-mixed (adiabatic) atmosphere
    double landAreaM2           = 0.0;  // the floor that is land
    double windowAreaM2         = 0.0;
    double volumeM3             = 0.0;  // of air inside
    double population           = 0.0;
    double hoopSpecificStrength = 0.0;  // J/kg = (m/s)^2 a free-standing hoop needs
    MaterialClass material      = MaterialClass::STEEL;
};

[[nodiscard]] double spinRate(double radiusM, double gravityG);  // rad/s for this gravity at radius
[[nodiscard]] HabitatMetrics computeMetrics(const HabitatSpec& spec);

/// Pressure at distance r from the axis relative to the floor, for an isothermal atmosphere.
[[nodiscard]] double pressureRatioAt(double omega, double radiusM, double r, double temperatureK);

/// Density of the air (kg/m^3) at radius r. Isothermal air in spin gravity thins toward the axis,
/// so there is less of it to fly on the higher you climb.
[[nodiscard]] double airDensityAt(const AtmosphereSpec& atmosphere, double omega, double radiusM,
                                  double r);

[[nodiscard]] MaterialClass materialClassFor(double specificStrength);
[[nodiscard]] const char*   materialClassName(MaterialClass material);
[[nodiscard]] bool          buildableToday(MaterialClass material);

}  // namespace StarshipSimulator
