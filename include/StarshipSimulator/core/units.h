#pragma once

// Physical constants in SI units.
namespace StarshipSimulator::units
{

inline constexpr double kStandardGravity        = 9.80665;  // m/s^2
inline constexpr double kSpecificGasConstantAir = 287.05;   // J/(kg K), dry air
inline constexpr double kSpecificHeatAir        = 1005.0;  // J/(kg K), dry air at constant pressure
inline constexpr double kSolarConstant          = 1361.0;  // W/m^2 at 1 AU
inline constexpr double kStandardAtmosphere     = 101325.0;  // Pa
inline constexpr double kSecondsPerMinute       = 60.0;

}  // namespace StarshipSimulator::units
