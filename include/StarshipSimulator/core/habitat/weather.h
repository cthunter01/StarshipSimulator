#pragma once

#include <cstdint>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"

// The habitat's weather and its year. Inside a cylinder the air co-rotates, the mirrors make the
// day, and the water that rises from the rivers and lakes comes back as a deck of cloud a few
// hundred metres up, below which it rains. None of it is simulated: the weather is a smooth,
// repeatable function of time, so everyone opening the same habitat file at the same moment sees
// the same sky.
namespace StarshipSimulator
{

/// How a habitat's climate is set up: its cloud deck, how often it rains, and the length of its
/// year (the mirrors' seasons).
struct ClimateSpec
{
    double cloudBaseM       = 420.0;  // the cloud deck, above the floor
    double cloudTopM        = 820.0;
    double cloudiness       = 0.45;   // average cover, 0..1 (0: never a cloud)
    double raininess        = 0.35;   // how much of the time showers pass
    double windSpeedMS      = 3.0;    // typical wind along the valley
    double mistiness        = 0.7;    // how thick the morning mist over the water gets
    double yearDays         = 120.0;  // a habitat year (the mirrors run the seasons)
    double seasonSwingHours = 2.0;    // how far the day length swings over that year
    double seasonAtEpoch    = 0.0;    // where in the year J2000 (2000-01-01 12:00) falls, 0..1:
                                      // the knob that puts a habitat's spring where you want it
};

/// How the plants look through the year.
struct SeasonLook
{
    double fresh   = 1.0;  // new green
    double gold    = 0.0;  // autumn colours and harvested fields
    double blossom = 0.0;  // spring blossom
};

/// The weather at one moment.
struct Weather
{
    double     cloudCover     = 0.0;   // 0 clear, 1 overcast
    double     rain           = 0.0;   // 0 dry, 1 downpour
    double     wetness        = 0.0;   // how wet the ground still is (rain, fading slowly)
    double     mist           = 0.0;   // ground mist over the water and fields
    double     windAlongMS    = 0.0;   // along the axis (+z)
    double     windAroundMS   = 0.0;   // around the habitat (the spin direction)
    double     season         = 0.0;   // 0..1 through the year: 0 spring, 0.5 autumn
    double     dayLengthHours = 14.0;  // this day, with the season's swing
    SeasonLook look;
};

/// The weather in a habitat at a moment. `seed` (the terrain seed) keeps two habitats' weather
/// from marching in step.
[[nodiscard]] Weather weatherAt(const ClimateSpec& climate, const DayScheduleSpec& day,
                                astro::SimTime time, double utcOffsetHours, std::uint64_t seed);

/// The day schedule of one day of the year: the same mirrors, swinging over a longer or shorter
/// day as the season turns.
[[nodiscard]] DayScheduleSpec seasonalDay(const DayScheduleSpec& day, const ClimateSpec& climate,
                                          double season);

/// How the plants look at a point in the year.
[[nodiscard]] SeasonLook seasonLook(double season);

/// Problems that make a climate unusable, as human-readable messages (empty when valid).
[[nodiscard]] bool validClimate(const ClimateSpec& climate);

}  // namespace StarshipSimulator
