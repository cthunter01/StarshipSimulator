#include "StarshipSimulator/core/habitat/weather.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/noise.h"
#include "StarshipSimulator/core/rng.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kWeatherHours = 20.0;  // how long a spell of weather lasts (most of a day)
constexpr double kShowerHours  = 3.5;   // and a shower
constexpr double kWindHours    = 2.5;
constexpr double kDryingHours  = 2.5;  // how long the ground stays wet after rain
constexpr double kMistHours    = 3.0;  // mist hangs about this long after sunrise
constexpr double kHoursPerDay  = 24.0;

/// Noise over time, 0..1, changing smoothly over `period` hours.
double drift(const SimplexNoise& noise, double at, double period, double lane, int octaves = 2)
{
    return std::clamp(0.5 + (0.75 * noise.fbm(Vec3d(at / period, lane, 0.5), octaves)), 0.0, 1.0);
}

/// Cloud cover from the slow drift, around a climate's average.
double coverFrom(double drifted, double average)
{
    return std::clamp(((drifted - 0.5) * 1.8) + average, 0.0, 1.0);
}

/// Rain leaves the ground wet; it dries out over a couple of hours. Looks back over the last few
/// hours rather than keeping state, so the weather stays a function of time.
double wetnessAt(const SimplexNoise& noise, double hours, double raininess, double average,
                 double coverNow)
{
    constexpr double kStep = 0.25;  // hours between look-back samples
    double           wet   = 0.0;
    for (int back = 0; back <= 14; ++back)
    {
        const double then = hours - (kStep * back);
        const double shower =
            std::max(0.0, drift(noise, then, kShowerHours, 4.0) - (1.0 - (0.55 * raininess))) * 4.0;
        const double cover =
            back == 0 ? coverNow : coverFrom(drift(noise, then, kWeatherHours, 1.0, 1), average);
        const double fell = std::clamp(shower, 0.0, 1.0) * glm::smoothstep(0.45, 0.8, cover);
        wet               = std::max(wet, fell * std::exp(-(kStep * back) / kDryingHours));
    }
    return std::clamp(wet, 0.0, 1.0);
}

}  // namespace

SeasonLook seasonLook(double season)
{
    // Spring (0), summer (0.25), autumn (0.5), winter (0.75).
    const double turn = std::fmod(std::fmod(season, 1.0) + 1.0, 1.0);
    SeasonLook   look;
    look.blossom = std::max(0.0, std::cos(2.0 * kPi * turn));  // strongest in spring
    look.blossom = std::pow(std::max(0.0, look.blossom), 2.0);
    look.gold    = std::pow(std::max(0.0, -std::cos(2.0 * kPi * turn)), 1.5);  // autumn
    look.fresh   = std::clamp(0.55 + (0.45 * std::cos(2.0 * kPi * (turn - 0.15))), 0.0, 1.0);
    return look;
}

DayScheduleSpec seasonalDay(const DayScheduleSpec& day, const ClimateSpec& climate, double season)
{
    DayScheduleSpec turned = day;
    const double    swing  = climate.seasonSwingHours * std::sin(2.0 * kPi * season);
    turned.dayLengthHours  = std::clamp(day.dayLengthHours + swing, 2.0, 22.0);
    // Longer days start earlier and the mirrors open wider at noon.
    turned.sunriseHour  = std::fmod(day.sunriseHour - (0.5 * swing) + kHoursPerDay, kHoursPerDay);
    turned.noonAngleDeg = std::clamp(day.noonAngleDeg - (2.0 * swing), 20.0, 80.0);
    return turned;
}

bool validClimate(const ClimateSpec& climate)
{
    return climate.cloudBaseM >= 50.0 && climate.cloudTopM > climate.cloudBaseM + 50.0 &&
           climate.cloudTopM <= 20000.0 && climate.cloudiness >= 0.0 && climate.cloudiness <= 1.0 &&
           climate.raininess >= 0.0 && climate.raininess <= 1.0 && climate.windSpeedMS >= 0.0 &&
           climate.windSpeedMS <= 40.0 && climate.mistiness >= 0.0 && climate.mistiness <= 1.0 &&
           climate.yearDays >= 1.0 && climate.yearDays <= 10000.0 &&
           climate.seasonSwingHours >= 0.0 && climate.seasonSwingHours <= 8.0 &&
           climate.seasonAtEpoch >= 0.0 && climate.seasonAtEpoch < 1.0;
}

Weather weatherAt(const ClimateSpec& climate, const DayScheduleSpec& day, astro::SimTime time,
                  double utcOffsetHours, std::uint64_t seed)
{
    const SimplexNoise noise(hashSeed(seed, 0x5EA5));
    const double       days  = time.days();
    const double       hours = days * kHoursPerDay;

    Weather      weather;
    const double turns          = climate.seasonAtEpoch + (days / std::max(climate.yearDays, 1.0));
    weather.season              = std::fmod(std::fmod(turns, 1.0) + 1.0, 1.0);
    weather.look                = seasonLook(weather.season);
    const DayScheduleSpec today = seasonalDay(day, climate, weather.season);
    weather.dayLengthHours      = today.dayLengthHours;

    // Cloud: a slow drift around the habitat's average cloudiness, a little thicker in the cooler
    // half of the year.
    const double base  = std::clamp(climate.cloudiness + (0.12 * weather.look.gold), 0.0, 1.0);
    weather.cloudCover = coverFrom(drift(noise, hours, kWeatherHours, 1.0, 1), base);

    // Showers: only from thick cloud.
    const double shower = drift(noise, hours, kShowerHours, 4.0);
    weather.rain    = std::clamp((shower - (1.0 - (0.55 * climate.raininess))) * 4.0, 0.0, 1.0) *
                      glm::smoothstep(0.45, 0.8, weather.cloudCover);
    weather.wetness = std::max(
        weather.rain, wetnessAt(noise, hours, climate.raininess, base, weather.cloudCover));

    // Mist: over the water in the first hours after the mirrors open, most on still, clear
    // mornings, and again after rain.
    const double local     = astro::hourOfDay(time, utcOffsetHours);
    double       sinceDawn = local - today.sunriseHour;
    sinceDawn = std::fmod(std::fmod(sinceDawn, kHoursPerDay) + kHoursPerDay, kHoursPerDay);
    // Most just after sunrise, some in the last hour and a half before it.
    double dawn = sinceDawn > kHoursPerDay - 1.5 ? 0.5 : 0.0;
    if (sinceDawn < kMistHours)
    {
        dawn = 1.0 - (sinceDawn / kMistHours);
    }
    const double calm = 1.0 - glm::smoothstep(0.3, 1.2, drift(noise, hours, kWindHours, 7.0));
    weather.mist      = std::clamp(
        climate.mistiness * ((dawn * (0.4 + (0.6 * calm)) * (1.0 - (0.5 * weather.cloudCover))) +
                             (0.5 * weather.wetness * (1.0 - weather.rain))),
        0.0, 1.0);

    // Wind: mostly along the valley, gusting.
    const double gust    = 0.4 + (1.3 * drift(noise, hours, kWindHours, 7.0));
    const double swirl   = (2.0 * drift(noise, hours, kWindHours * 2.0, 11.0)) - 1.0;
    weather.windAlongMS  = climate.windSpeedMS * gust * (1.0 + (0.6 * weather.rain));
    weather.windAroundMS = 0.25 * climate.windSpeedMS * swirl;
    return weather;
}

}  // namespace StarshipSimulator
