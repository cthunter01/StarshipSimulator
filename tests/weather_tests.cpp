#include "StarshipSimulator/core/habitat/weather.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/astro/astro_time.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

constexpr std::uint64_t kSeed = 1975;

astro::SimTime at(double days)
{
    return astro::SimTime::fromDays(days);
}

Weather weather(double days, const ClimateSpec& climate = {})
{
    return weatherAt(climate, DayScheduleSpec{}, at(days), 0.0, kSeed);
}

TEST(Weather, StaysInRangeAndIsTheSameEveryTime)
{
    const ClimateSpec climate;
    for (int step = 0; step < 400; ++step)
    {
        const double  days = 16600.0 + (step * 0.37);  // a few months from 2045
        const Weather now  = weather(days, climate);
        EXPECT_GE(now.cloudCover, 0.0);
        EXPECT_LE(now.cloudCover, 1.0);
        EXPECT_GE(now.rain, 0.0);
        EXPECT_LE(now.rain, 1.0);
        EXPECT_GE(now.wetness, now.rain - 1e-9);
        EXPECT_LE(now.wetness, 1.0);
        EXPECT_GE(now.mist, 0.0);
        EXPECT_LE(now.mist, 1.0);
        EXPECT_GE(now.windAlongMS, 0.0);
        EXPECT_LT(now.windAlongMS, 40.0);
        EXPECT_GE(now.season, 0.0);
        EXPECT_LT(now.season, 1.0);
        EXPECT_GT(now.dayLengthHours, 2.0);
        EXPECT_LT(now.dayLengthHours, 22.0);
        // The same moment always gives the same weather.
        const Weather again = weather(days, climate);
        EXPECT_EQ(again.cloudCover, now.cloudCover);
        EXPECT_EQ(again.rain, now.rain);
    }
}

TEST(Weather, ChangesSlowlyButDoesChange)
{
    std::vector<double> cover;
    cover.reserve(static_cast<std::size_t>(24) * 30);
    for (int hour = 0; hour < 24 * 30; ++hour)
    {
        cover.push_back(weather(16600.0 + (hour / 24.0)).cloudCover);
    }
    // Hour to hour it drifts; over a month it sees both clear and cloudy skies.
    for (std::size_t i = 1; i < cover.size(); ++i)
    {
        EXPECT_LT(std::abs(cover[i] - cover[i - 1]), 0.35) << "hour " << i;
    }
    EXPECT_LT(*std::ranges::min_element(cover), 0.15);
    EXPECT_GT(*std::ranges::max_element(cover), 0.85);
}

TEST(Weather, RainOnlyFallsFromThickCloudAndLeavesTheGroundWet)
{
    bool sawRain = false;
    for (int hour = 0; hour < 24 * 60; ++hour)
    {
        const double  days = 16600.0 + (hour / 24.0);
        const Weather now  = weather(days);
        if (now.rain > 0.05)
        {
            sawRain = true;
            EXPECT_GT(now.cloudCover, 0.4);
            EXPECT_GE(now.wetness, now.rain - 1e-9);
            // An hour after it stops, the ground is still damp.
            const Weather later = weather(days + (1.0 / 24.0));
            if (later.rain == 0.0)
            {
                EXPECT_GT(later.wetness, 0.0);
            }
        }
    }
    EXPECT_TRUE(sawRain);
}

TEST(Weather, DrierClimatesRainLess)
{
    const ClimateSpec dry{.cloudiness = 0.2, .raininess = 0.05};
    const ClimateSpec wet{.cloudiness = 0.8, .raininess = 0.9};
    double            dryHours = 0.0;
    double            wetHours = 0.0;
    for (int hour = 0; hour < 24 * 40; ++hour)
    {
        const double days = 16600.0 + (hour / 24.0);
        dryHours += weather(days, dry).rain > 0.05 ? 1.0 : 0.0;
        wetHours += weather(days, wet).rain > 0.05 ? 1.0 : 0.0;
    }
    EXPECT_LT(dryHours, wetHours);
    EXPECT_LT(dryHours / (24.0 * 40.0), 0.1);
    EXPECT_GT(wetHours / (24.0 * 40.0), 0.15);
}

TEST(Weather, MistLiesInTheHoursAfterSunrise)
{
    // A clear, calm morning: mist just after the mirrors open, gone by midday.
    const ClimateSpec     climate{.cloudiness = 0.15, .raininess = 0.0, .mistiness = 1.0};
    const DayScheduleSpec day;
    double                dawnMist = 0.0;
    double                noonMist = 0.0;
    for (int d = 0; d < 20; ++d)
    {
        const double midnight = std::floor(16600.0 + d) + 0.5;  // J2000 days start at noon
        dawnMist +=
            weatherAt(climate, day, at(midnight + ((day.sunriseHour + 0.5) / 24.0)), 0.0, kSeed)
                .mist;
        noonMist += weatherAt(climate, day, at(midnight + (12.5 / 24.0)), 0.0, kSeed).mist;
    }
    EXPECT_GT(dawnMist, 3.0 * std::max(noonMist, 0.1));
}

TEST(Weather, TheYearTurnsAndTheDaysGrowAndShrink)
{
    const ClimateSpec climate{.yearDays = 100.0, .seasonSwingHours = 3.0};
    double            longest  = 0.0;
    double            shortest = 24.0;
    for (int d = 0; d < 100; ++d)
    {
        const Weather now = weather(16600.0 + d, climate);
        longest           = std::max(longest, now.dayLengthHours);
        shortest          = std::min(shortest, now.dayLengthHours);
    }
    EXPECT_NEAR(longest - shortest, 6.0, 0.2);  // +/- 3 hours

    // Spring blossom, autumn gold, and both fade in between.
    EXPECT_GT(seasonLook(0.0).blossom, 0.9);
    EXPECT_LT(seasonLook(0.0).gold, 0.05);
    EXPECT_GT(seasonLook(0.5).gold, 0.9);
    EXPECT_LT(seasonLook(0.5).blossom, 0.05);
    EXPECT_GT(seasonLook(0.25).fresh, seasonLook(0.75).fresh);
    for (const double season : {0.0, 0.2, 0.4, 0.6, 0.8})
    {
        const SeasonLook look = seasonLook(season);
        EXPECT_GE(look.fresh, 0.0);
        EXPECT_LE(look.fresh, 1.0);
        EXPECT_LE(look.gold + look.blossom, 1.05);
    }
}

TEST(Weather, TheSeasonCanBeSetWhereTheYearStarts)
{
    // Moving the epoch moves the whole year: a quarter-turn later is a quarter of a year on.
    const ClimateSpec spring{.yearDays = 100.0, .seasonAtEpoch = 0.0};
    const ClimateSpec summer{.yearDays = 100.0, .seasonAtEpoch = 0.25};
    for (const double days : {16600.0, 16623.4, 16677.7})
    {
        const double a = weather(days, spring).season;
        const double b = weather(days, summer).season;
        EXPECT_NEAR(std::fmod(b - a + 1.0, 1.0), 0.25, 1e-9) << days;
    }
    EXPECT_FALSE(validClimate(ClimateSpec{.seasonAtEpoch = 1.0}));
    EXPECT_FALSE(validClimate(ClimateSpec{.seasonAtEpoch = -0.1}));
}

TEST(Weather, LongerDaysOpenTheMirrorsWiderAndEarlier)
{
    const ClimateSpec     climate{.seasonSwingHours = 2.0};
    const DayScheduleSpec day;
    const DayScheduleSpec summer = seasonalDay(day, climate, 0.25);  // longest day
    const DayScheduleSpec winter = seasonalDay(day, climate, 0.75);
    EXPECT_GT(summer.dayLengthHours, winter.dayLengthHours);
    EXPECT_LT(summer.sunriseHour, winter.sunriseHour);
    EXPECT_LT(summer.noonAngleDeg, winter.noonAngleDeg);  // sun higher at noon in summer
    EXPECT_GT(sunsetHour(summer), sunsetHour(winter));
}

TEST(Weather, ClimatesAreChecked)
{
    EXPECT_TRUE(validClimate(ClimateSpec{}));
    EXPECT_FALSE(validClimate(ClimateSpec{.cloudBaseM = 10.0}));
    EXPECT_FALSE(validClimate(ClimateSpec{.cloudBaseM = 900.0, .cloudTopM = 800.0}));
    EXPECT_FALSE(validClimate(ClimateSpec{.cloudiness = 1.4}));
    EXPECT_FALSE(validClimate(ClimateSpec{.raininess = -0.2}));
    EXPECT_FALSE(validClimate(ClimateSpec{.yearDays = 0.0}));
}

}  // namespace
