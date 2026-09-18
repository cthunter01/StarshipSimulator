#pragma once

// The habitat's day: the mirrors swing on a schedule. At sunrise they stand at 90 degrees (the sun
// image sits on the anti-sunward horizon), close to the noon angle (45 = overhead) at midday,
// return to 90 at sunset, then open past 90 so no sunlight enters and the windows show the stars.
namespace StarshipSimulator
{

struct DayScheduleSpec
{
    bool   enabled        = true;  // false: mirrors stay at the habitat's fixed opening angle
    double dayLengthHours = 14.0;
    double sunriseHour    = 6.0;    // local time
    double noonAngleDeg   = 45.0;   // mirror opening angle at midday
    double nightAngleDeg  = 105.0;  // mirror opening angle at midnight (past 90 = dark)
};

/// Mirror opening angle in degrees at a local hour of day (0..24).
[[nodiscard]] double scheduledMirrorAngleDeg(const DayScheduleSpec& schedule, double hourOfDay);

/// Local hours of sunset.
[[nodiscard]] double sunsetHour(const DayScheduleSpec& schedule);

}  // namespace StarshipSimulator
