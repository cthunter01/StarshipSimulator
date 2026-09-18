#include "StarshipSimulator/core/habitat/day_schedule.h"

#include <algorithm>
#include <cmath>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

double sunsetHour(const DayScheduleSpec& schedule)
{
    return std::fmod(schedule.sunriseHour + schedule.dayLengthHours, 24.0);
}

double scheduledMirrorAngleDeg(const DayScheduleSpec& schedule, double hourOfDay)
{
    const double dayLength    = std::clamp(schedule.dayLengthHours, 0.0, 24.0);
    const double nightLength  = 24.0 - dayLength;
    double       sinceSunrise = std::fmod(hourOfDay - schedule.sunriseHour, 24.0);
    if (sinceSunrise < 0.0)
    {
        sinceSunrise += 24.0;
    }
    if (sinceSunrise < dayLength)
    {
        const double f = sinceSunrise / dayLength;
        return 90.0 - ((90.0 - schedule.noonAngleDeg) * std::sin(kPi * f));
    }
    if (nightLength <= 0.0)
    {
        return 90.0;
    }
    const double g = (sinceSunrise - dayLength) / nightLength;
    return 90.0 + ((schedule.nightAngleDeg - 90.0) * std::sin(kPi * g));
}

}  // namespace StarshipSimulator
