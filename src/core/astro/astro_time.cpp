#include "StarshipSimulator/core/astro/astro_time.h"

#include <astronomy.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <string>
#include <string_view>

#include "StarshipSimulator/core/parse_number.h"

namespace StarshipSimulator::astro
{

namespace
{

constexpr std::int64_t kMicrosecondsPerDay = 86'400'000'000;
constexpr std::int64_t kSecondsPerHalfDay  = 43'200;  // J2000 is at noon
// 2000-01-01; J2000 itself is at 12:00 UT that day.
constexpr std::chrono::sys_days kJ2000Date{std::chrono::year{2000} / std::chrono::January / 1};

}  // namespace

double SimTime::days() const
{
    return static_cast<double>(microseconds) / static_cast<double>(kMicrosecondsPerDay);
}

SimTime SimTime::fromDays(double days)
{
    return {.microseconds = std::llround(days * static_cast<double>(kMicrosecondsPerDay))};
}

SimTime SimTime::fromTerrestrialDays(double days)
{
    return fromDays(Astronomy_TerrestrialTime(days).ut);
}

SimTime SimTime::plusSeconds(double seconds) const
{
    return {.microseconds = microseconds + std::llround(seconds * 1.0e6)};
}

SimTime fromCalendar(const CalendarTime& calendar)
{
    const std::chrono::sys_days date = std::chrono::year{calendar.year} /
                                       std::chrono::month{static_cast<unsigned>(calendar.month)} /
                                       std::chrono::day{static_cast<unsigned>(calendar.day)};
    const std::int64_t          days = (date - kJ2000Date).count();
    const std::int64_t          seconds = (std::int64_t{calendar.hour} * 3600) +
                                          (std::int64_t{calendar.minute} * 60) - kSecondsPerHalfDay;
    return {.microseconds = (days * kMicrosecondsPerDay) + (seconds * 1'000'000) +
                            std::llround(calendar.second * 1.0e6)};
}

CalendarTime toCalendar(SimTime time)
{
    // Shift the epoch from noon to midnight, then split into whole days and time of day.
    const std::int64_t sinceMidnight = time.microseconds + (kMicrosecondsPerDay / 2);
    std::int64_t       days          = sinceMidnight / kMicrosecondsPerDay;
    std::int64_t       timeOfDay     = sinceMidnight % kMicrosecondsPerDay;
    if (timeOfDay < 0)
    {
        timeOfDay += kMicrosecondsPerDay;
        --days;
    }
    const std::chrono::year_month_day date{kJ2000Date + std::chrono::days{days}};
    const std::int64_t                minutes = timeOfDay / 60'000'000;
    return {.year   = static_cast<int>(date.year()),
            .month  = static_cast<int>(static_cast<unsigned>(date.month())),
            .day    = static_cast<int>(static_cast<unsigned>(date.day())),
            .hour   = static_cast<int>(minutes / 60),
            .minute = static_cast<int>(minutes % 60),
            .second = static_cast<double>(timeOfDay % 60'000'000) / 1.0e6};
}

std::expected<SimTime, std::string> parseIsoTime(std::string_view text)
{
    const auto fail = [&]() {
        return std::unexpected(
            std::format("expected a UTC date like 2045-06-21T06:30:00Z, got '{}'", text));
    };
    std::string_view rest = text;
    if (!rest.empty() && rest.back() == 'Z')
    {
        rest.remove_suffix(1);
    }
    if (rest.size() < 10 || rest[4] != '-' || rest[7] != '-')
    {
        return fail();
    }
    CalendarTime calendar{.year = 0, .month = 0, .day = 0, .hour = 0, .minute = 0, .second = 0.0};
    const auto   year  = parseInt(rest.substr(0, 4));
    const auto   month = parseInt(rest.substr(5, 2));
    const auto   day   = parseInt(rest.substr(8, 2));
    if (!year || !month || !day || *month < 1 || *month > 12 || *day < 1 || *day > 31)
    {
        return fail();
    }
    calendar.year  = *year;
    calendar.month = *month;
    calendar.day   = *day;
    if (!(std::chrono::year{*year} / std::chrono::month{static_cast<unsigned>(*month)} /
          std::chrono::day{static_cast<unsigned>(*day)})
             .ok())
    {
        return fail();  // e.g. February 30
    }
    rest.remove_prefix(10);
    if (!rest.empty())
    {
        // "T06:30" or " 06:30", optionally ":ss".
        if ((rest[0] != 'T' && rest[0] != ' ') || (rest.size() != 6 && rest.size() != 9) ||
            rest[3] != ':')
        {
            return fail();
        }
        const auto         hour   = parseInt(rest.substr(1, 2));
        const auto         minute = parseInt(rest.substr(4, 2));
        std::optional<int> second = 0;
        if (rest.size() == 9)
        {
            second = rest[6] == ':' ? parseInt(rest.substr(7, 2)) : std::nullopt;
        }
        if (!hour || !minute || !second || *hour > 23 || *minute > 59 || *second > 60)
        {
            return fail();
        }
        calendar.hour   = *hour;
        calendar.minute = *minute;
        calendar.second = *second;
    }
    return fromCalendar(calendar);
}

std::string formatIsoTime(SimTime time)
{
    const CalendarTime c = toCalendar(time);
    return std::format("{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z", c.year, c.month, c.day, c.hour,
                       c.minute, static_cast<int>(std::floor(c.second)));
}

double hourOfDay(SimTime time, double utcOffsetHours)
{
    // J2000 is at 12:00 UT.
    const double hours = std::fmod((time.days() * 24.0) + 12.0 + utcOffsetHours, 24.0);
    return hours < 0.0 ? hours + 24.0 : hours;
}

}  // namespace StarshipSimulator::astro
