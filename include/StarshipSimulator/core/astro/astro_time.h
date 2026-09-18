#pragma once

#include <compare>
#include <cstdint>
#include <expected>
#include <string>
#include <string_view>

namespace StarshipSimulator::astro
{

/// A moment in (universal) time: microseconds since J2000, 2000-01-01 12:00 UT, the epoch Astronomy
/// Engine counts from. Integer microseconds keep time exact over centuries of time-lapse.
struct SimTime
{
    std::int64_t microseconds = 0;

    /// Days since J2000 (UT), as Astronomy Engine's `ut`.
    [[nodiscard]] double         days() const;
    [[nodiscard]] static SimTime fromDays(double days);
    /// From Terrestrial Time days since J2000 (as JPL ephemerides use); applies Delta T.
    [[nodiscard]] static SimTime fromTerrestrialDays(double days);

    [[nodiscard]] SimTime plusSeconds(double seconds) const;

    friend auto operator<=>(const SimTime&, const SimTime&) = default;
};

struct CalendarTime
{
    int    year   = 2000;
    int    month  = 1;
    int    day    = 1;
    int    hour   = 12;
    int    minute = 0;
    double second = 0.0;
};

[[nodiscard]] SimTime      fromCalendar(const CalendarTime& calendar);
[[nodiscard]] CalendarTime toCalendar(SimTime time);

/// Parses "2045-06-21T06:30:00Z", "2045-06-21 06:30" or "2045-06-21" (all UTC).
[[nodiscard]] std::expected<SimTime, std::string> parseIsoTime(std::string_view text);
/// "2045-06-21T06:30:00Z"
[[nodiscard]] std::string formatIsoTime(SimTime time);

/// Hours since the habitat's local midnight, 0..24.
[[nodiscard]] double hourOfDay(SimTime time, double utcOffsetHours);

}  // namespace StarshipSimulator::astro
