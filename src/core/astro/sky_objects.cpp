#include "StarshipSimulator/core/astro/sky_objects.h"

#include <cmath>
#include <cstddef>
#include <format>
#include <optional>
#include <string>
#include <string_view>

#include "StarshipSimulator/core/astro/ephemeris.h"
#include "StarshipSimulator/core/astro/star_catalog.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator::astro
{

namespace
{

constexpr double kLightYearsPerParsec = 3.261563777;
constexpr double kSearchRadius        = degreesToRadians(4.0);

double angleBetween(const Vec3d& a, const Vec3d& b)
{
    return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
}

/// "384,400"
std::string withThousands(double value)
{
    std::string digits = std::format("{:.0f}", value);
    for (auto i = static_cast<std::ptrdiff_t>(digits.size()) - 3; i > 0; i -= 3)
    {
        digits.insert(static_cast<std::size_t>(i), ",");
    }
    return digits;
}

Identified describeStar(const CatalogStar& star, const Vec3d& direction)
{
    std::string details;
    if (!star.properName.empty())
    {
        details = star.designation;
    }
    const std::string_view constellation = constellationName(star.constellation);
    if (!constellation.empty())
    {
        details += std::format("{}in {}", details.empty() ? "" : " ", constellation);
    }
    details += std::format("{}Magnitude {:.1f}", details.empty() ? "" : ". ", star.magnitude);
    if (star.distanceParsecs > 0.0)
    {
        const double lightYears = star.distanceParsecs * kLightYearsPerParsec;
        details += lightYears < 100.0
                       ? std::format(", {:.1f} light years away", lightYears)
                       : std::format(", {} light years away", withThousands(lightYears));
    }
    details +=
        std::format(", about {} K.",
                    withThousands(std::round(colorIndexToKelvin(star.colorIndex) / 100.0) * 100.0));
    return {.name = displayName(star), .details = details, .direction = direction};
}

}  // namespace

double illuminatedFraction(const VisibleBody& body)
{
    // Phase angle: between the directions from the body to the Sun and to the observer.
    const double phaseAngle = angleBetween(body.towardSun, -body.direction);
    return 0.5 * (1.0 + std::cos(phaseAngle));
}

const char* phaseName(double illuminatedFraction)
{
    if (illuminatedFraction < 0.03)
    {
        return "new";
    }
    if (illuminatedFraction < 0.40)
    {
        return "crescent";
    }
    if (illuminatedFraction < 0.60)
    {
        return "half lit";
    }
    if (illuminatedFraction < 0.97)
    {
        return "gibbous";
    }
    return "full";
}

std::string describeBody(const VisibleBody& body)
{
    const double lit = illuminatedFraction(body);
    if (body.body == Body::EARTH || body.body == Body::MOON)
    {
        return std::format("{}: {:.2f} degrees across, {} km away, {:.0f}% lit ({})",
                           bodyName(body.body), radiansToDegrees(2.0 * body.angularRadius),
                           withThousands(body.distanceKm), 100.0 * lit, phaseName(lit));
    }
    return std::format("{}: magnitude {:.1f}, {:.2f} AU away", bodyName(body.body), body.magnitude,
                       body.distanceKm / kKmPerAu);
}

std::optional<Identified> identifyInSky(const Vec3d& directionEqj, const SkyState& sky,
                                        const StarCatalog* catalog)
{
    const Vec3d d = glm::normalize(directionEqj);

    // Earth or the Moon when the crosshair is on (or right next to) the disk.
    for (const VisibleBody& body : sky.bodies)
    {
        const bool disk = body.body == Body::EARTH || body.body == Body::MOON;
        if (disk && angleBetween(d, body.direction) < body.angularRadius + degreesToRadians(0.5))
        {
            return Identified{.name      = bodyName(body.body),
                              .details   = describeBody(body),
                              .direction = body.direction};
        }
    }

    // Otherwise the best-matching planet or star.
    std::optional<Identified> best;
    double                    bestScore = 1.0;  // must be a match
    for (const VisibleBody& body : sky.bodies)
    {
        if (body.body == Body::EARTH || body.body == Body::MOON)
        {
            continue;
        }
        const double score = pointingScore(angleBetween(d, body.direction), body.magnitude);
        if (score < bestScore)
        {
            bestScore = score;
            best      = Identified{.name      = bodyName(body.body),
                                   .details   = std::format("Planet. {}.", describeBody(body)),
                                   .direction = body.direction};
        }
    }
    if (catalog != nullptr)
    {
        if (const auto index = catalog->identify(d, kSearchRadius))
        {
            const CatalogStar& star = catalog->stars[*index];
            const double score = pointingScore(angleBetween(d, star.direction), star.magnitude);
            if (score < bestScore)
            {
                best = describeStar(star, star.direction);
            }
        }
    }
    return best;
}

}  // namespace StarshipSimulator::astro
