#include "StarshipSimulator/core/habitat/MeridianProfile.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr int kHemisphereSegments = 60;  // 1.5 degrees each

/// Index of the segment [i, i+1] containing key, found with a projection onto the point member.
template <typename Projection>
std::size_t segmentIndex(const std::vector<ProfilePoint>& points, double key, Projection project)
{
    const auto upper = std::ranges::upper_bound(points, key, {}, project);
    const auto index = static_cast<std::size_t>(std::distance(points.begin(), upper));
    return std::clamp<std::size_t>(index, 1, points.size() - 1) - 1;
}

/// Appends endcap points going outward from the axis (for the anti-sunward end) at the end z =
/// zEnd; direction is +1 when the cylinder lies toward +z from the end.
void appendEndcap(std::vector<Vec2d>& points, const EndcapSpec& endcap, double radius, double zEnd,
                  double direction)
{
    switch (endcap.shape)
    {
        case EndcapShape::FLAT:
            points.emplace_back(zEnd, radius);
            break;
        case EndcapShape::HEMISPHERE:
            // A dome beyond the cylinder: pole at zEnd - direction * R, rim at zEnd.
            for (int i = kHemisphereSegments; i >= 0; --i)
            {
                const double angle = (kPi / 2.0) * i / kHemisphereSegments;
                points.emplace_back(zEnd - (direction * radius * std::sin(angle)),
                                    radius * std::cos(angle));
            }
            break;
        case EndcapShape::CONICAL_RAMP:
        {
            const double topRadius = endcap.rampTopRadiusFraction * radius;
            const double upper =
                (topRadius - endcap.hubRadiusM) / std::tan(degreesToRadians(endcap.upperSlopeDeg));
            const double lower =
                (radius - topRadius) / std::tan(degreesToRadians(endcap.rampSlopeDeg));
            points.emplace_back(zEnd, endcap.hubRadiusM);
            points.emplace_back(zEnd + (direction * upper), topRadius);
            points.emplace_back(zEnd + (direction * (upper + lower)), radius);
            break;
        }
    }
}

}  // namespace

MeridianProfile::MeridianProfile(const std::vector<Vec2d>& zr)
{
    if (zr.size() < 2)
    {
        throw std::invalid_argument("a meridian profile needs at least two points");
    }
    points_.reserve(zr.size());
    double u = 0.0;
    for (std::size_t i = 0; i < zr.size(); ++i)
    {
        if (i > 0)
        {
            if (!(zr[i].x > zr[i - 1].x))
            {
                throw std::invalid_argument("meridian profile z must increase strictly");
            }
            u += glm::distance(zr[i], zr[i - 1]);
        }
        points_.push_back({.z = zr[i].x, .r = zr[i].y, .u = u});
    }
}

std::optional<double> MeridianProfile::radiusAt(double z) const
{
    if (z < zMin() || z > zMax())
    {
        return std::nullopt;
    }
    const std::size_t   i = segmentIndex(points_, z, &ProfilePoint::z);
    const ProfilePoint& a = points_[i];
    const ProfilePoint& b = points_[i + 1];
    return std::lerp(a.r, b.r, (z - a.z) / (b.z - a.z));
}

Vec2d MeridianProfile::pointAt(double u) const
{
    const double        clamped = std::clamp(u, 0.0, length());
    const std::size_t   i       = segmentIndex(points_, clamped, &ProfilePoint::u);
    const ProfilePoint& a       = points_[i];
    const ProfilePoint& b       = points_[i + 1];
    const double        t       = (clamped - a.u) / (b.u - a.u);
    return {std::lerp(a.z, b.z, t), std::lerp(a.r, b.r, t)};
}

double MeridianProfile::arcAt(double z) const
{
    const double        clamped = std::clamp(z, zMin(), zMax());
    const std::size_t   i       = segmentIndex(points_, clamped, &ProfilePoint::z);
    const ProfilePoint& a       = points_[i];
    const ProfilePoint& b       = points_[i + 1];
    return std::lerp(a.u, b.u, (clamped - a.z) / (b.z - a.z));
}

Vec2d MeridianProfile::tangentAt(double u) const
{
    const std::size_t   i = segmentIndex(points_, std::clamp(u, 0.0, length()), &ProfilePoint::u);
    const ProfilePoint& a = points_[i];
    const ProfilePoint& b = points_[i + 1];
    return glm::normalize(Vec2d(b.z - a.z, b.r - a.r));
}

Vec2d MeridianProfile::inwardNormalAt(double u) const
{
    // Rotate the tangent so that on a cylinder floor (tangent +z) the normal points toward the
    // axis.
    const Vec2d tangent = tangentAt(u);
    return {tangent.y, -tangent.x};
}

MeridianProfile buildOneillProfile(const HabitatSpec& spec)
{
    const double       radius = spec.radiusM;
    const double       half   = spec.lengthM / 2.0;
    std::vector<Vec2d> points;
    appendEndcap(points, spec.antisunwardEndcap, radius, -half, 1.0);

    std::vector<Vec2d> sunward;
    appendEndcap(sunward, spec.sunwardEndcap, radius, half, -1.0);
    // The sunward endcap was built from the axis outward; walk it from the floor to the axis.
    std::ranges::reverse(sunward);
    points.insert(points.end(), sunward.begin(), sunward.end());
    return MeridianProfile(points);
}

}  // namespace StarshipSimulator

namespace StarshipSimulator
{

MeridianProfile buildFloorProfile(const HabitatSpec& spec)
{
    switch (spec.kind)
    {
        case HabitatKind::ONEILL_CYLINDER:
            return buildOneillProfile(spec);
        case HabitatKind::KALPANA_CYLINDER:
            // A plain cylinder between two flat end walls.
            return buildOneillProfile(normalizedForKind(spec));
        case HabitatKind::STANFORD_TORUS:
        case HabitatKind::BERNAL_SPHERE:
        case HabitatKind::BISHOP_RING:
            break;
    }
    throw std::invalid_argument(
        std::format("a {} has no floor profile yet", habitatKindName(spec.kind)));
}

}  // namespace StarshipSimulator
