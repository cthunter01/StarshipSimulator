#include "StarshipSimulator/core/habitat/Enclosure.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>

#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kHubHalfLength = 0.6;  // of the hub's radius: a drum a little wider than long

/// Where a torus's air is, with a margin kept from its walls.
struct TorusAir
{
    TorusShape shape;
    double     margin   = 0.0;
    bool       clamping = false;  // the floor's half is left to the ground: terrain and water

    [[nodiscard]] bool inTube(const Vec3d& p) const
    {
        const double t = std::hypot(p.x, p.y) - shape.centreRadiusM;
        if (clamping && t >= 0.0 && std::abs(p.z) < shape.tubeRadiusM)
        {
            return true;
        }
        const double room = shape.tubeRadiusM - margin;
        return room > 0.0 && (t * t) + (p.z * p.z) < room * room;
    }
    [[nodiscard]] bool inHub(const Vec3d& p) const
    {
        return std::hypot(p.x, p.y) < shape.hubRadiusM - margin &&
               std::abs(p.z) < shape.hubHalfLengthM - margin;
    }
    [[nodiscard]] bool inSpoke(const Vec3d& p, int k) const
    {
        const double angle = shape.spokeAngle(k);
        const double along = (p.x * std::cos(angle)) + (p.y * std::sin(angle));
        const double side  = (p.y * std::cos(angle)) - (p.x * std::sin(angle));
        const double room  = shape.spokeRadiusM - margin;
        return along >= 0.0 && along <= shape.centreRadiusM && room > 0.0 &&
               (side * side) + (p.z * p.z) < room * room;
    }
    [[nodiscard]] bool contains(const Vec3d& p) const
    {
        if (inTube(p) || inHub(p))
        {
            return true;
        }
        for (int k = 0; k < shape.spokes; ++k)
        {
            if (inSpoke(p, k))
            {
                return true;
            }
        }
        return false;
    }

    /// The nearest point in the tube.
    [[nodiscard]] Vec3d toTube(const Vec3d& p) const
    {
        const double r      = std::hypot(p.x, p.y);
        const Vec3d  radial = r > 1e-9 ? Vec3d(p.x / r, p.y / r, 0.0) : Vec3d(1.0, 0.0, 0.0);
        const double t      = r - shape.centreRadiusM;
        const double length = std::hypot(t, p.z);
        const double room   = std::max(shape.tubeRadiusM - margin, 0.0);
        const double scale  = length > room ? room / length : 1.0;
        return (radial * (shape.centreRadiusM + (t * scale))) + Vec3d(0.0, 0.0, p.z * scale);
    }
    /// The nearest point in the hub.
    [[nodiscard]] Vec3d toHub(const Vec3d& p) const
    {
        const double r     = std::hypot(p.x, p.y);
        const double room  = std::max(shape.hubRadiusM - margin, 0.0);
        const double scale = r > room ? room / r : 1.0;
        const double half  = std::max(shape.hubHalfLengthM - margin, 0.0);
        return {p.x * scale, p.y * scale, std::clamp(p.z, -half, half)};
    }
    /// The nearest point in spoke k.
    [[nodiscard]] Vec3d toSpoke(const Vec3d& p, int k) const
    {
        const double angle = shape.spokeAngle(k);
        const Vec3d  out(std::cos(angle), std::sin(angle), 0.0);
        const Vec3d  across(-std::sin(angle), std::cos(angle), 0.0);
        const double along  = std::clamp(glm::dot(p, out), 0.0, shape.centreRadiusM);
        const double side   = glm::dot(p, across);
        const double length = std::hypot(side, p.z);
        const double room   = std::max(shape.spokeRadiusM - margin, 0.0);
        const double scale  = length > room ? room / length : 1.0;
        return (out * along) + (across * (side * scale)) + Vec3d(0.0, 0.0, p.z * scale);
    }
};

}  // namespace

double TorusShape::spokeAngle(int k) const
{
    return 2.0 * kPi * static_cast<double>(k) / static_cast<double>(std::max(spokes, 1));
}

double TorusShape::ceilingRadiusAt(double z) const
{
    const double clamped = std::clamp(z, -tubeRadiusM, tubeRadiusM);
    return centreRadiusM - std::sqrt((tubeRadiusM * tubeRadiusM) - (clamped * clamped));
}

Vec3d TorusShape::tubePoint(double theta, double psi, double offsetM) const
{
    const double around = tubeRadiusM + offsetM;
    const double r      = centreRadiusM + (around * std::cos(psi));
    return {r * std::cos(theta), r * std::sin(theta), around * std::sin(psi)};
}

double TorusShape::tubeAngleOf(const Vec3d& point) const
{
    const double angle = std::atan2(point.z, std::hypot(point.x, point.y) - centreRadiusM);
    return angle < 0.0 ? angle + (2.0 * kPi) : angle;
}

TorusShape torusShape(const HabitatSpec& spec)
{
    const TorusSpec& torus = spec.torus;
    return {.centreRadiusM   = spec.radiusM - torus.tubeRadiusM,
            .tubeRadiusM     = torus.tubeRadiusM,
            .hubRadiusM      = torus.hubRadiusM,
            .hubHalfLengthM  = kHubHalfLength * torus.hubRadiusM,
            .spokes          = torus.spokes,
            .spokeRadiusM    = torus.spokeRadiusM,
            .windowHalfAngle = kPi * torus.ceilingWindowShare};
}

Enclosure::Enclosure(const HabitatSpec& spec, std::shared_ptr<const MeridianProfile> floor)
  : floor_(std::move(floor)),
    kind_(spec.kind),
    radiusM_(spec.radiusM),
    headroomM_(StarshipSimulator::headroomM(spec))
{
    if (spec.kind == HabitatKind::STANFORD_TORUS)
    {
        torus_ = torusShape(spec);
    }
}

bool Enclosure::contains(const Vec3d& point) const
{
    if (torus_)
    {
        return TorusAir{.shape = *torus_}.contains(point);
    }
    if (kind_ == HabitatKind::BERNAL_SPHERE)
    {
        return glm::length(point) < radiusM_;
    }
    const std::optional<double> floorRadius = floor_->radiusAt(point.z);
    return floorRadius && std::hypot(point.x, point.y) < *floorRadius;
}

Vec3d Enclosure::clampInside(const Vec3d& point, double marginM) const
{
    if (!torus_)
    {
        return point;
    }
    const TorusAir air{.shape = *torus_, .margin = marginM, .clamping = true};
    if (air.contains(point))
    {
        return point;
    }
    // Out through a wall: back to the nearest point of whichever part it is nearest.
    Vec3d        best     = air.toTube(point);
    double       nearest  = glm::distance(best, point);
    const Vec3d  hub      = air.toHub(point);
    const double toTheHub = glm::distance(hub, point);
    if (toTheHub < nearest)
    {
        best    = hub;
        nearest = toTheHub;
    }
    for (int k = 0; k < torus_->spokes; ++k)
    {
        const Vec3d  spoke = air.toSpoke(point, k);
        const double to    = glm::distance(spoke, point);
        if (to < nearest)
        {
            best    = spoke;
            nearest = to;
        }
    }
    return best;
}

}  // namespace StarshipSimulator
