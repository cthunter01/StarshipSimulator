#pragma once

#include <optional>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// One vertex of a meridian profile: axial position z, distance r from the spin axis, and arc
/// length u measured along the profile from its start.
struct ProfilePoint
{
    double z = 0.0;
    double r = 0.0;
    double u = 0.0;
};

/// The cross-section of a spinning habitat's inner surface in the (z, r) half-plane. Revolving it
/// around the spin axis gives the whole surface. z must increase along the profile, so the surface
/// radius is a function r(z). This one description covers cylinders, endcaps, tori and rings.
class MeridianProfile
{
public:
    /// Points as (z, r) pairs with strictly increasing z. Throws std::invalid_argument otherwise.
    explicit MeridianProfile(const std::vector<Vec2d>& zr);

    [[nodiscard]] const std::vector<ProfilePoint>& points() const { return points_; }
    [[nodiscard]] double                           length() const { return points_.back().u; }
    [[nodiscard]] double                           zMin() const { return points_.front().z; }
    [[nodiscard]] double                           zMax() const { return points_.back().z; }

    /// Surface radius at z, or nullopt beyond the ends.
    [[nodiscard]] std::optional<double> radiusAt(double z) const;
    /// (z, r) at arc length u (clamped to the profile).
    [[nodiscard]] Vec2d pointAt(double u) const;
    /// Arc length at axial position z (clamped).
    [[nodiscard]] double arcAt(double z) const;
    /// Unit tangent (dz/du, dr/du) at arc length u.
    [[nodiscard]] Vec2d tangentAt(double u) const;
    /// Unit normal (z, r components) pointing into the habitat interior at arc length u.
    [[nodiscard]] Vec2d inwardNormalAt(double u) const;

private:
    std::vector<ProfilePoint> points_;
};

/// The inner surface profile of an O'Neill cylinder with its endcaps (anti-sunward end first).
[[nodiscard]] MeridianProfile buildOneillProfile(const HabitatSpec& spec);

/// The floor's profile for any kind of habitat (anti-sunward end first). Throws
/// std::invalid_argument for a kind that cannot be built yet.
[[nodiscard]] MeridianProfile buildFloorProfile(const HabitatSpec& spec);

}  // namespace StarshipSimulator
