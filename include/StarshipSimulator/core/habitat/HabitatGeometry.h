#pragma once

#include <cstdint>
#include <optional>

#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/SimplexNoise.h"

namespace StarshipSimulator
{

enum class RegionKind : std::uint8_t
{
    Land,     // a valley floor strip
    Window,   // a window strip (walkable glass)
    Endcap,   // endcap slopes or domes
    Outside,  // beyond the ends of the habitat
};

struct Region
{
    RegionKind kind  = RegionKind::Outside;
    int        index = -1;  // strip number for Land and Window
};

[[nodiscard]] const char* regionKindName(RegionKind kind);

/// The ground below (or above) a point, measured along the local vertical (the radial line).
struct GroundSample
{
    double groundRadius      = 0.0;  // distance of the ground from the spin axis
    double heightAboveGround = 0.0;  // along "up" (toward the axis); negative below ground
    Vec3d  up{0.0, 0.0, 1.0};        // local up: toward the spin axis
    Vec3d  normal{0.0, 0.0, 1.0};    // ground surface normal, into the habitat
    double slopeRadians = 0.0;       // angle between normal and up
    Region region;
};

/// The walkable shape of an O'Neill cylinder in the rotating habitat frame (spin axis +Z, sun
/// toward +Z): the revolved meridian profile plus procedural terrain, with window strips along the
/// floor. Window i is centred on angle i * stripAngle, land strip i halfway to the next window.
class HabitatGeometry
{
public:
    /// Throws std::invalid_argument when the spec does not validate.
    explicit HabitatGeometry(const OneillCylinderSpec& spec);

    [[nodiscard]] const OneillCylinderSpec& spec() const { return spec_; }
    [[nodiscard]] const MeridianProfile&    profile() const { return profile_; }
    [[nodiscard]] const Landscape&          landscape() const { return landscape_; }
    [[nodiscard]] double                    radius() const { return spec_.radiusM; }
    [[nodiscard]] double                    omega() const { return omega_; }
    [[nodiscard]] double                    floorZMin() const { return floorZMin_; }
    [[nodiscard]] double                    floorZMax() const { return floorZMax_; }
    /// Range of z a person can occupy (inside the end walls, away from the domes' poles).
    [[nodiscard]] double walkableZMin() const { return walkableZMin_; }
    [[nodiscard]] double walkableZMax() const { return walkableZMax_; }

    [[nodiscard]] int    stripCount() const { return spec_.stripPairs; }
    [[nodiscard]] double stripAngle() const;
    [[nodiscard]] double windowHalfAngle() const;
    [[nodiscard]] double landHalfAngle() const;
    [[nodiscard]] double windowCenter(int index) const;
    [[nodiscard]] double landCenter(int index) const;

    [[nodiscard]] Region regionAt(double z, double theta) const;
    /// Soil height above the profile surface (toward the axis), metres; zero on windows, below
    /// zero in rivers and lakes (see Landscape).
    [[nodiscard]] double terrainHeight(double z, double theta) const;
    /// The hills and mountains alone, before water shapes them (smooth, non-negative).
    [[nodiscard]] double naturalHeight(double z, double theta) const;
    /// How wooded the ground is, 0..1: the landscape's woods, kept off water, walkways and cliffs.
    [[nodiscard]] double forestDensity(double z, double theta) const;
    /// Whether (z, theta) is under water, and how deep (m, 0 on land).
    [[nodiscard]] double waterDepth(double z, double theta) const;
    /// Distance of the ground from the axis at (z, theta), or nullopt beyond the ends.
    [[nodiscard]] std::optional<double> groundRadius(double z, double theta) const;
    /// Point on the ground at (z, theta); z is clamped to the profile.
    [[nodiscard]] Vec3d        surfacePoint(double z, double theta) const;
    [[nodiscard]] GroundSample ground(const Vec3d& position) const;
    /// Distance along a ray to the ground, if it is hit within maxDistance.
    [[nodiscard]] std::optional<double> raycast(const Vec3d& origin, const Vec3d& direction,
                                                double maxDistance) const;

    /// Spin gravity (m/s^2) at distance r from the axis.
    [[nodiscard]] double gravityAt(double r) const { return omega_ * omega_ * r; }

    /// Local up (toward the axis) at a position, or +X direction fallback on the axis itself.
    [[nodiscard]] static Vec3d localUp(const Vec3d& position);
    /// Angle around the spin axis in [0, 2*pi).
    [[nodiscard]] static double angleOf(const Vec3d& position);
    /// |a - b| wrapped to [0, pi].
    [[nodiscard]] static double angularDistance(double a, double b);

private:
    /// Distance (m) from a point on the surface to the nearest window strip, 0 inside one.
    [[nodiscard]] double distanceToWindow(double z, double theta, double radius) const;

    OneillCylinderSpec spec_;
    MeridianProfile    profile_;
    SimplexNoise       hills_;
    SimplexNoise       ridges_;
    double             omega_        = 0.0;
    double             floorZMin_    = 0.0;
    double             floorZMax_    = 0.0;
    double             walkableZMin_ = 0.0;
    double             walkableZMax_ = 0.0;
    Landscape          landscape_;
};

}  // namespace StarshipSimulator
