#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "StarshipSimulator/core/habitat/Enclosure.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/SimplexNoise.h"

namespace StarshipSimulator
{

enum class RegionKind : std::uint8_t
{
    LAND,     // a band of land (an O'Neill cylinder's valley floor)
    WINDOW,   // walkable glass (an O'Neill cylinder's window strip)
    ENDCAP,   // the slopes off the land: endcaps, domes
    OUTSIDE,  // beyond the ends of the habitat
};

struct Region
{
    RegionKind kind  = RegionKind::OUTSIDE;
    int        index = -1;  // the band for LAND, the window strip for WINDOW
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

/// The walkable shape of a spinning habitat in the rotating habitat frame (spin axis +Z, sun toward
/// +Z): the floor's meridian profile revolved about the axis, plus procedural terrain, and the
/// bands of land on it. On an O'Neill cylinder, window i is centred on angle i * stripAngle and
/// land strip (valley) i halfway to the next window.
class HabitatGeometry
{
public:
    /// Throws std::invalid_argument when the spec does not validate.
    explicit HabitatGeometry(const HabitatSpec& spec);

    [[nodiscard]] const HabitatSpec&     spec() const { return spec_; }
    [[nodiscard]] HabitatKind            kind() const { return spec_.kind; }
    [[nodiscard]] const MeridianProfile& profile() const { return *profile_; }
    [[nodiscard]] const std::shared_ptr<const MeridianProfile>& sharedProfile() const
    {
        return profile_;
    }
    [[nodiscard]] const Landscape& landscape() const { return landscape_; }
    /// The air inside: where people and cameras can be.
    [[nodiscard]] const Enclosure& enclosure() const { return enclosure_; }
    /// The floor's reference radius: gravity, air, water and clouds are measured from it.
    [[nodiscard]] double radius() const { return spec_.radiusM; }
    [[nodiscard]] double omega() const { return omega_; }
    /// The z range of the land (on an O'Neill cylinder: the level floor between the endcaps).
    [[nodiscard]] double floorZMin() const { return floorZMin_; }
    [[nodiscard]] double floorZMax() const { return floorZMax_; }
    /// The floor's radius at z (clamped to the profile), before any terrain.
    [[nodiscard]] double floorRadiusAt(double z) const;
    /// How far it is from the floor straight up to the far side (or the ceiling).
    [[nodiscard]] double spanAcrossM() const;

    [[nodiscard]] const std::vector<LandBand>& bands() const { return bands_; }
    [[nodiscard]] int             bandCount() const { return static_cast<int>(bands_.size()); }
    [[nodiscard]] const LandBand& band(int index) const;
    /// Whether (z, theta) is on a band of land.
    [[nodiscard]] bool onLand(double z, double theta) const;
    /// Distance (m) along the floor from a point to the nearest window strip, 0 inside one; it also
    /// grows beyond the ends of the level floor. `radius` is the floor's radius there.
    [[nodiscard]] double distanceToWindow(double z, double theta, double radius) const;
    /// Height of the water's surface above the floor datum at z, measured like terrain height. The
    /// surface is level under spin gravity: a cylinder around the axis. On a cylinder's level floor
    /// that is kWaterLevelM everywhere; on a sphere (or in a torus's tube) the floor rises away
    /// from the land's middle line and the water lies deeper under it.
    [[nodiscard]] double waterLevelAt(double z) const;
    /// How far the floor may rise above the band's middle line where there is still water (0 on a
    /// level floor): the water's surface can lie that far below the floor datum.
    [[nodiscard]] double maxWaterRiseM() const;
    /// How far across the band (arc metres from its middle line) water can lie, where the floor
    /// rises away from it; nullopt where it is level.
    [[nodiscard]] std::optional<double> waterReachAcrossM() const;
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
    /// The hills and mountains alone, before water shapes them (non-negative; smooth but for a
    /// torus's terraces).
    [[nodiscard]] double naturalHeight(double z, double theta) const;
    /// The terraces up a torus's walls, part of naturalHeight: a staircase of level shelves (0
    /// elsewhere). They have sharp edges, so the terrain grid takes them at its full resolution.
    [[nodiscard]] double terraceHeight(double z) const;
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
    [[nodiscard]] bool isSphere() const { return spec_.kind == HabitatKind::BERNAL_SPHERE; }
    /// Whether the floor curves up away from the land's middle line, to the polar windows of a
    /// sphere or up the walls of a torus's tube to its ceiling.
    [[nodiscard]] bool floorCurves() const
    {
        return isSphere() || spec_.kind == HabitatKind::STANFORD_TORUS;
    }
    /// Arc length along the floor from the land's edge, 0 on the land (the polar slopes).
    [[nodiscard]] double arcBeyondLand(double z) const;
    /// Arc length of one polar slope (or tube wall), from the land's edge to the profile's end.
    [[nodiscard]] double slopeArcM() const;

    HabitatSpec                            spec_;
    std::shared_ptr<const MeridianProfile> profile_;
    SimplexNoise                           hills_;
    SimplexNoise                           ridges_;
    double                                 omega_        = 0.0;
    double                                 floorZMin_    = 0.0;
    double                                 floorZMax_    = 0.0;
    double                                 walkableZMin_ = 0.0;
    double                                 walkableZMax_ = 0.0;
    std::vector<LandBand>                  bands_;
    Enclosure                              enclosure_;
    Landscape                              landscape_;
};

}  // namespace StarshipSimulator
