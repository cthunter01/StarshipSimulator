#pragma once

#include <memory>
#include <optional>

#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// A Stanford torus's air, in the habitat frame: a tube round the axis, spokes reaching in from it
/// and a hub at the middle. A point on the tube's circle at tube angle psi lies at
/// radial * (centre + a cos psi) + z * (a sin psi): psi = 0 is the floor's lowest line, pi the
/// ceiling's innermost, where the spokes meet the tube.
struct TorusShape
{
    double centreRadiusM   = 0.0;  // the axis to the middle of the tube
    double tubeRadiusM     = 0.0;
    double hubRadiusM      = 0.0;  // the hub is a drum round the axis
    double hubHalfLengthM  = 0.0;
    int    spokes          = 0;
    double spokeRadiusM    = 0.0;  // each spoke a tube along a radius, at z = 0
    double windowHalfAngle = 0.0;  // the ceiling's glass: within this tube angle of pi

    /// The angle round the axis of spoke k.
    [[nodiscard]] double spokeAngle(int k) const;
    /// The radius of the ceiling at z (its hub-facing half), for |z| up to the tube's radius.
    [[nodiscard]] double ceilingRadiusAt(double z) const;
    /// The point on the tube's surface at angle theta round the axis and tube angle psi, `offsetM`
    /// outside it.
    [[nodiscard]] Vec3d tubePoint(double theta, double psi, double offsetM = 0.0) const;
    /// The tube angle of a point (0 at the floor's lowest line, pi at the ceiling's innermost).
    [[nodiscard]] double tubeAngleOf(const Vec3d& point) const;
};

[[nodiscard]] TorusShape torusShape(const HabitatSpec& spec);

/// The habitat's air: the space inside it where a person, a thrown ball or a camera can be. In a
/// cylinder or a sphere that is everything above the floor, up to and across the axis; in a torus
/// the tube under its ceiling, the spokes and the hub; an open ring adds its walls (M10).
class Enclosure
{
public:
    Enclosure(const HabitatSpec& spec, std::shared_ptr<const MeridianProfile> floor);

    /// Whether a point is inside the air: above the floor (its datum, before any terrain) and
    /// within the habitat's length; in a sphere, anywhere inside its hull, the polar caps included;
    /// in a torus, inside its tube, a spoke or the hub.
    [[nodiscard]] bool contains(const Vec3d& point) const;

    /// The nearest point to `point` inside the air and at least `marginM` from the ceiling and
    /// the walls of a torus's tube, spokes and hub (the point itself when it is already there, and
    /// always in the habitats that have no ceiling: their floor and ends are kept elsewhere, as is
    /// the torus's floor, below which its terrain and water can reach).
    [[nodiscard]] Vec3d clampInside(const Vec3d& point, double marginM) const;

    /// How high the air goes above the floor before it meets the far side or the ceiling.
    [[nodiscard]] double headroomM() const { return headroomM_; }

    /// A torus's tube, spokes and hub; nothing for the other kinds.
    [[nodiscard]] const std::optional<TorusShape>& torus() const { return torus_; }

private:
    std::shared_ptr<const MeridianProfile> floor_;
    HabitatKind                            kind_      = HabitatKind::ONEILL_CYLINDER;
    double                                 radiusM_   = 0.0;
    double                                 headroomM_ = 0.0;
    std::optional<TorusShape>              torus_;
};

}  // namespace StarshipSimulator
