#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

// Where a habitat's land is. Whatever the habitat's shape, its land comes in bands: strips of floor
// that the rivers, towns, farms and tramways are laid out on as if on a flat plan, with x across
// the band and y along it. An O'Neill cylinder's valleys run along the axis; the land of a torus, a
// sphere or a short cylinder runs around it, once all the way round.
namespace StarshipSimulator
{

enum class BandAxis : std::uint8_t
{
    ALONG_Z,  // the band runs along the spin axis (plan y is z itself)
    AROUND,   // the band runs around the axis (plan y is the arc around it)
};

/// A point on the floor, by where it is along the axis and around it.
struct SurfaceSpot
{
    double z     = 0.0;
    double theta = 0.0;
};

/// One band of land, and the plan it is laid out on. The plan is true to scale at the band's middle
/// line and keeps one handedness for every kind of band: seen from above (from the axis), plan x
/// turned a quarter left gives plan y, so the same layout code builds the same towns on any band.
struct LandBand
{
    int      index       = 0;
    BandAxis axis        = BandAxis::ALONG_Z;
    double   centreTheta = 0.0;  // ALONG_Z: the band's middle line; AROUND: where plan y = 0
    double   centreU     = 0.0;  // AROUND: profile arc length of the middle line (plan x = 0)
    double   radiusM = 1.0;  // plan metres around the axis per radian (floor radius at the middle)
    double   alongMinM  = 0.0;  // plan y range; ALONG_Z: the floor's z range
    double   alongMaxM  = 0.0;
    double   halfWidthM = 0.0;    // plan x from -halfWidthM to +halfWidthM is land
    bool     wraps      = false;  // AROUND bands go all the way round: plan y wraps
    std::shared_ptr<const MeridianProfile> profile;  // AROUND: converts z to arc length and back

    /// (x across, y along) on the plan, metres.
    [[nodiscard]] Vec2d       toPlan(double z, double theta) const;
    [[nodiscard]] SurfaceSpot toSurface(const Vec2d& plan) const;
    [[nodiscard]] double      alongLengthM() const { return alongMaxM - alongMinM; }
    /// Unit direction of plan +y in the habitat frame, at angle theta around the axis.
    [[nodiscard]] Vec3d alongDirection(double theta) const;
};

/// The bands of land of a habitat, with its floor profile and the z range of its floor.
[[nodiscard]] std::vector<LandBand> planLandBands(
    const HabitatSpec& spec, const std::shared_ptr<const MeridianProfile>& floor, double floorZMin,
    double floorZMax);

}  // namespace StarshipSimulator
