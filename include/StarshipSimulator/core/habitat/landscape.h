#pragma once

#include <vector>

#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/procgen/noise.h"

// The valleys' water and woods. Water in a habitat is shallow: every tonne of it presses on the
// hull, so the rivers and lakes are wading depth. Their surfaces are equipotentials of spin
// gravity, cylinders around the axis, so a lake curves up with the floor. The rivers cannot run
// downhill (the floor is level along the axis): they are pumped streams, kept moving the way a
// fountain is.
namespace StarshipSimulator
{

/// Height of the water surface and the deepest water, in metres, measured like terrain height:
/// toward the axis from the floor datum.
inline constexpr double kWaterLevelM = -0.4;
inline constexpr double kWaterDepthM = 1.1;

/// Where the habitat's land is: what the landscape needs from the habitat geometry.
struct LandscapeFrame
{
    double radiusM         = 4000.0;
    double floorZMin       = -12000.0;
    double floorZMax       = 12000.0;
    int    stripCount      = 3;
    double stripAngle      = 0.0;  // radians between window centres
    double windowHalfAngle = 0.0;  // radians
};

/// A lake: an ellipse on the valley floor, longer along the axis than across.
struct Lake
{
    int    valley      = 0;
    double z           = 0.0;  // centre
    double theta       = 0.0;  // centre (radians)
    double halfLengthM = 0.0;  // along the axis
    double halfWidthM  = 0.0;  // around the habitat
};

class Landscape
{
public:
    Landscape(const TerrainSpec& terrain, const LandscapeFrame& frame);

    [[nodiscard]] bool                     hasRivers() const { return riverHalfWidthM_ > 0.0; }
    [[nodiscard]] const std::vector<Lake>& lakes() const { return lakes_; }

    /// The river's centreline in valley `valley` at axial position z (radians).
    [[nodiscard]] double riverAngle(int valley, double z) const;

    /// Signed distance along the floor from (z, theta) to the nearest shore, in metres: negative in
    /// the water. Large (at least `far`) away from all water; cheap to evaluate beyond that.
    [[nodiscard]] double shoreDistance(double z, double theta, double far = 1000.0) const;

    /// How strongly the land here tends toward woods, 0..1, before the geometry rules them out
    /// (water, walkways, cliffs). About `forestCover` of the floor has a value over one half.
    [[nodiscard]] double woodland(double z, double theta) const;

    /// Terrain height near water: the bed, the banks and the flat floodplain, blended into the
    /// natural height `natural` farther away. `shore` from shoreDistance().
    [[nodiscard]] static double shapeNearWater(double natural, double shore);

private:
    [[nodiscard]] double riverDistance(int valley, double z, double theta) const;
    [[nodiscard]] double meanderAngle(int valley, double z) const;  // from the noise
    [[nodiscard]] double lakeDistance(const Lake& lake, double z, double theta) const;
    [[nodiscard]] int    valleyAt(double theta) const;

    LandscapeFrame    frame_;
    double            riverHalfWidthM_ = 0.0;
    double            meanderM_        = 0.0;  // amplitude of the river's swing
    double            riverZMin_       = 0.0;
    double            riverZMax_       = 0.0;
    double            woodsThreshold_  = 0.0;
    std::vector<Lake> lakes_;
    // The rivers' centrelines (theta) and their slant (d arc / d z), tabulated along z.
    std::vector<std::vector<Vec2d>> riverTable_;
    double                          riverTableStep_ = 1.0;
    SimplexNoise                    meander_;
    SimplexNoise                    woods_;
};

}  // namespace StarshipSimulator
