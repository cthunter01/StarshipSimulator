#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "StarshipSimulator/core/units.h"

namespace StarshipSimulator
{

enum class EndcapShape : std::uint8_t
{
    FLAT,          // a flat end wall (vertical under spin gravity)
    HEMISPHERE,    // a dome beyond the end of the cylinder: a bowl that curves up overhead
    CONICAL_RAMP,  // mountains built inside the end: walkable slopes up toward the axis
};

struct EndcapSpec
{
    EndcapShape shape                 = EndcapShape::FLAT;
    double      rampSlopeDeg          = 25.0;  // conical ramp: slope of the lower part
    double      rampTopRadiusFraction = 0.5;   // radius (fraction of R) where the lower part ends
    double      upperSlopeDeg         = 35.0;  // slope from there up to the hub
    double      hubRadiusM            = 60.0;  // open hub around the axis
};

/// An endcap of the given shape with default ramp parameters.
[[nodiscard]] constexpr EndcapSpec makeEndcap(EndcapShape shape)
{
    EndcapSpec endcap;
    endcap.shape = shape;
    return endcap;
}

/// External mirrors hinged at the anti-sunward end of each window, reflecting sunlight inside.
struct MirrorSpec
{
    double openingAngleDeg = 60.0;  // angle between mirror and hull: 45 = noon, 90 = sunset
    double reflectivity    = 0.9;
};

/// Island Three comes as a counter-rotating pair: their spins cancel, so the pair can be turned to
/// keep facing the Sun without fighting gyroscopic torque. The partner flies alongside, its axis
/// parallel to ours (both point at the Sun), offset along the ecliptic.
struct PartnerSpec
{
    bool   enabled     = true;
    double separationM = 80000.0;  // axis to axis; the mirrors of both must clear each other
};

struct AtmosphereSpec
{
    double surfacePressurePa = units::kStandardAtmosphere;
    double temperatureK      = 293.0;
};

struct TerrainSpec
{
    std::uint64_t seed            = 1975;
    double        hillHeightM     = 40.0;   // rolling hills on the valley floors
    double        mountainHeightM = 120.0;  // extra relief on the endcaps
    double        featureSizeM    = 800.0;  // typical hill spacing
    double        riverWidthM     = 40.0;   // a meandering river along each valley; 0 for none
    int           lakesPerValley  = 2;      // shallow lakes on the river's course
    double        lakeRadiusM     = 300.0;  // typical lake half-width across the valley
    double        forestCover     = 0.35;   // share of the valley floors under woods
};

/// Towns and farms: villages of houses along the rivers, farmsteads out in the fields.
struct SettlementSpec
{
    int    townsPerValley = 4;
    double townRadiusM    = 260.0;  // typical half-length of a town along its main street
    int    farmsPerValley = 14;
};

/// An O'Neill cylinder: a spinning cylinder with land strips alternating with window strips.
/// Coordinates: spin axis +Z, the sun toward +Z, the cylinder section from z = -L/2 to +L/2.
struct OneillCylinderSpec
{
    double         radiusM           = 4000.0;
    double         lengthM           = 32000.0;
    double         surfaceGravityG   = 1.0;  // spin gravity on the floor, in standard g
    int            stripPairs        = 3;    // land + window pairs around the circumference
    double         windowFraction    = 0.5;  // share of the circumference that is window
    EndcapSpec     sunwardEndcap     = makeEndcap(EndcapShape::HEMISPHERE);
    EndcapSpec     antisunwardEndcap = makeEndcap(EndcapShape::CONICAL_RAMP);
    MirrorSpec     mirrors;
    PartnerSpec    partner;
    AtmosphereSpec atmosphere;
    TerrainSpec    terrain;
    SettlementSpec settlements;
    double         populationDensityPerKm2 = 5000.0;  // of land
};

/// Problems that make a spec unbuildable, as human-readable messages (empty when valid).
[[nodiscard]] std::vector<std::string> validate(const OneillCylinderSpec& spec);

/// Smallest axis-to-axis distance at which two such cylinders' mirrors cannot touch, however they
/// are turned: each mirror reaches out at most a cylinder length beyond the hull.
[[nodiscard]] double minimumPartnerSeparation(const OneillCylinderSpec& spec);

/// Axial depth of an endcap inside the cylinder section (0 for flat and hemisphere endcaps).
[[nodiscard]] double endcapDepthInside(const EndcapSpec& endcap, double radiusM);

[[nodiscard]] const char* endcapShapeName(EndcapShape shape);

}  // namespace StarshipSimulator
