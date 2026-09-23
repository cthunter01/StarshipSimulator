#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "StarshipSimulator/core/units.h"

namespace StarshipSimulator
{

/// The kinds of spinning habitat the simulator can build. They share one spec (HabitatSpec); the
/// fields a kind does not use are ignored and not written to its file.
enum class HabitatKind : std::uint8_t
{
    ONEILL_CYLINDER,   // land strips along the axis alternating with window strips (Island Three)
    KALPANA_CYLINDER,  // a short cylinder without windows, lit through its glass end caps
    STANFORD_TORUS,    // a tube around a wheel, with windows on the tube's hub-facing side
    BERNAL_SPHERE,     // a sphere with a band of land around its equator and windows at the poles
    BISHOP_RING,       // an open-topped ring whose walls hold the air in (M10)
};

/// Where the daylight comes in, which follows from the kind of habitat.
enum class DaylightKind : std::uint8_t
{
    MIRROR_STRIPS,    // a hinged mirror outside each window strip (O'Neill cylinder)
    END_CAPS,         // mirrors outside the transparent end caps (Kalpana One)
    POLAR_WINDOWS,    // mirrors outside the windows at the poles (Bernal sphere)
    OVERHEAD_MIRROR,  // a mirror over the hub, louvres over the ceiling windows (Stanford torus)
    OPEN_SKY,         // the Sun itself (open rings)
};

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

/// The mirrors that bring the sunlight in. Their opening angle sets how much light gets in: on an
/// O'Neill cylinder it is the hinged mirrors' angle to the hull (45 = noon, 90 = sunset); the other
/// kinds turn their mirrors or louvres the same way, so one angle runs the day in every habitat.
struct MirrorSpec
{
    double openingAngleDeg = 60.0;
    double reflectivity    = 0.9;
};

/// Island Three comes as a counter-rotating pair: their spins cancel, so the pair can be turned to
/// keep facing the Sun without fighting gyroscopic torque. The partner flies alongside, its axis
/// parallel to ours (both point at the Sun), offset along the ecliptic. O'Neill cylinders only.
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
    double        hillHeightM     = 40.0;   // rolling hills on the floor
    double        mountainHeightM = 120.0;  // extra relief off the floor (endcaps, slopes)
    double        featureSizeM    = 800.0;  // typical hill spacing
    double        riverWidthM     = 40.0;   // a meandering river along each band; 0 for none
    int           lakesPerValley  = 2;      // shallow lakes on the river's course, per band
    double        lakeRadiusM     = 300.0;  // typical lake half-width across the band
    double        forestCover     = 0.35;   // share of the floor under woods
};

/// Towns and farms: villages of houses along the rivers, farmsteads out in the fields.
struct SettlementSpec
{
    int    townsPerValley = 4;      // per band of land
    double townRadiusM    = 260.0;  // typical half-length of a town along its main street
    int    farmsPerValley = 14;     // per band of land
};

/// A Stanford torus: a tube (minor radius a) bent into a wheel. The floor is the outer half of the
/// tube, HabitatSpec::radiusM from the axis at its lowest point; the ceiling is the half facing the
/// hub, with the windows.
struct TorusSpec
{
    double tubeRadiusM        = 65.0;  // the 130 m tube
    double hubRadiusM         = 65.0;  // the 130 m hub
    int    spokes             = 6;
    double spokeRadiusM       = 7.5;    // 15 m spokes
    double landHalfAngleDeg   = 30.0;   // the floor is land where the tube slopes less than this
    double ceilingWindowShare = 0.333;  // share of the tube's circumference that is window
    int    sections           = 6;      // alternating farmland and towns around the ring (0: mixed)
    double shieldM            = 1.7;    // the non-rotating regolith shield (almanac only)
};

/// A Bernal sphere: land in a band around the equator, windows at the poles.
struct SphereSpec
{
    double landLatitudeDeg   = 35.0;  // the band of land reaches this far north and south
    double windowLatitudeDeg = 55.0;  // beyond this the caps around the poles are glass
};

/// An open-topped ring (M10): no roof, the air held down by spin gravity between two walls. With a
/// ring's width in HabitatSpec::lengthM.
struct RingSpec
{
    double wallHeightM = 200000.0;
    double sunTiltDeg  = 0.0;  // a Banks orbital leans on its star so that the spin is the day
};

/// One spinning habitat, whatever its kind. Coordinates: spin axis +Z, the sun toward +Z, the
/// middle of the habitat at z = 0; "up" is toward the axis everywhere.
struct HabitatSpec
{
    HabitatKind    kind            = HabitatKind::ONEILL_CYLINDER;
    double         radiusM         = 4000.0;   // spin axis to the floor (a torus: its lowest point)
    double         lengthM         = 32000.0;  // cylinders: the length; a ring: its width
    double         surfaceGravityG = 1.0;      // spin gravity on the floor, in standard g
    int            stripPairs      = 3;    // O'Neill: land + window pairs around the circumference
    double         windowFraction  = 0.5;  // O'Neill: share of the circumference that is window
    EndcapSpec     sunwardEndcap   = makeEndcap(EndcapShape::HEMISPHERE);  // O'Neill
    EndcapSpec     antisunwardEndcap = makeEndcap(EndcapShape::CONICAL_RAMP);
    TorusSpec      torus;
    SphereSpec     sphere;
    RingSpec       ring;
    MirrorSpec     mirrors;
    PartnerSpec    partner;  // O'Neill
    AtmosphereSpec atmosphere;
    TerrainSpec    terrain;
    SettlementSpec settlements;
    double         populationDensityPerKm2 = 5000.0;  // of land
};

/// Problems that make a spec unbuildable, as human-readable messages (empty when valid).
[[nodiscard]] std::vector<std::string> validate(const HabitatSpec& spec);

/// Whether the simulator can build a world of this kind yet (every kind can be read and written).
[[nodiscard]] bool habitatKindBuilt(HabitatKind kind);

/// Smallest axis-to-axis distance at which two such cylinders' mirrors cannot touch, however they
/// are turned: each mirror reaches out at most a cylinder length beyond the hull.
[[nodiscard]] double minimumPartnerSeparation(const HabitatSpec& spec);

/// Axial depth of an endcap inside the cylinder section (0 for flat and hemisphere endcaps).
[[nodiscard]] double endcapDepthInside(const EndcapSpec& endcap, double radiusM);

[[nodiscard]] const char* endcapShapeName(EndcapShape shape);

[[nodiscard]] const char*                habitatKindKey(HabitatKind kind);   // "stanford_torus"
[[nodiscard]] const char*                habitatKindName(HabitatKind kind);  // "Stanford torus"
[[nodiscard]] std::optional<HabitatKind> habitatKindFromKey(std::string_view key);
[[nodiscard]] std::vector<HabitatKind>   allHabitatKinds();
[[nodiscard]] DaylightKind               daylightKindFor(HabitatKind kind);

/// How many bands of land the habitat has (the O'Neill cylinder's valleys; one elsewhere).
[[nodiscard]] int bandCount(const HabitatSpec& spec);

/// How far above the floor the air goes before it meets the far side, the ceiling or the top of the
/// walls: room for clouds, birds and flight.
[[nodiscard]] double headroomM(const HabitatSpec& spec);

}  // namespace StarshipSimulator
