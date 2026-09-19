#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/trees.h"

// Towns and farms. Villages of whitewashed houses with tiled roofs line the rivers, around a
// square with a fountain and a hall with a bell tower, like the settlements in the 1970s paintings
// of O'Neill's habitats; farmsteads stand out in the fields. Everything is laid out on the valley
// floor unrolled flat: a cylinder unrolls without stretching, so distances on the plan are true.
namespace StarshipSimulator
{

/// A patch of the valley floor unrolled flat around an origin: x metres around the habitat (in
/// the spin direction, increasing theta), y metres along the axis (+z).
struct FloorPlane
{
    double z0     = 0.0;
    double theta0 = 0.0;
    double radius = 1.0;  // of the floor

    [[nodiscard]] double theta(double x) const { return theta0 + (x / radius); }
    [[nodiscard]] double z(double y) const { return z0 + y; }
    /// The point on the plan at (x, y), `height` above the floor (toward the axis).
    [[nodiscard]] Vec3d point(const Vec2d& p, double height) const;
    /// Where (z, theta) lies on the plan.
    [[nodiscard]] Vec2d toPlan(double z, double theta) const;
};

enum class RoofKind : std::uint8_t
{
    Hip,
    Gable,
    Flat,     // a roof terrace behind a parapet
    Pyramid,  // bell towers
};

enum class BuildingUse : std::uint8_t
{
    House,
    Shop,  // shop windows and an awning on the ground floor
    Hall,  // the town hall on the square
    Tower,
    Farmhouse,
    Barn,
};

/// A building: storeys on a rectangular footprint, and a roof.
struct Building
{
    BuildingUse   use        = BuildingUse::House;
    std::size_t   settlement = 0;
    Vec2d         centre{0.0};          // on the settlement's plan
    double        angle = 0.0;          // radians from the plan's x axis to the footprint's x axis
    Vec2d         halfSize{5.0};        // along the footprint's own x and y axes (m)
    double        floorHeight   = 0.0;  // ground floor level, terrain height (m, toward the axis)
    double        foundation    = 1.0;  // the walls reach this far below the ground floor (m)
    int           storeys       = 1;
    double        storeyHeightM = 3.0;
    RoofKind      roof          = RoofKind::Hip;
    double        roofPitchDeg  = 30.0;
    std::uint8_t  wallColour    = 0;
    std::uint8_t  roofColour    = 0;
    std::uint8_t  shutterColour = 0;  // 0: none
    int           frontSide     = 0;  // the wall with the door: 0 -y, 1 +x, 2 +y, 3 -x; -1 none
    std::uint32_t seed          = 0;

    [[nodiscard]] double wallHeight() const { return storeys * storeyHeightM; }
};

enum class FurnitureKind : std::uint8_t
{
    Lamp,  // a street lamp, lit at night
    Bench,
    Planter,   // a stone trough of flowers
    Fountain,  // the square's fountain
    Stall,     // a market stall with an awning
};

struct Furniture
{
    FurnitureKind kind       = FurnitureKind::Lamp;
    std::size_t   settlement = 0;
    Vec2d         position{0.0};  // on the plan
    double        angle  = 0.0;   // radians, like Building::angle
    double        height = 0.0;   // terrain height under it (m)
};

/// An arched stone bridge over a river, from bank to bank.
struct Bridge
{
    std::size_t settlement = 0;
    Vec2d       from{0.0};  // on the plan: the two ends, on dry land
    Vec2d       to{0.0};
    double      fromHeight = 0.0;  // terrain height at the ends (m)
    double      toHeight   = 0.0;
    double      halfWidth  = 3.0;
    double      rise       = 2.5;  // how far the arch's middle rises above the ends' mean (m)
};

/// A street: a paved strip between two points on the plan.
struct Street
{
    Vec2d  from{0.0};
    Vec2d  to{0.0};
    double halfWidth = 3.0;
};

/// What the ground of a town looks like, 1 m per texel over its plan (RGBA8): R the signed
/// distance to paving (streets, squares, footpaths and the ground under buildings; 128 at the
/// edge, 16 per metre, negative inside), G the paving's style (0 street, 255 stone), B how
/// built-up the ground is (gardens and lawns rather than fields), A the light of street lamps at
/// night.
struct GroundMap
{
    Vec2d                     origin{0.0};  // plan position of texel (0, 0)'s corner
    double                    texelM = 1.0;
    std::uint32_t             width  = 0;
    std::uint32_t             height = 0;
    std::vector<std::uint8_t> texels;

    /// Bilinear sample at a plan position, channels 0..1 (zero outside the map).
    [[nodiscard]] Vec4d sample(const Vec2d& p) const;
    /// Signed distance to paving (m) at a plan position (large outside the map).
    [[nodiscard]] double pavingDistance(const Vec2d& p) const;
};

inline constexpr double kGroundMapDistanceRange = 8.0;  // R spans -4..4 m

enum class SettlementKind : std::uint8_t
{
    Town,
    Farm,
};

struct Settlement
{
    SettlementKind      kind = SettlementKind::Town;
    std::string         name;
    int                 valley = 0;
    FloorPlane          plane;           // centred on the settlement
    double              radiusM = 0.0;   // roughly how far it reaches from its centre
    Vec2d               boundsMin{0.0};  // plan extents of everything built
    Vec2d               boundsMax{0.0};
    std::vector<Street> streets;
    GroundMap           ground;  // towns only
    std::size_t         buildingCount = 0;
};

/// A tree planted by people: along streets and the river front, in gardens.
struct StandingTree
{
    std::size_t settlement = 0;
    Vec3d       position{0.0};  // habitat frame, on the ground
    TreeSpecies species = TreeSpecies::Broadleaf;
    double      heightM = 8.0;
};

struct Settlements
{
    std::vector<Settlement>    places;  // towns first, then farms
    std::vector<Building>      buildings;
    std::vector<Furniture>     furniture;
    std::vector<Bridge>        bridges;
    std::vector<PropPlacement> props;
    std::vector<StandingTree>  trees;

    /// Whether wild trees are kept off (z, theta): towns and farmyards.
    [[nodiscard]] bool keepsTreesOff(double z, double theta) const;
    /// The town whose ground map covers (z, theta), if any.
    [[nodiscard]] const Settlement* townAt(double z, double theta) const;
    [[nodiscard]] std::size_t       townCount() const;
};

/// Plans the habitat's towns and farms on its terrain (heights from the grid, as drawn).
/// Deterministic for a given habitat.
[[nodiscard]] Settlements planSettlements(const HabitatGeometry& geometry, const TerrainGrid& grid);

/// Marks the settlements in the terrain's land-cover map: no painted woods where towns and farms
/// stand, and channel B set over each town's ground map (where the shader looks for streets).
void stampSettlements(TerrainGrid& grid, const Settlements& settlements);

/// Adds the settlements' own trees to a tree layer, one tile per settlement.
void addStandingTrees(TreeLayer& layer, const Settlements& settlements);

}  // namespace StarshipSimulator
