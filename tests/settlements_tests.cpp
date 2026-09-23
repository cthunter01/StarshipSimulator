#include "StarshipSimulator/core/procgen/settlements.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/trees.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

const HabitatGeometry& islandThree()
{
    static const HabitatGeometry kGeometry{HabitatSpec{}};
    return kGeometry;
}

/// Island Three's terrain on a coarse grid (quick to sample), and its settlements.
struct Planned
{
    TerrainGrid grid        = sampleTerrain(islandThree(), 8.0);
    Settlements settlements = planSettlements(islandThree(), grid);
};

const Planned& planned()
{
    static const Planned kPlan;
    return kPlan;
}

std::array<Vec2d, 4> footprint(const Building& b)
{
    const Vec2d x = Vec2d(std::cos(b.angle), std::sin(b.angle)) * b.halfSize.x;
    const Vec2d y = Vec2d(-std::sin(b.angle), std::cos(b.angle)) * b.halfSize.y;
    return {b.centre - x - y, b.centre + x - y, b.centre + x + y, b.centre - x + y};
}

/// Whether two footprints overlap (separating axes), allowing `slack` metres of contact.
bool overlap(const Building& a, const Building& b, double slack)
{
    const auto ca = footprint(a);
    const auto cb = footprint(b);
    for (const double angle : {a.angle, a.angle + (0.5 * kPi), b.angle, b.angle + (0.5 * kPi)})
    {
        const Vec2d axis(std::cos(angle), std::sin(angle));
        double      aLow  = 1e30;
        double      aHigh = -1e30;
        double      bLow  = 1e30;
        double      bHigh = -1e30;
        for (const Vec2d& p : ca)
        {
            aLow  = std::min(aLow, glm::dot(p, axis));
            aHigh = std::max(aHigh, glm::dot(p, axis));
        }
        for (const Vec2d& p : cb)
        {
            bLow  = std::min(bLow, glm::dot(p, axis));
            bHigh = std::max(bHigh, glm::dot(p, axis));
        }
        if (aHigh <= bLow + slack || bHigh <= aLow + slack)
        {
            return false;
        }
    }
    return true;
}

double segmentDistance(const Vec2d& p, const Vec2d& a, const Vec2d& b)
{
    const Vec2d  ab = b - a;
    const double t  = std::clamp(glm::dot(p - a, ab) / glm::dot(ab, ab), 0.0, 1.0);
    return glm::length(p - (a + (ab * t)));
}

TEST(Settlements, EveryValleyHasTownsAndFarms)
{
    const Settlements& s = planned().settlements;
    for (int valley = 0; valley < islandThree().stripCount(); ++valley)
    {
        const auto towns = std::ranges::count_if(s.places, [&](const Settlement& p) {
            return p.valley == valley && p.kind == SettlementKind::TOWN;
        });
        const auto farms = std::ranges::count_if(s.places, [&](const Settlement& p) {
            return p.valley == valley && p.kind == SettlementKind::FARM;
        });
        EXPECT_GE(towns, 3) << "valley " << valley;
        EXPECT_GE(farms, 8) << "valley " << valley;
    }
    for (const Settlement& place : s.places)
    {
        if (place.kind == SettlementKind::TOWN)
        {
            EXPECT_GE(place.buildingCount, 40U) << place.name;
            EXPECT_FALSE(place.name.empty());
            EXPECT_GT(place.ground.width, 100U);
        }
    }
    EXPECT_FALSE(s.furniture.empty());
    EXPECT_FALSE(s.props.empty());
    EXPECT_FALSE(s.trees.empty());
    EXPECT_FALSE(s.bridges.empty());
}

TEST(Settlements, PlanningIsDeterministic)
{
    const Settlements  again = planSettlements(islandThree(), planned().grid);
    const Settlements& s     = planned().settlements;
    ASSERT_EQ(again.buildings.size(), s.buildings.size());
    ASSERT_EQ(again.props.size(), s.props.size());
    for (std::size_t i = 0; i < s.buildings.size(); i += 17)
    {
        EXPECT_EQ(again.buildings[i].centre, s.buildings[i].centre);
        EXPECT_EQ(again.buildings[i].seed, s.buildings[i].seed);
    }
}

TEST(Settlements, BuildingsStandOnDryValleyFloor)
{
    const HabitatGeometry& geometry = islandThree();
    const Settlements&     s        = planned().settlements;
    for (const Building& b : s.buildings)
    {
        const Settlement& place = s.places[b.settlement];
        for (const Vec2d& corner : footprint(b))
        {
            const double z     = place.plane.z(corner.y);
            const double theta = place.plane.theta(corner.x);
            EXPECT_GT(geometry.landscape().shoreDistance(z, theta), 4.0);
            const Region region = geometry.regionAt(z, theta);
            EXPECT_EQ(region.kind, RegionKind::LAND);
            EXPECT_EQ(region.index, place.valley);
            // The walls reach below the ground everywhere under them.
            EXPECT_LT(b.floorHeight - b.foundation, planned().grid.groundHeight(z, theta) - 0.5);
            EXPECT_GE(b.floorHeight, planned().grid.groundHeight(z, theta));
        }
        EXPECT_GE(b.storeys, 1);
        EXPECT_LE(b.storeys, 7);
    }
}

TEST(Settlements, BuildingsDoNotOverlapOrStandInTheStreet)
{
    const Settlements& s = planned().settlements;
    for (std::size_t i = 0; i < s.buildings.size(); ++i)
    {
        const Building&   a     = s.buildings[i];
        const Settlement& place = s.places[a.settlement];
        for (std::size_t j = i + 1; j < s.buildings.size(); ++j)
        {
            const Building& b = s.buildings[j];
            if (b.settlement == a.settlement && glm::distance(a.centre, b.centre) < 60.0)
            {
                // Towers stand against their hall; row houses share walls.
                const bool attached = a.use == BuildingUse::TOWER || b.use == BuildingUse::TOWER;
                EXPECT_FALSE(overlap(a, b, attached ? 1.0 : 0.05))
                    << place.name << ": buildings " << i << " and " << j;
            }
        }
        for (const Street& street : place.streets)
        {
            for (const Vec2d& corner : footprint(a))
            {
                EXPECT_GT(segmentDistance(corner, street.from, street.to), street.halfWidth - 0.05)
                    << place.name << ": building " << i << " at " << a.centre.x << ", "
                    << a.centre.y;
            }
            EXPECT_GT(segmentDistance(a.centre, street.from, street.to), street.halfWidth)
                << place.name << ": building " << i;
        }
    }
}

TEST(Settlements, GroundMapsPaveTheStreets)
{
    const Settlements& s = planned().settlements;
    for (const Settlement& town : s.places)
    {
        if (town.kind != SettlementKind::TOWN)
        {
            continue;
        }
        for (const Street& street : town.streets)
        {
            const Vec2d middle = 0.5 * (street.from + street.to);
            EXPECT_LT(town.ground.pavingDistance(middle), -0.5) << town.name;
            // The town's ground around its streets is built up: lawns, not fields.
            EXPECT_GT(town.ground.sample(middle).z, 0.3) << town.name;
        }
        // Lamps light the ground at night.
        const auto lamp = std::ranges::find_if(s.furniture, [&](const Furniture& f) {
            return f.kind == FurnitureKind::LAMP && &s.places[f.settlement] == &town;
        });
        ASSERT_NE(lamp, s.furniture.end());
        EXPECT_GT(town.ground.sample(lamp->position).w, 0.5);
        // Nothing paved far outside.
        EXPECT_GT(town.ground.pavingDistance(Vec2d(5000.0)), 3.9);
    }
}

TEST(Settlements, WildTreesKeepOutOfTownsAndFarmyards)
{
    const Settlements& s = planned().settlements;
    for (const Settlement& place : s.places)
    {
        EXPECT_TRUE(s.keepsTreesOff(place.plane.z0, place.plane.theta0)) << place.name;
        // A kilometre away along the valley is open country (or another settlement's edge).
        const Vec2d away(0.0, place.kind == SettlementKind::TOWN ? 1500.0 : 300.0);
        if (s.townAt(place.plane.z(away.y), place.plane.theta(away.x)) == nullptr)
        {
            EXPECT_FALSE(s.keepsTreesOff(place.plane.z(away.y), place.plane.theta(away.x)))
                << place.name;
        }
    }
    const Settlement& first = s.places.front();
    EXPECT_EQ(s.townAt(first.plane.z0, first.plane.theta0), &first);
}

TEST(Settlements, TownTreesJoinTheTreeLayer)
{
    TreeLayer layer;
    addStandingTrees(layer, planned().settlements);
    std::size_t count = 0;
    for (const TreeTile& tile : layer.tiles)
    {
        for (const std::uint32_t n : tile.counts)
        {
            count += n;
        }
    }
    EXPECT_EQ(count, planned().settlements.trees.size());
    EXPECT_EQ(layer.instances.size(), count);
}

TEST(Settlements, StampingClearsTheCanopyAndMarksTowns)
{
    TerrainGrid        grid = planned().grid;
    const Settlements& s    = planned().settlements;
    stampSettlements(grid, s);
    for (const Settlement& place : s.places)
    {
        const Vec2d cell   = grid.cellAt(place.plane.z0, place.plane.theta0) / 2.0;
        const auto  column = static_cast<std::size_t>(std::lround(cell.x)) % grid.coverColumns;
        const auto  row    = static_cast<std::size_t>(std::lround(cell.y));
        EXPECT_EQ(grid.cover[((row * grid.coverColumns) + column) * 4], 0) << place.name;
        EXPECT_EQ(grid.cover[(((row * grid.coverColumns) + column) * 4) + 2],
                  place.kind == SettlementKind::TOWN ? 255 : 0)
            << place.name;
    }
}

}  // namespace
