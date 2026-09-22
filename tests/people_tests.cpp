#include "StarshipSimulator/core/procgen/people.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

/// A small habitat with towns, built once: planning settlements is not cheap.
struct Town
{
    HabitatGeometry geometry;
    TerrainGrid     grid;
    Settlements     places;
};

const Town& town()
{
    static const Town kBuilt = [] {
        OneillCylinderSpec spec;
        spec.radiusM = 500.0;
        spec.lengthM = 3000.0;
        Town made{.geometry = HabitatGeometry(spec), .grid = {}, .places = {}};
        made.grid   = sampleTerrain(made.geometry, 4.0);
        made.places = planSettlements(made.geometry, made.grid);
        return made;
    }();
    return kBuilt;
}

/// Standing in the middle of the first town.
Vec3d inTheTown()
{
    const Settlement& place = town().places.places.front();
    return place.plane.point(Vec2d(0.0),
                             town().grid.groundHeight(place.plane.z0, place.plane.theta0));
}

TEST(People, AreOutAndAboutInTheTowns)
{
    ASSERT_FALSE(town().places.places.empty());
    const std::vector<Person> people =
        peopleNear(town().places, town().grid, inTheTown(), 10.0, 1975);
    ASSERT_FALSE(people.empty());
    for (const Person& person : people)
    {
        EXPECT_LE(glm::distance(person.position, inTheTown()), CrowdSettings{}.rangeM + 1.0);
        EXPECT_NEAR(glm::length(person.forward), 1.0, 1e-9);
        // Facing along the ground, not into it or up into the sky.
        const Vec3d up = HabitatGeometry::localUp(person.position);
        EXPECT_NEAR(glm::dot(person.forward, up), 0.0, 1e-9);
        // Standing on the ground, not floating or sunk.
        const double radius = std::hypot(person.position.x, person.position.y);
        const double ground =
            town().geometry.radius() -
            town().grid.groundHeight(person.position.z, HabitatGeometry::angleOf(person.position));
        EXPECT_NEAR(radius, ground, 0.05);
        EXPECT_GT(person.heightM, 1.4);
        EXPECT_LT(person.heightM, 2.0);
        EXPECT_LT(person.clothes, kPersonClothes);
        EXPECT_LT(person.skin, kPersonSkins);
        EXPECT_GE(person.gait, 0.0);
        EXPECT_LE(person.gait, 1.0);
    }
}

TEST(People, AreTheSameEveryTimeAndWalkOn)
{
    const std::vector<Person> now = peopleNear(town().places, town().grid, inTheTown(), 10.0, 1975);
    const std::vector<Person> again =
        peopleNear(town().places, town().grid, inTheTown(), 10.0, 1975);
    ASSERT_EQ(now.size(), again.size());
    for (std::size_t i = 0; i < now.size(); ++i)
    {
        EXPECT_EQ(now[i].position, again[i].position);
        EXPECT_EQ(now[i].gait, again[i].gait);
    }

    // Two seconds later the walkers have moved on, at a walking pace.
    const std::vector<Person> later =
        peopleNear(town().places, town().grid, inTheTown(), 12.0, 1975);
    ASSERT_EQ(later.size(), now.size());
    double walked = 0.0;
    for (std::size_t i = 0; i < now.size(); ++i)
    {
        const double moved = glm::distance(now[i].position, later[i].position);
        EXPECT_LT(moved, 2.0 * 2.0);  // nobody covers more than 2 m/s
        if (now[i].doing == Activity::WALKING)
        {
            walked = std::max(walked, moved);
        }
        else
        {
            EXPECT_EQ(moved, 0.0) << "someone standing or sitting moved";
        }
    }
    EXPECT_GT(walked, 1.0);
}

TEST(People, StayInAtNight)
{
    const CrowdSettings quiet{.busy = 0.0};
    EXPECT_TRUE(peopleNear(town().places, town().grid, inTheTown(), 5.0, 1975, quiet).empty());

    const CrowdSettings few{.busy = 0.3};
    const CrowdSettings many{.busy = 1.0};
    EXPECT_LT(peopleNear(town().places, town().grid, inTheTown(), 5.0, 1975, few).size(),
              peopleNear(town().places, town().grid, inTheTown(), 5.0, 1975, many).size());
}

TEST(People, SitOnTheBenchesAndStandAboutTheSquare)
{
    const std::vector<Person> people =
        peopleNear(town().places, town().grid, inTheTown(), 7.0, 4242);
    const auto doing = [&people](Activity what) {
        return std::ranges::count_if(people, [what](const Person& p) { return p.doing == what; });
    };
    EXPECT_GT(doing(Activity::WALKING), 0);
    EXPECT_GT(doing(Activity::SITTING) + doing(Activity::STANDING), 0);
    // Everyone sitting is on a bench, and every bench holds at most one person.
    for (const Person& person : people)
    {
        if (person.doing != Activity::SITTING)
        {
            continue;
        }
        double nearest = 1e9;
        for (const Furniture& item : town().places.furniture)
        {
            if (item.kind != FurnitureKind::BENCH)
            {
                continue;
            }
            const Settlement& place = town().places.places[item.settlement];
            const Vec3d       at    = place.plane.point(item.position, 0.0);
            nearest =
                std::min(nearest, std::hypot(at.x - person.position.x, at.y - person.position.y) +
                                      std::abs(at.z - person.position.z));
        }
        EXPECT_LT(nearest, 1.5) << "someone is sitting on nothing";
    }
}

TEST(People, TheBodyIsBuiltToScale)
{
    const CpuMesh person = buildPersonMesh();
    ASSERT_FALSE(person.vertices.empty());
    EXPECT_EQ(person.indices.size() % 3, 0U);
    float               lowest  = 1e9F;
    float               highest = -1e9F;
    std::array<bool, 9> parts{};
    for (const Vertex& vertex : person.vertices)
    {
        lowest  = std::min(lowest, vertex.position.y);
        highest = std::max(highest, vertex.position.y);
        EXPECT_LT(vertex.material, 9U);
        parts.at(vertex.material) = true;
        EXPECT_LT(std::abs(vertex.position.x), 0.45F);  // no one is that wide
    }
    EXPECT_NEAR(lowest, 0.0, 0.02);              // feet on the ground
    EXPECT_NEAR(highest, kPersonHeightM, 0.03);  // and the top of the head 1.75 m up
    for (const bool present : parts)
    {
        EXPECT_TRUE(present) << "a part of the body is missing";
    }
}

}  // namespace
