#include "StarshipSimulator/physics/PhysicsWorld.h"

#include <cmath>
#include <cstddef>
#include <memory>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/CharacterMover.h"
#include "StarshipSimulator/core/physics/PlayerController.h"
#include "StarshipSimulator/core/physics/RotatingFrame.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/trees.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

constexpr double kStep   = 1.0 / 120.0;
constexpr double kRadius = 250.0;

/// A small, fast-spinning habitat (about 2 rpm at 1 g) with a flat floor: Coriolis effects are
/// large there. Optionally with one lake per valley.
std::shared_ptr<const HabitatGeometry> smallHabitat(bool lakes = false)
{
    HabitatSpec spec;
    spec.radiusM                 = kRadius;
    spec.lengthM                 = 800.0;
    spec.windowFraction          = 0.4;
    spec.sunwardEndcap           = makeEndcap(EndcapShape::HEMISPHERE);
    spec.antisunwardEndcap       = makeEndcap(EndcapShape::HEMISPHERE);
    spec.partner.enabled         = false;
    spec.terrain.hillHeightM     = 0.0;
    spec.terrain.mountainHeightM = 0.0;
    spec.terrain.riverWidthM     = 0.0;
    spec.terrain.lakesPerValley  = lakes ? 1 : 0;
    spec.terrain.lakeRadiusM     = 30.0;
    return std::make_shared<const HabitatGeometry>(spec);
}

struct TestWorld
{
    explicit TestWorld(bool lakes = false)
      : geometry(smallHabitat(lakes)),
        terrain(std::make_shared<const TerrainGrid>(sampleTerrain(*geometry, 2.0, 2))),
        physics(geometry, terrain, PhysicsSettings{.threads = 0})
    {
    }
    std::shared_ptr<const HabitatGeometry> geometry;
    std::shared_ptr<const TerrainGrid>     terrain;
    PhysicsWorld                           physics;
};

/// A point on the valley floor at theta = pi (up is +x), `height` above it.
Vec3d floorPoint(double height, double z = 0.0)
{
    return {-kRadius + height, 0.0, z};
}

/// Where a prop's centre of mass is: its origin is on the ground under it.
Vec3d ballCentre(const PropState& ball)
{
    return ball.position + (ball.orientation * Vec3d(0.0, 0.12, 0.0));
}

PropPlacement ballAt(const Vec3d& centre)
{
    // The ball's local y along the habitat's up at theta = pi (+x).
    const Quatd upright = floorOrientation(centre);
    return {.kind        = PropKind::BALL,
            .position    = centre - (upright * Vec3d(0.0, 0.12, 0.0)),
            .orientation = upright};
}

TEST(PhysicsWorld, BallComesToRestOnTheFloor)
{
    TestWorld         world;
    const std::size_t ball = world.physics.addProp(ballAt(floorPoint(1.0)), Vec3d(0.0), true);
    // It bounces, and rolls off sideways (the Coriolis force turned its fall) before it stops.
    for (int i = 0; i < 1200; ++i)
    {
        world.physics.step(kStep, floorPoint(1.7));
    }
    const PropState& state = world.physics.props()[ball];
    const Vec3d      c     = ballCentre(state);
    EXPECT_NEAR(std::hypot(c.x, c.y), kRadius - 0.12, 0.01);
    EXPECT_LT(glm::length(state.velocity), 0.05);
    EXPECT_GT(world.physics.terrainTiles(), 0U);
}

TEST(PhysicsWorld, ThrownBallFollowsTheExactFreeFlight)
{
    TestWorld           world;
    const RotatingFrame frame(world.geometry->omega());
    const BodyState     start{.position = floorPoint(1.5), .velocity = Vec3d(4.0, -3.0, 5.0)};
    const std::size_t   ball = world.physics.addProp(ballAt(start.position), start.velocity);
    for (int i = 1; i <= 96; ++i)  // 0.8 s, before it comes down
    {
        world.physics.step(kStep, floorPoint(1.7));
        if (i % 24 == 0)
        {
            const Vec3d expected = propagateFreeFlight(frame, start, i * kStep).position;
            EXPECT_LT(glm::distance(ballCentre(world.physics.props()[ball]), expected), 0.01)
                << "after " << i * kStep << " s";
        }
    }
}

TEST(PhysicsWorld, DroppedBallLandsAntispinward)
{
    TestWorld           world;
    const RotatingFrame frame(world.geometry->omega());
    const BodyState     start{.position = floorPoint(5.0), .velocity = Vec3d(0.0)};
    const auto          impact = predictImpact(frame, *world.geometry, start, 5.0);
    ASSERT_TRUE(impact.has_value());
    const std::size_t ball = world.physics.addProp(ballAt(start.position), Vec3d(0.0), true);
    // Step until it is down to the ground (its centre a radius above the floor).
    Vec3d landed(0.0);
    for (int i = 0; i < 600; ++i)
    {
        world.physics.step(kStep, floorPoint(1.7));
        landed = ballCentre(world.physics.props()[ball]);
        if (std::hypot(landed.x, landed.y) >= kRadius - 0.125)
        {
            break;
        }
    }
    // At theta = pi, against the spin is +y. (The exact impact is for a point, not a ball.)
    const Vec3d exact = impact.value_or(Impact{}).state.position;
    EXPECT_GT(landed.y, 0.3);
    EXPECT_NEAR(landed.y, exact.y, 0.05);
}

TEST(PhysicsWorld, BallFloatsOnALake)
{
    TestWorld world(true);
    ASSERT_FALSE(world.geometry->landscape().lakes().empty());
    const Lake& lake = world.geometry->landscape().lakes().front();
    const Vec3d centre =
        world.geometry->surfacePoint(lake.z, lake.theta) +
        (HabitatGeometry::localUp(world.geometry->surfacePoint(lake.z, lake.theta)) * 3.0);
    ASSERT_GT(world.geometry->waterDepth(lake.z, lake.theta), 0.5);
    const std::size_t ball = world.physics.addProp(ballAt(centre), Vec3d(0.0), true);
    for (int i = 0; i < 720; ++i)
    {
        world.physics.step(kStep, centre);
    }
    // Floating: its centre near the water surface, well above the lake bed.
    const Vec3d  c       = ballCentre(world.physics.props()[ball]);
    const double surface = kRadius - kWaterLevelM;
    EXPECT_NEAR(std::hypot(c.x, c.y), surface, 0.12);
}

TEST(PhysicsWorld, CharacterStandsOnTheFloorAllAround)
{
    TestWorld world;
    for (const double theta : {kPi, kPi / 3.0, 5.0 * kPi / 3.0})
    {
        const Vec3d   feet = world.geometry->surfacePoint(10.0, theta);
        const Vec3d   up   = HabitatGeometry::localUp(feet);
        CharacterMove moved{.feet = feet + (up * 0.05)};
        for (int i = 0; i < 120; ++i)
        {
            moved = world.physics.character().move(moved.feet, Vec3d(0.0), up, 9.8, kStep,
                                                   MoveMode::WALK);
        }
        EXPECT_TRUE(moved.supported) << "theta " << theta;
        EXPECT_NEAR(std::hypot(moved.feet.x, moved.feet.y), kRadius, 0.05) << "theta " << theta;
        EXPECT_NEAR(moved.feet.z, 10.0, 0.01);
    }
}

TEST(PhysicsWorld, WallsStopTheCharacter)
{
    TestWorld       world;
    StaticColliders colliders;
    // A wall across the valley, 5 m ahead (+z), 3 m high and 1 m thick.
    const Vec3d base = floorPoint(0.0, 5.0);
    colliders.boxes.push_back({.centre      = base + Vec3d(1.5, 0.0, 0.5),
                               .orientation = floorOrientation(base),
                               .halfExtents = Vec3d(10.0, 1.5, 0.5)});
    world.physics.addColliders(colliders);
    EXPECT_EQ(world.physics.staticBodies(), 1U);

    const Vec3d   up(1.0, 0.0, 0.0);
    CharacterMove moved{.feet = floorPoint(0.0)};
    for (int i = 0; i < 480; ++i)  // 4 s at 2 m/s: 8 m if nothing were in the way
    {
        moved = world.physics.character().move(moved.feet, Vec3d(0.0, 0.0, 2.0), up, 9.8, kStep,
                                               MoveMode::WALK);
    }
    EXPECT_LT(moved.feet.z, 5.0 - 0.25);
    EXPECT_GT(moved.feet.z, 4.0);
}

TEST(PhysicsWorld, TreeTrunksStopTheCharacter)
{
    TestWorld world;
    // One 15 m oak, 4 m ahead.
    auto        trees = std::make_shared<TreeLayer>();
    const Vec3d root  = floorPoint(0.0, 4.0);
    trees->instances.push_back(
        {.position = Vec3f(0.0F), .packed = packTree(15.0, 0.0, TreeSpecies::BROADLEAF, 0.5)});
    TreeTile tile;
    tile.origin    = root;
    tile.boundsMin = Vec3f(-30.0F);
    tile.boundsMax = Vec3f(30.0F);
    tile.counts    = {1, 0, 0};
    trees->tiles.push_back(tile);
    world.physics.setTrees(trees);
    world.physics.step(kStep, floorPoint(1.7));
    EXPECT_EQ(world.physics.treeTiles(), 1U);

    const Vec3d   up(1.0, 0.0, 0.0);
    CharacterMove moved{.feet = floorPoint(0.0)};
    for (int i = 0; i < 360; ++i)  // 3 s at 2 m/s
    {
        moved = world.physics.character().move(moved.feet, Vec3d(0.0, 0.0, 2.0), up, 9.8, kStep,
                                               MoveMode::WALK);
    }
    // Stopped against the trunk (about 0.4 m thick) instead of walking through.
    EXPECT_LT(moved.feet.z, 4.0 - 0.5);
}

TEST(PhysicsWorld, PlayerWalksAndJumpsWithTheMover)
{
    TestWorld        world;
    PlayerController player;
    player.setMover(&world.physics.character());
    player.placeOnGround(*world.geometry, 0.0, kPi);
    const LookRig look(Vec3d(1.0, 0.0, 0.0), Vec3d(0.0, 0.0, 1.0));  // looking along +z
    for (int i = 0; i < 240; ++i)
    {
        player.step(MoveIntent{.forward = 1.0}, look, *world.geometry, kStep);
        world.physics.step(kStep, player.eyePosition());
    }
    EXPECT_TRUE(player.grounded());
    EXPECT_NEAR(player.eyePosition().z, 2.0 * player.settings.walkSpeed, 0.3);
    EXPECT_NEAR(std::hypot(player.eyePosition().x, player.eyePosition().y),
                kRadius - player.settings.eyeHeight, 0.05);

    // A jump straight up lands ahead, in the spin direction (-y here): the opposite of a drop.
    // (On Earth, where up points away from the axis, it is the other way round.)
    const Vec3d before = player.eyePosition();
    player.step(MoveIntent{.jump = true}, look, *world.geometry, kStep);
    int steps = 1;
    while (!player.grounded() && steps < 360)
    {
        player.step(MoveIntent{}, look, *world.geometry, kStep);
        world.physics.step(kStep, player.eyePosition());
        ++steps;
    }
    EXPECT_TRUE(player.grounded());
    const double w = world.geometry->omega();
    const double v = player.settings.jumpSpeed;
    const double g = world.geometry->gravityAt(kRadius);
    EXPECT_NEAR(before.y - player.eyePosition().y, 4.0 / 3.0 * w * v * v * v / (g * g), 0.02);
    EXPECT_NEAR(steps * kStep, 2.0 * v / g, 0.05);
}

TEST(PhysicsWorld, PushedCrateSlidesAndSettles)
{
    TestWorld         world;
    const Vec3d       spot  = floorPoint(0.0);
    const std::size_t crate = world.physics.addProp(
        {.kind = PropKind::CRATE, .position = spot, .orientation = floorOrientation(spot)});
    EXPECT_FALSE(world.physics.props()[crate].awake);
    world.physics.push(crate, Vec3d(0.0, 0.0, 40.0), spot + Vec3d(0.3, 0.0, 0.0));
    for (int i = 0; i < 600; ++i)
    {
        world.physics.step(kStep, spot);
    }
    const PropState& state = world.physics.props()[crate];
    EXPECT_GT(state.position.z, 0.1);  // it slid along the push
    EXPECT_FALSE(state.awake);         // and went back to sleep
    EXPECT_NEAR(std::hypot(state.position.x, state.position.y), kRadius, 0.02);
}

TEST(PhysicsWorld, RaysHitPropsAndTheGround)
{
    TestWorld         world;
    const Vec3d       spot  = floorPoint(0.0, 3.0);
    const std::size_t crate = world.physics.addProp(
        {.kind = PropKind::CRATE, .position = spot, .orientation = floorOrientation(spot)});
    world.physics.step(kStep, floorPoint(1.7));
    const auto hit = world.physics.raycast(floorPoint(0.3), Vec3d(0.0, 0.0, 1.0), 10.0);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit.value_or(RayHit{}).prop, crate);
    EXPECT_NEAR(hit.value_or(RayHit{}).distance, 2.7, 0.05);
    const auto down = world.physics.raycast(floorPoint(1.7), Vec3d(-1.0, 0.0, 0.0), 10.0);
    ASSERT_TRUE(down.has_value());
    EXPECT_FALSE(down.value_or(RayHit{}).prop.has_value());
    EXPECT_NEAR(down.value_or(RayHit{}).distance, 1.7, 0.01);
}

TEST(PhysicsWorld, TerrainTilesFollowThePlayerAndAreDropped)
{
    TestWorld world;
    // Walk the focus once around the habitat: tiles are built ahead and dropped behind.
    for (int i = 0; i < 3600; ++i)
    {
        const double theta = kPi + (2.0 * kPi * i / 3600.0);
        world.physics.step(
            kStep, world.geometry->surfacePoint(0.0, theta) +
                       (HabitatGeometry::localUp(world.geometry->surfacePoint(0.0, theta)) * 1.7));
    }
    EXPECT_GT(world.physics.terrainTiles(), 4U);
    EXPECT_LT(world.physics.terrainTiles(), 120U);
}

}  // namespace
