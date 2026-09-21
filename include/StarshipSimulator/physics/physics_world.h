#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <span>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/character_mover.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/people.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/core/procgen/trees.h"

// Rigid bodies in the spinning habitat (Jolt Physics, double precision, behind this interface).
// Everything is simulated in the rotating habitat frame: every moving body feels the centrifugal
// "gravity" w^2 r outward and the Coriolis force -2 w x v, so a rolling ball curves and a thrown
// one lands off to the side, as they would in the real thing. Terrain collision is built from the
// terrain grid in tiles around the player and around anything moving.
namespace StarshipSimulator
{

/// A prop as it is now.
struct PropState
{
    PropKind kind = PropKind::Crate;
    Vec3d    position{0.0};
    Quatd    orientation{1.0, 0.0, 0.0, 0.0};
    Vec3d    velocity{0.0};
    float    tint    = 0.0F;
    bool     awake   = false;  // moving (asleep: resting where it is)
    bool     removed = false;
};

struct RayHit
{
    double                     distance = 0.0;
    Vec3d                      point{0.0};
    Vec3d                      normal{0.0};
    std::optional<std::size_t> prop;  // when it hit a prop: its index
};

struct PhysicsSettings
{
    unsigned threads         = 2;      // worker threads for the solver (0: the calling thread)
    double   terrainRadiusM  = 100.0;  // terrain collision kept this far around the player
    double   characterRadius = 0.3;    // m
    double   characterHeight = 1.8;    // m, feet to the top of the head
    double   characterMassKg = 70.0;
    double   maxSlopeDeg     = 38.0;   // steeper ground is a wall
    double   stepUpM         = 0.4;    // kerbs and stairs the character climbs
    double   stepDownM       = 0.5;    // drops it follows while walking
    double   pushStrengthN   = 250.0;  // how hard it can push props
};

class PhysicsWorld
{
public:
    /// The world starts with the terrain (built on demand) and nothing else.
    PhysicsWorld(std::shared_ptr<const HabitatGeometry> geometry,
                 std::shared_ptr<const TerrainGrid> terrain, const PhysicsSettings& settings = {});
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&)            = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;
    PhysicsWorld(PhysicsWorld&&)                 = delete;
    PhysicsWorld& operator=(PhysicsWorld&&)      = delete;

    /// Adds buildings, bridges and street furniture (they never move).
    void addColliders(const StaticColliders& colliders);
    /// Makes the trees' trunks solid, in tiles near the player (the layer is kept, not copied).
    void setTrees(std::shared_ptr<const TreeLayer> trees);
    /// Makes the people near `focus` solid: a pool of standing capsules kept where they are, so
    /// you bump into them instead of walking through. Call it before each step.
    void setPeople(std::span<const Person> people, const Vec3d& focus, double radiusM = 30.0);
    /// Makes the trams near `focus` solid, so you can stand on one and be carried along.
    void setTrams(std::span<const Tram> trams, const Vec3d& focus, double radiusM = 90.0);

    /// Adds a prop. It sleeps where it is placed until something touches it, unless it is `awake`
    /// or given a velocity. Returns its index in props().
    std::size_t addProp(const PropPlacement& placement, const Vec3d& velocity = Vec3d(0.0),
                        bool awake = false);
    void        removeProp(std::size_t index);
    /// Gives a prop a push: an impulse (N s) at a point on it.
    void push(std::size_t index, const Vec3d& impulse, const Vec3d& at);

    /// Advances dt seconds. `focus` (the player) keeps the terrain around it solid.
    void step(double dt, const Vec3d& focus);

    [[nodiscard]] std::span<const PropState> props() const;
    [[nodiscard]] std::size_t                awakeProps() const;
    [[nodiscard]] std::size_t                terrainTiles() const;
    [[nodiscard]] std::size_t                treeTiles() const;
    [[nodiscard]] std::size_t                staticBodies() const;

    /// The first thing a ray hits (terrain tiles only where they are loaded).
    [[nodiscard]] std::optional<RayHit> raycast(const Vec3d& origin, const Vec3d& direction,
                                                double maxDistance) const;

    /// The player's body: a capsule standing on the ground.
    [[nodiscard]] CharacterMover& character();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace StarshipSimulator
