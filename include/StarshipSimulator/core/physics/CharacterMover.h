#pragma once

#include <cstdint>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// How a character move treats the ground.
enum class MoveMode : std::uint8_t
{
    WALK,  // follows the ground down small drops and climbs steps
    FREE,  // a plain sweep: falling, jumping, flying
};

/// The outcome of one character move.
struct CharacterMove
{
    Vec3d feet{0.0};          // where the character now stands (the bottom of its body)
    Vec3d velocity{0.0};      // after sliding along whatever it touched
    bool  supported = false;  // standing on ground it can walk on
    bool  blocked   = false;  // something changed the requested motion
};

/// Moves a character's body through the world with collisions. The physics library provides one
/// (a capsule among the terrain, buildings and props), so the player controller does not depend
/// on the physics engine.
class CharacterMover
{
public:
    CharacterMover()                                 = default;
    CharacterMover(const CharacterMover&)            = delete;
    CharacterMover& operator=(const CharacterMover&) = delete;
    CharacterMover(CharacterMover&&)                 = delete;
    CharacterMover& operator=(CharacterMover&&)      = delete;
    virtual ~CharacterMover()                        = default;

    /// Moves the feet at `velocity` for dt seconds. `up` points from the feet to the head (toward
    /// the spin axis); `gravity` (m/s^2) is how hard the character presses on what it stands on.
    [[nodiscard]] virtual CharacterMove move(const Vec3d& feet, const Vec3d& velocity,
                                             const Vec3d& up, double gravity, double dt,
                                             MoveMode mode) = 0;
};

}  // namespace StarshipSimulator
