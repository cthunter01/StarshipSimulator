#pragma once

#include <cstdint>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/units.h"

namespace StarshipSimulator
{

enum class Locomotion : std::uint8_t
{
    Walk,
    Fly,
};

/// What the player is asking for this step, independent of the input device.
struct MoveIntent
{
    double forward = 0.0;  // -1..1
    double right   = 0.0;  // -1..1
    double up      = 0.0;  // -1..1, fly mode only
    bool   fast    = false;
    bool   jump    = false;
};

struct FlySettings
{
    double walkSpeed      = 1.4;   // m/s
    double runSpeed       = 5.0;   // m/s
    double flySpeed       = 25.0;  // m/s, adjustable with the mouse wheel
    double fastMultiplier = 10.0;  // fly speed multiplier while "fast" is held
    double eyeHeight      = 1.7;   // m above the ground
    double gravity        = units::kStandardGravity;
    double jumpSpeed      = 4.0;   // m/s
    double responseTime   = 0.12;  // s, how quickly velocity follows the input
};

/// M0 test controller over a flat ground plane at z = 0 (up is +Z): walk with gravity or fly
/// freely. M1 replaces it with the rotating-frame player controller for spin habitats.
class FlyController
{
public:
    explicit FlyController(const Vec3d& eyePosition = Vec3d(0.0, 0.0, 1.7));

    void step(const MoveIntent& intent, const LookRig& look, double dt);

    /// Moves the eye to a new position and stops all motion.
    void teleport(const Vec3d& eyePosition);
    void setLocomotion(Locomotion locomotion);

    [[nodiscard]] const Vec3d& eyePosition() const { return eye_; }
    [[nodiscard]] const Vec3d& velocity() const { return velocity_; }
    [[nodiscard]] Locomotion   locomotion() const { return locomotion_; }
    [[nodiscard]] bool         grounded() const { return grounded_; }

    FlySettings settings;

private:
    void stepWalk(const MoveIntent& intent, const LookRig& look, double dt);
    void stepFly(const MoveIntent& intent, const LookRig& look, double dt);

    Vec3d      eye_;
    Vec3d      velocity_{0.0};
    Locomotion locomotion_ = Locomotion::Walk;
    bool       grounded_   = false;
};

}  // namespace StarshipSimulator
