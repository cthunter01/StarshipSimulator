#pragma once

#include <cstdint>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

enum class Locomotion : std::uint8_t
{
    Walk,  // physical: spin gravity, Coriolis while airborne, ground contact
    Fly,   // a drone camera for exploring: no gravity, stays above the ground
};

/// What the player is asking for this step, independent of the input device.
struct MoveIntent
{
    double forward = 0.0;  // -1..1
    double right   = 0.0;  // -1..1
    double up      = 0.0;  // -1..1, fly mode and zero-g manoeuvring
    bool   fast    = false;
    bool   jump    = false;
};

struct PlayerSettings
{
    double walkSpeed      = 1.4;    // m/s
    double runSpeed       = 5.0;    // m/s
    double flySpeed       = 60.0;   // m/s, adjustable with the mouse wheel
    double fastMultiplier = 10.0;   // fly speed multiplier while "fast" is held
    double eyeHeight      = 1.7;    // m
    double jumpSpeed      = 3.5;    // m/s
    double maxSlopeDeg    = 38.0;   // steeper ground cannot be walked up
    double stepDown       = 0.8;    // m of drop followed while walking before becoming airborne
    double airThrust      = 0.6;    // m/s^2 of "swimming" control while airborne
    double responseTime   = 0.12;   // s, how quickly walking/flying velocity follows the input
    bool   comfortMode    = false;  // no Coriolis on the player (thrown objects keep it)
};

/// Moves the player through a spinning habitat. Everything is in the rotating habitat frame: while
/// walking the ground carries you; once airborne you are a free body feeling centrifugal "gravity"
/// and (unless in comfort mode) the Coriolis force, so jumps land slightly off to one side.
class PlayerController
{
public:
    void step(const MoveIntent& intent, const LookRig& look, const HabitatGeometry& geometry,
              double dt);

    /// Stands the player on the ground at (z, theta).
    void placeOnGround(const HabitatGeometry& geometry, double z, double theta);
    /// Moves the eye to a position and stops all motion (no ground check).
    void teleport(const Vec3d& eyePosition);
    void setLocomotion(Locomotion locomotion);

    [[nodiscard]] const Vec3d& eyePosition() const { return eye_; }
    [[nodiscard]] const Vec3d& velocity() const { return velocity_; }
    [[nodiscard]] Locomotion   locomotion() const { return locomotion_; }
    [[nodiscard]] bool         grounded() const { return grounded_; }
    /// The "up" the camera should use: local up, held steady while floating near the axis.
    [[nodiscard]] const Vec3d& viewUp() const { return viewUp_; }

    PlayerSettings settings;

private:
    void stepWalking(const MoveIntent& intent, const LookRig& look, const HabitatGeometry& geometry,
                     double dt);
    void stepAirborne(const MoveIntent& intent, const LookRig& look,
                      const HabitatGeometry& geometry, double dt);
    void stepFlying(const MoveIntent& intent, const LookRig& look, const HabitatGeometry& geometry,
                    double dt);
    /// Keeps the eye inside the habitat (above the ground, inside the ends). Returns true if the
    /// ground was touched.
    bool constrain(const HabitatGeometry& geometry);
    void updateViewUp(double dt);

    Vec3d      eye_{-4000.0, 0.0, 0.0};
    Vec3d      velocity_{0.0};
    Vec3d      viewUp_{1.0, 0.0, 0.0};
    Locomotion locomotion_ = Locomotion::Walk;
    bool       grounded_   = false;
};

}  // namespace StarshipSimulator
