#pragma once

#include <cstdint>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/CharacterMover.h"

namespace StarshipSimulator
{

enum class Locomotion : std::uint8_t
{
    WALK,   // physical: spin gravity, Coriolis while airborne, ground contact
    FLY,    // a drone camera for exploring: no gravity, stays above the ground
    WINGS,  // strapped into a pair of wings: real lift and drag, so it only works up near the axis
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
    // Strap-on wings, as people were expected to fly with near the axis of an O'Neill cylinder.
    double wingAreaM2   = 14.0;   // of both wings together
    double wingMassKg   = 82.0;   // you and the wings
    double flapPowerW   = 320.0;  // what a fit person can sustain
    double maxLiftCoeff = 1.5;    // before the wing stalls
    double dragCoeff    = 0.055;  // of the wing and you, at zero lift
};

/// Moves the player through a spinning habitat. Everything is in the rotating habitat frame: while
/// walking the ground carries you; once airborne you are a free body feeling centrifugal "gravity"
/// and (unless in comfort mode) the Coriolis force, so jumps land slightly off to one side.
/// Without a mover the player collides only with the bare terrain (HabitatGeometry); with one
/// (the physics world's character) also with buildings and props.
class PlayerController
{
public:
    /// Uses `mover` (not owned; may be null) for collisions from now on.
    void setMover(CharacterMover* mover) { mover_ = mover; }
    void step(const MoveIntent& intent, const LookRig& look, const HabitatGeometry& geometry,
              double dt);

    /// Stands the player on the ground at (z, theta).
    void placeOnGround(const HabitatGeometry& geometry, double z, double theta);
    /// Moves the eye to a position and stops all motion (no ground check).
    void teleport(const Vec3d& eyePosition);
    /// Sets the player moving through the air (stepping off a platform, or a test launch).
    void launch(const Vec3d& velocity)
    {
        velocity_ = velocity;
        grounded_ = false;
    }
    void setLocomotion(Locomotion locomotion);

    [[nodiscard]] const Vec3d& eyePosition() const { return eye_; }
    [[nodiscard]] const Vec3d& velocity() const { return velocity_; }
    [[nodiscard]] Locomotion   locomotion() const { return locomotion_; }
    [[nodiscard]] bool         grounded() const { return grounded_; }
    /// How the wings are doing: airspeed, the lift they are making over your weight, and whether
    /// they have stalled. Only meaningful in Locomotion::WINGS.
    struct WingState
    {
        double airspeed       = 0.0;  // m/s
        double liftOverWeight = 0.0;
        double climbMS        = 0.0;
        bool   stalled        = false;
    };
    [[nodiscard]] const WingState& wings() const { return wings_; }
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
    void stepWinged(const MoveIntent& intent, const LookRig& look, const HabitatGeometry& geometry,
                    double dt);
    /// With a mover: moves the body from the eye's position at `velocity`, then puts the eye on
    /// top of where it ended up.
    CharacterMove moveBody(const Vec3d& velocity, const HabitatGeometry& geometry, double dt,
                           MoveMode mode);
    /// Keeps the eye inside the habitat (above the ground, inside the ends). Returns true if the
    /// ground was touched.
    bool constrain(const HabitatGeometry& geometry);
    void updateViewUp(double dt);

    Vec3d           eye_{-4000.0, 0.0, 0.0};
    Vec3d           velocity_{0.0};
    Vec3d           viewUp_{1.0, 0.0, 0.0};
    Locomotion      locomotion_ = Locomotion::WALK;
    bool            grounded_   = false;
    WingState       wings_;
    CharacterMover* mover_ = nullptr;
};

}  // namespace StarshipSimulator
