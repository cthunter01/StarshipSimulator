#include "StarshipSimulator/core/physics/PlayerController.h"

#include <algorithm>
#include <cmath>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/CharacterMover.h"
#include "StarshipSimulator/core/physics/RotatingFrame.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kAxisZoneRadius = 30.0;  // m: inside this, "up" is held steady
constexpr double kUpResponseTime = 0.4;   // s: how fast the view re-aligns after leaving the axis
// With a mover the body stands on the collision terrain, which can differ from the analytic ground
// by a few centimetres: the analytic ground only catches a fall through the world.
constexpr double kFallThroughM = 1.0;

/// Limits a direction built from stick inputs to unit length, so diagonals are not faster.
Vec3d clampLength(const Vec3d& v)
{
    const double length = glm::length(v);
    return length > 1.0 ? v / length : v;
}

double responseFactor(double dt, double responseTime)
{
    return responseTime > 0.0 ? 1.0 - std::exp(-dt / responseTime) : 1.0;
}

/// The point on the radial line through position at distance radius from the axis.
Vec3d atRadius(const Vec3d& position, double radius)
{
    const double r = std::hypot(position.x, position.y);
    if (r < 1e-9)
    {
        return position;
    }
    return {position.x * (radius / r), position.y * (radius / r), position.z};
}

Vec3d removeComponent(const Vec3d& v, const Vec3d& axis)
{
    return v - (glm::dot(v, axis) * axis);
}

}  // namespace

void PlayerController::placeOnGround(const HabitatGeometry& geometry, double z, double theta)
{
    const double clamped = std::clamp(z, geometry.walkableZMin(), geometry.walkableZMax());
    const Vec3d  ground  = geometry.surfacePoint(clamped, theta);
    eye_                 = ground + (HabitatGeometry::localUp(ground) * settings.eyeHeight);
    velocity_            = Vec3d(0.0);
    viewUp_              = HabitatGeometry::localUp(ground);
    grounded_            = locomotion_ == Locomotion::WALK;
}

void PlayerController::teleport(const Vec3d& eyePosition)
{
    eye_      = eyePosition;
    velocity_ = Vec3d(0.0);
    grounded_ = false;
    if (std::hypot(eye_.x, eye_.y) > kAxisZoneRadius)
    {
        viewUp_ = HabitatGeometry::localUp(eye_);
    }
}

void PlayerController::setLocomotion(Locomotion locomotion)
{
    locomotion_ = locomotion;
    grounded_   = false;
}

void PlayerController::step(const MoveIntent& intent, const LookRig& look,
                            const HabitatGeometry& geometry, double dt)
{
    if (!(dt > 0.0))
    {
        return;
    }
    if (locomotion_ == Locomotion::FLY)
    {
        stepFlying(intent, look, geometry, dt);
    }
    else if (locomotion_ == Locomotion::WINGS && !grounded_)
    {
        stepWinged(intent, look, geometry, dt);
    }
    else if (grounded_)
    {
        stepWalking(intent, look, geometry, dt);
    }
    else
    {
        stepAirborne(intent, look, geometry, dt);
    }
    updateViewUp(dt);
}

void PlayerController::stepWalking(const MoveIntent& intent, const LookRig& look,
                                   const HabitatGeometry& geometry, double dt)
{
    const Vec3d  up    = HabitatGeometry::localUp(eye_);
    const double speed = intent.fast ? settings.runSpeed : settings.walkSpeed;
    const Vec3d  heading =
        clampLength((look.horizontalForward() * intent.forward) + (look.right() * intent.right));
    Vec3d walk = removeComponent(velocity_, up);
    walk += ((heading * speed) - walk) * responseFactor(dt, settings.responseTime);

    if (intent.jump)
    {
        velocity_ = walk + (up * settings.jumpSpeed);
        grounded_ = false;
        stepAirborne({}, look, geometry, dt);
        return;
    }
    if (mover_ != nullptr)
    {
        const CharacterMove moved = moveBody(walk, geometry, dt, MoveMode::WALK);
        velocity_                 = removeComponent(moved.velocity, HabitatGeometry::localUp(eye_));
        grounded_                 = moved.supported;  // walked off an edge: falling
        return;
    }

    const GroundSample here     = geometry.ground(eye_);
    Vec3d              proposed = eye_ + (walk * dt);
    proposed.z = std::clamp(proposed.z, geometry.walkableZMin(), geometry.walkableZMax());
    const GroundSample there = geometry.ground(proposed);

    // Too steep to walk up? (Downhill is always allowed.)
    const double rise     = here.groundRadius - there.groundRadius;  // ground moves toward the axis
    const double distance = glm::length(walk) * dt;
    const double maxRise  = std::tan(degreesToRadians(settings.maxSlopeDeg)) * distance;
    const bool   tooSteep =
        rise > maxRise + 1e-9 && there.slopeRadians > degreesToRadians(settings.maxSlopeDeg);
    if (tooSteep)
    {
        velocity_ = Vec3d(0.0);
        return;
    }

    const double eyeRadius = there.groundRadius - settings.eyeHeight;
    if (there.heightAboveGround - settings.eyeHeight > settings.stepDown)
    {
        // Walked off a ledge: keep going as a free body.
        eye_      = proposed;
        velocity_ = walk;
        grounded_ = false;
        return;
    }
    eye_      = atRadius(proposed, eyeRadius);
    velocity_ = removeComponent(walk, HabitatGeometry::localUp(eye_));
}

void PlayerController::stepAirborne(const MoveIntent& intent, const LookRig& look,
                                    const HabitatGeometry& geometry, double dt)
{
    const Vec3d up     = HabitatGeometry::localUp(eye_);
    const Vec3d thrust = clampLength((look.forward() * intent.forward) +
                                     (look.right() * intent.right) + (up * intent.up)) *
                         settings.airThrust;
    BodyState   state{.position = eye_, .velocity = velocity_};
    stepFreeBody(state, RotatingFrame(geometry.omega()), thrust, dt, !settings.comfortMode);
    if (mover_ != nullptr)
    {
        // The exact free flight decides where the body wants to go; the mover what's in the way.
        const CharacterMove moved =
            moveBody((state.position - eye_) / dt, geometry, dt, MoveMode::FREE);
        const Vec3d landingUp = HabitatGeometry::localUp(eye_);
        if (moved.supported && glm::dot(state.velocity, landingUp) <= 0.0)
        {
            grounded_ = true;
            velocity_ = removeComponent(state.velocity, landingUp);
        }
        else
        {
            velocity_ = moved.blocked ? moved.velocity : state.velocity;
        }
        return;
    }
    eye_      = state.position;
    velocity_ = state.velocity;
    if (constrain(geometry))
    {
        grounded_ = true;
        velocity_ = removeComponent(velocity_, HabitatGeometry::localUp(eye_));
    }
}

void PlayerController::stepWinged(const MoveIntent& intent, const LookRig& look,
                                  const HabitatGeometry& geometry, double dt)
{
    // Strap-on wings, in the air that turns with the habitat. Near the floor a person cannot lift
    // their own weight; a few hundred metres up, where the spin gravity has fallen away, they can.
    const HabitatSpec& spec   = geometry.spec();
    const double       radius = std::hypot(eye_.x, eye_.y);
    const double       density =
        airDensityAt(spec.atmosphere, geometry.omega(), geometry.radius(), radius);
    const double weight = settings.wingMassKg * geometry.gravityAt(radius);

    // The air co-rotates, so what the wings feel is simply the velocity in this frame.
    const double airspeed = glm::length(velocity_);
    const Vec3d  up       = HabitatGeometry::localUp(eye_);
    Vec3d        thrust(0.0);
    Vec3d        force(0.0);
    wings_ = WingState{};
    if (airspeed > 0.5)
    {
        const Vec3d ahead = velocity_ / airspeed;
        // The wing is held across the way you are looking: its angle of attack is how far your
        // aim is above the path you are actually flying.
        const Vec3d  aim    = look.forward();
        const Vec3d  lift   = glm::length(glm::cross(ahead, aim)) > 1e-6
                                  ? glm::normalize(glm::cross(glm::cross(ahead, aim), ahead))
                                  : up;
        const double attack = std::asin(std::clamp(glm::dot(aim, lift), -1.0, 1.0));
        // A thin wing: lift climbs with the angle of attack until it stalls, and drag climbs with
        // the square of the lift it is making.
        const double stallAt = degreesToRadians(15.0);
        const bool   stalled = attack > stallAt;
        const double coeff =
            stalled ? settings.maxLiftCoeff * std::max(0.25, 1.0 - ((attack - stallAt) * 2.2))
                    : settings.maxLiftCoeff * (attack / stallAt);
        const double pressure = 0.5 * density * airspeed * airspeed * settings.wingAreaM2;
        const double lifted   = coeff * pressure;
        const double dragged  = (settings.dragCoeff + (0.045 * coeff * coeff)) * pressure;
        force                 = (lift * lifted) - (ahead * dragged);
        wings_.airspeed       = airspeed;
        wings_.liftOverWeight = weight > 1e-6 ? lifted / weight : 1e3;
        wings_.stalled        = stalled;
    }
    // Flapping: a fit person's power, which buys speed rather than a hover.
    if (intent.jump)
    {
        const double push =
            settings.flapPowerW / std::max(2.0, airspeed);  // newtons, from power over speed
        thrust += look.forward() * (push / settings.wingMassKg);
    }
    // Leaning with A and D banks you, which is all the steering there is: the wings do the rest.
    thrust += look.right() * (intent.right * 1.2);
    thrust += force / settings.wingMassKg;

    BodyState state{.position = eye_, .velocity = velocity_};
    stepFreeBody(state, RotatingFrame(geometry.omega()), thrust, dt, !settings.comfortMode);
    wings_.climbMS = glm::dot(state.velocity, up);
    if (mover_ != nullptr)
    {
        const CharacterMove moved =
            moveBody((state.position - eye_) / dt, geometry, dt, MoveMode::FREE);
        const Vec3d landingUp = HabitatGeometry::localUp(eye_);
        if (moved.supported && glm::dot(state.velocity, landingUp) <= 0.0)
        {
            grounded_ = true;
            velocity_ = removeComponent(state.velocity, landingUp);
        }
        else
        {
            velocity_ = moved.blocked ? moved.velocity : state.velocity;
        }
        return;
    }
    eye_      = state.position;
    velocity_ = state.velocity;
    if (constrain(geometry))
    {
        grounded_ = true;
        velocity_ = removeComponent(velocity_, HabitatGeometry::localUp(eye_));
    }
}

void PlayerController::stepFlying(const MoveIntent& intent, const LookRig& look,
                                  const HabitatGeometry& geometry, double dt)
{
    const double speed     = settings.flySpeed * (intent.fast ? settings.fastMultiplier : 1.0);
    const Vec3d  direction = clampLength((look.forward() * intent.forward) +
                                         (look.right() * intent.right) + (look.up() * intent.up));
    velocity_ += ((direction * speed) - velocity_) * responseFactor(dt, settings.responseTime);
    if (mover_ != nullptr)
    {
        velocity_ = moveBody(velocity_, geometry, dt, MoveMode::FREE).velocity;
        return;
    }
    eye_ += velocity_ * dt;
    if (constrain(geometry))
    {
        const Vec3d up = HabitatGeometry::localUp(eye_);
        velocity_ -= std::min(0.0, glm::dot(velocity_, up)) * up;  // no pushing into the ground
    }
}

CharacterMove PlayerController::moveBody(const Vec3d& velocity, const HabitatGeometry& geometry,
                                         double dt, MoveMode mode)
{
    const Vec3d         up      = HabitatGeometry::localUp(eye_);
    const double        gravity = geometry.gravityAt(std::hypot(eye_.x, eye_.y));
    const CharacterMove moved =
        mover_->move(eye_ - (up * settings.eyeHeight), velocity, up, gravity, dt, mode);
    eye_ = moved.feet + (HabitatGeometry::localUp(moved.feet) * settings.eyeHeight);
    constrain(geometry);
    return moved;
}

bool PlayerController::constrain(const HabitatGeometry& geometry)
{
    if (eye_.z < geometry.walkableZMin() || eye_.z > geometry.walkableZMax())
    {
        eye_.z      = std::clamp(eye_.z, geometry.walkableZMin(), geometry.walkableZMax());
        velocity_.z = 0.0;
    }
    const GroundSample ground = geometry.ground(eye_);
    const double       slack  = mover_ != nullptr ? kFallThroughM : 0.0;
    if (ground.heightAboveGround >= settings.eyeHeight - slack)
    {
        return false;
    }
    eye_ = atRadius(eye_, std::max(0.0, ground.groundRadius - settings.eyeHeight));
    return true;
}

void PlayerController::updateViewUp(double dt)
{
    if (std::hypot(eye_.x, eye_.y) < kAxisZoneRadius)
    {
        return;  // floating at the axis: keep the current orientation
    }
    const Vec3d target = HabitatGeometry::localUp(eye_);
    viewUp_ = glm::normalize(glm::mix(viewUp_, target, responseFactor(dt, kUpResponseTime)));
    if (glm::dot(viewUp_, target) > 0.9999)
    {
        viewUp_ = target;
    }
}

}  // namespace StarshipSimulator
