#include "StarshipSimulator/core/physics/player_controller.h"

#include <algorithm>
#include <cmath>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/rotating_frame.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kAxisZoneRadius = 30.0;  // m: inside this, "up" is held steady
constexpr double kUpResponseTime = 0.4;   // s: how fast the view re-aligns after leaving the axis

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
    grounded_            = locomotion_ == Locomotion::Walk;
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
    if (locomotion_ == Locomotion::Fly)
    {
        stepFlying(intent, look, geometry, dt);
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
    eye_ += velocity_ * dt;
    if (constrain(geometry))
    {
        const Vec3d up = HabitatGeometry::localUp(eye_);
        velocity_ -= std::min(0.0, glm::dot(velocity_, up)) * up;  // no pushing into the ground
    }
}

bool PlayerController::constrain(const HabitatGeometry& geometry)
{
    if (eye_.z < geometry.walkableZMin() || eye_.z > geometry.walkableZMax())
    {
        eye_.z      = std::clamp(eye_.z, geometry.walkableZMin(), geometry.walkableZMax());
        velocity_.z = 0.0;
    }
    const GroundSample ground = geometry.ground(eye_);
    if (ground.heightAboveGround >= settings.eyeHeight)
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
