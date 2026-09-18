#include "StarshipSimulator/core/fly_controller.h"

#include <cmath>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr Vec3d kUp(0.0, 0.0, 1.0);

/// Limits a direction built from stick inputs to unit length, so diagonals are not faster.
Vec3d clampLength(const Vec3d& v)
{
    const double length = glm::length(v);
    return length > 1.0 ? v / length : v;
}

/// Fraction of the remaining difference closed in one step of an exponential response.
double responseFactor(double dt, double responseTime)
{
    return responseTime > 0.0 ? 1.0 - std::exp(-dt / responseTime) : 1.0;
}

}  // namespace

FlyController::FlyController(const Vec3d& eyePosition) : eye_(eyePosition) { }

void FlyController::teleport(const Vec3d& eyePosition)
{
    eye_      = eyePosition;
    velocity_ = Vec3d(0.0);
    grounded_ = false;
}

void FlyController::setLocomotion(Locomotion locomotion)
{
    locomotion_ = locomotion;
    grounded_   = false;
}

void FlyController::step(const MoveIntent& intent, const LookRig& look, double dt)
{
    if (!(dt > 0.0))
    {
        return;
    }
    if (locomotion_ == Locomotion::Walk)
    {
        stepWalk(intent, look, dt);
    }
    else
    {
        stepFly(intent, look, dt);
    }
}

void FlyController::stepWalk(const MoveIntent& intent, const LookRig& look, double dt)
{
    const double speed = intent.fast ? settings.runSpeed : settings.walkSpeed;
    const Vec3d  heading =
        clampLength((look.horizontalForward() * intent.forward) + (look.right() * intent.right));
    const Vec3d target = heading * speed;

    // Full control on the ground, a little in the air.
    const double control = grounded_ ? 1.0 : 0.1;
    const double blend   = responseFactor(dt, settings.responseTime) * control;
    velocity_.x += (target.x - velocity_.x) * blend;
    velocity_.y += (target.y - velocity_.y) * blend;

    if (grounded_ && intent.jump)
    {
        velocity_.z = settings.jumpSpeed;
        grounded_   = false;
    }
    velocity_.z -= settings.gravity * dt;
    eye_ += velocity_ * dt;

    // Flat ground at z = 0.
    if (eye_.z <= settings.eyeHeight)
    {
        eye_.z      = settings.eyeHeight;
        velocity_.z = 0.0;
        grounded_   = true;
    }
    else
    {
        grounded_ = false;
    }
}

void FlyController::stepFly(const MoveIntent& intent, const LookRig& look, double dt)
{
    const double speed     = settings.flySpeed * (intent.fast ? settings.fastMultiplier : 1.0);
    const Vec3d  direction = clampLength((look.forward() * intent.forward) +
                                         (look.right() * intent.right) + (kUp * intent.up));
    velocity_ += ((direction * speed) - velocity_) * responseFactor(dt, settings.responseTime);
    eye_ += velocity_ * dt;
    grounded_ = false;
}

}  // namespace StarshipSimulator
