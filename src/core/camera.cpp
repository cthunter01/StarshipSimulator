#include "StarshipSimulator/core/camera.h"

#include <algorithm>
#include <cmath>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

Mat4d reverseZInfinitePerspective(double verticalFovRadians, double aspect, double nearPlaneMeters)
{
    const double focal = 1.0 / std::tan(verticalFovRadians / 2.0);
    Mat4d        m(0.0);
    m[0][0] = focal / aspect;
    m[1][1] = focal;
    m[2][3] = -1.0;             // clip.w = -z_view
    m[3][2] = nearPlaneMeters;  // clip.z = near, so depth = near / -z_view
    return m;
}

Mat4d viewRotation(const Quatd& orientation)
{
    return glm::mat4_cast(glm::conjugate(orientation));
}

Mat4d cameraRelativeViewProjection(const Camera& camera, double aspect)
{
    return reverseZInfinitePerspective(camera.verticalFovRadians, aspect, camera.nearPlaneMeters) *
           viewRotation(camera.orientation);
}

Vec3f cameraRelative(const Vec3d& world, const Vec3d& cameraPosition)
{
    return Vec3f(world - cameraPosition);
}

LookRig::LookRig(const Vec3d& up, const Vec3d& north)
{
    setFrame(up, north);
}

void LookRig::setFrame(const Vec3d& up, const Vec3d& north)
{
    up_                = glm::normalize(up);
    Vec3d tangentNorth = north - (glm::dot(north, up_) * up_);
    if (glm::dot(tangentNorth, tangentNorth) < 1e-18)
    {
        // north is parallel to up: pick any direction in the tangent plane.
        const Vec3d helper = std::abs(up_.x) < 0.9 ? Vec3d(1.0, 0.0, 0.0) : Vec3d(0.0, 1.0, 0.0);
        tangentNorth       = helper - (glm::dot(helper, up_) * up_);
    }
    north_ = glm::normalize(tangentNorth);
}

void LookRig::applyLook(double yawDelta, double pitchDelta)
{
    setAngles(yaw_ + yawDelta, pitch_ + pitchDelta);
}

void LookRig::setAngles(double yawRadians, double pitchRadians)
{
    yaw_   = std::remainder(yawRadians, 2.0 * kPi);
    pitch_ = std::clamp(pitchRadians, -kMaxPitch, kMaxPitch);
}

Vec3d LookRig::horizontalForward() const
{
    const Vec3d east = glm::cross(north_, up_);
    return (north_ * std::cos(yaw_)) - (east * std::sin(yaw_));
}

Vec3d LookRig::forward() const
{
    return (horizontalForward() * std::cos(pitch_)) + (up_ * std::sin(pitch_));
}

Vec3d LookRig::right() const
{
    return glm::normalize(glm::cross(horizontalForward(), up_));
}

Quatd LookRig::orientation() const
{
    const Vec3d viewForward = forward();
    const Vec3d viewRight   = right();
    const Vec3d viewUp      = glm::cross(viewRight, viewForward);
    return glm::normalize(glm::quat_cast(Mat3d(viewRight, viewUp, -viewForward)));
}

}  // namespace StarshipSimulator
