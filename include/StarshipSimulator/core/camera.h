#pragma once

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// A viewpoint in double-precision world space. The camera looks along its local -Z, local +Y is
/// up.
struct Camera
{
    Vec3d  position{0.0};
    Quatd  orientation{1.0, 0.0, 0.0, 0.0};  // camera-to-world rotation
    double verticalFovRadians = degreesToRadians(70.0);
    double nearPlaneMeters    = 0.05;
};

/// Reverse-Z perspective projection with an infinite far plane, for clip-space depth 0..1:
/// points on the near plane map to depth 1, points at infinity to depth 0. Use a GREATER depth test
/// and clear depth to 0. Float depth then has nearly uniform relative precision out to any
/// distance.
[[nodiscard]] Mat4d reverseZInfinitePerspective(double verticalFovRadians, double aspect,
                                                double nearPlaneMeters);

/// World-to-view rotation only. Geometry is rendered camera-relative, so the view has no
/// translation.
[[nodiscard]] Mat4d viewRotation(const Quatd& orientation);

/// Projection times rotation-only view, for camera-relative positions.
[[nodiscard]] Mat4d cameraRelativeViewProjection(const Camera& camera, double aspect);

/// Position relative to the camera, computed in double and only then rounded to float.
[[nodiscard]] Vec3f cameraRelative(const Vec3d& world, const Vec3d& cameraPosition);

/// Mouse-look relative to a local "up" and a reference "north" in the tangent plane.
/// On flat ground up is +Z and north +Y. In a spin habitat up points toward the axis and north is
/// the axis direction; the rig keeps yaw and pitch while the up vector changes as you walk around.
class LookRig
{
public:
    LookRig() = default;
    LookRig(const Vec3d& up, const Vec3d& north);

    /// Changes the local frame, keeping yaw and pitch. north is projected onto the plane normal to
    /// up.
    void setFrame(const Vec3d& up, const Vec3d& north);

    /// Positive yaw turns left (counter-clockwise about up); positive pitch looks up. Radians.
    void applyLook(double yawDelta, double pitchDelta);
    void setAngles(double yawRadians, double pitchRadians);

    [[nodiscard]] double yaw() const { return yaw_; }
    [[nodiscard]] double pitch() const { return pitch_; }

    [[nodiscard]] Vec3d up() const { return up_; }
    [[nodiscard]] Vec3d forward() const;  // view direction
    [[nodiscard]] Vec3d horizontalForward()
        const;  // view direction projected onto the tangent plane
    [[nodiscard]] Vec3d right() const;
    [[nodiscard]] Quatd orientation() const;  // camera-to-world, for Camera::orientation

    static constexpr double kMaxPitch = degreesToRadians(89.5);

private:
    Vec3d  up_{0.0, 0.0, 1.0};
    Vec3d  north_{0.0, 1.0, 0.0};
    double yaw_   = 0.0;
    double pitch_ = 0.0;
};

}  // namespace StarshipSimulator
