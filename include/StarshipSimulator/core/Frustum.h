#pragma once

#include <array>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// View frustum planes for culling, from a (camera-relative, reverse-Z infinite) view-projection.
/// There is no far plane: the view extends to infinity.
class Frustum
{
public:
    explicit Frustum(const Mat4d& viewProjection);

    /// True if the box (in the same camera-relative space) may be visible.
    [[nodiscard]] bool intersects(const Vec3d& boxMin, const Vec3d& boxMax) const;

private:
    std::array<Vec4d, 5> planes_{};  // left, right, bottom, top, near; inside where dot >= 0
};

}  // namespace StarshipSimulator
