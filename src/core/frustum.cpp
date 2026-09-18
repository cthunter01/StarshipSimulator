#include "StarshipSimulator/core/frustum.h"

#include <algorithm>
#include <array>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

Frustum::Frustum(const Mat4d& viewProjection)
{
    // Gribb/Hartmann: planes are sums of the matrix rows. GLM is column-major: m[column][row].
    const Mat4d  m = glm::transpose(viewProjection);  // rows as columns
    const Vec4d& x = m[0];
    const Vec4d& y = m[1];
    const Vec4d& z = m[2];
    const Vec4d& w = m[3];
    planes_ = {w + x, w - x, w + y, w - y, w - z};  // depth <= 1 is the near plane (reverse-Z)
}

bool Frustum::intersects(const Vec3d& boxMin, const Vec3d& boxMax) const
{
    return std::ranges::all_of(planes_, [&](const Vec4d& plane) {
        // The box corner furthest along the plane normal.
        const Vec3d corner(plane.x >= 0.0 ? boxMax.x : boxMin.x,
                           plane.y >= 0.0 ? boxMax.y : boxMin.y,
                           plane.z >= 0.0 ? boxMax.z : boxMin.z);
        return glm::dot(Vec3d(plane), corner) + plane.w >= 0.0;
    });
}

}  // namespace StarshipSimulator
