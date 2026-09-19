#include "StarshipSimulator/core/physics/colliders.h"

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

Quatd floorOrientation(const Vec3d& position, double yaw)
{
    const Vec3d up = HabitatGeometry::localUp(position);
    const Vec3d along(0.0, 0.0, 1.0);
    const Vec3d across = glm::cross(up, along);  // x = y cross z
    const Quatd frame  = glm::quat_cast(Mat3d(across, up, along));
    return glm::normalize(frame * glm::angleAxis(yaw, Vec3d(0.0, 1.0, 0.0)));
}

}  // namespace StarshipSimulator
