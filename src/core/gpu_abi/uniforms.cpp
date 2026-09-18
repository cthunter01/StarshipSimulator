#include "StarshipSimulator/core/gpu_abi/uniforms.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "StarshipSimulator/core/camera.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator::gpu
{

FrameUniforms makeFrameUniforms(const Camera& camera, std::uint32_t width, std::uint32_t height)
{
    const double safeWidth      = std::max(1.0, static_cast<double>(width));
    const double safeHeight     = std::max(1.0, static_cast<double>(height));
    const Mat4d  viewProjection = cameraRelativeViewProjection(camera, safeWidth / safeHeight);

    // The grid only needs the camera position modulo its period, which float holds precisely.
    const double gridX = std::fmod(camera.position.x, kGridPeriodMeters);
    const double gridY = std::fmod(camera.position.y, kGridPeriodMeters);

    FrameUniforms frame;
    frame.viewProjection        = Mat4f(viewProjection);
    frame.inverseViewProjection = Mat4f(glm::inverse(viewProjection));
    frame.gridOrigin   = Vec4f(Vec4d(gridX, gridY, camera.position.z, camera.nearPlaneMeters));
    frame.viewport     = Vec4f(Vec4d(safeWidth, safeHeight, 1.0 / safeWidth, 1.0 / safeHeight));
    frame.sunDirection = Vec4f(glm::normalize(Vec3f(0.4F, 0.3F, 0.85F)), 0.0F);
    return frame;
}

}  // namespace StarshipSimulator::gpu
