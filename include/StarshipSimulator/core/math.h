#pragma once

// The project's single entry point to GLM. Use these aliases rather than glm:: types directly.
// Conventions:
//   - World positions are double precision (Vec3d); GPU data is float and camera-relative (Vec3f).
//   - Right-handed coordinates. In a spin habitat the spin axis is +Z.
//   - Cameras look along their local -Z with local +Y up (GLM convention).
#include <numbers>

#include <glm/glm.hpp>                   // IWYU pragma: export
#include <glm/gtc/matrix_transform.hpp>  // IWYU pragma: export
#include <glm/gtc/quaternion.hpp>        // IWYU pragma: export

namespace StarshipSimulator
{

using Vec2d = glm::dvec2;
using Vec3d = glm::dvec3;
using Vec4d = glm::dvec4;
using Mat3d = glm::dmat3;
using Mat4d = glm::dmat4;
using Quatd = glm::dquat;

using Vec2f = glm::vec2;
using Vec3f = glm::vec3;
using Vec4f = glm::vec4;
using Mat3f = glm::mat3;
using Mat4f = glm::mat4;

inline constexpr double kPi = std::numbers::pi;

[[nodiscard]] constexpr double degreesToRadians(double degrees)
{
    return degrees * (kPi / 180.0);
}
[[nodiscard]] constexpr double radiansToDegrees(double radians)
{
    return radians * (180.0 / kPi);
}

}  // namespace StarshipSimulator
