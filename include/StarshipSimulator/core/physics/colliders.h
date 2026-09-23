#pragma once

#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"

// Collision shapes of the things built in the habitat (buildings, bridges, street furniture), as
// plain data for the physics library. Local axes: x across, y up (toward the spin axis), z along;
// `orientation` turns them into the habitat frame.
namespace StarshipSimulator
{

struct StaticBox
{
    Vec3d centre{0.0};
    Quatd orientation{1.0, 0.0, 0.0, 0.0};
    Vec3d halfExtents{0.5};
};

/// A convex shape: the hull of its points (local, relative to `origin`).
struct StaticHull
{
    Vec3d              origin{0.0};
    Quatd              orientation{1.0, 0.0, 0.0, 0.0};
    std::vector<Vec3f> points;
};

/// Triangles, as a surface to collide with (a torus's ceiling, spokes and hub): vertices relative
/// to `origin`, three indices to a triangle.
struct StaticMesh
{
    Vec3d                      origin{0.0};
    std::vector<Vec3f>         vertices;
    std::vector<std::uint32_t> indices;
};

struct StaticColliders
{
    std::vector<StaticBox>  boxes;
    std::vector<StaticHull> hulls;
    std::vector<StaticMesh> meshes;
};

/// The rotation that takes local axes (x across, y up, z along) to the habitat frame at a point on
/// the floor: y toward the spin axis, z along it (+Z), turned by `yaw` (radians) about y.
[[nodiscard]] Quatd floorOrientation(const Vec3d& position, double yaw = 0.0);
/// The same with local z pointing `along` instead (a unit vector across the local up), for plans
/// laid out on a band of land that runs around the axis.
[[nodiscard]] Quatd floorOrientation(const Vec3d& position, double yaw, const Vec3d& along);

}  // namespace StarshipSimulator
