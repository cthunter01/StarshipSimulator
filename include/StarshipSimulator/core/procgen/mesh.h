#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// The vertex format shared by all meshes. Positions are float and relative to the mesh origin
/// (a double-precision world position), so vertices stay precise however far the mesh is from
/// (0,0,0). Must match the vertex inputs of shaders/mesh.vert.
struct Vertex
{
    Vec3f         position{0.0F};
    Vec3f         normal{0.0F, 0.0F, 1.0F};
    Vec2f         uv{0.0F};
    std::uint32_t material = 0;
};
static_assert(sizeof(Vertex) == 36, "Vertex layout must match the GPU vertex input description");
static_assert(offsetof(Vertex, normal) == 12);
static_assert(offsetof(Vertex, uv) == 24);
static_assert(offsetof(Vertex, material) == 32);

/// Triangle list with counter-clockwise front faces.
struct CpuMesh
{
    std::vector<Vertex>        vertices;
    std::vector<std::uint32_t> indices;
};

/// An axis-aligned box centred on the origin, with flat normals and outward-facing triangles.
[[nodiscard]] CpuMesh makeBox(const Vec3f& halfExtents, std::uint32_t material = 0);

/// A UV sphere centred on the origin with outward normals.
[[nodiscard]] CpuMesh makeSphere(float radius, int segments, std::uint32_t material = 0);

}  // namespace StarshipSimulator
