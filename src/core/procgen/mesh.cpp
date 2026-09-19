#include "StarshipSimulator/core/procgen/mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

CpuMesh makeBox(const Vec3f& halfExtents, std::uint32_t material)
{
    struct Face
    {
        Vec3f normal;
        Vec3f tangent;    // u direction
        Vec3f bitangent;  // v direction, tangent x bitangent == normal (counter-clockwise)
    };
    const std::array<Face, 6>  faces{{
        {.normal = {1, 0, 0}, .tangent = {0, 1, 0}, .bitangent = {0, 0, 1}},
        {.normal = {-1, 0, 0}, .tangent = {0, 0, 1}, .bitangent = {0, 1, 0}},
        {.normal = {0, 1, 0}, .tangent = {0, 0, 1}, .bitangent = {1, 0, 0}},
        {.normal = {0, -1, 0}, .tangent = {1, 0, 0}, .bitangent = {0, 0, 1}},
        {.normal = {0, 0, 1}, .tangent = {1, 0, 0}, .bitangent = {0, 1, 0}},
        {.normal = {0, 0, -1}, .tangent = {0, 1, 0}, .bitangent = {1, 0, 0}},
    }};
    const std::array<Vec2f, 4> corners{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}};

    CpuMesh mesh;
    mesh.vertices.reserve(faces.size() * corners.size());
    mesh.indices.reserve(faces.size() * 6);
    for (const Face& face : faces)
    {
        const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
        for (const Vec2f& corner : corners)
        {
            const Vec3f unit =
                face.normal + (face.tangent * corner.x) + (face.bitangent * corner.y);
            mesh.vertices.push_back({.position = unit * halfExtents,
                                     .normal   = face.normal,
                                     .uv       = (corner + Vec2f(1.0F)) * 0.5F,
                                     .material = material});
        }
        for (const std::uint32_t corner : {0U, 1U, 2U, 0U, 2U, 3U})
        {
            mesh.indices.push_back(base + corner);
        }
    }
    return mesh;
}

CpuMesh makeSphere(float radius, int segments, std::uint32_t material)
{
    const int rings   = std::max(2, segments / 2);
    const int columns = std::max(3, segments);
    CpuMesh   mesh;
    for (int i = 0; i <= rings; ++i)
    {
        const double polar = kPi * i / rings;
        for (int k = 0; k <= columns; ++k)
        {
            const double azimuth = 2.0 * kPi * k / columns;
            const Vec3f  normal(Vec3d(std::sin(polar) * std::cos(azimuth),
                                      std::sin(polar) * std::sin(azimuth), std::cos(polar)));
            mesh.vertices.push_back({.position = normal * radius,
                                     .normal   = normal,
                                     .uv       = Vec2f(Vec2d(static_cast<double>(k) / columns,
                                                             static_cast<double>(i) / rings)),
                                     .material = material});
        }
    }
    const auto at = [columns](int ring, int column) {
        return static_cast<std::uint32_t>((ring * (columns + 1)) + column);
    };
    for (int i = 0; i < rings; ++i)
    {
        for (int k = 0; k < columns; ++k)
        {
            // Counter-clockwise seen from outside.
            mesh.indices.insert(mesh.indices.end(), {at(i, k), at(i + 1, k), at(i, k + 1),
                                                     at(i + 1, k), at(i + 1, k + 1), at(i, k + 1)});
        }
    }
    return mesh;
}

CpuMesh makeCylinder(float radius, float halfHeight, int sides, std::uint32_t material)
{
    const int count = std::max(3, sides);
    CpuMesh   mesh;
    // Sides: a ring of vertices at the bottom and the top, the seam duplicated for the uv.
    for (int i = 0; i <= count; ++i)
    {
        const double angle = 2.0 * kPi * i / count;
        const Vec3f  outward(Vec3d(std::cos(angle), 0.0, std::sin(angle)));
        const auto   u = static_cast<float>(static_cast<double>(i) / count);
        for (const float y : {-1.0F, 1.0F})
        {
            mesh.vertices.push_back(
                {.position = Vec3f(outward.x * radius, y * halfHeight, outward.z * radius),
                 .normal   = outward,
                 .uv       = Vec2f(u, (y + 1.0F) * 0.5F),
                 .material = material});
        }
    }
    for (int i = 0; i < count; ++i)
    {
        const auto a = static_cast<std::uint32_t>(2 * i);
        // Counter-clockwise seen from outside: bottom, top, next bottom; next bottom, top, next
        // top.
        mesh.indices.insert(mesh.indices.end(), {a, a + 1, a + 2, a + 2, a + 1, a + 3});
    }
    // Caps: a fan around a centre vertex each.
    for (const float y : {-1.0F, 1.0F})
    {
        const Vec3f normal(0.0F, y, 0.0F);
        const auto  centre = static_cast<std::uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({.position = Vec3f(0.0F, y * halfHeight, 0.0F),
                                 .normal   = normal,
                                 .uv       = Vec2f(0.0F),
                                 .material = material});
        for (int i = 0; i < count; ++i)
        {
            const double angle = 2.0 * kPi * i / count;
            const Vec2f  at(Vec2d(std::cos(angle), std::sin(angle)) * static_cast<double>(radius));
            mesh.vertices.push_back({.position = Vec3f(at.x, y * halfHeight, at.y),
                                     .normal   = normal,
                                     .uv       = at,
                                     .material = material});
        }
        for (int i = 0; i < count; ++i)
        {
            const std::uint32_t a = centre + 1 + static_cast<std::uint32_t>(i);
            const std::uint32_t b = centre + 1 + static_cast<std::uint32_t>((i + 1) % count);
            if (y > 0.0F)
            {
                mesh.indices.insert(mesh.indices.end(), {centre, b, a});
            }
            else
            {
                mesh.indices.insert(mesh.indices.end(), {centre, a, b});
            }
        }
    }
    return mesh;
}

void appendMesh(CpuMesh& to, const CpuMesh& from, const Mat4d& transform)
{
    const Mat3d normalMatrix = glm::transpose(glm::inverse(Mat3d(transform)));
    const auto  base         = static_cast<std::uint32_t>(to.vertices.size());
    to.vertices.reserve(to.vertices.size() + from.vertices.size());
    for (const Vertex& vertex : from.vertices)
    {
        Vertex moved   = vertex;
        moved.position = Vec3f(Vec3d(transform * Vec4d(Vec3d(vertex.position), 1.0)));
        moved.normal   = Vec3f(glm::normalize(normalMatrix * Vec3d(vertex.normal)));
        to.vertices.push_back(moved);
    }
    to.indices.reserve(to.indices.size() + from.indices.size());
    for (const std::uint32_t index : from.indices)
    {
        to.indices.push_back(base + index);
    }
}

}  // namespace StarshipSimulator
