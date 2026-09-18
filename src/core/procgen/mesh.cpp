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

}  // namespace StarshipSimulator
