#include "StarshipSimulator/core/procgen/mesh.h"

#include <array>
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

}  // namespace StarshipSimulator
