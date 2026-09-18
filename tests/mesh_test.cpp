#include "StarshipSimulator/core/procgen/mesh.h"

#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/math.h"

namespace
{

using StarshipSimulator::makeBox;
using StarshipSimulator::Vec3f;

TEST(Mesh, BoxHasFourVerticesAndTwoTrianglesPerFace)
{
    const auto box = makeBox(Vec3f(1.0F, 2.0F, 3.0F));
    EXPECT_EQ(box.vertices.size(), 24U);
    EXPECT_EQ(box.indices.size(), 36U);
    for (const auto index : box.indices)
    {
        EXPECT_LT(index, box.vertices.size());
    }
}

TEST(Mesh, BoxVerticesLieOnItsSurface)
{
    const Vec3f half(1.0F, 2.0F, 3.0F);
    for (const auto& vertex : makeBox(half).vertices)
    {
        // The coordinate along the face normal is on the face; the others are within it.
        EXPECT_FLOAT_EQ(glm::dot(vertex.position, vertex.normal),
                        glm::dot(half, glm::abs(vertex.normal)));
        EXPECT_LE(std::abs(vertex.position.x), half.x);
        EXPECT_LE(std::abs(vertex.position.y), half.y);
        EXPECT_LE(std::abs(vertex.position.z), half.z);
    }
}

TEST(Mesh, BoxTrianglesAreCounterClockwiseSeenFromOutside)
{
    const auto box = makeBox(Vec3f(0.5F));
    for (std::size_t i = 0; i + 2 < box.indices.size(); i += 3)
    {
        const auto& a          = box.vertices[box.indices[i]];
        const auto& b          = box.vertices[box.indices[i + 1]];
        const auto& c          = box.vertices[box.indices[i + 2]];
        const Vec3f faceNormal = glm::cross(b.position - a.position, c.position - a.position);
        EXPECT_GT(glm::dot(faceNormal, a.normal), 0.0F) << "triangle " << i / 3;
    }
}

}  // namespace
