#include "StarshipSimulator/core/procgen/props.h"

#include <array>
#include <cstddef>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"

namespace StarshipSimulator
{

namespace
{

using Shape = PropPart::Shape;

std::array<PropInfo, kPropKindCount> makeInfos()
{
    std::array<PropInfo, kPropKindCount> infos{};
    // A football: bouncy, rolls a long way, floats high.
    infos.at(static_cast<std::size_t>(PropKind::Ball)) = {
        .parts          = {{.shape       = Shape::Sphere,
                            .centre      = Vec3f(0.0F, 0.12F, 0.0F),
                            .halfExtents = Vec3f(0.12F)}},
        .massKg         = 0.43F,
        .friction       = 0.6F,
        .restitution    = 0.7F,
        .angularDamping = 0.5F,
        .buoyancy       = 3.0F};
    infos.at(static_cast<std::size_t>(PropKind::Crate)) = {
        .parts          = {{.shape       = Shape::Box,
                            .centre      = Vec3f(0.0F, 0.3F, 0.0F),
                            .halfExtents = Vec3f(0.3F)}},
        .massKg         = 12.0F,
        .friction       = 0.6F,
        .restitution    = 0.15F,
        .angularDamping = 0.2F,
        .buoyancy       = 1.3F};
    infos.at(static_cast<std::size_t>(PropKind::Barrel)) = {
        .parts          = {{.shape       = Shape::Cylinder,
                            .centre      = Vec3f(0.0F, 0.45F, 0.0F),
                            .halfExtents = Vec3f(0.3F, 0.45F, 0.3F)}},
        .massKg         = 25.0F,
        .friction       = 0.5F,
        .restitution    = 0.1F,
        .angularDamping = 0.3F,
        .buoyancy       = 1.5F};
    // A round bale, standing on its end.
    infos.at(static_cast<std::size_t>(PropKind::HayBale)) = {
        .parts          = {{.shape       = Shape::Cylinder,
                            .centre      = Vec3f(0.0F, 0.6F, 0.0F),
                            .halfExtents = Vec3f(0.6F, 0.6F, 0.6F)}},
        .massKg         = 200.0F,
        .friction       = 0.8F,
        .restitution    = 0.05F,
        .angularDamping = 0.4F,
        .buoyancy       = 1.2F};
    // A cafe chair: seat on legs, and a back.
    infos.at(static_cast<std::size_t>(PropKind::Chair)) = {
        .parts          = {{.shape       = Shape::Box,
                            .centre      = Vec3f(0.0F, 0.235F, 0.0F),
                            .halfExtents = Vec3f(0.21F, 0.235F, 0.21F)},
                           {.shape       = Shape::Box,
                            .centre      = Vec3f(0.0F, 0.66F, -0.195F),
                            .halfExtents = Vec3f(0.21F, 0.19F, 0.02F)}},
        .massKg         = 4.0F,
        .friction       = 0.5F,
        .restitution    = 0.1F,
        .angularDamping = 0.2F,
        .buoyancy       = 0.6F};
    // A round cafe table on a pedestal.
    infos.at(static_cast<std::size_t>(PropKind::Table)) = {
        .parts          = {{.shape       = Shape::Cylinder,
                            .centre      = Vec3f(0.0F, 0.72F, 0.0F),
                            .halfExtents = Vec3f(0.36F, 0.015F, 0.36F)},
                           {.shape       = Shape::Box,
                            .centre      = Vec3f(0.0F, 0.36F, 0.0F),
                            .halfExtents = Vec3f(0.03F, 0.345F, 0.03F)},
                           {.shape       = Shape::Cylinder,
                            .centre      = Vec3f(0.0F, 0.015F, 0.0F),
                            .halfExtents = Vec3f(0.22F, 0.015F, 0.22F)}},
        .massKg         = 12.0F,
        .friction       = 0.5F,
        .restitution    = 0.1F,
        .angularDamping = 0.2F,
        .buoyancy       = 0.5F};
    return infos;
}

Mat4d at(double x, double y, double z)
{
    return glm::translate(Mat4d(1.0), Vec3d(x, y, z));
}

}  // namespace

const char* propKindName(PropKind kind)
{
    switch (kind)
    {
        case PropKind::Ball:
            return "ball";
        case PropKind::Crate:
            return "crate";
        case PropKind::Barrel:
            return "barrel";
        case PropKind::HayBale:
            return "hay bale";
        case PropKind::Chair:
            return "chair";
        case PropKind::Table:
            return "table";
    }
    return "prop";
}

const PropInfo& propInfo(PropKind kind)
{
    static const std::array<PropInfo, kPropKindCount> infos = makeInfos();
    return infos.at(static_cast<std::size_t>(kind));
}

CpuMesh makePropMesh(PropKind kind)
{
    CpuMesh mesh;
    switch (kind)
    {
        case PropKind::Ball:
            appendMesh(mesh, makeSphere(0.12F, 20, propMaterial::kBall), at(0.0, 0.12, 0.0));
            break;
        case PropKind::Crate:
            appendMesh(mesh, makeBox(Vec3f(0.3F), propMaterial::kWood), at(0.0, 0.3, 0.0));
            break;
        case PropKind::Barrel:
            appendMesh(mesh, makeCylinder(0.3F, 0.45F, 16, propMaterial::kStaves),
                       at(0.0, 0.45, 0.0));
            break;
        case PropKind::HayBale:
            appendMesh(mesh, makeCylinder(0.6F, 0.6F, 20, propMaterial::kStraw), at(0.0, 0.6, 0.0));
            break;
        case PropKind::Chair:
            appendMesh(mesh, makeBox(Vec3f(0.21F, 0.015F, 0.21F), propMaterial::kWood),
                       at(0.0, 0.455, 0.0));
            appendMesh(mesh, makeBox(Vec3f(0.21F, 0.14F, 0.012F), propMaterial::kWood),
                       at(0.0, 0.72, -0.2));
            for (const double x : {-0.19, 0.19})
            {
                for (const double z : {-0.19, 0.19})
                {
                    appendMesh(mesh, makeCylinder(0.012F, 0.22F, 6, propMaterial::kMetal),
                               at(x, 0.22, z));
                }
                appendMesh(mesh, makeCylinder(0.012F, 0.2F, 6, propMaterial::kMetal),
                           at(x, 0.64, -0.2));
            }
            break;
        case PropKind::Table:
            appendMesh(mesh, makeCylinder(0.36F, 0.015F, 24, propMaterial::kTableTop),
                       at(0.0, 0.72, 0.0));
            appendMesh(mesh, makeCylinder(0.03F, 0.345F, 8, propMaterial::kMetal),
                       at(0.0, 0.36, 0.0));
            appendMesh(mesh, makeCylinder(0.22F, 0.015F, 16, propMaterial::kMetal),
                       at(0.0, 0.015, 0.0));
            break;
    }
    return mesh;
}

}  // namespace StarshipSimulator
