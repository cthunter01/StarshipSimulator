#include "StarshipSimulator/core/procgen/buildings.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/rng.h"

namespace StarshipSimulator
{

namespace
{

using buildingMaterial::kAwning;
using buildingMaterial::kFlatRoof;
using buildingMaterial::kFoliage;
using buildingMaterial::kLampGlass;
using buildingMaterial::kMetal;
using buildingMaterial::kRoofTiles;
using buildingMaterial::kSoffit;
using buildingMaterial::kStone;
using buildingMaterial::kWall;
using buildingMaterial::kWater;
using buildingMaterial::kWood;

constexpr double kEaveM          = 0.35;  // roofs overhang the walls
constexpr double kParapetM       = 0.6;   // around flat roofs
constexpr double kParapetInsetM  = 0.25;
constexpr double kBridgeDeckM    = 0.7;  // thickness of a bridge's arch at its crown
constexpr double kBridgeParapetM = 1.0;

constexpr Vec3d kUp(0.0, 1.0, 0.0);

/// Writes faces into a mesh: positions in some local frame, placed into the mesh's frame by
/// `place`. Faces are turned to face along a hint; normals come from the placed triangles.
class MeshWriter
{
public:
    MeshWriter(CpuMesh& mesh, std::function<Vec3d(const Vec3d&)> place)
      : mesh_(&mesh), place_(std::move(place))
    {
    }

    void face(std::span<const Vec3d> points, std::span<const Vec2d> uvs, std::uint32_t material,
              const Vec3d& facing)
    {
        // Placing can mirror (the plan's frame is left-handed), so wind the placed triangles.
        std::array<Vec3d, 4> placed{};
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            placed.at(i) = place_(points[i]);
        }
        const Vec3d towards = place_(points[0] + (facing * 0.01)) - placed[0];
        Vec3d       normal  = glm::cross(placed[1] - placed[0], placed[2] - placed[0]);
        const bool  flip    = glm::dot(normal, towards) < 0.0;
        normal          = glm::length(normal) > 0.0 ? glm::normalize(flip ? -normal : normal) : kUp;
        const auto base = static_cast<std::uint32_t>(mesh_->vertices.size());
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            mesh_->vertices.push_back({.position = Vec3f(placed.at(i)),
                                       .normal   = Vec3f(normal),
                                       .uv       = Vec2f(uvs[i]),
                                       .material = material});
        }
        const std::array<std::uint32_t, 6> quad{0, 1, 2, 0, 2, 3};
        const std::size_t                  count = points.size() == 4 ? 6 : 3;
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::uint32_t corner = flip ? quad.at(count - 1 - i) : quad.at(i);
            mesh_->indices.push_back(base + corner);
        }
    }

    void quad(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d,
              std::array<Vec2d, 4> uvs, std::uint32_t material, const Vec3d& facing)
    {
        const std::array<Vec3d, 4> points{a, b, c, d};
        face(points, uvs, material, facing);
    }

    void triangle(const Vec3d& a, const Vec3d& b, const Vec3d& c, std::array<Vec2d, 3> uvs,
                  std::uint32_t material, const Vec3d& facing)
    {
        const std::array<Vec3d, 3> points{a, b, c};
        face(points, uvs, material, facing);
    }

    /// A box's sides and top (not its bottom), uv in metres.
    void box(const Vec3d& centre, const Vec3d& half, std::uint32_t material)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            for (const double sign : {-1.0, 1.0})
            {
                if (axis == 1 && sign < 0.0)
                {
                    continue;
                }
                Vec3d n(0.0);
                n[axis]         = sign;
                const int uAxis = axis == 0 ? 2 : 0;
                const int vAxis = axis == 1 ? 2 : 1;
                Vec3d     du(0.0);
                Vec3d     dv(0.0);
                du[uAxis]      = half[uAxis];
                dv[vAxis]      = half[vAxis];
                const Vec3d c  = centre + (n * half);
                const auto  uv = [&](double su, double sv) {
                    return Vec2d(c[uAxis] + (su * half[uAxis]), c[vAxis] + (sv * half[vAxis]));
                };
                quad(c - du - dv, c + du - dv, c + du + dv, c - du + dv,
                     {uv(-1, -1), uv(1, -1), uv(1, 1), uv(-1, 1)}, material, n);
            }
        }
    }

    /// Appends a ready-made mesh (in the local frame).
    void mesh(const CpuMesh& local)
    {
        const auto base = static_cast<std::uint32_t>(mesh_->vertices.size());
        for (const Vertex& vertex : local.vertices)
        {
            const Vec3d p      = place_(Vec3d(vertex.position));
            const Vec3d q      = place_(Vec3d(vertex.position) + (Vec3d(vertex.normal) * 0.01));
            Vertex      placed = vertex;
            placed.position    = Vec3f(p);
            placed.normal      = Vec3f(glm::normalize(q - p));
            mesh_->vertices.push_back(placed);
        }
        for (const std::uint32_t index : local.indices)
        {
            mesh_->indices.push_back(base + index);
        }
    }

private:
    CpuMesh*                           mesh_;
    std::function<Vec3d(const Vec3d&)> place_;
};

/// A building's frame: local x and z along its footprint's axes, y up, origin on the ground floor
/// at the footprint's centre.
struct BuildingFrame
{
    Vec3d base{0.0};  // habitat frame
    Quatd orientation{1.0, 0.0, 0.0, 0.0};

    [[nodiscard]] Vec3d toHabitat(const Vec3d& local) const { return base + (orientation * local); }
};

BuildingFrame frameOf(const Settlement& place, const Vec2d& centre, double height, double angle)
{
    const Vec3d base = place.plane.point(centre, height);
    return {.base = base, .orientation = floorOrientation(base, -angle)};
}

FacadeStyle styleOf(BuildingUse use)
{
    switch (use)
    {
        case BuildingUse::Tower:
            return FacadeStyle::Tower;
        case BuildingUse::Barn:
            return FacadeStyle::Barn;
        case BuildingUse::Hall:
            return FacadeStyle::Hall;
        case BuildingUse::House:
        case BuildingUse::Shop:
        case BuildingUse::Farmhouse:
            break;
    }
    return FacadeStyle::House;
}

/// The roof's outline: where its faces meet, in the building's frame (for the mesh and collider).
struct RoofShape
{
    std::vector<std::array<Vec3d, 4>> quads;      // (a, b, c, d); triangles repeat d = c
    std::vector<bool>                 triangles;  // per quad: really a triangle
    std::vector<Vec3d>                facing;
    std::vector<Vec3d>                eaves;   // per face: direction along its eave
    std::vector<std::array<Vec3d, 3>> gables;  // gable-end wall triangles
    std::vector<Vec3d>                gableFacing;
    std::vector<Vec3d>                points;  // everything, for the collider
};

RoofShape roofShape(const Building& b)
{
    RoofShape    roof;
    const double H      = b.wallHeight();
    const double t      = std::tan(degreesToRadians(b.roofPitchDeg));
    const bool   alongX = b.halfSize.x >= b.halfSize.y;
    const double L      = std::max(b.halfSize.x, b.halfSize.y);
    const double S      = std::min(b.halfSize.x, b.halfSize.y);
    const Vec3d  r      = alongX ? Vec3d(1.0, 0.0, 0.0) : Vec3d(0.0, 0.0, 1.0);
    const Vec3d  c      = alongX ? Vec3d(0.0, 0.0, 1.0) : Vec3d(1.0, 0.0, 0.0);
    const auto   at     = [&](double along, double across, double y) {
        return (r * along) + (c * across) + Vec3d(0.0, y, 0.0);
    };
    const double yEave  = H - (kEaveM * t);
    const double yRidge = H + (S * t);
    const auto   add    = [&](std::array<Vec3d, 4> q, bool triangle, const Vec3d& facing,
                              const Vec3d& eave) {
        roof.quads.push_back(q);
        roof.triangles.push_back(triangle);
        roof.facing.push_back(facing);
        roof.eaves.push_back(eave);
        for (const Vec3d& p : q)
        {
            roof.points.push_back(p);
        }
    };

    if (b.roof == RoofKind::Gable)
    {
        const double end = L + kEaveM;
        for (const double side : {-1.0, 1.0})
        {
            add({at(-end, side * (S + kEaveM), yEave), at(end, side * (S + kEaveM), yEave),
                 at(end, 0.0, yRidge), at(-end, 0.0, yRidge)},
                false, (c * side) + kUp, r);
            roof.gables.push_back(
                {at(side * L, -S, H), at(side * L, S, H), at(side * L, 0.0, yRidge)});
            roof.gableFacing.push_back(r * side);
        }
        return roof;
    }
    // Hip roofs, and pyramids when the footprint is square.
    const double ridge = b.roof == RoofKind::Pyramid ? 0.0 : std::max(L - S, 0.0);
    const double peak  = b.roof == RoofKind::Pyramid ? H + (S * t) : yRidge;
    for (const double side : {-1.0, 1.0})
    {
        add({at(-(L + kEaveM), side * (S + kEaveM), yEave),
             at(L + kEaveM, side * (S + kEaveM), yEave), at(ridge, 0.0, peak),
             at(-ridge, 0.0, peak)},
            ridge <= 0.0, (c * side) + kUp, r);
        add({at(side * (L + kEaveM), -(S + kEaveM), yEave),
             at(side * (L + kEaveM), S + kEaveM, yEave), at(side * ridge, 0.0, peak),
             at(side * ridge, 0.0, peak)},
            true, (r * side) + kUp, c);
    }
    return roof;
}

void writeRoof(MeshWriter& writer, const Building& b, const RoofShape& roof, const Facade& wall)
{
    const std::uint32_t tiles =
        packFacade({.surface = kRoofTiles, .colour = b.roofColour, .seed = b.seed});
    const std::uint32_t soffit = packFacade({.surface = kSoffit, .colour = b.wallColour});
    for (std::size_t i = 0; i < roof.quads.size(); ++i)
    {
        const auto& q      = roof.quads[i];
        const Vec3d eave   = roof.eaves[i];
        const Vec3d normal = glm::normalize(glm::cross(q[1] - q[0], q[2] - q[0]));
        Vec3d       slope  = glm::normalize(glm::cross(normal, eave));
        slope              = slope.y < 0.0 ? -slope : slope;
        std::array<Vec2d, 4> uv{};
        for (std::size_t k = 0; k < 4; ++k)
        {
            uv.at(k) = Vec2d(glm::dot(q.at(k), eave), glm::dot(q.at(k) - q[0], slope));
        }
        if (roof.triangles[i])
        {
            writer.triangle(q[0], q[1], q[2], {uv[0], uv[1], uv[2]}, tiles, roof.facing[i]);
            writer.triangle(q[0], q[1], q[2], {uv[0], uv[1], uv[2]}, soffit, -roof.facing[i]);
        }
        else
        {
            writer.quad(q[0], q[1], q[2], q[3], uv, tiles, roof.facing[i]);
            writer.quad(q[0], q[1], q[2], q[3], uv, soffit, -roof.facing[i]);
        }
    }
    Facade gable  = wall;
    gable.windows = 0;
    gable.door    = false;
    gable.shop    = false;
    for (std::size_t i = 0; i < roof.gables.size(); ++i)
    {
        const auto& g = roof.gables[i];
        writer.triangle(g[0], g[1], g[2],
                        {Vec2d(-1.0, g[0].y), Vec2d(-1.0, g[1].y), Vec2d(-1.0, g[2].y)},
                        packFacade(gable), roof.gableFacing[i]);
    }
}

/// A wall from a to b (bottom corners, seen from outside left to right), y0 to y1.
void writeWall(MeshWriter& writer, const Vec3d& a, const Vec3d& b, double y0, double y1,
               Facade facade, const Vec3d& outward)
{
    const double length = glm::distance(a, b);
    const int    bays   = facade.surface == kWall && facade.style != FacadeStyle::Barn
                              ? static_cast<int>(std::floor((length - 1.0) / kWindowBayM))
                              : 0;
    facade.windows      = static_cast<std::uint32_t>(std::clamp(bays, 0, 63));
    const double margin = 0.5 * (length - (facade.windows * kWindowBayM));
    const double u0     = -margin / kWindowBayM;
    const double u1     = (length - margin) / kWindowBayM;
    const Vec3d  lift0(0.0, y0, 0.0);
    const Vec3d  lift1(0.0, y1, 0.0);
    writer.quad(a + lift0, b + lift0, b + lift1, a + lift1,
                {Vec2d(u0, y0), Vec2d(u1, y0), Vec2d(u1, y1), Vec2d(u0, y1)}, packFacade(facade),
                outward);
}

void writeFlatRoof(MeshWriter& writer, const Building& b)
{
    const double        H      = b.wallHeight();
    const double        x      = b.halfSize.x - kParapetInsetM;
    const double        z      = b.halfSize.y - kParapetInsetM;
    const double        floor  = H + 0.1;
    const double        top    = H + kParapetM;
    const std::uint32_t roof   = packFacade({.surface = kFlatRoof, .colour = b.roofColour});
    const std::uint32_t coping = packFacade({.surface = kStone});
    writer.quad(Vec3d(-x, floor, -z), Vec3d(x, floor, -z), Vec3d(x, floor, z), Vec3d(-x, floor, z),
                {Vec2d(-x, -z), Vec2d(x, -z), Vec2d(x, z), Vec2d(-x, z)}, roof, kUp);
    const Vec3d                outer(b.halfSize.x, 0.0, b.halfSize.y);
    const std::array<Vec3d, 4> inner{Vec3d(-x, 0.0, -z), Vec3d(x, 0.0, -z), Vec3d(x, 0.0, z),
                                     Vec3d(-x, 0.0, z)};
    const std::array<Vec3d, 4> edge{Vec3d(-outer.x, 0.0, -outer.z), Vec3d(outer.x, 0.0, -outer.z),
                                    Vec3d(outer.x, 0.0, outer.z), Vec3d(-outer.x, 0.0, outer.z)};
    const Facade               wall{.surface = kWall, .colour = b.wallColour};
    for (std::size_t i = 0; i < 4; ++i)
    {
        const std::size_t j      = (i + 1) % 4;
        const Vec3d       inward = -glm::normalize(inner.at(i) + inner.at(j));
        // The parapet's inner face, and its stone coping.
        writer.quad(inner.at(i) + Vec3d(0.0, floor, 0.0), inner.at(j) + Vec3d(0.0, floor, 0.0),
                    inner.at(j) + Vec3d(0.0, top, 0.0), inner.at(i) + Vec3d(0.0, top, 0.0),
                    {Vec2d(-1.0, floor), Vec2d(-1.0, floor), Vec2d(-1.0, top), Vec2d(-1.0, top)},
                    packFacade(wall), inward);
        writer.quad(edge.at(i) + Vec3d(0.0, top, 0.0), edge.at(j) + Vec3d(0.0, top, 0.0),
                    inner.at(j) + Vec3d(0.0, top, 0.0), inner.at(i) + Vec3d(0.0, top, 0.0),
                    {Vec2d(0.0), Vec2d(1.0, 0.0), Vec2d(1.0), Vec2d(0.0, 1.0)}, coping, kUp);
    }
}

void writeBuilding(MeshWriter& writer, const Building& b)
{
    const FacadeStyle style = styleOf(b.use);
    const double      top   = b.wallHeight() + (b.roof == RoofKind::Flat ? kParapetM : 0.0);
    const double      hx    = b.halfSize.x;
    const double      hz    = b.halfSize.y;
    SplitMix64        random(b.seed);
    // Walls, from the side at -z (front side 0) round: -z, +x, +z, -x.
    const std::array<Vec3d, 4> normals{Vec3d(0.0, 0.0, -1.0), Vec3d(1.0, 0.0, 0.0),
                                       Vec3d(0.0, 0.0, 1.0), Vec3d(-1.0, 0.0, 0.0)};
    for (int side = 0; side < 4; ++side)
    {
        const Vec3d  n       = normals.at(static_cast<std::size_t>(side));
        const Vec3d  along   = glm::cross(kUp, n);
        const double halfLen = std::abs(glm::dot(along, Vec3d(hx, 0.0, hz)));
        const Vec3d  centre  = n * std::abs(glm::dot(n, Vec3d(hx, 0.0, hz)));
        const bool   front   = side == b.frontSide;
        const Facade facade{.surface = kWall,
                            .colour  = b.wallColour,
                            .door    = front && b.use != BuildingUse::Tower,
                            .shop    = front && b.use == BuildingUse::Shop,
                            .style   = style,
                            .shutter = b.shutterColour,
                            // Towers keep their height in storeys here, for the belfry and clock.
                            .seed = b.use == BuildingUse::Tower
                                        ? static_cast<std::uint32_t>(b.storeys)
                                        : static_cast<std::uint32_t>(random.next())};
        writeWall(writer, centre - (along * halfLen), centre + (along * halfLen), -b.foundation,
                  top, facade, n);
        if (facade.shop)
        {
            // A striped awning over the shop window.
            const std::uint32_t awning = packFacade(
                {.surface = kAwning, .colour = static_cast<std::uint32_t>(random.next() % 6)});
            const double               w   = halfLen - 0.5;
            const Vec3d                a   = centre - (along * w) + Vec3d(0.0, 2.95, 0.0);
            const Vec3d                bb  = centre + (along * w) + Vec3d(0.0, 2.95, 0.0);
            const Vec3d                out = (n * 1.4) + Vec3d(0.0, -0.45, 0.0);
            const std::array<Vec2d, 4> uv{Vec2d(0.0), Vec2d(2.0 * w, 0.0), Vec2d(2.0 * w, 1.0),
                                          Vec2d(0.0, 1.0)};
            writer.quad(a, bb, bb + out, a + out, uv, awning, kUp + n);
            writer.quad(a, bb, bb + out, a + out, uv, awning, -kUp - n);
        }
    }
    const Facade plain{.surface = kWall, .colour = b.wallColour, .style = style, .seed = b.seed};
    if (b.roof == RoofKind::Flat)
    {
        writeFlatRoof(writer, b);
        return;
    }
    writeRoof(writer, b, roofShape(b), plain);
    if (b.use != BuildingUse::Tower && random.uniform() < 0.45)
    {
        // A chimney through the roof.
        const double t       = std::tan(degreesToRadians(b.roofPitchDeg));
        const double S       = std::min(hx, hz);
        const Vec3d  spot    = hx >= hz ? Vec3d(random.uniform(-0.6, 0.6) * (hx - S), 0.0, 0.3 * S)
                                        : Vec3d(0.3 * S, 0.0, random.uniform(-0.6, 0.6) * (hz - S));
        const double bottom  = b.wallHeight() - 0.3;
        const double peak    = b.wallHeight() + (S * t) + 0.6;
        Facade       chimney = plain;
        chimney.windows      = 0;
        writer.box(spot + Vec3d(0.0, 0.5 * (bottom + peak), 0.0),
                   Vec3d(0.35, 0.5 * (peak - bottom), 0.35), packFacade(chimney));
    }
}

// ---- Street furniture ------------------------------------------------------------------------

void writeFurniture(MeshWriter& writer, const Furniture& item, SplitMix64& random)
{
    const std::uint32_t metal = packFacade({.surface = kMetal});
    const std::uint32_t wood  = packFacade({.surface = kWood});
    const std::uint32_t stone = packFacade({.surface = kStone});
    switch (item.kind)
    {
        case FurnitureKind::Lamp:
            writer.mesh([&] {
                CpuMesh post;
                appendMesh(post, makeCylinder(0.06F, 1.8F, 8, metal),
                           glm::translate(Mat4d(1.0), Vec3d(0.0, 1.8, 0.0)));
                return post;
            }());
            writer.box(Vec3d(0.0, 3.85, 0.0), Vec3d(0.17, 0.24, 0.17),
                       packFacade({.surface = kLampGlass}));
            writer.box(Vec3d(0.0, 4.14, 0.0), Vec3d(0.23, 0.05, 0.23), metal);
            break;
        case FurnitureKind::Bench:
            writer.box(Vec3d(0.0, 0.45, 0.0), Vec3d(0.8, 0.03, 0.22), wood);
            writer.box(Vec3d(0.0, 0.74, -0.2), Vec3d(0.8, 0.2, 0.03), wood);
            for (const double x : {-0.7, 0.7})
            {
                writer.box(Vec3d(x, 0.21, 0.0), Vec3d(0.03, 0.21, 0.2), metal);
            }
            break;
        case FurnitureKind::Planter:
        {
            writer.box(Vec3d(0.0, 0.3, 0.0), Vec3d(0.6, 0.3, 0.3), stone);
            CpuMesh flowers;
            appendMesh(
                flowers,
                makeSphere(0.35F, 10,
                           packFacade({.surface = kFoliage,
                                       .colour  = static_cast<std::uint32_t>(random.next() % 4)})),
                glm::scale(glm::translate(Mat4d(1.0), Vec3d(0.0, 0.72, 0.0)),
                           Vec3d(1.55, 0.7, 0.75)));
            writer.mesh(flowers);
            break;
        }
        case FurnitureKind::Fountain:
        {
            // A round basin (a ring wall around water), a column and a bowl.
            constexpr int kSides = 24;
            for (int i = 0; i < kSides; ++i)
            {
                const double               a0 = 2.0 * kPi * i / kSides;
                const double               a1 = 2.0 * kPi * (i + 1) / kSides;
                const Vec3d                d0(std::cos(a0), 0.0, std::sin(a0));
                const Vec3d                d1(std::cos(a1), 0.0, std::sin(a1));
                const Vec3d                mid = glm::normalize(d0 + d1);
                const std::array<Vec2d, 4> uv{Vec2d(0.0), Vec2d(1.0, 0.0), Vec2d(1.0),
                                              Vec2d(0.0, 1.0)};
                for (const double r : {3.0, 2.7})
                {
                    writer.quad(d0 * r, d1 * r, (d1 * r) + Vec3d(0.0, 0.55, 0.0),
                                (d0 * r) + Vec3d(0.0, 0.55, 0.0), uv, stone, r > 2.8 ? mid : -mid);
                }
                writer.quad((d0 * 2.7) + Vec3d(0.0, 0.55, 0.0), (d1 * 2.7) + Vec3d(0.0, 0.55, 0.0),
                            (d1 * 3.0) + Vec3d(0.0, 0.55, 0.0), (d0 * 3.0) + Vec3d(0.0, 0.55, 0.0),
                            uv, stone, kUp);
                writer.triangle(Vec3d(0.0, 0.42, 0.0), (d0 * 2.7) + Vec3d(0.0, 0.42, 0.0),
                                (d1 * 2.7) + Vec3d(0.0, 0.42, 0.0),
                                {Vec2d(0.0), Vec2d(d0.x, d0.z) * 2.7, Vec2d(d1.x, d1.z) * 2.7},
                                packFacade({.surface = kWater}), kUp);
            }
            CpuMesh column;
            appendMesh(column, makeCylinder(0.3F, 0.75F, 12, stone),
                       glm::translate(Mat4d(1.0), Vec3d(0.0, 0.75, 0.0)));
            appendMesh(column, makeCylinder(1.1F, 0.12F, 20, stone),
                       glm::translate(Mat4d(1.0), Vec3d(0.0, 1.45, 0.0)));
            appendMesh(column, makeSphere(0.28F, 10, stone),
                       glm::translate(Mat4d(1.0), Vec3d(0.0, 1.85, 0.0)));
            writer.mesh(column);
            break;
        }
        case FurnitureKind::Stall:
        {
            writer.box(Vec3d(0.0, 0.45, 0.0), Vec3d(1.2, 0.45, 0.45), wood);
            for (const double x : {-1.15, 1.15})
            {
                for (const double z : {-0.7, 0.4})
                {
                    writer.box(Vec3d(x, 1.2, z), Vec3d(0.05, 1.2, 0.05), wood);
                }
            }
            const std::uint32_t awning = packFacade(
                {.surface = kAwning, .colour = static_cast<std::uint32_t>(random.next() % 6)});
            const std::array<Vec2d, 4> uv{Vec2d(0.0), Vec2d(2.6, 0.0), Vec2d(2.6, 1.6),
                                          Vec2d(0.0, 1.6)};
            const Vec3d                a(-1.3, 2.25, 0.8);
            const Vec3d                b(1.3, 2.25, 0.8);
            const Vec3d                c(1.3, 2.55, -0.85);
            const Vec3d                d(-1.3, 2.55, -0.85);
            writer.quad(a, b, c, d, uv, awning, kUp);
            writer.quad(a, b, c, d, uv, awning, -kUp);
            break;
        }
    }
}

// ---- Bridges ------------------------------------------------------------------------------------

/// Height of a bridge's deck (m, terrain-height datum) at s in 0..1 from one end to the other.
double deckHeight(const Bridge& bridge, double s)
{
    return std::lerp(bridge.fromHeight, bridge.toHeight, s) + 0.1 +
           (4.0 * bridge.rise * s * (1.0 - s));
}

int bridgeSegments(const Bridge& bridge)
{
    return std::max(8, static_cast<int>(glm::distance(bridge.from, bridge.to) / 3.0));
}

void writeBridge(MeshWriter& writer, const Bridge& bridge)
{
    const std::uint32_t stone  = packFacade({.surface = kStone});
    const std::uint32_t paving = packFacade({.surface = kStone, .colour = 1});
    const Vec2d         dir    = glm::normalize(bridge.to - bridge.from);
    const Vec2d         side(-dir.y, dir.x);
    const int           segments = bridgeSegments(bridge);
    const double        length   = glm::distance(bridge.from, bridge.to);
    // Points are (plan x, plan y, height) and placed on the plan by the writer.
    const auto at = [&](double s, double across, double lift) {
        const Vec2d p = glm::mix(bridge.from, bridge.to, s) + (side * across);
        return Vec3d(p.x, p.y, deckHeight(bridge, s) + lift);
    };
    const Vec3d  up(0.0, 0.0, 1.0);
    const double w = bridge.halfWidth;
    for (int i = 0; i < segments; ++i)
    {
        const double s0 = static_cast<double>(i) / segments;
        const double s1 = static_cast<double>(i + 1) / segments;
        const double u0 = s0 * length;
        const double u1 = s1 * length;
        // Thicker toward the ends, where the arch springs from the banks.
        const auto under = [&](double s) {
            return -kBridgeDeckM - (1.6 * std::pow(std::abs((2.0 * s) - 1.0), 3.0));
        };
        const Vec3d sideways(side.x, side.y, 0.0);
        writer.quad(at(s0, -w, 0.0), at(s1, -w, 0.0), at(s1, w, 0.0), at(s0, w, 0.0),
                    {Vec2d(-w, u0), Vec2d(-w, u1), Vec2d(w, u1), Vec2d(w, u0)}, paving, up);
        writer.quad(at(s0, -w, under(s0)), at(s1, -w, under(s1)), at(s1, w, under(s1)),
                    at(s0, w, under(s0)),
                    {Vec2d(-w, u0), Vec2d(-w, u1), Vec2d(w, u1), Vec2d(w, u0)}, stone, -up);
        for (const double sgn : {-1.0, 1.0})
        {
            const double outer = sgn * (w + 0.3);
            const double inner = sgn * w;
            const Vec3d  out   = sideways * sgn;
            // Outer face: from the arch's underside to the parapet top.
            writer.quad(
                at(s0, outer, under(s0)), at(s1, outer, under(s1)), at(s1, outer, kBridgeParapetM),
                at(s0, outer, kBridgeParapetM),
                {Vec2d(u0, under(s0)), Vec2d(u1, under(s1)), Vec2d(u1, 1.0), Vec2d(u0, 1.0)}, stone,
                out);
            writer.quad(at(s0, inner, 0.0), at(s1, inner, 0.0), at(s1, inner, kBridgeParapetM),
                        at(s0, inner, kBridgeParapetM),
                        {Vec2d(u0, 0.0), Vec2d(u1, 0.0), Vec2d(u1, 1.0), Vec2d(u0, 1.0)}, stone,
                        -out);
            writer.quad(at(s0, inner, kBridgeParapetM), at(s1, inner, kBridgeParapetM),
                        at(s1, outer, kBridgeParapetM), at(s0, outer, kBridgeParapetM),
                        {Vec2d(u0, 0.0), Vec2d(u1, 0.0), Vec2d(u1, 0.3), Vec2d(u0, 0.3)}, stone,
                        up);
            writer.quad(at(s0, inner, under(s0)), at(s1, inner, under(s1)),
                        at(s1, outer, under(s1)), at(s0, outer, under(s0)),
                        {Vec2d(u0, 0.0), Vec2d(u1, 0.0), Vec2d(u1, 0.3), Vec2d(u0, 0.3)}, stone,
                        -up);
        }
    }
}

// ---- Colliders ---------------------------------------------------------------------------------

void addBox(StaticColliders& out, const BuildingFrame& frame, const Vec3d& centre,
            const Vec3d& half)
{
    out.boxes.push_back(
        {.centre = frame.toHabitat(centre), .orientation = frame.orientation, .halfExtents = half});
}

}  // namespace

std::uint32_t packFacade(const Facade& facade)
{
    return (facade.surface & 0xFU) | ((facade.colour & 0xFU) << 4U) |
           ((facade.windows & 0x3FU) << 8U) | (static_cast<std::uint32_t>(facade.door) << 14U) |
           (static_cast<std::uint32_t>(facade.shop) << 15U) |
           ((static_cast<std::uint32_t>(facade.style) & 0x3U) << 16U) |
           ((facade.shutter & 0x7U) << 18U) | ((facade.seed & 0x7FFU) << 21U);
}

Facade unpackFacade(std::uint32_t material)
{
    return {.surface = material & 0xFU,
            .colour  = (material >> 4U) & 0xFU,
            .windows = (material >> 8U) & 0x3FU,
            .door    = ((material >> 14U) & 1U) != 0,
            .shop    = ((material >> 15U) & 1U) != 0,
            .style   = static_cast<FacadeStyle>((material >> 16U) & 0x3U),
            .shutter = (material >> 18U) & 0x7U,
            .seed    = (material >> 21U) & 0x7FFU};
}

double storeyHeight(FacadeStyle style)
{
    switch (style)
    {
        case FacadeStyle::Tower:
            return 3.4;
        case FacadeStyle::Barn:
            return 6.0;
        case FacadeStyle::Hall:
            return 4.2;
        case FacadeStyle::House:
            break;
    }
    return 3.0;
}

std::vector<SettlementMesh> buildSettlementMeshes(const Settlements& settlements)
{
    std::vector<SettlementMesh> meshes(settlements.places.size());
    for (std::size_t i = 0; i < meshes.size(); ++i)
    {
        meshes[i].settlement = i;
        meshes[i].origin     = settlements.places[i].plane.point(Vec2d(0.0), 0.0);
    }
    for (const Building& b : settlements.buildings)
    {
        SettlementMesh&     target = meshes.at(b.settlement);
        const BuildingFrame frame =
            frameOf(settlements.places.at(b.settlement), b.centre, b.floorHeight, b.angle);
        MeshWriter writer(target.mesh, [&](const Vec3d& local) {
            return frame.toHabitat(local) - target.origin;
        });
        writeBuilding(writer, b);
    }
    SplitMix64 random(0xF0421);
    for (const Furniture& item : settlements.furniture)
    {
        SettlementMesh&     target = meshes.at(item.settlement);
        const BuildingFrame frame =
            frameOf(settlements.places.at(item.settlement), item.position, item.height, item.angle);
        MeshWriter writer(target.mesh, [&](const Vec3d& local) {
            return frame.toHabitat(local) - target.origin;
        });
        writeFurniture(writer, item, random);
    }
    for (const Bridge& bridge : settlements.bridges)
    {
        SettlementMesh&   target = meshes.at(bridge.settlement);
        const Settlement& place  = settlements.places.at(bridge.settlement);
        MeshWriter        writer(target.mesh, [&](const Vec3d& p) {
            return place.plane.point(Vec2d(p.x, p.y), p.z) - target.origin;
        });
        writeBridge(writer, bridge);
    }
    for (SettlementMesh& m : meshes)
    {
        Vec3f low(std::numeric_limits<float>::max());
        Vec3f high(std::numeric_limits<float>::lowest());
        for (const Vertex& v : m.mesh.vertices)
        {
            low  = glm::min(low, v.position);
            high = glm::max(high, v.position);
        }
        m.boundsMin = low;
        m.boundsMax = high;
    }
    std::erase_if(meshes, [](const SettlementMesh& m) { return m.mesh.indices.empty(); });
    return meshes;
}

StaticColliders settlementColliders(const Settlements& settlements)
{
    StaticColliders out;
    for (const Building& b : settlements.buildings)
    {
        const BuildingFrame frame =
            frameOf(settlements.places.at(b.settlement), b.centre, b.floorHeight, b.angle);
        const double top = b.wallHeight() + (b.roof == RoofKind::Flat ? kParapetM : 0.0);
        addBox(out, frame, Vec3d(0.0, 0.5 * (top - b.foundation), 0.0),
               Vec3d(b.halfSize.x, 0.5 * (top + b.foundation), b.halfSize.y));
        if (b.roof != RoofKind::Flat)
        {
            const RoofShape roof = roofShape(b);
            StaticHull hull{.origin = frame.base, .orientation = frame.orientation, .points = {}};
            for (const Vec3d& p : roof.points)
            {
                hull.points.emplace_back(p);
            }
            out.hulls.push_back(std::move(hull));
        }
    }
    for (const Furniture& item : settlements.furniture)
    {
        const BuildingFrame frame =
            frameOf(settlements.places.at(item.settlement), item.position, item.height, item.angle);
        switch (item.kind)
        {
            case FurnitureKind::Lamp:
                addBox(out, frame, Vec3d(0.0, 2.1, 0.0), Vec3d(0.12, 2.1, 0.12));
                break;
            case FurnitureKind::Bench:
                addBox(out, frame, Vec3d(0.0, 0.4, -0.05), Vec3d(0.8, 0.4, 0.25));
                break;
            case FurnitureKind::Planter:
                addBox(out, frame, Vec3d(0.0, 0.45, 0.0), Vec3d(0.6, 0.45, 0.3));
                break;
            case FurnitureKind::Stall:
                addBox(out, frame, Vec3d(0.0, 0.45, 0.0), Vec3d(1.2, 0.45, 0.45));
                break;
            case FurnitureKind::Fountain:
            {
                StaticHull basin{
                    .origin = frame.base, .orientation = frame.orientation, .points = {}};
                for (int i = 0; i < 16; ++i)
                {
                    const double a = 2.0 * kPi * i / 16.0;
                    for (const double y : {-0.3, 0.55})
                    {
                        basin.points.emplace_back(Vec3d(3.0 * std::cos(a), y, 3.0 * std::sin(a)));
                    }
                }
                out.hulls.push_back(std::move(basin));
                addBox(out, frame, Vec3d(0.0, 1.0, 0.0), Vec3d(0.3, 1.0, 0.3));
                break;
            }
        }
    }
    for (const Bridge& bridge : settlements.bridges)
    {
        const Settlement& place    = settlements.places.at(bridge.settlement);
        const Vec2d       dir      = glm::normalize(bridge.to - bridge.from);
        const double      angle    = std::atan2(dir.y, dir.x) - (0.5 * kPi);  // local x across
        const int         segments = bridgeSegments(bridge);
        const double      length   = glm::distance(bridge.from, bridge.to) / segments;
        for (int i = 0; i < segments; ++i)
        {
            const double  s0    = static_cast<double>(i) / segments;
            const double  s1    = static_cast<double>(i + 1) / segments;
            const double  h0    = deckHeight(bridge, s0);
            const double  h1    = deckHeight(bridge, s1);
            const Vec2d   mid   = glm::mix(bridge.from, bridge.to, 0.5 * (s0 + s1));
            const double  pitch = std::atan2(h1 - h0, length);
            BuildingFrame frame = frameOf(place, mid, 0.5 * (h0 + h1), angle);
            // Tilt the deck up along its length (local +z).
            frame.orientation =
                glm::normalize(frame.orientation * glm::angleAxis(-pitch, Vec3d(1.0, 0.0, 0.0)));
            const double half = 0.5 * length * 1.02;
            addBox(out, frame, Vec3d(0.0, -0.35, 0.0), Vec3d(bridge.halfWidth + 0.3, 0.35, half));
            for (const double side : {-1.0, 1.0})
            {
                addBox(out, frame, Vec3d(side * (bridge.halfWidth + 0.15), 0.5, 0.0),
                       Vec3d(0.15, 0.5, half));
            }
        }
    }
    return out;
}

}  // namespace StarshipSimulator
