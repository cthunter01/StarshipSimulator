#include "StarshipSimulator/core/procgen/hull_mesh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/mesh.h"

namespace StarshipSimulator
{

namespace
{

constexpr int kDomeRings   = 12;
constexpr int kAroundCells = 192;  // hull segments around the axis (window edges exact)

/// A point on the hull's outline in the (z, r) half-plane, with its outward normal.
struct OutlinePoint
{
    double z  = 0.0;
    double r  = 0.0;
    double nz = 0.0;  // outward normal, axial part
    double nr = 1.0;  // outward normal, radial part
};

/// One band of the outline to revolve (consecutive points share smooth normals).
struct Band
{
    std::vector<OutlinePoint> points;
    bool                      windows = false;  // the cylinder wall alongside the floor
};

/// The flat end wall (facing -1 or +1 along z) or a dome beyond it.
Band endBand(const EndcapSpec& endcap, double radius, double z, double facing)
{
    Band band;
    if (endcap.shape == EndcapShape::HEMISPHERE)
    {
        for (int i = 0; i <= kDomeRings; ++i)
        {
            const double a = (kPi / 2.0) * i / kDomeRings;  // 0 at the rim, pi/2 at the pole
            band.points.push_back({.z  = z + (facing * radius * std::sin(a)),
                                   .r  = radius * std::cos(a),
                                   .nz = facing * std::sin(a),
                                   .nr = std::cos(a)});
        }
    }
    else
    {
        band.points.push_back({.z = z, .r = radius, .nz = facing, .nr = 0.0});
        band.points.push_back({.z = z, .r = 0.0, .nz = facing, .nr = 0.0});
    }
    return band;
}

/// The outline from the anti-sunward end (-z) to the sunward end (+z), outward along r.
std::vector<Band> hullOutline(const HabitatGeometry& geometry)
{
    const HabitatSpec& spec   = geometry.spec();
    const double       radius = spec.radiusM;
    const double       half   = spec.lengthM / 2.0;
    const auto         wall   = [&](double z0, double z1, bool windows) {
        Band band;
        band.windows = windows;
        band.points  = {{.z = z0, .r = radius, .nz = 0.0, .nr = 1.0},
                        {.z = z1, .r = radius, .nz = 0.0, .nr = 1.0}};
        return band;
    };

    std::vector<Band> bands;
    Band              antisunward = endBand(spec.antisunwardEndcap, radius, -half, -1.0);
    std::ranges::reverse(antisunward.points);  // walk from the axis out to the rim
    bands.push_back(antisunward);
    if (geometry.floorZMin() > -half)
    {
        bands.push_back(wall(-half, geometry.floorZMin(), false));
    }
    bands.push_back(wall(geometry.floorZMin(), geometry.floorZMax(), true));
    if (geometry.floorZMax() < half)
    {
        bands.push_back(wall(geometry.floorZMax(), half, false));
    }
    bands.push_back(endBand(spec.sunwardEndcap, radius, half, 1.0));
    return bands;
}

Vertex hullVertex(const OutlinePoint& point, double angle, double radius, std::uint32_t materialId)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {.position = Vec3f(Vec3d(point.r * c, point.r * s, point.z)),
            .normal   = Vec3f(Vec3d(point.nr * c, point.nr * s, point.nz)),
            .uv       = Vec2f(Vec2d(angle * radius, point.z)),
            .material = materialId};
}

}  // namespace

CpuMesh buildHullMesh(const HabitatGeometry& geometry)
{
    const double    radius = geometry.radius();
    const AngleGrid grid   = buildAngleGrid(geometry, 2.0 * kPi * radius / kAroundCells);
    CpuMesh         mesh;
    for (const Band& band : hullOutline(geometry))
    {
        for (std::size_t p = 0; p + 1 < band.points.size(); ++p)
        {
            const OutlinePoint& a = band.points[p];
            const OutlinePoint& b = band.points[p + 1];
            for (std::size_t s = 0; s + 1 < grid.angles.size(); ++s)
            {
                const std::uint32_t id =
                    band.windows && grid.windowSegment[s] ? material::kGlass : material::kMetal;
                const double t0   = grid.angles[s];
                const double t1   = grid.angles[s + 1];
                const auto   base = static_cast<std::uint32_t>(mesh.vertices.size());
                mesh.vertices.push_back(hullVertex(a, t0, radius, id));
                mesh.vertices.push_back(hullVertex(a, t1, radius, id));
                mesh.vertices.push_back(hullVertex(b, t1, radius, id));
                mesh.vertices.push_back(hullVertex(b, t0, radius, id));
                // The outline runs from the anti-sunward axis round to the sunward axis and the
                // angles increase counter-clockwise about +z, so (a0, a1, b1) winds
                // counter-clockwise seen from outside. Triangles touching the axis are degenerate.
                for (const std::uint32_t i : {0U, 1U, 2U, 0U, 2U, 3U})
                {
                    mesh.indices.push_back(base + i);
                }
            }
        }
    }
    return mesh;
}

Mat4d partnerTransform(double separationM, double spinPhase)
{
    // Our frame has turned by +phase and the partner's by -phase from the inertial frame.
    const Mat4d unspin = glm::rotate(Mat4d(1.0), -spinPhase, Vec3d(0.0, 0.0, 1.0));
    return unspin * glm::translate(Mat4d(1.0), Vec3d(separationM, 0.0, 0.0)) * unspin;
}

}  // namespace StarshipSimulator
