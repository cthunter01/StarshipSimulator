#include "StarshipSimulator/core/procgen/enclosure_mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/habitat/Enclosure.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"
#include "StarshipSimulator/core/procgen/mesh.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kHullGapM    = 1.0;    // the hull's skin, this far outside the ceiling and walls
constexpr double kCollarM     = 4.0;    // the plate round a spoke's foot reaches this far past it
constexpr double kChunkArcM   = 240.0;  // of ceiling or hull, round the ring, to a chunk
constexpr double kSpokeChunkM = 100.0;  // of a spoke, along it
constexpr double kTubeRowDeg  = 2.5;    // between rows of the ceiling, round the tube
constexpr double kHullRowDeg  = 6.0;    // and of the hull, seen only from afar
constexpr int    kSpokeSides  = 24;
constexpr int kCollarSteps = 32;  // round a plate's hole: a multiple of 8, so it meets the corners
constexpr int kCollarRings = 3;
constexpr int kHubSides    = 72;
constexpr double kHubHoleM = 2.0;      // the plates round the spokes' holes in the hub's wall
constexpr double kSpokeOverlap = 1.0;  // a spoke reaches this far down into the tube: its collar

struct SurfacePoint
{
    Vec3d position{0.0};
    Vec3d normal{0.0, 0.0, 1.0};
    Vec2d uv{0.0};
};

Vec3d radial(double theta)
{
    return {std::cos(theta), std::sin(theta), 0.0};
}

void setBounds(MeshChunk& chunk)
{
    if (chunk.mesh.vertices.empty())
    {
        return;
    }
    Vec3f low  = chunk.mesh.vertices.front().position;
    Vec3f high = low;
    for (const Vertex& vertex : chunk.mesh.vertices)
    {
        low  = glm::min(low, vertex.position);
        high = glm::max(high, vertex.position);
    }
    chunk.boundsMin = low;
    chunk.boundsMax = high;
}

/// A grid of rows x columns vertices over a surface, surface(s, t) for s and t in [0, 1], its
/// triangles wound to face the way the surface's normals point.
template <typename Surface>
MeshChunk buildPatch(ChunkKind kind, std::uint32_t material, std::size_t rows, std::size_t columns,
                     const Surface& surface)
{
    MeshChunk chunk;
    chunk.kind   = kind;
    chunk.origin = surface(0.5, 0.5).position;
    chunk.mesh.vertices.reserve(rows * columns);
    for (std::size_t i = 0; i < rows; ++i)
    {
        const double s = static_cast<double>(i) / static_cast<double>(rows - 1);
        for (std::size_t k = 0; k < columns; ++k)
        {
            const double       t     = static_cast<double>(k) / static_cast<double>(columns - 1);
            const SurfacePoint point = surface(s, t);
            chunk.mesh.vertices.push_back({.position = Vec3f(point.position - chunk.origin),
                                           .normal   = Vec3f(point.normal),
                                           .uv       = Vec2f(point.uv),
                                           .material = material});
        }
    }
    // Counter-clockwise seen from the side the normals face.
    const SurfacePoint corner = surface(0.0, 0.0);
    const Vec3d        down   = surface(1.0 / static_cast<double>(rows - 1), 0.0).position;
    const Vec3d        across = surface(0.0, 1.0 / static_cast<double>(columns - 1)).position;
    const bool         flip =
        glm::dot(glm::cross(down - corner.position, across - corner.position), corner.normal) < 0.0;
    for (std::size_t i = 0; i + 1 < rows; ++i)
    {
        for (std::size_t k = 0; k + 1 < columns; ++k)
        {
            const auto a = static_cast<std::uint32_t>((i * columns) + k);
            const auto b = static_cast<std::uint32_t>(((i + 1) * columns) + k);
            const auto c = a + 1;
            const auto d = b + 1;
            if (flip)
            {
                chunk.mesh.indices.insert(chunk.mesh.indices.end(), {a, c, b, b, c, d});
            }
            else
            {
                chunk.mesh.indices.insert(chunk.mesh.indices.end(), {a, b, c, b, d, c});
            }
        }
    }
    setBounds(chunk);
    return chunk;
}

std::size_t stepsFor(double length, double step)
{
    return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(length / step)));
}

/// Splits [from, to] into pieces of about `size`, sharing their ends exactly.
std::vector<std::array<double, 2>> pieces(double from, double to, double size)
{
    std::vector<std::array<double, 2>> out;
    if (to <= from)
    {
        return out;
    }
    const std::size_t count = stepsFor(to - from, size);
    for (std::size_t i = 0; i < count; ++i)
    {
        out.push_back(
            {std::lerp(from, to, static_cast<double>(i) / static_cast<double>(count)),
             std::lerp(from, to, static_cast<double>(i + 1) / static_cast<double>(count))});
    }
    return out;
}

/// Builds the torus's surfaces into `chunks`.
class EnclosureMesher
{
public:
    EnclosureMesher(const TorusShape& shape, double cellSizeM, std::vector<MeshChunk>& chunks)
      : shape_(shape), cell_(cellSizeM), chunks_(&chunks)
    {
    }

    void build()
    {
        ceilingAndHull();
        for (int k = 0; k < shape_.spokes; ++k)
        {
            spoke(k);
        }
        hub();
    }

private:
    [[nodiscard]] double innerRadius() const { return shape_.centreRadiusM - shape_.tubeRadiusM; }
    [[nodiscard]] double plateHalf() const { return shape_.spokeRadiusM + kCollarM; }
    /// Half the plate round a spoke's foot, as an angle round the axis and round the tube.
    [[nodiscard]] double plateTheta() const { return plateHalf() / innerRadius(); }
    [[nodiscard]] double platePsi() const { return plateHalf() / shape_.tubeRadiusM; }

    /// A strip of the tube's surface: theta in [theta0, theta1] round the axis, psi in [psi0,
    /// psi1] round the tube, `offset` outside it, facing in (or out, for the hull).
    void tubeStrip(double theta0, double theta1, double psi0, double psi1, double offset,
                   bool outward, std::uint32_t material, ChunkKind kind, double rowDeg,
                   double columnM)
    {
        const double a       = shape_.tubeRadiusM + offset;
        const auto   rows    = stepsFor(psi1 - psi0, degreesToRadians(rowDeg)) + 1;
        const auto   columns = stepsFor((theta1 - theta0) * shape_.centreRadiusM, columnM) + 1;
        const double inner   = innerRadius();
        chunks_->push_back(buildPatch(kind, material, rows, columns, [&](double s, double t) {
            const double psi   = std::lerp(psi0, psi1, s);
            const double theta = std::lerp(theta0, theta1, t);
            const Vec3d  out   = (radial(theta) * std::cos(psi)) + Vec3d(0.0, 0.0, std::sin(psi));
            // Metres: round the ring at the ceiling's innermost line, and round the tube.
            return SurfacePoint{.position = shape_.tubePoint(theta, psi, offset),
                                .normal   = outward ? out : -out,
                                .uv       = Vec2d(theta * inner, a * (psi - kPi))};
        }));
    }

    /// The ceiling (metal where it is not window) and, outside it all, the hull.
    void ceilingAndHull()
    {
        const double window = shape_.windowHalfAngle;
        const double metal0 = kPi / 2.0;
        const double metal1 = (3.0 * kPi) / 2.0;
        for (const auto& [theta0, theta1] :
             pieces(0.0, 2.0 * kPi, kChunkArcM / shape_.centreRadiusM))
        {
            if (kPi - window > metal0)
            {
                tubeStrip(theta0, theta1, metal0, kPi - window, 0.0, false, material::kMetal,
                          ChunkKind::TERRAIN, kTubeRowDeg, cell_);
                tubeStrip(theta0, theta1, kPi + window, metal1, 0.0, false, material::kMetal,
                          ChunkKind::TERRAIN, kTubeRowDeg, cell_);
            }
            // The hull: all the way round the tube but for the windows, and a lip at their edges.
            tubeStrip(theta0, theta1, window - kPi, kPi - window, kHullGapM, true, material::kHull,
                      ChunkKind::TERRAIN, kHullRowDeg, 3.0 * cell_);
            lip(theta0, theta1, kPi - window, 1.0);
            lip(theta0, theta1, window - kPi, -1.0);
        }
        glass();
    }

    /// Where the hull meets the glass: a narrow band from the ceiling out to the hull, facing the
    /// windows.
    void lip(double theta0, double theta1, double psi, double facing)
    {
        const auto columns = stepsFor((theta1 - theta0) * shape_.centreRadiusM, 3.0 * cell_) + 1;
        chunks_->push_back(
            buildPatch(ChunkKind::TERRAIN, material::kHull, 2, columns, [&](double s, double t) {
                const double theta = std::lerp(theta0, theta1, t);
                const Vec3d  along =
                    (radial(theta) * -std::sin(psi)) + Vec3d(0.0, 0.0, std::cos(psi));
                return SurfacePoint{.position = shape_.tubePoint(theta, psi, s * kHullGapM),
                                    .normal   = along * facing,
                                    .uv       = Vec2d(theta * innerRadius(), s)};
            }));
    }

    /// The windows round the ceiling's innermost line, with a plate round each spoke's foot.
    void glass()
    {
        const double window = shape_.windowHalfAngle;
        if (window <= 0.0)
        {
            return;
        }
        const auto pane = [&](double theta0, double theta1, double psi0, double psi1) {
            for (const auto& [from, to] : pieces(theta0, theta1, kChunkArcM / innerRadius()))
            {
                tubeStrip(from, to, psi0, psi1, 0.0, false, material::kGlass, ChunkKind::GLASS,
                          kTubeRowDeg, cell_);
            }
        };
        if (shape_.spokes == 0)
        {
            pane(0.0, 2.0 * kPi, kPi - window, kPi + window);
            return;
        }
        const double across = plateTheta();
        const double up     = std::min(platePsi(), window);
        for (int k = 0; k < shape_.spokes; ++k)
        {
            const double spoke = shape_.spokeAngle(k);
            const double next  = shape_.spokeAngle(k + 1);
            pane(spoke + across, next - across, kPi - window, kPi + window);
            // Beside the plate, toward either side of the tube.
            pane(spoke - across, spoke + across, kPi - window, kPi - up);
            pane(spoke - across, spoke + across, kPi + up, kPi + window);
            collar(spoke);
        }
    }

    /// A square plate round the spoke's foot, with the spoke's round hole through it; and its back,
    /// part of the hull.
    void collar(double spoke)
    {
        const double inner = innerRadius();
        const double a     = shape_.tubeRadiusM;
        for (const bool hull : {false, true})
        {
            const double offset = hull ? kHullGapM : 0.0;
            chunks_->push_back(buildPatch(
                ChunkKind::TERRAIN, hull ? material::kHull : material::kMetal, kCollarRings + 1,
                kCollarSteps + 1, [&](double s, double t) {
                    // From the hole's edge out to the plate's square edge, in plan metres: u round
                    // the ring, v round the tube.
                    const double beta = 2.0 * kPi * t;
                    const Vec2d  dir(std::cos(beta), std::sin(beta));
                    const double edge  = plateHalf() / std::max(std::abs(dir.x), std::abs(dir.y));
                    const Vec2d  plan  = dir * std::lerp(shape_.spokeRadiusM, edge, s);
                    const double theta = spoke + (plan.x / inner);
                    const double psi   = kPi + (plan.y / a);
                    const Vec3d  out =
                        (radial(theta) * std::cos(psi)) + Vec3d(0.0, 0.0, std::sin(psi));
                    return SurfacePoint{.position = shape_.tubePoint(theta, psi, offset),
                                        .normal   = hull ? out : -out,
                                        .uv       = plan};
                }));
        }
    }

    /// A spoke: a tube along the radius at its angle, from the hub's wall to just inside the
    /// torus's; its inside and its outside.
    void spoke(int k)
    {
        const double angle  = shape_.spokeAngle(k);
        const Vec3d  out    = radial(angle);
        const Vec3d  across = radial(angle + (kPi / 2.0));
        const double foot   = innerRadius() + kSpokeOverlap;
        for (const bool hull : {false, true})
        {
            const double radius = shape_.spokeRadiusM + (hull ? 0.5 : 0.0);
            // Inside, from a little within the hub's wall (whose curve would leave a gap round it).
            const double from = shape_.hubRadiusM + (hull ? 0.5 : -kSpokeOverlap);
            const double to   = hull ? innerRadius() - kHullGapM : foot;
            for (const auto& [r0, r1] : pieces(from, to, kSpokeChunkM))
            {
                chunks_->push_back(
                    buildPatch(ChunkKind::TERRAIN, hull ? material::kHull : material::kMetal, 2,
                               kSpokeSides + 1, [&](double s, double t) {
                                   const double beta = 2.0 * kPi * t;
                                   const Vec3d  side =
                                       (across * std::cos(beta)) + Vec3d(0.0, 0.0, std::sin(beta));
                                   const double r = std::lerp(r0, r1, s);
                                   return SurfacePoint{.position = (out * r) + (side * radius),
                                                       .normal   = hull ? side : -side,
                                                       .uv       = Vec2d(beta * radius, r)};
                               }));
            }
        }
    }

    /// The hub: a drum round the axis, its wall the floor of the low-gravity rooms, the spokes
    /// coming in through holes in it; and its outside.
    void hub()
    {
        const double h    = shape_.hubRadiusM;
        const double half = shape_.hubHalfLengthM;
        const auto   wall = [&](double theta0, double theta1, double z0, double z1) {
            const auto columns = stepsFor((theta1 - theta0) * kHubSides / (2.0 * kPi), 1.0) + 1;
            const auto rows    = stepsFor(z1 - z0, 4.0 * cell_) + 1;
            chunks_->push_back(buildPatch(
                ChunkKind::TERRAIN, material::kMetal, rows, columns, [&](double s, double t) {
                    const double theta = std::lerp(theta0, theta1, t);
                    const double z     = std::lerp(z0, z1, s);
                    return SurfacePoint{.position = (radial(theta) * h) + Vec3d(0.0, 0.0, z),
                                        .normal   = -radial(theta),
                                        .uv       = Vec2d(theta * h, z)};
                }));
        };
        const double hole = shape_.spokeRadiusM + kHubHoleM;
        if (shape_.spokes == 0)
        {
            wall(0.0, 2.0 * kPi, -half, half);
        }
        for (int k = 0; k < shape_.spokes; ++k)
        {
            const double spoke = shape_.spokeAngle(k);
            const double next  = shape_.spokeAngle(k + 1);
            const double wide  = hole / h;
            wall(spoke + wide, next - wide, -half, half);
            wall(spoke - wide, spoke + wide, -half, -hole);
            wall(spoke - wide, spoke + wide, hole, half);
            chunks_->push_back(buildPatch(
                ChunkKind::TERRAIN, material::kMetal, kCollarRings + 1, kCollarSteps + 1,
                [&](double s, double t) {
                    const double beta = 2.0 * kPi * t;
                    const Vec2d  dir(std::cos(beta), std::sin(beta));
                    const double edge  = hole / std::max(std::abs(dir.x), std::abs(dir.y));
                    const Vec2d  plan  = dir * std::lerp(shape_.spokeRadiusM, edge, s);
                    const double theta = spoke + (plan.x / h);
                    return SurfacePoint{.position = (radial(theta) * h) + Vec3d(0.0, 0.0, plan.y),
                                        .normal   = -radial(theta),
                                        .uv       = Vec2d(theta * h, plan.y)};
                }));
        }
        // Its two ends, and the outside: wall and ends.
        for (const double side : {-1.0, 1.0})
        {
            disk(side * half, h, -side, material::kMetal);
            disk(side * (half + 0.5), h + 0.5, side, material::kHull);
        }
        chunks_->push_back(buildPatch(
            ChunkKind::TERRAIN, material::kHull, 2, kHubSides + 1, [&](double s, double t) {
                const double theta = 2.0 * kPi * t;
                const double z     = std::lerp(-half - 0.5, half + 0.5, s);
                return SurfacePoint{.position = (radial(theta) * (h + 0.5)) + Vec3d(0.0, 0.0, z),
                                    .normal   = radial(theta),
                                    .uv       = Vec2d(theta * h, z)};
            }));
    }

    /// A flat disk square to the axis at z, facing +Z (facing > 0) or -Z.
    void disk(double z, double radius, double facing, std::uint32_t material)
    {
        const auto rings = stepsFor(radius, 4.0 * cell_) + 1;
        chunks_->push_back(
            buildPatch(ChunkKind::TERRAIN, material, rings, kHubSides + 1, [&](double s, double t) {
                const double theta = 2.0 * kPi * t;
                // Not quite to the axis, so no triangle there has no area.
                const double r = radius * std::max(s, 0.02);
                return SurfacePoint{.position = (radial(theta) * r) + Vec3d(0.0, 0.0, z),
                                    .normal   = Vec3d(0.0, 0.0, facing),
                                    .uv       = Vec2d(theta * radius, r)};
            }));
    }

    TorusShape              shape_;
    double                  cell_;
    std::vector<MeshChunk>* chunks_;
};

}  // namespace

StaticColliders enclosureColliders(const HabitatGeometry&        geometry,
                                   const std::vector<MeshChunk>& chunks)
{
    StaticColliders colliders;
    if (!geometry.enclosure().torus())
    {
        return colliders;
    }
    for (const MeshChunk& chunk : chunks)
    {
        if (chunk.mesh.vertices.empty() || chunk.mesh.vertices.front().material == material::kHull)
        {
            continue;
        }
        StaticMesh& mesh = colliders.meshes.emplace_back();
        mesh.origin      = chunk.origin;
        mesh.vertices.reserve(chunk.mesh.vertices.size());
        for (const Vertex& vertex : chunk.mesh.vertices)
        {
            mesh.vertices.push_back(vertex.position);
        }
        mesh.indices = chunk.mesh.indices;
    }
    return colliders;
}

std::vector<MeshChunk> buildEnclosureMeshes(const HabitatGeometry& geometry, double cellSizeM)
{
    std::vector<MeshChunk> chunks;
    if (const auto& torus = geometry.enclosure().torus())
    {
        EnclosureMesher(*torus, cellSizeM, chunks).build();
    }
    return chunks;
}

}  // namespace StarshipSimulator
