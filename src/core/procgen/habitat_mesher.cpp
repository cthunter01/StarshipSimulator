#include "StarshipSimulator/core/procgen/habitat_mesher.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <thread>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/meridian_profile.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"

namespace StarshipSimulator
{

namespace
{

enum class JobKind : std::uint8_t
{
    Terrain,
    Glass,
    Disk,
};

/// One chunk to build. Terrain: vertex rows [row0, row1] x segments [seg0, seg1).
/// Glass: z in [z0, z1] x segments. Disk: a flat disk at z0 with radius z1 facing +-Z.
struct Job
{
    JobKind       kind     = JobKind::Terrain;
    std::uint32_t material = material::kValley;
    std::size_t   row0     = 0;
    std::size_t   row1     = 0;
    std::size_t   seg0     = 0;
    std::size_t   seg1     = 0;
    double        z0       = 0.0;
    double        z1       = 0.0;
    double        facing   = 1.0;
};

Job terrainJob(std::uint32_t materialId, std::size_t row0, std::size_t row1, std::size_t seg0,
               std::size_t seg1)
{
    Job job;
    job.kind     = JobKind::Terrain;
    job.material = materialId;
    job.row0     = row0;
    job.row1     = row1;
    job.seg0     = seg0;
    job.seg1     = seg1;
    return job;
}

Job glassJob(std::size_t seg0, std::size_t seg1, double z0, double z1)
{
    Job job;
    job.kind     = JobKind::Glass;
    job.material = material::kGlass;
    job.seg0     = seg0;
    job.seg1     = seg1;
    job.z0       = z0;
    job.z1       = z1;
    return job;
}

Job diskJob(std::uint32_t materialId, double z, double radius, double facing)
{
    Job job;
    job.kind     = JobKind::Disk;
    job.material = materialId;
    job.z0       = z;
    job.z1       = radius;
    job.facing   = facing;
    return job;
}

/// Everything the chunk builders share (read-only while building).
struct Grid
{
    const HabitatGeometry* geometry = nullptr;
    std::vector<double>    rows;  // arc length along the profile of each vertex row
    AngleGrid              angles;

    [[nodiscard]] std::size_t segmentCount() const { return angles.windowSegment.size(); }

    /// Angle of grid column k, for any integer k (wraps around by whole turns).
    [[nodiscard]] double angle(std::ptrdiff_t k) const
    {
        const auto n     = static_cast<std::ptrdiff_t>(segmentCount());
        const auto turns = (k >= 0 ? k : k - n + 1) / n;
        const auto index = static_cast<std::size_t>(k - (turns * n));
        return angles.angles[index] + (2.0 * kPi * static_cast<double>(turns));
    }
};

std::vector<double> buildRows(const MeridianProfile& profile, double cellSize)
{
    std::vector<double> rows;
    const auto&         points = profile.points();
    for (std::size_t i = 0; i + 1 < points.size(); ++i)
    {
        const double length = points[i + 1].u - points[i].u;
        const int    n      = std::max(1, static_cast<int>(std::ceil(length / cellSize)));
        for (int j = 0; j < n; ++j)
        {
            rows.push_back(points[i].u + (length * j / n));
        }
    }
    rows.push_back(profile.length());
    return rows;
}

std::size_t rowIndexAt(const std::vector<double>& rows, double u)
{
    const auto closest = std::ranges::min_element(
        rows, [u](double a, double b) { return std::abs(a - u) < std::abs(b - u); });
    return static_cast<std::size_t>(std::distance(rows.begin(), closest));
}

Vec3d radial(double theta)
{
    return {std::cos(theta), std::sin(theta), 0.0};
}

Vec3d terrainPoint(const HabitatGeometry& geometry, double u, double theta)
{
    const Vec2d  zr = geometry.profile().pointAt(u);
    const double r  = std::max(0.0, zr.y - geometry.terrainHeight(zr.x, theta));
    return {r * std::cos(theta), r * std::sin(theta), zr.x};
}

void appendQuadIndices(std::vector<std::uint32_t>& indices, std::uint32_t a, std::uint32_t b,
                       std::uint32_t c, std::uint32_t d, bool flip)
{
    // a = (i, k), b = (i + 1, k), c = (i, k + 1), d = (i + 1, k + 1); counter-clockwise seen from
    // the side the normal points to.
    if (flip)
    {
        indices.insert(indices.end(), {a, c, b, b, c, d});
    }
    else
    {
        indices.insert(indices.end(), {a, b, c, b, d, c});
    }
}

void finish(MeshChunk& chunk)
{
    Vec3f low(0.0F);
    Vec3f high(0.0F);
    if (!chunk.mesh.vertices.empty())
    {
        low  = chunk.mesh.vertices.front().position;
        high = low;
    }
    for (const Vertex& vertex : chunk.mesh.vertices)
    {
        low  = glm::min(low, vertex.position);
        high = glm::max(high, vertex.position);
    }
    chunk.boundsMin = low;
    chunk.boundsMax = high;
}

/// Adds a quad-grid of (rows x columns) vertices' triangles to the mesh.
void appendGridIndices(CpuMesh& mesh, std::size_t rows, std::size_t columns, bool flip)
{
    for (std::size_t i = 0; i + 1 < rows; ++i)
    {
        for (std::size_t k = 0; k + 1 < columns; ++k)
        {
            const auto at = [columns](std::size_t row, std::size_t column) {
                return static_cast<std::uint32_t>((row * columns) + column);
            };
            appendQuadIndices(mesh.indices, at(i, k), at(i + 1, k), at(i, k + 1), at(i + 1, k + 1),
                              flip);
        }
    }
}

MeshChunk buildTerrainChunk(const Grid& grid, const Job& job)
{
    const HabitatGeometry& geometry = *grid.geometry;
    const double           hullR    = geometry.radius();
    const std::size_t      rows     = job.row1 - job.row0 + 1;
    const std::size_t      columns  = job.seg1 - job.seg0 + 1;

    // Positions with a one-vertex border, for normals by central differences.
    const std::size_t  paddedColumns = columns + 2;
    std::vector<Vec3d> padded((rows + 2) * paddedColumns);
    for (std::size_t i = 0; i < rows + 2; ++i)
    {
        const std::size_t row = std::clamp<std::size_t>(job.row0 + i, 1, grid.rows.size()) -
                                1;  // row0 - 1 + i, clamped
        for (std::size_t k = 0; k < paddedColumns; ++k)
        {
            const double theta = grid.angle(static_cast<std::ptrdiff_t>(job.seg0 + k) - 1);
            padded[(i * paddedColumns) + k] = terrainPoint(geometry, grid.rows[row], theta);
        }
    }

    MeshChunk chunk;
    chunk.kind                 = ChunkKind::Terrain;
    const std::size_t midRow   = (job.row0 + job.row1) / 2;
    const double      midTheta = grid.angle(static_cast<std::ptrdiff_t>((job.seg0 + job.seg1) / 2));
    const Vec2d       midZr    = geometry.profile().pointAt(grid.rows[midRow]);
    chunk.origin               = (radial(midTheta) * midZr.y) + Vec3d(0.0, 0.0, midZr.x);

    chunk.mesh.vertices.reserve(rows * columns);
    for (std::size_t i = 0; i < rows; ++i)
    {
        const double u       = grid.rows[job.row0 + i];
        const Vec2d  inward2 = geometry.profile().inwardNormalAt(u);
        for (std::size_t k = 0; k < columns; ++k)
        {
            const std::size_t p     = ((i + 1) * paddedColumns) + (k + 1);
            const double      theta = grid.angle(static_cast<std::ptrdiff_t>(job.seg0 + k));
            const Vec3d       fallback =
                (radial(theta) * inward2.y) + Vec3d(0.0, 0.0, inward2.x);  // profile normal
            const Vec3d alongU   = padded[p + paddedColumns] - padded[p - paddedColumns];
            const Vec3d alongArc = padded[p + 1] - padded[p - 1];
            Vec3d       normal   = glm::cross(alongU, alongArc);
            normal = glm::dot(normal, normal) > 1e-12 ? glm::normalize(normal) : fallback;
            if (glm::dot(normal, fallback) < 0.0)
            {
                normal = -normal;
            }
            chunk.mesh.vertices.push_back({.position = Vec3f(padded[p] - chunk.origin),
                                           .normal   = Vec3f(normal),
                                           .uv       = Vec2f(Vec2d(theta * hullR, u)),
                                           .material = job.material});
        }
    }
    appendGridIndices(chunk.mesh, rows, columns, false);
    finish(chunk);
    return chunk;
}

MeshChunk buildGlassChunk(const Grid& grid, const Job& job, double cellSize)
{
    const double      radius  = grid.geometry->radius();
    const std::size_t columns = job.seg1 - job.seg0 + 1;
    const auto        steps =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil((job.z1 - job.z0) / cellSize)));

    MeshChunk chunk;
    chunk.kind            = ChunkKind::Glass;
    const double midTheta = grid.angle(static_cast<std::ptrdiff_t>((job.seg0 + job.seg1) / 2));
    chunk.origin          = (radial(midTheta) * radius) + Vec3d(0.0, 0.0, 0.5 * (job.z0 + job.z1));
    chunk.mesh.vertices.reserve((steps + 1) * columns);
    for (std::size_t i = 0; i <= steps; ++i)
    {
        const double z =
            std::lerp(job.z0, job.z1, static_cast<double>(i) / static_cast<double>(steps));
        for (std::size_t k = 0; k < columns; ++k)
        {
            const double theta = grid.angle(static_cast<std::ptrdiff_t>(job.seg0 + k));
            const Vec3d  point = (radial(theta) * radius) + Vec3d(0.0, 0.0, z);
            chunk.mesh.vertices.push_back({.position = Vec3f(point - chunk.origin),
                                           .normal   = Vec3f(-radial(theta)),
                                           .uv       = Vec2f(Vec2d(theta * radius, z)),
                                           .material = material::kGlass});
        }
    }
    appendGridIndices(chunk.mesh, steps + 1, columns, false);
    finish(chunk);
    return chunk;
}

MeshChunk buildDiskChunk(const Grid& grid, const Job& job, double cellSize)
{
    const double      radius  = job.z1;
    const double      hullR   = grid.geometry->radius();
    const std::size_t columns = grid.segmentCount() + 1;
    const auto        rings =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(radius / cellSize)));

    MeshChunk chunk;
    chunk.kind   = ChunkKind::Terrain;
    chunk.origin = Vec3d(0.0, 0.0, job.z0);
    chunk.mesh.vertices.reserve((rings + 1) * columns);
    for (std::size_t i = 0; i <= rings; ++i)
    {
        const double r = radius * static_cast<double>(i) / static_cast<double>(rings);
        for (std::size_t k = 0; k < columns; ++k)
        {
            const double theta = grid.angle(static_cast<std::ptrdiff_t>(k));
            chunk.mesh.vertices.push_back(
                {.position = Vec3f(radial(theta) * r),
                 .normal   = Vec3f(0.0F, 0.0F, static_cast<float>(job.facing)),
                 .uv       = Vec2f(Vec2d(theta * hullR, r)),
                 .material = job.material});
        }
    }
    // Rows go outward and columns counter-clockwise, so the unflipped winding faces +Z.
    appendGridIndices(chunk.mesh, rings + 1, columns, job.facing < 0.0);
    finish(chunk);
    return chunk;
}

/// Splits [first, last] into pieces of about `size`, as (start, end) pairs sharing boundaries.
std::vector<std::pair<std::size_t, std::size_t>> split(std::size_t first, std::size_t last,
                                                       std::size_t size)
{
    std::vector<std::pair<std::size_t, std::size_t>> pieces;
    if (last <= first)
    {
        return pieces;
    }
    const std::size_t count = std::max<std::size_t>(1, ((last - first) + (size / 2)) / size);
    for (std::size_t i = 0; i < count; ++i)
    {
        pieces.emplace_back(first + (((last - first) * i) / count),
                            first + (((last - first) * (i + 1)) / count));
    }
    return pieces;
}

void addEndDisks(const HabitatGeometry& geometry, std::vector<Job>& jobs)
{
    const OneillCylinderSpec& spec    = geometry.spec();
    const MeridianProfile&    profile = geometry.profile();
    const auto                add     = [&](const EndcapSpec& endcap, double z, double facing) {
        if (endcap.shape == EndcapShape::ConicalRamp)
        {
            jobs.push_back(diskJob(material::kMetal, z, endcap.hubRadiusM, facing));
        }
        else if (endcap.shape == EndcapShape::Flat)
        {
            jobs.push_back(diskJob(material::kEndcap, z, spec.radiusM, facing));
        }
    };
    add(spec.antisunwardEndcap, profile.zMin(), 1.0);
    add(spec.sunwardEndcap, profile.zMax(), -1.0);
}

std::vector<Job> planJobs(const HabitatGeometry& geometry, const Grid& grid,
                          const MeshingSettings& settings)
{
    const MeridianProfile& profile    = geometry.profile();
    const std::size_t      floorFrom  = rowIndexAt(grid.rows, profile.arcAt(geometry.floorZMin()));
    const std::size_t      floorTo    = rowIndexAt(grid.rows, profile.arcAt(geometry.floorZMax()));
    const auto             chunkCells = std::max<std::size_t>(
        1, static_cast<std::size_t>(settings.chunkSizeM / settings.cellSizeM));

    // Row ranges: endcap / floor / endcap, each split into chunk-sized pieces.
    struct RowRange
    {
        std::size_t first;
        std::size_t last;
        bool        floor;
    };
    std::vector<RowRange> rowRanges;
    const std::size_t     lastRow = grid.rows.size() - 1;
    for (const auto& [first, last, floor] :
         {RowRange{.first = 0, .last = floorFrom, .floor = false},
          RowRange{.first = floorFrom, .last = floorTo, .floor = true},
          RowRange{.first = floorTo, .last = lastRow, .floor = false}})
    {
        for (const auto& [a, b] : split(first, last, chunkCells))
        {
            rowRanges.push_back({.first = a, .last = b, .floor = floor});
        }
    }

    // Segment runs: alternating window and land strips, each split into chunk-sized pieces.
    struct SegmentRange
    {
        std::size_t first;
        std::size_t last;
        bool        window;
    };
    std::vector<SegmentRange> segmentRanges;
    const std::size_t         segments = grid.segmentCount();
    std::size_t               runStart = 0;
    for (std::size_t k = 1; k <= segments; ++k)
    {
        if (k == segments || grid.angles.windowSegment[k] != grid.angles.windowSegment[runStart])
        {
            for (const auto& [a, b] : split(runStart, k, chunkCells))
            {
                segmentRanges.push_back(
                    {.first = a, .last = b, .window = grid.angles.windowSegment[runStart]});
            }
            runStart = k;
        }
    }

    std::vector<Job> jobs;
    for (const RowRange& rows : rowRanges)
    {
        for (const SegmentRange& segs : segmentRanges)
        {
            if (rows.floor && segs.window)
            {
                const double z0 = profile.pointAt(grid.rows[rows.first]).x;
                const double z1 = profile.pointAt(grid.rows[rows.last]).x;
                jobs.push_back(glassJob(segs.first, segs.last, z0, z1));
            }
            else
            {
                jobs.push_back(terrainJob(rows.floor ? material::kValley : material::kEndcap,
                                          rows.first, rows.last, segs.first, segs.last));
            }
        }
    }
    addEndDisks(geometry, jobs);
    return jobs;
}

}  // namespace

AngleGrid buildAngleGrid(const HabitatGeometry& geometry, double cellSizeM)
{
    const double radius    = geometry.radius();
    const double windowArc = 2.0 * geometry.windowHalfAngle();
    const double landArc   = 2.0 * geometry.landHalfAngle();
    const auto   cellsFor  = [&](double arc) {
        return std::max(1, static_cast<int>(std::lround(arc * radius / cellSizeM)));
    };
    const int windowCells = cellsFor(windowArc);
    const int landCells   = cellsFor(landArc);

    // Start at the leading edge of window 0 and go once around.
    const double start = -geometry.windowHalfAngle();
    AngleGrid    grid;
    for (int strip = 0; strip < geometry.stripCount(); ++strip)
    {
        const double windowStart = start + (strip * geometry.stripAngle());
        for (int i = 0; i < windowCells; ++i)
        {
            grid.angles.push_back(windowStart + (windowArc * i / windowCells));
            grid.windowSegment.push_back(true);
        }
        const double landStart = windowStart + windowArc;
        for (int i = 0; i < landCells; ++i)
        {
            grid.angles.push_back(landStart + (landArc * i / landCells));
            grid.windowSegment.push_back(false);
        }
    }
    grid.angles.push_back(start + (2.0 * kPi));
    return grid;
}

HabitatMeshes buildHabitatMeshes(const HabitatGeometry& geometry, const MeshingSettings& settings)
{
    Grid grid;
    grid.geometry = &geometry;
    grid.rows     = buildRows(geometry.profile(), settings.cellSizeM);
    grid.angles   = buildAngleGrid(geometry, settings.cellSizeM);

    const std::vector<Job>   jobs = planJobs(geometry, grid, settings);
    std::vector<MeshChunk>   chunks(jobs.size());
    std::atomic<std::size_t> next{0};
    const auto               work = [&]() {
        for (std::size_t i = next++; i < jobs.size(); i = next++)
        {
            const Job& job = jobs[i];
            switch (job.kind)
            {
                case JobKind::Terrain:
                    chunks[i] = buildTerrainChunk(grid, job);
                    break;
                case JobKind::Glass:
                    chunks[i] = buildGlassChunk(grid, job, settings.glassCellSizeM);
                    break;
                case JobKind::Disk:
                    chunks[i] = buildDiskChunk(grid, job, settings.cellSizeM);
                    break;
            }
        }
    };
    const unsigned threads =
        settings.threads > 0 ? settings.threads : std::max(1U, std::thread::hardware_concurrency());
    {
        std::vector<std::jthread> workers;
        for (unsigned t = 1; t < threads; ++t)
        {
            workers.emplace_back(work);
        }
        work();
    }

    HabitatMeshes meshes;
    meshes.chunks = std::move(chunks);
    for (const MeshChunk& chunk : meshes.chunks)
    {
        meshes.vertexCount += chunk.mesh.vertices.size();
        meshes.triangleCount += chunk.mesh.indices.size() / 3;
    }
    return meshes;
}

}  // namespace StarshipSimulator
