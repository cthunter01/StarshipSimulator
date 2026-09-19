#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"

namespace StarshipSimulator
{

/// Vertex material ids (Vertex::material), interpreted by the habitat shaders.
namespace material
{
inline constexpr std::uint32_t kValley = 0;  // valley floor land
inline constexpr std::uint32_t kEndcap = 1;  // endcap slopes and domes
inline constexpr std::uint32_t kGlass  = 2;  // window strips
inline constexpr std::uint32_t kMetal  = 3;  // hubs and bare structure
}  // namespace material

enum class ChunkKind : std::uint8_t
{
    Terrain,  // opaque ground
    Glass,    // window strips, drawn transparent after everything else
};

/// A piece of the habitat surface. Vertices are float and relative to a double-precision origin.
struct MeshChunk
{
    ChunkKind kind = ChunkKind::Terrain;
    Vec3d     origin{0.0};
    CpuMesh   mesh;
    Vec3f     boundsMin{0.0F};  // local, relative to origin
    Vec3f     boundsMax{0.0F};
};

struct MeshingSettings
{
    double   cellSizeM      = 25.0;    // terrain grid spacing
    double   chunkSizeM     = 1000.0;  // approximate chunk edge length
    double   glassCellSizeM = 500.0;   // along the axis; glass is flat
    unsigned threads        = 0;       // 0: one per hardware thread
    bool     terrain        = true;    // false: glass and end walls only (the GPU draws the land)
};

struct HabitatMeshes
{
    std::vector<MeshChunk> chunks;
    std::size_t            vertexCount   = 0;
    std::size_t            triangleCount = 0;
};

/// Angles around the axis shared by all chunks, so neighbouring chunks meet exactly and every
/// land/window boundary lies on a grid line. angles.size() == windowSegment.size() + 1.
struct AngleGrid
{
    std::vector<double> angles;         // increasing, spanning exactly 2*pi
    std::vector<bool>   windowSegment;  // per segment: inside a window strip
};

[[nodiscard]] AngleGrid buildAngleGrid(const HabitatGeometry& geometry, double cellSizeM);

/// Generates the habitat surface: terrain chunks (valley floors, endcaps, hubs) and glass chunks.
/// Deterministic for a given geometry and settings (except for the thread count, which does not
/// change the result).
[[nodiscard]] HabitatMeshes buildHabitatMeshes(const HabitatGeometry& geometry,
                                               const MeshingSettings& settings);

}  // namespace StarshipSimulator
