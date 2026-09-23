#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

// The tramway: one line down each valley, calling at the towns, on a track that follows the ground
// and crosses the river on a low viaduct; where the land runs around the axis instead, one line
// once round it. Where the trams are follows from the clock, like the weather and the birds, so
// nothing has to be simulated or saved.
namespace StarshipSimulator
{

/// A stop: a platform beside the track, named after the place it serves.
struct TramStop
{
    std::string name;
    double      alongM = 0.0;  // how far along the line it is
    Vec3d       position{0.0};
    double      dwellS = 14.0;  // how long a tram waits there
};

/// A point on the track: where the rails run, how high they sit, and whether the ground is under
/// them at all.
struct TrackPoint
{
    Vec3d  position{0.0};  // on the rail head
    double alongM = 0.0;
    double railHeightM =
        0.0;  // of the rail head above the meridian profile (radius = profile - it)
    double aboveGroundM = 0.0;    // how far the formation stands above the land as it was found
    bool   carried      = false;  // on a trestle or a viaduct, with nothing but air underneath
    double planM = 0.0;  // where on the line's own plan: z down a valley, metres round a loop
};

enum class LineKind : std::uint8_t
{
    VALLEY,  // a tramway down the valley floor, calling at the towns
    ENDCAP,  // a funicular up the endcap's ramp to the hub, where there is no gravity left
    LOOP,    // a tramway once round a band of land that runs around the axis, always one way
};

/// One line, running the length of a valley (or up an endcap) and back, or round a loop.
struct TramLine
{
    LineKind kind    = LineKind::VALLEY;
    int      valley  = 0;    // the band of land it serves
    double   theta   = 0.0;  // the line runs along the axis at this angle (a loop: where it starts)
    double   z       = 0.0;  // a loop runs around the axis at this z
    double   radiusM = 1.0;  // of the floor it runs on, for arc lengths
    std::vector<TrackPoint> track;  // evenly spaced along the line
    std::vector<TramStop>   stops;
    double                  lengthM  = 0.0;
    double                  topSpeed = 16.0;  // m/s between stops
    int                     trams    = 3;     // sharing the line, spread over the timetable
    double                  journeyS = 0.0;   // one end to the other, stops included
};

/// Where a tram is now.
struct Tram
{
    std::size_t line = 0;
    Vec3d       position{0.0};  // on the track, under the middle of the car
    Vec3d       forward{0.0, 0.0, 1.0};
    double      speedMS = 0.0;
    double      alongM  = 0.0;
    bool        atStop  = false;
    std::size_t stop    = 0;  // which one, when stopped
};

/// Plans where the habitat's lines run: a tramway down each valley, and a funicular from the foot
/// of the antisunward endcap's ramp up to the hub at the axis; or a loop round a band of land. The
/// alignment is smoothed into something that could be built -- as straight as it can be while
/// keeping its cuttings shallow -- so it does not follow every bump in the ground. Deterministic;
/// the stops come later.
[[nodiscard]] std::vector<TramLine> planTramLines(const HabitatGeometry& geometry,
                                                  const TerrainGrid&     grid);

/// Cuts and fills the land along the lines so the track lies on it: the corridor is graded to the
/// alignment and blended back into the natural ground at its edges, and the woods are cleared from
/// it. Where the ground falls too far below the alignment it is left alone and the track is carried
/// on a trestle instead. Call it before planning anything else that stands on the ground.
void gradeForTrack(TerrainGrid& grid, const std::vector<TramLine>& lines);

/// Adds the stops: every town a valley line passes and halts in between, stations at the ends of a
/// funicular and halts at the terraces on the way.
void addTramStops(std::vector<TramLine>& lines, const Settlements& settlements);

/// Whether a point on the floor is within `clearM` of a line's track: where trees are kept off
/// and the ground is levelled for the formation.
[[nodiscard]] bool nearTrack(const std::vector<TramLine>& lines, double z, double theta,
                             double clearM);

/// The trams of every line at a moment (`seconds` is wall-clock time, like the spin).
[[nodiscard]] std::vector<Tram> tramsAt(const std::vector<TramLine>& lines, double seconds);

/// Where a tram is when it has travelled `alongM` down a line (on a loop, as far round as that is).
[[nodiscard]] TrackPoint pointAlong(const TramLine& line, double alongM);

/// The track's mesh, in chunks: rails, sleepers, ballast, and piers where it crosses water. Each
/// chunk's vertices are relative to its own origin.
struct TrackChunk
{
    Vec3d   origin{0.0};
    CpuMesh mesh;
    Vec3d   centre{0.0};  // of its bounding sphere, relative to the origin
    double  radius = 0.0;
};

[[nodiscard]] std::vector<TrackChunk> buildTrackMeshes(const std::vector<TramLine>& lines);

/// How big a tram car is: its body, for the physics that carries you when you ride on one.
inline constexpr double kTramBodyHeightM = 3.1;
inline constexpr double kTramBodyLengthM = 12.4;
inline constexpr double kTramBodyWidthM  = 2.5;

/// One tram car, 12 m long, facing +z with its floor at y = 0 and its middle at x = 0.
[[nodiscard]] CpuMesh buildTramMesh();

/// Vertex materials of the transit meshes.
namespace transit_material
{
inline constexpr std::uint32_t kBallast  = 0;
inline constexpr std::uint32_t kRail     = 1;
inline constexpr std::uint32_t kPier     = 2;
inline constexpr std::uint32_t kPlatform = 3;
inline constexpr std::uint32_t kBody     = 4;  // the tram's painted sides
inline constexpr std::uint32_t kGlass    = 5;  // its windows
inline constexpr std::uint32_t kRoof     = 6;
inline constexpr std::uint32_t kSkirt    = 7;  // its dark underframe and doors
inline constexpr std::uint32_t kDeck     = 8;  // the girder deck of a trestle or a viaduct
}  // namespace transit_material

}  // namespace StarshipSimulator
