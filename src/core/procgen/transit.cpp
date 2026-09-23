#include "StarshipSimulator/core/procgen/transit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/habitat/Enclosure.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kStepM        = 12.0;  // between points on the track
constexpr double kSoundM       = 4.0;   // between soundings of the ground under a line
constexpr double kMaxCutM      = 9.0;   // the deepest cutting the line will take
constexpr double kMaxRiseM     = 24.0;  // and the tallest trestle it will stand on
constexpr double kMaxFillM     = 6.0;   // above that it goes on a trestle rather than a bank
constexpr double kOvershootM   = 3.0;   // how far a vertical curve may stray outside the band
constexpr double kSideSlope    = 1.5;   // of cuttings and embankments: 1 down to 1.5 along
constexpr int    kSmoothPasses = 12;
constexpr int    kRoundPasses  = 60;    // vertical curves: how far the corners are rounded off
constexpr double kRailM        = 0.15;  // how far the rail stands above the sleepers
constexpr double kBedM         = 0.5;   // how deep the ballast and sleepers are
constexpr double kGradeM       = 5.0;   // half width of the levelled formation (track and platform)
constexpr double kBlendM       = 9.0;   // and of the slope back into the natural ground beside it

constexpr double kGaugeM      = 1.435;  // between the rails, as on Earth
constexpr double kRailHeadM   = 0.28;   // rail head above the ground
constexpr double kBallastM    = 1.75;   // half width of the formation
constexpr double kViaductM    = 1.6;    // rail head above the water where it crosses
constexpr double kChunkM      = 480.0;  // of track to a mesh chunk
constexpr double kStopEveryM  = 900.0;  // a halt out in the fields, where no town is near
constexpr double kPlatformM   = 14.0;   // long
constexpr double kTramLengthM = kTramBodyLengthM;
constexpr double kTramWidthM  = kTramBodyWidthM;
constexpr double kTramHeightM = kTramBodyHeightM;
constexpr double kHubRadiusM  = 70.0;  // the funicular stops where the ramp reaches the hub
constexpr double kLoopAcross  = 0.55;  // a loop runs this far across its band, off the river
constexpr double kRimMarginM  = 3.0;   // a sphere's funicular stops this far short of the window
constexpr double kSphereFootM = 15.0;  // and starts this far inside the land, below the fields
// A torus: its loop runs down its towns' main streets, beside the middle line, where the lifts come
// down the spokes to stations on the floor.
constexpr double kTubeLoopAcrossM = -12.0;
constexpr double kLiftSpeedMS     = 10.0;
constexpr double kLiftFloorMS2    = 1.0;   // how hard a lift speeds up or slows down at the floor
constexpr double kLiftHubMS2      = 0.25;  // and near the hub, where you weigh next to nothing
constexpr double kLiftPadM        = 6.0;   // the level ground round a lift's foot
constexpr double kLiftPlatformM   = 0.5;   // its station's platform, above the ground
constexpr double kLiftDwellS      = 20.0;
constexpr double kLiftFrameM      = 12.0;  // between the frames of its tower
constexpr double kLiftColumnM     = 2.6;   // the tower's columns, out from its middle

/// The line runs down the valley, off to one side of the river's meander.
double lineAngle(const HabitatGeometry& geometry, int valley)
{
    return geometry.landCenter(valley) + (0.55 * geometry.landHalfAngle());
}

/// Adds a box between two points, `halfWidth` to each side and `thickness` thick, to a mesh.
void addBeam(CpuMesh& mesh, const Vec3d& origin, const Vec3d& from, const Vec3d& to,
             const Vec3d& side, const Vec3d& up, double halfWidth, double thickness,
             std::uint32_t material)
{
    const Vec3d along = to - from;
    if (glm::length(along) < 1e-6)
    {
        return;
    }
    const Vec3d middle = ((from + to) * 0.5) - origin;
    const Vec3d ahead  = glm::normalize(along);
    const Mat4d place(Vec4d(side, 0.0), Vec4d(up, 0.0), Vec4d(ahead, 0.0), Vec4d(middle, 1.0));
    appendMesh(mesh, makeBox(Vec3f(1.0F), material),
               place * glm::scale(Mat4d(1.0),
                                  Vec3d(halfWidth, 0.5 * thickness, 0.5 * glm::length(along))));
}

}  // namespace

TrackPoint pointAlong(const TramLine& line, double alongM)
{
    if (line.track.empty())
    {
        return {};
    }
    double where = std::clamp(alongM, 0.0, line.lengthM);
    if (line.kind == LineKind::LOOP && line.lengthM > 0.0)
    {
        where = std::fmod(alongM, line.lengthM);  // round and round
        where = where < 0.0 ? where + line.lengthM : where;
    }
    const auto last = line.track.size() - 1;
    // The points are not evenly spaced (the ground rises and falls), so look the place up.
    const auto after = std::ranges::lower_bound(line.track, where, {}, &TrackPoint::alongM);
    const auto index =
        after == line.track.begin()
            ? std::size_t{0}
            : std::min(static_cast<std::size_t>(after - line.track.begin()) - 1, last);
    if (index == last)
    {
        return line.track[last];
    }
    const TrackPoint& a = line.track[index];
    const TrackPoint& b = line.track[index + 1];
    const double      t = b.alongM > a.alongM
                              ? std::clamp((where - a.alongM) / (b.alongM - a.alongM), 0.0, 1.0)
                              : 0.0;
    return {.position = glm::mix(a.position, b.position, t),
            .alongM   = where,
            .carried  = t < 0.5 ? a.carried : b.carried};
}

namespace
{

/// One sample of the ground under a line, and how far the alignment may stray from it there.
struct GroundSample
{
    double at     = 0.0;  // on the line's plan: z down a valley, metres round a loop
    double ground = 0.0;  // the land's height above the meridian profile
    double low    = 0.0;  // the deepest cutting the line will take here
    double high   = 0.0;  // and the tallest trestle
};

/// Where a point on a line's plan lies on the floor.
SurfaceSpot spotOf(const TramLine& line, double at)
{
    if (line.kind == LineKind::LOOP)
    {
        return {.z = line.z, .theta = line.theta + (at / line.radiusM)};
    }
    return {.z = at, .theta = line.theta};
}

/// One sounding of the ground under a line.
GroundSample soundAt(const TramLine& line, const HabitatGeometry& geometry, const TerrainGrid& grid,
                     double at, double cutM, double riseM)
{
    const SurfaceSpot spot   = spotOf(line, at);
    const double      ground = grid.groundHeight(spot.z, spot.theta);
    GroundSample sample{.at = at, .ground = ground, .low = ground - cutM, .high = ground + riseM};
    // Over water the formation has to clear the surface, on a viaduct.
    if (geometry.waterDepth(spot.z, spot.theta) > 0.0)
    {
        sample.low = std::max(sample.low, geometry.waterLevelAt(spot.z) + kViaductM);
    }
    return sample;
}

/// Samples the ground under a line, finely, from one end of it to the other.
std::vector<GroundSample> soundGround(const TramLine& line, const HabitatGeometry& geometry,
                                      const TerrainGrid& grid, double from, double to, double cutM,
                                      double riseM)
{
    std::vector<GroundSample> samples;
    const double              step  = (to > from ? 1.0 : -1.0) * kSoundM;
    const auto                count = static_cast<int>(std::abs(to - from) / kSoundM);
    samples.reserve(static_cast<std::size_t>(count) + 1);
    for (int i = 0; i <= count; ++i)
    {
        samples.push_back(soundAt(line, geometry, grid, from + (step * i), cutM, riseM));
    }
    return samples;
}

/// Samples the ground once round a loop, a whole number of soundings, the last just short of
/// where the first was taken.
std::vector<GroundSample> soundLoop(const TramLine& line, const HabitatGeometry& geometry,
                                    const TerrainGrid& grid, double cutM, double riseM)
{
    const double              round = 2.0 * kPi * line.radiusM;
    const auto                count = static_cast<int>(std::ceil(round / kSoundM));
    const double              step  = round / count;
    std::vector<GroundSample> samples;
    samples.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        samples.push_back(soundAt(line, geometry, grid, step * i, cutM, riseM));
    }
    return samples;
}

/// Smooths an alignment out of the ground it has been given: as straight as it can be made while
/// staying inside the band each sample allows. Coarse sweeps first, so a long line comes out
/// genuinely straight rather than merely locally smooth.
void smoothAlignment(std::vector<double>& height, const std::vector<GroundSample>& ground)
{
    const auto count = static_cast<std::ptrdiff_t>(height.size());
    if (count < 3)
    {
        return;
    }
    for (std::ptrdiff_t stride = std::max<std::ptrdiff_t>(1, count / 8); stride >= 1; stride /= 2)
    {
        for (int pass = 0; pass < kSmoothPasses; ++pass)
        {
            for (std::ptrdiff_t i = stride; i + stride < count; ++i)
            {
                const auto   at   = static_cast<std::size_t>(i);
                const double even = 0.5 * (height[static_cast<std::size_t>(i - stride)] +
                                           height[static_cast<std::size_t>(i + stride)]);
                height[at]        = std::clamp(even, ground[at].low, ground[at].high);
            }
        }
    }
    // Where the band pinched the line it will have left a corner. Round those into vertical
    // curves, so the track really is smooth at the scale the ground is built from; a curve cuts a
    // little deeper than the band asked for, which is what vertical curves do.
    const auto round = [&](int passes) {
        for (int pass = 0; pass < passes; ++pass)
        {
            for (std::ptrdiff_t i = 1; i + 1 < count; ++i)
            {
                const auto at = static_cast<std::size_t>(i);
                height[at]    = 0.5 * (height[at] + (0.5 * (height[at - 1] + height[at + 1])));
            }
        }
    };
    round(kRoundPasses);
    for (std::size_t i = 0; i < height.size(); ++i)
    {
        height[i] =
            std::clamp(height[i], ground[i].low - kOvershootM, ground[i].high + kOvershootM);
    }
    round(kRoundPasses / 8);
}

/// The same for a loop, whose alignment runs back into itself.
void smoothLoop(std::vector<double>& height, const std::vector<GroundSample>& ground)
{
    const auto count = static_cast<std::ptrdiff_t>(height.size());
    if (count < 3)
    {
        return;
    }
    const auto wrap = [count](std::ptrdiff_t i) {
        return static_cast<std::size_t>(((i % count) + count) % count);
    };
    for (std::ptrdiff_t stride = std::max<std::ptrdiff_t>(1, count / 8); stride >= 1; stride /= 2)
    {
        for (int pass = 0; pass < kSmoothPasses; ++pass)
        {
            for (std::ptrdiff_t i = 0; i < count; ++i)
            {
                const auto   at   = static_cast<std::size_t>(i);
                const double even = 0.5 * (height[wrap(i - stride)] + height[wrap(i + stride)]);
                height[at]        = std::clamp(even, ground[at].low, ground[at].high);
            }
        }
    }
    const auto round = [&](int passes) {
        for (int pass = 0; pass < passes; ++pass)
        {
            for (std::ptrdiff_t i = 0; i < count; ++i)
            {
                const auto at = static_cast<std::size_t>(i);
                height[at] =
                    0.5 * (height[at] + (0.5 * (height[wrap(i - 1)] + height[wrap(i + 1)])));
            }
        }
    };
    round(kRoundPasses);
    for (std::size_t i = 0; i < height.size(); ++i)
    {
        height[i] =
            std::clamp(height[i], ground[i].low - kOvershootM, ground[i].high + kOvershootM);
    }
    round(kRoundPasses / 8);
}

/// Lays a line's track: its alignment, and a point about every kStepM along it for the mesh.
void layTrack(TramLine& line, const HabitatGeometry& geometry, const TerrainGrid& grid, double from,
              double to, double railM)
{
    // A funicular up a rough ramp is happier cutting than a tramway across gentle fields is.
    const bool                      ramp  = line.kind == LineKind::ENDCAP;
    const bool                      loop  = line.kind == LineKind::LOOP;
    const double                    cutM  = ramp ? 14.0 : kMaxCutM;
    const double                    riseM = ramp ? 16.0 : kMaxRiseM;
    const std::vector<GroundSample> ground =
        loop ? soundLoop(line, geometry, grid, cutM, riseM)
             : soundGround(line, geometry, grid, from, to, cutM, riseM);
    if (ground.size() < 3)
    {
        return;
    }
    std::vector<double> formation;
    formation.reserve(ground.size());
    for (const GroundSample& sample : ground)
    {
        formation.push_back(std::clamp(sample.ground, sample.low, sample.high));
    }
    if (loop)
    {
        smoothLoop(formation, ground);
    }
    else
    {
        smoothAlignment(formation, ground);
    }

    // Down to the points the track is actually built from, about kStepM apart along it.
    double along = 0.0;
    Vec3d  last(0.0);
    for (std::size_t i = 0; i < ground.size(); ++i)
    {
        const SurfaceSpot spot = spotOf(line, ground[i].at);
        const double      z    = spot.z;
        const double      rail = formation[i] + railM;
        const double      base = geometry.profile().radiusAt(z).value_or(geometry.radius());
        const double      r    = base - rail;
        const Vec3d       at(r * std::cos(spot.theta), r * std::sin(spot.theta), z);
        const double      above = formation[i] - ground[i].ground;
        const bool        atEnd = !loop && i + 1 == ground.size();
        if (!line.track.empty())
        {
            const double reach = glm::distance(at, last);
            if (reach < kStepM && !atEnd)
            {
                continue;  // still inside this stretch of track
            }
            along += reach;
        }
        line.track.push_back({.position     = at,
                              .alongM       = along,
                              .railHeightM  = rail,
                              .aboveGroundM = above,
                              .carried      = above > kMaxFillM,
                              .planM        = ground[i].at});
        last = at;
    }
    if (loop)
    {
        // Back to the start, which closes the loop.
        TrackPoint close = line.track.front();
        along += glm::distance(close.position, last);
        close.alongM = along;
        close.planM  = 2.0 * kPi * line.radiusM;
        line.track.push_back(close);
    }
    line.lengthM = along;
}

/// Where a valley line calls: at each of its towns, and at halts out in the fields between them.
void callAtTowns(TramLine& line, const Settlements& settlements)
{
    double sinceStop = kStopEveryM;
    for (const TrackPoint& point : line.track)
    {
        const double theta = HabitatGeometry::angleOf(point.position);
        const auto*  town  = settlements.townAt(point.position.z, theta);
        const bool serves = town != nullptr && town->valley == line.valley &&
                            (line.stops.empty() || point.alongM - line.stops.back().alongM > 250.0);
        sinceStop += kStepM;
        if ((!serves && sinceStop < kStopEveryM) || point.carried)
        {
            continue;
        }
        line.stops.push_back({.name     = town != nullptr ? town->name : std::string("halt"),
                              .alongM   = point.alongM,
                              .position = point.position,
                              .dwellS   = town != nullptr ? 16.0 : 8.0});
        sinceStop = 0.0;
    }
}

/// The stretch of a loop's track nearest a point, off any trestle.
const TrackPoint* nearestOnTrack(const TramLine& line, const Vec3d& to)
{
    const TrackPoint* nearest = nullptr;
    for (const TrackPoint& point : line.track)
    {
        if (!point.carried && (nearest == nullptr || glm::distance(point.position, to) <
                                                         glm::distance(nearest->position, to)))
        {
            nearest = &point;
        }
    }
    return nearest;
}

/// Halts in the long gaps between a loop's stops, all the way round if it has none.
std::vector<TramStop> loopHalts(const TramLine& line)
{
    std::vector<TramStop> halts;
    const auto            count = static_cast<std::ptrdiff_t>(line.stops.size());
    for (std::ptrdiff_t i = 0; i < std::max<std::ptrdiff_t>(count, 1); ++i)
    {
        const double from = count > 0 ? line.stops[static_cast<std::size_t>(i)].alongM : 0.0;
        const double to =
            count > 1 ? line.stops[static_cast<std::size_t>((i + 1) % count)].alongM : from;
        const double gap =
            count > 1 ? std::fmod(to - from + line.lengthM, line.lengthM) : line.lengthM;
        const int extra = static_cast<int>(gap / kStopEveryM);
        for (int k = 1; k <= extra; ++k)
        {
            const TrackPoint here = pointAlong(line, from + (gap * k / (extra + 1)));
            if (!here.carried)
            {
                halts.push_back({.name     = "halt",
                                 .alongM   = here.alongM,
                                 .position = here.position,
                                 .dwellS   = 8.0});
            }
        }
    }
    return halts;
}

/// Where a loop calls: at the stretch of track nearest each town on its band, and at halts where
/// the towns are far apart.
void callRound(TramLine& line, const Settlements& settlements)
{
    for (const Settlement& place : settlements.places)
    {
        if (place.kind != SettlementKind::TOWN || place.valley != line.valley)
        {
            continue;
        }
        if (const TrackPoint* nearest = nearestOnTrack(line, place.plane.point(Vec2d(0.0), 0.0)))
        {
            line.stops.push_back({.name     = place.name,
                                  .alongM   = nearest->alongM,
                                  .position = nearest->position,
                                  .dwellS   = 16.0});
        }
    }
    std::ranges::sort(line.stops, [](const TramStop& a, const TramStop& b) {
        return a.alongM != b.alongM ? a.alongM < b.alongM : a.name < b.name;
    });
    // Two towns served from one place get one stop.
    const auto close = std::ranges::unique(line.stops, [](const TramStop& a, const TramStop& b) {
        return b.alongM - a.alongM < 60.0;
    });
    line.stops.erase(close.begin(), close.end());
    const std::vector<TramStop> halts = loopHalts(line);
    line.stops.insert(line.stops.end(), halts.begin(), halts.end());
    std::ranges::stable_sort(line.stops, {}, &TramStop::alongM);
}

/// How far down the endcap the ramp still has ground to run on, before it reaches the hub. A
/// sphere's polar slope has no hub: it runs on to the rim of the window.
double rampFoot(const HabitatGeometry& geometry, double theta, double from)
{
    double top = from;
    bool   hub = false;
    for (int step = 1; step * 20.0 < from - geometry.profile().zMin(); ++step)
    {
        const double                z      = from - (step * 20.0);
        const std::optional<double> ground = geometry.groundRadius(z, theta);
        if (!ground || *ground < kHubRadiusM)
        {
            hub = true;
            break;
        }
        top = z;
    }
    if (!hub && geometry.kind() == HabitatKind::BERNAL_SPHERE)
    {
        return geometry.profile().zMin() + kRimMarginM;
    }
    return top;
}

/// Where a funicular calls: a station at each end, and a halt at the terraces in between.
void callAtTerraces(TramLine& line)
{
    constexpr int kLegs = 4;
    for (int leg = 0; leg <= kLegs; ++leg)
    {
        const double     at   = (line.lengthM * leg) / kLegs;
        const TrackPoint here = pointAlong(line, at);
        std::string      name = "terrace";
        if (leg == 0)
        {
            name = line.footName;
        }
        else if (leg == kLegs)
        {
            name = line.summitName;
        }
        const bool end = leg == 0 || leg == kLegs;
        line.stops.push_back({.name     = std::move(name),
                              .alongM   = here.alongM,
                              .position = here.position,
                              .dwellS   = end ? 24.0 : 10.0});
    }
}

/// A lift's stations: on the floor, named after the town beside it, and at the hub.
void callAtSpoke(TramLine& line, const Settlements& settlements)
{
    const Vec3d foot = line.track.front().position;
    std::string name = "the fields";
    for (const Settlement& place : settlements.places)
    {
        if (place.kind == SettlementKind::TOWN &&
            glm::distance(place.plane.point(Vec2d(0.0), 0.0), foot) < place.radiusM + 120.0)
        {
            name = place.name + " station";
            break;
        }
    }
    line.footName = name;
    line.stops.push_back(
        {.name = std::move(name), .alongM = 0.0, .position = foot, .dwellS = kLiftDwellS});
    line.stops.push_back({.name     = line.summitName,
                          .alongM   = line.lengthM,
                          .position = line.track.back().position,
                          .dwellS   = kLiftDwellS});
}

/// A sphere's funicular: from the land's antisunward edge up the polar slope to the window's rim,
/// halfway round from where the band's plan starts (the towns keep to the middles of the band's
/// stretches, so that is always between two of them).
TramLine sphereFunicular(const HabitatGeometry& geometry, const TerrainGrid& grid,
                         const LandBand& band)
{
    TramLine line;
    line.kind         = LineKind::ENDCAP;
    line.valley       = band.index;
    line.theta        = std::fmod(band.centreTheta + kPi, 2.0 * kPi);
    line.topSpeed     = 26.0;
    line.trams        = 2;
    line.footName     = "the fields";
    line.summitName   = "the window";
    const double foot = geometry.floorZMin() + kSphereFootM;
    const double top  = rampFoot(geometry, line.theta, foot);
    line.radiusM      = geometry.floorRadiusAt(0.5 * (foot + top));
    layTrack(line, geometry, grid, foot, top, kRailHeadM);
    return line;
}

/// A torus's lift up spoke k: from a station on the floor at the spoke's foot, straight up the
/// radius through the tube and the spoke to the hub's floor.
TramLine spokeLift(const HabitatGeometry& geometry, const TerrainGrid& grid,
                   const TorusShape& torus, int k)
{
    TramLine line;
    line.kind           = LineKind::SPOKE;
    line.valley         = 0;
    line.theta          = torus.spokeAngle(k);
    line.topSpeed       = kLiftSpeedMS;
    line.trams          = 1;
    line.footName       = "the floor";
    line.summitName     = "the hub";
    const double floor  = geometry.floorRadiusAt(0.0);
    const double ground = floor - grid.groundHeight(0.0, line.theta);
    const double foot   = ground - kLiftPlatformM;
    const double top    = torus.hubRadiusM;
    line.radiusM        = foot;
    const int steps     = std::max(2, static_cast<int>(std::ceil((foot - top) / kStepM)));
    for (int i = 0; i <= steps; ++i)
    {
        const double r = std::lerp(foot, top, static_cast<double>(i) / steps);
        line.track.push_back(
            {.position     = Vec3d(r * std::cos(line.theta), r * std::sin(line.theta), 0.0),
             .alongM       = foot - r,
             .railHeightM  = floor - r,
             .aboveGroundM = ground - r,
             .carried      = true,
             .planM        = foot - r});
    }
    line.lengthM = foot - top;
    return line;
}

}  // namespace

std::vector<TramLine> planTramLines(const HabitatGeometry& geometry, const TerrainGrid& grid)
{
    std::vector<TramLine> lines;
    for (int valley = 0; valley < geometry.bandCount(); ++valley)
    {
        const LandBand& band = geometry.band(valley);
        if (band.axis == BandAxis::AROUND)
        {
            // Once round the band, to one side of the river (a torus's: down its main streets).
            const double      across = geometry.kind() == HabitatKind::STANFORD_TORUS
                                           ? kTubeLoopAcrossM
                                           : kLoopAcross * band.halfWidthM;
            const SurfaceSpot start  = band.toSurface(Vec2d(across, 0.0));
            TramLine          line;
            line.kind    = LineKind::LOOP;
            line.valley  = valley;
            line.theta   = start.theta;
            line.z       = start.z;
            line.radiusM = geometry.profile().radiusAt(start.z).value_or(geometry.radius());
            layTrack(line, geometry, grid, 0.0, 0.0, kRailHeadM);
            if (line.track.size() >= 2)
            {
                lines.push_back(std::move(line));
            }
            continue;
        }
        // The tramway runs the length of the valley floor (the endcaps' ramps are for the lifts).
        TramLine line;
        line.kind    = LineKind::VALLEY;
        line.valley  = valley;
        line.theta   = lineAngle(geometry, valley);
        line.radiusM = geometry.radius();
        layTrack(line, geometry, grid, geometry.floorZMin() + 150.0, geometry.floorZMax() - 150.0,
                 kRailHeadM);
        if (line.track.size() >= 2)
        {
            lines.push_back(std::move(line));
        }
    }

    // A torus's lifts, one up each spoke to the hub.
    if (const auto& torus = geometry.enclosure().torus())
    {
        for (int k = 0; k < torus->spokes; ++k)
        {
            lines.push_back(spokeLift(geometry, grid, *torus, k));
        }
    }

    // The funicular up each valley's end of the antisunward ramp, to the hub at the axis.
    for (int valley = 0; valley < geometry.bandCount(); ++valley)
    {
        if (geometry.kind() == HabitatKind::BERNAL_SPHERE)
        {
            TramLine line = sphereFunicular(geometry, grid, geometry.band(valley));
            if (line.track.size() >= 2)
            {
                lines.push_back(std::move(line));
            }
            continue;
        }
        if (geometry.band(valley).axis != BandAxis::ALONG_Z)
        {
            continue;
        }
        TramLine line;
        line.kind         = LineKind::ENDCAP;
        line.valley       = valley;
        line.theta        = geometry.landCenter(valley);
        line.radiusM      = geometry.radius();
        line.topSpeed     = 26.0;  // it has a long climb
        line.trams        = 2;
        const double foot = geometry.floorZMin() + 40.0;
        layTrack(line, geometry, grid, foot, rampFoot(geometry, line.theta, foot), kRailHeadM);
        if (line.track.size() >= 2 && line.lengthM > 100.0)
        {
            lines.push_back(std::move(line));
        }
    }
    return lines;
}

void addTramStops(std::vector<TramLine>& lines, const Settlements& settlements)
{
    for (TramLine& line : lines)
    {
        line.stops.clear();
        if (line.kind == LineKind::VALLEY)
        {
            callAtTowns(line, settlements);
        }
        else if (line.kind == LineKind::LOOP)
        {
            callRound(line, settlements);
        }
        else if (line.kind == LineKind::SPOKE)
        {
            callAtSpoke(line, settlements);
        }
        else
        {
            callAtTerraces(line);
        }
        double dwell = 0.0;
        for (const TramStop& stop : line.stops)
        {
            dwell += stop.dwellS;
        }
        line.journeyS = (line.lengthM / line.topSpeed) + dwell;
    }
}

namespace
{

/// The alignment's formation level at a z on a line, and whether the track is carried there.
double formationAt(const TramLine& line, double z, bool& carried)
{
    // The track runs from one end of the line to the other without ever turning back, so its
    // points are in order of z one way or the other.
    const bool  down = line.track.back().position.z < line.track.front().position.z;
    std::size_t low  = 0;
    std::size_t high = line.track.size() - 1;
    while (high - low > 1)
    {
        const std::size_t middle = (low + high) / 2;
        const bool        before =
            down ? line.track[middle].position.z > z : line.track[middle].position.z < z;
        (before ? low : high) = middle;
    }
    const TrackPoint& a    = line.track[low];
    const TrackPoint& b    = line.track[high];
    const double      span = b.position.z - a.position.z;
    const double t = std::abs(span) > 1e-9 ? std::clamp((z - a.position.z) / span, 0.0, 1.0) : 0.0;
    // Only where both ends of the stretch are flying: the ground is still graded up to a
    // trestle's feet, which is what its abutments stand on.
    carried = a.carried && b.carried;
    return std::lerp(a.railHeightM, b.railHeightM, t) - kRailHeadM;
}

}  // namespace

namespace
{

/// Cuts and fills the heights of one line's corridor at one row of the grid.
void gradeRow(TerrainGrid& grid, const TramLine& line, std::uint32_t row, double z, double base,
              double formation)
{
    const TerrainGridLayout& layout = grid.layout;
    const double             arc    = (2.0 * kPi * base) / layout.columns;
    const auto middle = static_cast<std::int64_t>(std::llround(grid.cellAt(z, line.theta).x));
    const auto wrap   = [&](std::int64_t c) {
        return static_cast<std::uint32_t>(((c % layout.columns) + layout.columns) % layout.columns);
    };
    // The side slopes run at a constant angle, so the deeper the earthwork the wider it spreads,
    // as a cutting or an embankment really does.
    const double depth  = std::abs(formation - grid.height(wrap(middle), row));
    const double batter = kBlendM + (kSideSlope * depth);
    const auto   reach  = static_cast<std::int64_t>((kGradeM + batter) / arc) + 1;
    for (std::int64_t c = middle - reach; c <= middle + reach; ++c)
    {
        const std::uint32_t column = wrap(c);
        const double        across =
            std::abs(std::remainder(layout.theta(column) - line.theta, 2.0 * kPi)) * base;
        if (across > kGradeM + batter)
        {
            continue;
        }
        // Level across the formation, then blend back into the land it was cut from.
        const double blend = glm::smoothstep(kGradeM, kGradeM + batter, across);
        grid.setHeight(column, row, std::lerp(formation, grid.height(column, row), blend));
    }
}

/// The alignment's formation level on a loop, `round` metres round it from its start.
double loopFormationAt(const TramLine& line, double round, bool& carried)
{
    // The points are in order of how far round they are, the last one closing the loop.
    const auto        after = std::ranges::upper_bound(line.track, round, {}, &TrackPoint::planM);
    const std::size_t high  = std::clamp<std::size_t>(
        static_cast<std::size_t>(after - line.track.begin()), 1, line.track.size() - 1);
    const TrackPoint& a    = line.track[high - 1];
    const TrackPoint& b    = line.track[high];
    const double      span = b.planM - a.planM;
    const double      t    = span > 1e-9 ? std::clamp((round - a.planM) / span, 0.0, 1.0) : 0.0;
    carried                = a.carried && b.carried;
    return std::lerp(a.railHeightM, b.railHeightM, t) - kRailHeadM;
}

/// Cuts and fills the heights of a loop's corridor at one column of the grid: the mirror of
/// gradeRow, for a line running around the axis rather than along it.
void gradeColumn(TerrainGrid& grid, const TramLine& line, std::uint32_t column, double theta,
                 double formation)
{
    const TerrainGridLayout& layout    = grid.layout;
    const double             middleRow = grid.cellAt(line.z, theta).y;
    const auto               last      = static_cast<std::int64_t>(layout.rows()) - 1;
    const auto               middle    = std::clamp<std::int64_t>(std::llround(middleRow), 0, last);
    const double             depth =
        std::abs(formation - grid.height(column, static_cast<std::uint32_t>(middle)));
    // The shelf is level across, at one distance from the axis: where the floor slopes (a
    // sphere) the rows uphill of the middle are cut to that radius, not to the same height.
    const auto middleRadius = static_cast<double>(grid.profile[static_cast<std::size_t>(middle)].y);
    const double batter     = kBlendM + (kSideSlope * depth);
    const auto   reach      = static_cast<std::int64_t>((kGradeM + batter) / layout.cellU) + 1;
    for (std::int64_t row = std::max<std::int64_t>(middle - reach, 0);
         row <= std::min(middle + reach, last); ++row)
    {
        const auto   at     = static_cast<std::uint32_t>(row);
        const double across = std::abs(static_cast<double>(row) - middleRow) * layout.cellU;
        if (across > kGradeM + batter)
        {
            continue;
        }
        const double blend = glm::smoothstep(kGradeM, kGradeM + batter, across);
        const double level = formation + (static_cast<double>(grid.profile[at].y) - middleRadius);
        grid.setHeight(column, at, std::lerp(level, grid.height(column, at), blend));
    }
}

/// Grades a loop's whole corridor, column by column, and clears the woods from it.
void gradeLoop(TerrainGrid& grid, const TramLine& line)
{
    const TerrainGridLayout& layout = grid.layout;
    const double             round  = 2.0 * kPi * line.radiusM;
    for (std::uint32_t column = 0; column < layout.columns; ++column)
    {
        const double theta     = layout.theta(column);
        double       angle     = std::remainder(theta - line.theta, 2.0 * kPi);
        angle                  = angle < 0.0 ? angle + (2.0 * kPi) : angle;
        bool         carried   = false;
        const double formation = loopFormationAt(line, angle / (2.0 * kPi) * round, carried);
        if (!carried)
        {
            gradeColumn(grid, line, column, theta, formation);
        }
    }
    // The woods, in the cover map's coarser rows (two grid rows each).
    const double cell      = 2.0 * layout.cellU;
    const double middleRow = grid.cellAt(line.z, line.theta).y / 2.0;
    const auto   middle    = static_cast<std::int64_t>(std::llround(middleRow));
    const auto   reach     = static_cast<std::int64_t>((kGradeM + kBlendM) / cell) + 1;
    for (std::int64_t row = std::max<std::int64_t>(middle - reach, 0);
         row <= std::min<std::int64_t>(middle + reach, grid.coverRows - 1); ++row)
    {
        if (std::abs(static_cast<double>(row) - middleRow) * cell >= kGradeM + kBlendM)
        {
            continue;
        }
        for (std::uint32_t column = 0; column < grid.coverColumns; ++column)
        {
            grid.cover[((static_cast<std::size_t>(row) * grid.coverColumns) + column) * 4] = 0;
        }
    }
}

/// Levels the ground round a lift's foot for its station, and clears the woods from it.
void gradePad(TerrainGrid& grid, const TramLine& line)
{
    const TerrainGridLayout& layout = grid.layout;
    const Vec3d              foot   = line.track.front().position;
    const double             level  = line.track.front().railHeightM - kLiftPlatformM;
    const Vec2d              middle = grid.cellAt(foot.z, line.theta);
    const double             reach  = kLiftPadM + kBlendM;
    const double             arc    = 2.0 * kPi * line.radiusM / layout.columns;
    const auto               rows   = static_cast<std::int64_t>(reach / layout.cellU) + 1;
    const auto               cols   = static_cast<std::int64_t>(reach / arc) + 1;
    const auto               last   = static_cast<std::int64_t>(layout.rows()) - 1;
    const auto               row0   = static_cast<std::int64_t>(std::llround(middle.y));
    const auto               col0   = static_cast<std::int64_t>(std::llround(middle.x));
    for (std::int64_t r = std::max<std::int64_t>(row0 - rows, 0); r <= std::min(row0 + rows, last);
         ++r)
    {
        for (std::int64_t c = col0 - cols; c <= col0 + cols; ++c)
        {
            const double away = std::hypot((static_cast<double>(r) - middle.y) * layout.cellU,
                                           (static_cast<double>(c) - middle.x) * arc);
            if (away > reach)
            {
                continue;
            }
            const auto column = static_cast<std::uint32_t>(((c % layout.columns) + layout.columns) %
                                                           layout.columns);
            const auto at     = static_cast<std::uint32_t>(r);
            const double blend = glm::smoothstep(kLiftPadM, reach, away);
            grid.setHeight(column, at, std::lerp(level, grid.height(column, at), blend));
        }
    }
    for (std::int64_t r = std::max<std::int64_t>((row0 - rows) / 2, 0);
         r <= std::min<std::int64_t>((row0 + rows) / 2, grid.coverRows - 1); ++r)
    {
        for (std::int64_t c = (col0 - cols) / 2; c <= (col0 + cols) / 2; ++c)
        {
            const auto column = static_cast<std::uint32_t>(
                ((c % grid.coverColumns) + grid.coverColumns) % grid.coverColumns);
            grid.cover[((static_cast<std::size_t>(r) * grid.coverColumns) + column) * 4] = 0;
        }
    }
}

/// Clears the painted woods from a line's corridor, in the cover map's coarser cells.
void clearWoods(TerrainGrid& grid, const TramLine& line, double from, double to)
{
    const TerrainGridLayout& layout = grid.layout;
    for (std::uint32_t row = 0; row < grid.coverRows; ++row)
    {
        const auto at   = std::min(2U * row, layout.cells);
        const auto z    = static_cast<double>(grid.profile[at].x);
        const auto base = static_cast<double>(grid.profile[at].y);
        if (z < from || z > to || base < 1.0)
        {
            continue;
        }
        const double arc   = (2.0 * kPi * base) / grid.coverColumns;
        const auto   reach = static_cast<std::int64_t>((kGradeM + kBlendM) / arc) + 1;
        const auto   middle =
            static_cast<std::int64_t>(std::llround(grid.cellAt(z, line.theta).x / 2.0));
        for (std::int64_t c = middle - reach; c <= middle + reach; ++c)
        {
            const auto column = static_cast<std::uint32_t>(
                ((c % grid.coverColumns) + grid.coverColumns) % grid.coverColumns);
            const double across =
                std::abs(std::remainder(layout.theta(2.0 * column) - line.theta, 2.0 * kPi)) * base;
            if (across < kGradeM + kBlendM)
            {
                grid.cover[((static_cast<std::size_t>(row) * grid.coverColumns) + column) * 4] = 0;
            }
        }
    }
}

}  // namespace

void gradeForTrack(TerrainGrid& grid, const std::vector<TramLine>& lines)
{
    for (const TramLine& line : lines)
    {
        if (line.track.size() < 2)
        {
            continue;
        }
        if (line.kind == LineKind::LOOP)
        {
            gradeLoop(grid, line);
            continue;
        }
        if (line.kind == LineKind::SPOKE)
        {
            gradePad(grid, line);  // it stands on the floor only at its foot
            continue;
        }
        const double from = std::min(line.track.front().position.z, line.track.back().position.z);
        const double to   = std::max(line.track.front().position.z, line.track.back().position.z);
        for (std::uint32_t row = 0; row < grid.layout.rows(); ++row)
        {
            const auto z    = static_cast<double>(grid.profile[row].x);
            const auto base = static_cast<double>(grid.profile[row].y);
            if (z < from || z > to || base < 1.0)
            {
                continue;
            }
            bool         carried   = false;
            const double formation = formationAt(line, z, carried);
            if (!carried)  // nothing to grade under a trestle: the track flies over the ground
            {
                gradeRow(grid, line, row, z, base, formation);
            }
        }
        clearWoods(grid, line, from, to);
    }
}

bool nearTrack(const std::vector<TramLine>& lines, double z, double theta, double clearM)
{
    return std::ranges::any_of(lines, [z, theta, clearM](const TramLine& line) {
        if (line.track.size() < 2)
        {
            return false;
        }
        if (line.kind == LineKind::LOOP)
        {
            return std::abs(z - line.z) < clearM;  // it goes all the way round at one z
        }
        if (line.kind == LineKind::SPOKE)
        {
            // Only its foot stands on the floor, in its station.
            const double across =
                std::abs(std::remainder(theta - line.theta, 2.0 * kPi)) * line.radiusM;
            return std::hypot(across, z - line.z) < clearM + kLiftPadM;
        }
        const double from = std::min(line.track.front().position.z, line.track.back().position.z);
        const double to   = std::max(line.track.front().position.z, line.track.back().position.z);
        // A line runs at one angle the whole way, so the distance across to it is an arc.
        const double across =
            std::abs(std::remainder(theta - line.theta, 2.0 * kPi)) * line.radiusM;
        return z >= from - clearM && z <= to + clearM && across < clearM;
    });
}

namespace
{

/// Where a tram is after `into` seconds of one leg of its run, and which stop it is standing at.
struct Progress
{
    double      travelled = 0.0;  // along this leg, from where it set off
    bool        atStop    = false;
    std::size_t stop      = 0;
};

Progress runLeg(const TramLine& line, double into, bool back)
{
    Progress progress;
    double   left = into;
    for (std::size_t s = 0; s < line.stops.size(); ++s)
    {
        const std::size_t index = back ? line.stops.size() - 1 - s : s;
        const TramStop&   stop  = line.stops[index];
        const double      reach = back ? line.lengthM - stop.alongM : stop.alongM;
        const double      run   = (reach - progress.travelled) / line.topSpeed;
        if (left < run)
        {
            return {.travelled = progress.travelled + (left * line.topSpeed)};
        }
        left -= run;
        progress.travelled = reach;
        if (left < stop.dwellS)
        {
            return {.travelled = reach, .atStop = true, .stop = index};
        }
        left -= stop.dwellS;
    }
    return {.travelled = progress.travelled + (left * line.topSpeed)};
}

/// One tram of a loop, at a moment: always going the same way round.
Tram loopTramAt(const TramLine& line, std::size_t index, double seconds, int which)
{
    const double phase =
        std::fmod((seconds / line.journeyS) + (static_cast<double>(which) / line.trams), 1.0);
    const Progress   leg   = runLeg(line, phase * line.journeyS, false);
    const TrackPoint here  = pointAlong(line, leg.travelled);
    const TrackPoint ahead = pointAlong(line, leg.travelled + 4.0);
    const Vec3d      step  = ahead.position - here.position;
    return {.line     = index,
            .position = here.position,
            .forward  = glm::length(step) > 1e-6 ? glm::normalize(step) : Vec3d(0.0, 0.0, 1.0),
            .speedMS  = leg.atStop ? 0.0 : line.topSpeed,
            .alongM   = here.alongM,
            .atStop   = leg.atStop,
            .stop     = leg.stop};
}

/// How far a lift has gone `t` seconds after leaving one end of a leg `length` long: speeding up
/// at `start` m/s^2, running at `top`, slowing at `end` m/s^2 to stop at the far end.
double liftTravelled(double t, double length, double top, double start, double end)
{
    // Short legs never reach full speed: the peak where the two ramps meet.
    const double peak = std::min(top, std::sqrt(2.0 * length * start * end / (start + end)));
    const double t1   = peak / start;
    const double t3   = peak / end;
    const double s1   = 0.5 * peak * t1;
    const double s3   = 0.5 * peak * t3;
    const double t2   = (length - s1 - s3) / peak;
    if (t < t1)
    {
        return 0.5 * start * t * t;
    }
    if (t < t1 + t2)
    {
        return s1 + (peak * (t - t1));
    }
    const double left = std::max(0.0, t1 + t2 + t3 - t);
    return length - (0.5 * end * left * left);
}

/// How long a lift takes over a leg `length` long (see liftTravelled).
double liftLegSeconds(double length, double top, double start, double end)
{
    const double peak = std::min(top, std::sqrt(2.0 * length * start * end / (start + end)));
    const double s1   = 0.5 * peak * peak / start;
    const double s3   = 0.5 * peak * peak / end;
    return (peak / start) + (peak / end) + ((length - s1 - s3) / peak);
}

/// A lift, at a moment: waiting on the floor, going up, waiting at the hub, coming down. It moves
/// gently near the hub, where you weigh next to nothing and a hard stop would throw you up off its
/// floor.
Tram liftAt(const TramLine& line, std::size_t index, double seconds, int which)
{
    const double leg   = liftLegSeconds(line.lengthM, line.topSpeed, kLiftFloorMS2, kLiftHubMS2);
    const double cycle = (2.0 * leg) + (2.0 * kLiftDwellS);
    double       t =
        std::fmod((seconds / cycle) + (static_cast<double>(which) / line.trams), 1.0) * cycle;
    Tram tram{.line = index, .lift = true};
    // Waiting at the floor, going up, waiting at the hub, coming down.
    const double up   = kLiftDwellS;
    const double hub  = up + leg;
    const double down = hub + kLiftDwellS;
    if (t < up)
    {
        tram.atStop = true;
        tram.stop   = 0;
    }
    else if (t < hub)
    {
        tram.alongM =
            liftTravelled(t - up, line.lengthM, line.topSpeed, kLiftFloorMS2, kLiftHubMS2);
        tram.speedMS = line.topSpeed;
    }
    else if (t < down)
    {
        tram.alongM = line.lengthM;
        tram.atStop = true;
        tram.stop   = 1;
    }
    else
    {
        t -= down;
        tram.alongM  = line.lengthM -
                       liftTravelled(t, line.lengthM, line.topSpeed, kLiftHubMS2, kLiftFloorMS2);
        tram.speedMS = line.topSpeed;
    }
    tram.position = pointAlong(line, tram.alongM).position;
    tram.forward  = Vec3d(-std::sin(line.theta), std::cos(line.theta), 0.0);  // round the ring
    return tram;
}

/// One tram of a line, at a moment.
Tram tramAt(const TramLine& line, std::size_t index, double seconds, int which)
{
    if (line.kind == LineKind::LOOP)
    {
        return loopTramAt(line, index, seconds, which);
    }
    if (line.kind == LineKind::SPOKE)
    {
        return liftAt(line, index, seconds, which);
    }
    const double cycle = 2.0 * line.journeyS;  // down the line and back again
    const double phase =
        std::fmod((seconds / cycle) + (static_cast<double>(which) / line.trams), 1.0);
    const bool     back = phase >= 0.5;
    const Progress leg  = runLeg(line, (back ? phase - 0.5 : phase) * 2.0 * line.journeyS, back);
    const double   alongM =
        std::clamp(back ? line.lengthM - leg.travelled : leg.travelled, 0.0, line.lengthM);
    const TrackPoint here = pointAlong(line, alongM);
    const TrackPoint ahead =
        pointAlong(line, std::clamp(alongM + (back ? -4.0 : 4.0), 0.0, line.lengthM));
    const Vec3d step = ahead.position - here.position;
    return {.line     = index,
            .position = here.position,
            .forward  = glm::length(step) > 1e-6 ? glm::normalize(step)
                                                 : Vec3d(0.0, 0.0, back ? -1.0 : 1.0),
            .speedMS  = leg.atStop ? 0.0 : line.topSpeed,
            .alongM   = alongM,
            .atStop   = leg.atStop,
            .stop     = leg.stop};
}

}  // namespace

std::vector<Tram> tramsAt(const std::vector<TramLine>& lines, double seconds)
{
    std::vector<Tram> trams;
    for (std::size_t l = 0; l < lines.size(); ++l)
    {
        const TramLine& line = lines[l];
        if (line.track.size() < 2 || line.journeyS <= 0.0)
        {
            continue;
        }
        for (int which = 0; which < line.trams; ++which)
        {
            trams.push_back(tramAt(line, l, seconds, which));
        }
    }
    return trams;
}

namespace
{

/// One stretch of track between two points: the ballast and sleepers it lies on, its two rails,
/// and a pier under it where it crosses the water.
void addTrackSegment(CpuMesh& mesh, const Vec3d& origin, const TrackPoint& a, const TrackPoint& b,
                     bool bent)
{
    const Vec3d down  = -HabitatGeometry::localUp(a.position);  // away from the axis
    const Vec3d ahead = glm::normalize(b.position - a.position);
    const Vec3d side  = glm::normalize(glm::cross(-down, ahead));
    const Vec3d over  = -down;

    // The formation carries the rails: its top is a rail's height below the rail head, and the
    // rails stand on it (everything is measured from the rail head).
    const Vec3d bed = down * (kRailM + (0.5 * kBedM));
    addBeam(mesh, origin, a.position + bed, b.position + bed, side, over, kBallastM, kBedM,
            a.carried ? transit_material::kDeck : transit_material::kBallast);
    for (const double rail : {-0.5 * kGaugeM, 0.5 * kGaugeM})
    {
        const Vec3d offset = (side * rail) + (down * (0.5 * kRailM));
        addBeam(mesh, origin, a.position + offset, b.position + offset, side, over, 0.038, kRailM,
                transit_material::kRail);
    }
    if (!bent)
    {
        return;
    }
    // A trestle bent: a pair of legs splayed out to the ground with a cap across them, and a
    // brace between them. They stand on the land as it was left, since a trestle grades nothing.
    const double drop = std::max(a.aboveGroundM, 0.5) + 1.2;
    const Vec3d  head = a.position + (down * (kRailM + kBedM));
    const Vec3d  cap  = head + (down * 0.45);
    addBeam(mesh, origin, cap - (side * kBallastM), cap + (side * kBallastM), ahead, down, 0.36,
            0.7, transit_material::kPier);
    std::array<Vec3d, 2> feet{};
    for (int leg = 0; leg < 2; ++leg)
    {
        const double lean = leg == 0 ? -1.0 : 1.0;
        const Vec3d  top  = cap + (side * (lean * (kBallastM - 0.5)));
        feet.at(static_cast<std::size_t>(leg)) =
            top + (down * drop) + (side * (lean * 0.32 * drop));
        addBeam(mesh, origin, top, feet.at(static_cast<std::size_t>(leg)), ahead,
                glm::normalize(feet.at(static_cast<std::size_t>(leg)) - top), 0.30, 0.60,
                transit_material::kPier);
    }
    if (drop > 4.0)
    {
        // Braced across, half way down, so it does not look like a pair of stilts.
        const Vec3d left  = glm::mix(cap - (side * (kBallastM - 0.5)), feet[0], 0.55);
        const Vec3d right = glm::mix(cap + (side * (kBallastM - 0.5)), feet[1], 0.55);
        addBeam(mesh, origin, left, right, ahead, down, 0.22, 0.34, transit_material::kPier);
    }
}

/// Closes a chunk off: works out the sphere that holds it, for culling.
void closeChunk(std::vector<TrackChunk>& chunks, TrackChunk& chunk)
{
    if (chunk.mesh.indices.empty())
    {
        return;
    }
    Vec3f low(1e9F);
    Vec3f high(-1e9F);
    for (const Vertex& vertex : chunk.mesh.vertices)
    {
        low  = glm::min(low, vertex.position);
        high = glm::max(high, vertex.position);
    }
    chunk.centre = Vec3d((low + high) * 0.5F);
    chunk.radius = glm::length(Vec3d(high - low)) * 0.5;
    chunks.push_back(std::move(chunk));
    chunk = TrackChunk{};
}

/// The platform beside a stop.
TrackChunk platformAt(const TramLine& line, const TramStop& stop)
{
    const TrackPoint here  = pointAlong(line, stop.alongM);
    const TrackPoint ahead = pointAlong(line, std::min(stop.alongM + 4.0, line.lengthM));
    const Vec3d      over  = HabitatGeometry::localUp(here.position);
    const Vec3d      along = glm::length(ahead.position - here.position) > 1e-6
                                 ? glm::normalize(ahead.position - here.position)
                                 : Vec3d(0.0, 0.0, 1.0);
    const Vec3d      side  = glm::normalize(glm::cross(over, along));

    TrackChunk stand;
    stand.origin = here.position + (side * (kBallastM + 1.4));
    addBeam(stand.mesh, stand.origin, stand.origin - (along * (0.5 * kPlatformM)),
            stand.origin + (along * (0.5 * kPlatformM)), side, over, 1.4, 0.5,
            transit_material::kPlatform);
    stand.radius = kPlatformM;
    return stand;
}

/// A lift's tower: four columns at the corners of its shaft, from its station up the spoke to the
/// hub, framed together every few metres; and the station's platform round its foot.
void liftTower(std::vector<TrackChunk>& chunks, const TramLine& line)
{
    const Vec3d                out = HabitatGeometry::localUp(line.track.front().position) * -1.0;
    const Vec3d                around = Vec3d(-std::sin(line.theta), std::cos(line.theta), 0.0);
    const Vec3d                axis(0.0, 0.0, 1.0);
    const std::array<Vec3d, 4> corners{
        (around + axis) * kLiftColumnM, (around - axis) * kLiftColumnM,
        (-around - axis) * kLiftColumnM, (-around + axis) * kLiftColumnM};
    const double foot   = line.track.front().aboveGroundM + line.radiusM;  // the ground's radius
    const double top    = std::hypot(line.track.back().position.x, line.track.back().position.y);
    const auto   pieces = static_cast<int>(std::ceil((foot - top) / kChunkM));
    for (int piece = 0; piece < pieces; ++piece)
    {
        const double from = foot - (kChunkM * piece);
        const double to   = std::max(from - kChunkM, top);
        TrackChunk   chunk;
        chunk.origin = out * (0.5 * (from + to));
        for (const Vec3d& corner : corners)
        {
            addBeam(chunk.mesh, chunk.origin, (out * from) + corner, (out * to) + corner, around,
                    axis, 0.18, 0.36, transit_material::kPier);
        }
        const auto frames = static_cast<int>(std::floor((from - to) / kLiftFrameM));
        for (int frame = 1; frame <= frames; ++frame)
        {
            const double r = from - (kLiftFrameM * frame);
            for (std::size_t c = 0; c < corners.size(); ++c)
            {
                const Vec3d a    = (out * r) + corners.at(c);
                const Vec3d b    = (out * r) + corners.at((c + 1) % corners.size());
                const Vec3d side = glm::normalize(glm::cross(out, b - a));
                addBeam(chunk.mesh, chunk.origin, a, b, side, out, 0.12, 0.3,
                        transit_material::kDeck);
            }
        }
        closeChunk(chunks, chunk);
    }
    // The platform the cabin stops on at the floor, reaching out along the ring either side.
    TrackChunk   stand;
    const double deck = line.track.front().aboveGroundM + line.radiusM - kLiftPlatformM;
    stand.origin      = out * (foot - (0.5 * kLiftPlatformM));
    const Vec3d half  = around * ((0.5 * kLiftCabinM) + 4.0);
    addBeam(stand.mesh, stand.origin, (out * (deck + (0.5 * kLiftPlatformM))) - half,
            (out * (deck + (0.5 * kLiftPlatformM))) + half, axis, -out, kLiftCabinM, kLiftPlatformM,
            transit_material::kPlatform);
    closeChunk(chunks, stand);
}

}  // namespace

std::vector<TrackChunk> buildTrackMeshes(const std::vector<TramLine>& lines)
{
    std::vector<TrackChunk> chunks;
    for (const TramLine& line : lines)
    {
        if (line.kind == LineKind::SPOKE && !line.track.empty())
        {
            liftTower(chunks, line);
            continue;
        }
        TrackChunk chunk;
        double     chunkFrom = 0.0;
        for (std::size_t i = 0; i + 1 < line.track.size(); ++i)
        {
            if (chunk.mesh.indices.empty())
            {
                chunk.origin = line.track[i].position;
                chunkFrom    = line.track[i].alongM;
            }
            addTrackSegment(chunk.mesh, chunk.origin, line.track[i], line.track[i + 1],
                            line.track[i].carried && i % 2 == 0);
            if (line.track[i + 1].alongM - chunkFrom > kChunkM)
            {
                closeChunk(chunks, chunk);
            }
        }
        closeChunk(chunks, chunk);
        for (const TramStop& stop : line.stops)
        {
            chunks.push_back(platformAt(line, stop));
        }
    }
    return chunks;
}

CpuMesh buildTramMesh()
{
    using namespace transit_material;  // NOLINT(google-build-using-namespace): the names, just here
    CpuMesh    tram;
    const auto put = [&tram](const CpuMesh& part, const Vec3d& at, const Vec3d& half) {
        appendMesh(tram, part, glm::translate(Mat4d(1.0), at) * glm::scale(Mat4d(1.0), half));
    };
    const double halfLength = 0.5 * kTramLengthM;
    const double halfWidth  = 0.5 * kTramWidthM;

    // Underframe and bogies, a body with a window band, and a shallow roof.
    put(makeBox(Vec3f(1.0F), kSkirt), Vec3d(0.0, 0.42, 0.0),
        Vec3d(halfWidth - 0.06, 0.42, halfLength));
    for (const double end : {-1.0, 1.0})
    {
        put(makeBox(Vec3f(1.0F), kSkirt), Vec3d(0.0, 0.22, end * (halfLength - 1.9)),
            Vec3d(halfWidth - 0.2, 0.22, 1.0));
    }
    put(makeBox(Vec3f(1.0F), kBody), Vec3d(0.0, 1.55, 0.0), Vec3d(halfWidth, 0.72, halfLength));
    put(makeBox(Vec3f(1.0F), kGlass), Vec3d(0.0, 2.05, 0.0),
        Vec3d(halfWidth + 0.015, 0.44, halfLength - 0.35));
    put(makeBox(Vec3f(1.0F), kRoof), Vec3d(0.0, kTramHeightM - 0.14, 0.0),
        Vec3d(halfWidth - 0.05, 0.14, halfLength - 0.12));
    // Rounded ends: a narrower nose at each end of the body.
    for (const double end : {-1.0, 1.0})
    {
        put(makeBox(Vec3f(1.0F), kBody), Vec3d(0.0, 1.75, end * (halfLength + 0.22)),
            Vec3d(halfWidth - 0.35, 0.95, 0.22));
    }
    return tram;
}

CpuMesh buildLiftMesh()
{
    using namespace transit_material;  // NOLINT(google-build-using-namespace): the names, just here
    CpuMesh    lift;
    const auto put = [&lift](const CpuMesh& part, const Vec3d& at, const Vec3d& half) {
        appendMesh(lift, part, glm::translate(Mat4d(1.0), at) * glm::scale(Mat4d(1.0), half));
    };
    const double half   = 0.5 * kLiftCabinM;
    const double height = kLiftCabinHeightM;
    // A floor, a post at each corner, glass sides with a rail along them, and a roof.
    put(makeBox(Vec3f(1.0F), kSkirt), Vec3d(0.0, -0.15, 0.0), Vec3d(half, 0.15, half));
    for (const double x : {-1.0, 1.0})
    {
        for (const double z : {-1.0, 1.0})
        {
            put(makeBox(Vec3f(1.0F), kBody),
                Vec3d(x * (half - 0.08), 0.5 * height, z * (half - 0.08)),
                Vec3d(0.08, 0.5 * height, 0.08));
        }
        put(makeBox(Vec3f(1.0F), kGlass), Vec3d(x * (half - 0.03), 0.5 * height, 0.0),
            Vec3d(0.03, (0.5 * height) - 0.05, half - 0.16));
        put(makeBox(Vec3f(1.0F), kBody), Vec3d(x * (half - 0.12), 1.0, 0.0),
            Vec3d(0.04, 0.04, half - 0.16));
    }
    put(makeBox(Vec3f(1.0F), kRoof), Vec3d(0.0, height + 0.1, 0.0), Vec3d(half, 0.1, half));
    return lift;
}

}  // namespace StarshipSimulator
