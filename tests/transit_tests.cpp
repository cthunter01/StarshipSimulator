#include "StarshipSimulator/core/procgen/transit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

/// A small habitat with its towns and its tramway, built once.
struct Network
{
    HabitatGeometry       geometry;
    TerrainGrid           grid;     // as the track left it
    TerrainGrid           natural;  // as it was before
    Settlements           places;
    std::vector<TramLine> lines;
};

const Network& network()
{
    static const Network kBuilt = [] {
        OneillCylinderSpec spec;
        spec.radiusM = 500.0;
        spec.lengthM = 4000.0;
        Network made{.geometry = HabitatGeometry(spec),
                     .grid     = {},
                     .natural  = {},
                     .places   = {},
                     .lines    = {}};
        made.grid  = sampleTerrain(made.geometry, 4.0);
        made.lines = planTramLines(made.geometry, made.grid);
        // The land is graded to the track before anything else is built on it.
        made.natural = made.grid;
        gradeForTrack(made.grid, made.lines);
        made.places = planSettlements(made.geometry, made.grid);
        addTramStops(made.lines, made.places);
        return made;
    }();
    return kBuilt;
}

TEST(Transit, RunsALineDownEveryValleyAndUpEveryEndcap)
{
    const Network& net = network();
    ASSERT_FALSE(net.lines.empty());
    const auto valleys = std::ranges::count_if(
        net.lines, [](const TramLine& l) { return l.kind == LineKind::Valley; });
    const auto endcaps = std::ranges::count_if(
        net.lines, [](const TramLine& l) { return l.kind == LineKind::Endcap; });
    EXPECT_EQ(valleys, net.geometry.stripCount());
    EXPECT_EQ(endcaps, net.geometry.stripCount());

    for (const TramLine& line : net.lines)
    {
        EXPECT_GE(line.track.size(), 2U);
        EXPECT_GT(line.lengthM, 100.0);
        EXPECT_GE(line.stops.size(), 2U);
        EXPECT_GT(line.journeyS, line.lengthM / line.topSpeed);  // the stops take time too
        // The track measures itself correctly, and its points are in order.
        EXPECT_EQ(line.track.front().alongM, 0.0);
        EXPECT_NEAR(line.track.back().alongM, line.lengthM, 1e-9);
        for (std::size_t i = 1; i < line.track.size(); ++i)
        {
            EXPECT_GT(line.track[i].alongM, line.track[i - 1].alongM);
            EXPECT_NEAR(glm::distance(line.track[i].position, line.track[i - 1].position),
                        line.track[i].alongM - line.track[i - 1].alongM, 1e-6);
        }
        // It stays inside the habitat, and the whole line runs at one angle around it.
        for (const TrackPoint& point : line.track)
        {
            const double radius = std::hypot(point.position.x, point.position.y);
            EXPECT_LE(radius, net.geometry.radius() + 1.0);
            EXPECT_NEAR(
                std::remainder(HabitatGeometry::angleOf(point.position) - line.theta, 2.0 * kPi),
                0.0, 1e-9);
        }
    }
}

TEST(Transit, TheEndcapLineClimbsFromTheValleyToTheAxis)
{
    const Network& net   = network();
    const auto     found = std::ranges::find_if(
        net.lines, [](const TramLine& l) { return l.kind == LineKind::Endcap; });
    ASSERT_NE(found, net.lines.end());
    const TramLine* lift = &*found;
    const double    bottom =
        std::hypot(lift->track.front().position.x, lift->track.front().position.y);
    const double top = std::hypot(lift->track.back().position.x, lift->track.back().position.y);
    EXPECT_GT(bottom, 0.9 * net.geometry.radius());  // it starts on the valley floor
    EXPECT_LT(top, 0.3 * net.geometry.radius());     // and ends up near the axis
    // Up there the spin gravity has all but gone: that is the point of going.
    EXPECT_LT(net.geometry.gravityAt(top), 0.3 * net.geometry.gravityAt(bottom));
}

TEST(Transit, PointAlongFollowsTheTrack)
{
    const TramLine& line = network().lines.front();
    EXPECT_EQ(pointAlong(line, -10.0).position, line.track.front().position);
    EXPECT_EQ(pointAlong(line, line.lengthM + 10.0).position, line.track.back().position);
    for (int step = 0; step < 37; ++step)
    {
        const double     at   = (line.lengthM * step) / 37.0;
        const TrackPoint here = pointAlong(line, at);
        EXPECT_NEAR(here.alongM, at, 1e-9);
        // It really is on the track: within half a step of the nearest point of it.
        double nearest = 1e9;
        for (const TrackPoint& point : line.track)
        {
            nearest = std::min(nearest, glm::distance(point.position, here.position));
        }
        EXPECT_LT(nearest, 8.0) << "at " << at;
    }
}

TEST(Transit, TramsRunToTheTimetable)
{
    const Network& net = network();
    // Over a whole cycle every tram stays on its line, and each line's trams keep apart.
    const double cycle      = 2.0 * net.lines.front().journeyS;
    bool         sawStopped = false;
    bool         sawMoving  = false;
    for (int step = 0; step < 97; ++step)
    {
        const double            t     = (cycle * step) / 97.0;
        const std::vector<Tram> trams = tramsAt(net.lines, t);
        EXPECT_FALSE(trams.empty());
        for (const Tram& tram : trams)
        {
            const TramLine& line = net.lines[tram.line];
            EXPECT_GE(tram.alongM, 0.0);
            EXPECT_LE(tram.alongM, line.lengthM);
            EXPECT_NEAR(glm::length(tram.forward), 1.0, 1e-9);
            EXPECT_LT(glm::distance(tram.position, pointAlong(line, tram.alongM).position), 1e-6);
            sawStopped = sawStopped || tram.atStop;
            sawMoving  = sawMoving || !tram.atStop;
            if (tram.atStop)
            {
                EXPECT_EQ(tram.speedMS, 0.0);
                EXPECT_LT(std::abs(line.stops[tram.stop].alongM - tram.alongM), 1.0);
            }
        }
    }
    EXPECT_TRUE(sawStopped) << "no tram ever called at a stop";
    EXPECT_TRUE(sawMoving);
}

TEST(Transit, TramsAreWhereTheClockSaysAndMoveOn)
{
    const Network&          net   = network();
    const std::vector<Tram> now   = tramsAt(net.lines, 300.0);
    const std::vector<Tram> again = tramsAt(net.lines, 300.0);
    ASSERT_EQ(now.size(), again.size());
    for (std::size_t i = 0; i < now.size(); ++i)
    {
        EXPECT_EQ(now[i].position, again[i].position);
    }
    // A second later, the ones that were running have moved about a second's worth.
    const std::vector<Tram> later = tramsAt(net.lines, 301.0);
    ASSERT_EQ(later.size(), now.size());
    for (std::size_t i = 0; i < now.size(); ++i)
    {
        const double moved = glm::distance(now[i].position, later[i].position);
        EXPECT_LT(moved, net.lines[now[i].line].topSpeed * 1.5);
        if (!now[i].atStop && !later[i].atStop)
        {
            EXPECT_GT(moved, 1.0);
        }
    }
}

TEST(Transit, TheTrackAndTheCarsAreBuiltToSize)
{
    const std::vector<TrackChunk> chunks = buildTrackMeshes(network().lines);
    ASSERT_FALSE(chunks.empty());
    for (const TrackChunk& chunk : chunks)
    {
        EXPECT_FALSE(chunk.mesh.vertices.empty());
        EXPECT_EQ(chunk.mesh.indices.size() % 3, 0U);
        EXPECT_GT(chunk.radius, 0.0);
        for (const Vertex& vertex : chunk.mesh.vertices)
        {
            // Chunk-local, so the float positions stay small and precise.
            EXPECT_LT(glm::length(vertex.position), 1000.0F);
        }
    }

    const CpuMesh tram = buildTramMesh();
    ASSERT_FALSE(tram.vertices.empty());
    Vec3f low(1e9F);
    Vec3f high(-1e9F);
    for (const Vertex& vertex : tram.vertices)
    {
        low  = glm::min(low, vertex.position);
        high = glm::max(high, vertex.position);
    }
    EXPECT_NEAR(low.y, 0.0F, 0.01F);                                   // floor on the rails
    EXPECT_NEAR(high.y, static_cast<float>(kTramBodyHeightM), 0.01F);  // and the roof on top
    EXPECT_NEAR(high.x - low.x, static_cast<float>(kTramBodyWidthM), 0.1F);
    EXPECT_LT(high.z - low.z, static_cast<float>(kTramBodyLengthM) + 1.0F);
}

TEST(Transit, TheTrackLiesOnTheGroundItGraded)
{
    const Network& net   = network();
    int            cut   = 0;
    int            fill  = 0;
    int            flown = 0;
    for (const TramLine& line : net.lines)
    {
        for (const TrackPoint& point : line.track)
        {
            const double theta  = HabitatGeometry::angleOf(point.position);
            const double ground = net.grid.groundHeight(point.position.z, theta);
            const double rails  = point.railHeightM;
            if (point.carried)
            {
                // Over a trestle the land is left as it was found, well below the rails.
                ++flown;
                EXPECT_GT(point.aboveGroundM, 1.0);
                EXPECT_LT(point.aboveGroundM, 30.0);
                continue;
            }
            // Everywhere else the rails sit a rail's height above the ground, not in it.
            EXPECT_NEAR(rails - ground, 0.28, 0.35)
                << "the track is " << (rails - ground) << " m above the ground at z "
                << point.position.z;
            cut += point.aboveGroundM < -0.2 ? 1 : 0;
            fill += point.aboveGroundM > 0.2 ? 1 : 0;
        }
    }
    // A line that long over that much country has to do some of all three.
    EXPECT_GT(cut, 0);
    EXPECT_GT(fill, 0);
    EXPECT_GT(flown, 0);
}

TEST(Transit, TheEarthworksAreLocalAndTheAlignmentIsSmooth)
{
    const Network& net = network();
    // Away from the corridor the land is exactly as it was.
    const TramLine& line = net.lines.front();
    const double    z    = line.track[line.track.size() / 2].position.z;
    for (const double across : {30.0, 80.0, 200.0})
    {
        const double theta = line.theta + (across / line.radiusM);
        EXPECT_EQ(net.grid.groundHeight(z, theta), net.natural.groundHeight(z, theta))
            << across << " m from the track";
    }
    // And the graded corridor differs from the natural land somewhere: it did do some work.
    double moved = 0.0;
    for (const TrackPoint& point : line.track)
    {
        const double theta = HabitatGeometry::angleOf(point.position);
        moved = std::max(moved, std::abs(net.grid.groundHeight(point.position.z, theta) -
                                         net.natural.groundHeight(point.position.z, theta)));
    }
    EXPECT_GT(moved, 0.2);

    // The alignment rises and falls far more gently than the land it crosses.
    for (const TramLine& each : net.lines)
    {
        double steepest = 0.0;
        double land     = 0.0;
        for (std::size_t i = 1; i < each.track.size(); ++i)
        {
            const double run = each.track[i].alongM - each.track[i - 1].alongM;
            if (run < 1e-6)
            {
                continue;
            }
            const auto height = [&](const TrackPoint& p) {
                return p.railHeightM - p.aboveGroundM;  // the land as it was found
            };
            steepest =
                std::max(steepest,
                         std::abs(each.track[i].railHeightM - each.track[i - 1].railHeightM) / run);
            land =
                std::max(land, std::abs(height(each.track[i]) - height(each.track[i - 1])) / run);
        }
        EXPECT_LT(steepest, land + 1e-9) << "the alignment is rougher than the ground";
    }
}

TEST(Transit, KeepsTreesOffTheTrack)
{
    const Network&  net  = network();
    const TramLine& line = net.lines.front();
    const double    z    = line.track[line.track.size() / 2].position.z;
    EXPECT_TRUE(nearTrack(net.lines, z, line.theta, 6.5));
    EXPECT_TRUE(nearTrack(net.lines, z, line.theta + (5.0 / line.radiusM), 6.5));
    EXPECT_FALSE(nearTrack(net.lines, z, line.theta + (20.0 / line.radiusM), 6.5));
    EXPECT_FALSE(nearTrack(net.lines, net.geometry.floorZMax() + 5000.0, line.theta, 6.5));
}

}  // namespace
