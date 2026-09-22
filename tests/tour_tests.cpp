#include "StarshipSimulator/core/tour.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

const HabitatGeometry& island()
{
    static const HabitatGeometry kGeometry{OneillCylinderSpec{}};
    return kGeometry;
}

/// A habitat a fortieth of Island Three's size: the same tours have to fit in this one too.
const HabitatGeometry& playground()
{
    static const HabitatGeometry kGeometry{[] {
        OneillCylinderSpec spec;
        spec.radiusM           = 250.0;
        spec.lengthM           = 800.0;
        spec.sunwardEndcap     = makeEndcap(EndcapShape::FLAT);
        spec.antisunwardEndcap = makeEndcap(EndcapShape::FLAT);
        return spec;
    }()};
    return kGeometry;
}

void checkStop(const TourStop& stop, const HabitatGeometry& geometry, const std::string& tour)
{
    EXPECT_GT(stop.caption.size(), 40U) << "a stop with nothing to read";
    EXPECT_GE(stop.holdS, 4.0) << "no time to read it";
    // Inside the habitat, and not buried in the ground or outside the hull.
    const double radius = std::hypot(stop.eye.x, stop.eye.y);
    EXPECT_LT(radius, geometry.radius()) << tour;
    EXPECT_GT(stop.eye.z, geometry.profile().zMin()) << tour;
    EXPECT_LT(stop.eye.z, geometry.profile().zMax()) << tour;
    EXPECT_GE(stop.pitchDeg, -90.0);
    EXPECT_LE(stop.pitchDeg, 90.0);
}

void checkTours(const HabitatGeometry& geometry)
{
    const std::vector<Tour> tours = habitatTours(geometry, "Test Habitat");
    ASSERT_GE(tours.size(), 2U);
    for (const Tour& tour : tours)
    {
        EXPECT_FALSE(tour.name.empty());
        EXPECT_FALSE(tour.blurb.empty());
        EXPECT_GE(tour.stops.size(), 2U);
        EXPECT_GT(tour.lengthS(), 20.0);
        EXPECT_LT(tour.lengthS(), 8.0 * 60.0) << tour.name << " goes on too long";
        for (const TourStop& stop : tour.stops)
        {
            checkStop(stop, geometry, tour.name);
        }
    }
}

TEST(Tour, EveryTourIsWorthTakingAndFitsTheHabitat)
{
    checkTours(island());
    checkTours(playground());
}

TEST(Tour, TalksAboutTheHabitatItIsIn)
{
    // The captions quote the habitat's own numbers: a tour of a 500 m cylinder must not tell you
    // it is eight kilometres across.
    const std::vector<Tour> big   = habitatTours(island(), "Island Three");
    const std::vector<Tour> small = habitatTours(playground(), "Playground");
    EXPECT_NE(big.front().name.find("Island Three"), std::string::npos);
    EXPECT_NE(small.front().name.find("Playground"), std::string::npos);
    const std::string& opening = big.front().stops.front().caption;
    const std::string& tiny    = small.front().stops.front().caption;
    EXPECT_NE(opening.find("8.0 kilometres"), std::string::npos) << opening;
    EXPECT_NE(tiny.find("500 metres"), std::string::npos) << tiny;
    EXPECT_EQ(tiny.find("kilometres"), std::string::npos) << tiny;
}

TEST(Tour, FliesFromStopToStopAndStopsAtTheEnd)
{
    const std::vector<Tour> tours = habitatTours(island(), "Island Three");
    const Tour&             tour  = tours.front();
    EXPECT_EQ(tourAt(tour, 0.0).eye, tour.stops.front().eye);

    // Halfway through the first hold, it is standing on the first stop with something to read.
    const TourFrame reading = tourAt(tour, tour.stops.front().travelS + 2.0);
    EXPECT_EQ(reading.stop, 0U);
    EXPECT_EQ(reading.caption, tour.stops.front().caption);
    EXPECT_GT(reading.captionFade, 0.5);
    EXPECT_FALSE(reading.finished);

    // It never jumps: between one moment and the next the camera moves a sensible distance.
    Vec3d        last  = tourAt(tour, 0.0).eye;
    double       most  = 0.0;
    const double step  = 1.0 / 30.0;
    const int    steps = static_cast<int>(tour.lengthS() / step);
    for (int i = 1; i <= steps; ++i)
    {
        const Vec3d now = tourAt(tour, i * step).eye;
        most            = std::max(most, glm::distance(now, last));
        last            = now;
    }
    EXPECT_LT(most, 40.0) << "the camera jumps between stops";

    const TourFrame over = tourAt(tour, tour.lengthS() + 5.0);
    EXPECT_TRUE(over.finished);
    EXPECT_EQ(over.eye, tour.stops.back().eye);
    EXPECT_EQ(tourAt(tour, -3.0).eye, tourAt(tour, 0.0).eye);  // before the start is the start
}

TEST(Tour, TurnsTheShorterWayRound)
{
    Tour tour;
    tour.name  = "turn";
    tour.blurb = "b";
    TourStop one;
    one.caption  = "Facing almost due north, about to turn a little to the left.";
    one.yawDeg   = 350.0;
    one.travelS  = 0.0;
    one.holdS    = 1.0;
    TourStop two = one;
    two.yawDeg   = 10.0;
    two.travelS  = 2.0;
    tour.stops   = {one, two};

    // 350 -> 10 is twenty degrees the short way, not three hundred and forty the long way.
    const double half = tourAt(tour, one.holdS + 1.0).yawDeg;
    EXPECT_GT(half, 350.0);
    EXPECT_LT(half, 370.0);
}

TEST(Tour, AnEmptyTourIsOverBeforeItStarts)
{
    const Tour      nothing;
    const TourFrame frame = tourAt(nothing, 0.0);
    EXPECT_TRUE(frame.finished);
    EXPECT_EQ(nothing.lengthS(), 0.0);
}

}  // namespace
