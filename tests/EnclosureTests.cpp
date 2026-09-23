#include "StarshipSimulator/core/habitat/Enclosure.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

HabitatSpec torusSpec()
{
    HabitatSpec spec              = {};
    spec.kind                     = HabitatKind::STANFORD_TORUS;
    spec.radiusM                  = 895.0;
    spec.torus.tubeRadiusM        = 65.0;
    spec.torus.hubRadiusM         = 65.0;
    spec.torus.spokes             = 6;
    spec.torus.spokeRadiusM       = 7.5;
    spec.torus.ceilingWindowShare = 1.0 / 3.0;
    return spec;
}

Enclosure torusAir()
{
    const HabitatSpec spec = torusSpec();
    return {spec, std::make_shared<const MeridianProfile>(buildFloorProfile(spec))};
}

/// A point `r` from the axis at `degrees` round it.
Vec3d at(double r, double degrees, double z = 0.0)
{
    const double angle = degreesToRadians(degrees);
    return {r * std::cos(angle), r * std::sin(angle), z};
}

TEST(Enclosure, ATorusHoldsItsAirInTheTubeTheSpokesAndTheHub)
{
    const Enclosure  air   = torusAir();
    const TorusShape torus = air.torus().value_or(TorusShape{});
    ASSERT_TRUE(air.torus().has_value());
    // The tube: just above the floor, and 100 m up it; 140 m up is past the ceiling.
    EXPECT_TRUE(air.contains(at(894.0, 30.0)));
    EXPECT_FALSE(air.contains(at(896.0, 30.0)));
    EXPECT_TRUE(air.contains(at(795.0, 30.0)));
    EXPECT_FALSE(air.contains(at(755.0, 30.0)));
    EXPECT_TRUE(air.contains(at(830.0, 30.0, 60.0)));
    EXPECT_FALSE(air.contains(at(830.0, 30.0, 70.0)));
    // Up a spoke (one every 60 degrees, from 0), and beside it.
    EXPECT_TRUE(air.contains(at(755.0, 0.0)));
    EXPECT_TRUE(air.contains(at(400.0, 60.0, 5.0)));
    EXPECT_FALSE(air.contains(at(400.0, 60.0, 9.0)));
    EXPECT_FALSE(air.contains(at(400.0, 30.0)));
    // The hub, a drum 130 m across and 78 m long.
    EXPECT_TRUE(air.contains(Vec3d(0.0)));
    EXPECT_TRUE(air.contains(at(60.0, 30.0, 30.0)));
    EXPECT_FALSE(air.contains(at(60.0, 30.0, 45.0)));
    EXPECT_FALSE(air.contains(at(70.0, 30.0)));
    // The ceiling above the floor's lowest line.
    EXPECT_NEAR(torus.ceilingRadiusAt(0.0), 765.0, 1e-9);
    EXPECT_NEAR(torus.tubeAngleOf(at(765.0, 10.0)), kPi, 1e-9);
    EXPECT_NEAR(torus.tubeAngleOf(at(895.0, 10.0)), 0.0, 1e-9);
}

TEST(Enclosure, ClampingKeepsYouUnderTheCeiling)
{
    const Enclosure air = torusAir();
    // Inside with room to spare: left alone.
    EXPECT_EQ(air.clampInside(at(850.0, 30.0), 1.0), at(850.0, 30.0));
    EXPECT_EQ(air.clampInside(at(400.0, 0.0), 1.0), at(400.0, 0.0));
    // Through the ceiling between the spokes: back under it.
    const Vec3d back = air.clampInside(at(750.0, 30.0), 1.0);
    EXPECT_NEAR(glm::length(back - at(766.0, 30.0)), 0.0, 1e-9);
    EXPECT_TRUE(air.contains(back));
    // Out through a spoke's wall: back into the spoke.
    const Vec3d spoke = air.clampInside(at(400.0, 0.0, 12.0), 1.0);
    EXPECT_NEAR(spoke.z, 6.5, 1e-9);
    EXPECT_TRUE(air.contains(spoke));
}

TEST(Enclosure, OtherKindsHaveNoCeiling)
{
    const HabitatSpec spec = {};
    const Enclosure   air(spec, std::make_shared<const MeridianProfile>(buildFloorProfile(spec)));
    EXPECT_FALSE(air.torus().has_value());
    EXPECT_EQ(air.clampInside(Vec3d(0.0, 0.0, 99999.0), 1.0), Vec3d(0.0, 0.0, 99999.0));
    EXPECT_TRUE(air.contains(Vec3d(3999.0, 0.0, 0.0)));
    EXPECT_FALSE(air.contains(Vec3d(4001.0, 0.0, 0.0)));
}

}  // namespace
