#include "StarshipSimulator/core/habitat/land_layout.h"

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

/// The inside of a sphere of radius R from 80 degrees south to 80 degrees north, as a profile.
std::shared_ptr<const MeridianProfile> sphereProfile(double radius)
{
    std::vector<Vec2d> points;
    for (int latitude = -80; latitude <= 80; ++latitude)
    {
        const double angle = degreesToRadians(static_cast<double>(latitude));
        points.emplace_back(radius * std::sin(angle), radius * std::cos(angle));
    }
    return std::make_shared<const MeridianProfile>(points);
}

/// The world point of a plan point on a band, on the floor.
Vec3d worldOf(const LandBand& band, const Vec2d& plan)
{
    const SurfaceSpot spot = band.toSurface(plan);
    const double      r    = band.profile->radiusAt(spot.z).value_or(0.0);
    return {r * std::cos(spot.theta), r * std::sin(spot.theta), spot.z};
}

/// Plan x, plan y and up at a plan point must be left-handed, like the valleys' plan: the town
/// layouts turn streets and houses with that handedness. On the band's middle line the plan is
/// also true to scale both ways and lies flat on the floor.
void expectPlanHandedness(const LandBand& band, const Vec2d& plan)
{
    const Vec3d here   = worldOf(band, plan);
    const Vec3d xAxis  = worldOf(band, plan + Vec2d(1.0, 0.0)) - here;
    const Vec3d yAxis  = worldOf(band, plan + Vec2d(0.0, 1.0)) - here;
    const Vec3d up     = HabitatGeometry::localUp(here);
    const Vec3d normal = glm::cross(xAxis, yAxis);
    EXPECT_LT(glm::dot(normal, up), 0.0) << "band " << band.index;
    if (plan.x == 0.0)
    {
        EXPECT_LT(glm::dot(normal, up), -0.999 * glm::length(normal));
        EXPECT_NEAR(glm::length(xAxis), 1.0, 1e-3);
        EXPECT_NEAR(glm::length(yAxis), 1.0, 1e-3);
    }
}

TEST(LandLayout, AnOneillCylindersBandsAreItsValleys)
{
    const HabitatGeometry geometry{HabitatSpec{}};  // Island Three
    ASSERT_EQ(geometry.bandCount(), 3);
    for (int i = 0; i < geometry.bandCount(); ++i)
    {
        const LandBand& band = geometry.band(i);
        EXPECT_EQ(band.axis, BandAxis::ALONG_Z);
        EXPECT_DOUBLE_EQ(band.centreTheta, geometry.landCenter(i));
        EXPECT_DOUBLE_EQ(band.halfWidthM, geometry.landHalfAngle() * geometry.radius());
        EXPECT_EQ(band.alongMinM, geometry.floorZMin());
        EXPECT_EQ(band.alongMaxM, geometry.floorZMax());
        EXPECT_FALSE(band.wraps);
        // Along a valley is z itself; across is the arc around from its middle line.
        const Vec2d plan = band.toPlan(-2000.0, geometry.landCenter(i) + (300.0 / 4000.0));
        EXPECT_NEAR(plan.x, 300.0, 1e-9);
        EXPECT_EQ(plan.y, -2000.0);
        expectPlanHandedness(band, plan);
        expectPlanHandedness(band, Vec2d(0.0, -2000.0));
        EXPECT_TRUE(geometry.onLand(-2000.0, geometry.landCenter(i)));
    }
    EXPECT_FALSE(geometry.onLand(-2000.0, geometry.windowCenter(0)));
    // Water lies level: on a cylinder's floor, at the same height everywhere.
    EXPECT_EQ(geometry.waterLevelAt(-2000.0), kWaterLevelM);
    EXPECT_EQ(geometry.waterLevelAt(9000.0), kWaterLevelM);
}

TEST(LandLayout, BandsAroundTheAxisGoAllTheWayRound)
{
    HabitatSpec spec;
    spec.kind                           = HabitatKind::BERNAL_SPHERE;
    spec.radiusM                        = 250.0;
    const auto                  profile = sphereProfile(250.0);
    const double                edge    = 250.0 * std::sin(degreesToRadians(35.0));
    const std::vector<LandBand> bands   = planLandBands(spec, profile, -edge, edge);
    ASSERT_EQ(bands.size(), 1U);
    const LandBand& band = bands.front();
    EXPECT_EQ(band.axis, BandAxis::AROUND);
    EXPECT_TRUE(band.wraps);
    EXPECT_NEAR(band.radiusM, 250.0, 0.05);                             // the equator
    EXPECT_NEAR(band.alongLengthM(), 2.0 * kPi * 250.0, 0.5);           // once round
    EXPECT_NEAR(band.halfWidthM, 250.0 * degreesToRadians(35.0), 0.5);  // 35 degrees of arc

    for (const Vec2d plan : {Vec2d(0.0), Vec2d(40.0, 300.0), Vec2d(-120.0, -700.0)})
    {
        const SurfaceSpot spot = band.toSurface(plan);
        const Vec2d       back = band.toPlan(spot.z, spot.theta);
        EXPECT_NEAR(back.x, plan.x, 1e-6);
        EXPECT_NEAR(back.y, plan.y, 1e-6);
        expectPlanHandedness(band, plan);
    }
    // Plan y wraps: half way round in either direction is the same place.
    const SurfaceSpot east = band.toSurface(Vec2d(0.0, kPi * band.radiusM));
    const SurfaceSpot west = band.toSurface(Vec2d(0.0, -kPi * band.radiusM));
    EXPECT_NEAR(std::remainder(east.theta - west.theta, 2.0 * kPi), 0.0, 1e-9);
    // Along the band points around the axis, spinward (increasing angle).
    EXPECT_NEAR(glm::dot(band.alongDirection(0.0), Vec3d(0.0, 1.0, 0.0)), 1.0, 1e-12);
}

}  // namespace
