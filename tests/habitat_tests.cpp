#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/day_schedule.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/habitat/mirror_optics.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/units.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

/// Island Three without terrain relief, for exact geometric checks.
OneillCylinderSpec flatIslandThree()
{
    OneillCylinderSpec spec;
    spec.terrain.hillHeightM     = 0.0;
    spec.terrain.mountainHeightM = 0.0;
    spec.terrain.riverWidthM     = 0.0;
    spec.terrain.lakesPerValley  = 0;
    return spec;
}

void expectNear(const Vec3d& actual, const Vec3d& expected, double tolerance)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

// ---- Metrics -----------------------------------------------------------------------------------

TEST(Metrics, IslandThreeReferenceNumbers)
{
    const HabitatMetrics m = computeMetrics(OneillCylinderSpec{});
    EXPECT_NEAR(m.omega, 0.049514, 1e-6);
    EXPECT_NEAR(m.periodS, 126.90, 0.01);
    EXPECT_NEAR(m.rpm, 0.4728, 1e-4);
    EXPECT_NEAR(m.rimSpeed, 198.06, 0.01);
    EXPECT_NEAR(m.floorGravity, 9.80665, 1e-9);
    EXPECT_NEAR(m.coriolisWalkingRatio, 0.01414, 1e-4);
    EXPECT_NEAR(m.axisPressureRatio, 0.792, 1e-3);
    EXPECT_NEAR(m.axisTemperatureDropK, 19.5, 0.05);
    EXPECT_NEAR(m.landAreaM2 / 1e6, 402.1, 0.1);
    EXPECT_NEAR(m.hoopSpecificStrength / 1e6, 0.0392, 1e-4);
    EXPECT_EQ(m.material, MaterialClass::STEEL);
    EXPECT_NEAR(m.population, 2.01e6, 0.01e6);
}

TEST(Metrics, BishopRingNeedsFutureMaterials)
{
    const double v2 = units::kStandardGravity * 1.0e6;  // 1000 km radius at 1 g
    EXPECT_EQ(materialClassFor(v2), MaterialClass::FUTURE_MATERIALS);
    EXPECT_FALSE(buildableToday(materialClassFor(v2)));
}

TEST(Metrics, SmallerHabitatsSpinFaster)
{
    EXPECT_NEAR(spinRate(250.0, 1.0), 0.19806, 1e-5);
    EXPECT_GT(spinRate(250.0, 1.0), spinRate(4000.0, 1.0));
}

// ---- Spec validation ---------------------------------------------------------------------------

TEST(HabitatSpec, DefaultsAreValidAndNonsenseIsNot)
{
    EXPECT_TRUE(validate(OneillCylinderSpec{}).empty());
    OneillCylinderSpec bad;
    bad.radiusM    = -5.0;
    bad.stripPairs = 0;
    EXPECT_GE(validate(bad).size(), 2U);
    EXPECT_THROW(HabitatGeometry{bad}, std::invalid_argument);

    OneillCylinderSpec tooShort;
    tooShort.lengthM = 8000.0;  // the conical ramp alone is 7 km deep
    EXPECT_FALSE(validate(tooShort).empty());
}

// ---- Meridian profile --------------------------------------------------------------------------

TEST(MeridianProfile, IslandThreeHasRampFloorAndDome)
{
    const MeridianProfile profile = buildOneillProfile(OneillCylinderSpec{});
    const double          lower   = 2000.0 / std::tan(degreesToRadians(25.0));
    const double          upper   = 1940.0 / std::tan(degreesToRadians(35.0));
    const double          floorZ  = -16000.0 + lower + upper;

    EXPECT_DOUBLE_EQ(profile.zMin(), -16000.0);
    EXPECT_DOUBLE_EQ(profile.zMax(), 20000.0);
    EXPECT_DOUBLE_EQ(profile.radiusAt(0.0).value_or(-1.0), 4000.0);
    EXPECT_NEAR(profile.radiusAt(floorZ).value_or(-1.0), 4000.0, 1e-9);
    EXPECT_NEAR(profile.radiusAt(floorZ - 100.0).value_or(-1.0),
                4000.0 - (100.0 * std::tan(degreesToRadians(25.0))), 1e-6);
    EXPECT_NEAR(profile.radiusAt(-16000.0).value_or(-1.0), 60.0, 1e-9);
    // Hemisphere: r = R cos(a) at z = L/2 + R sin(a); polyline error below half a metre.
    EXPECT_NEAR(profile.radiusAt(18000.0).value_or(-1.0), 4000.0 * std::cos(degreesToRadians(30.0)),
                0.5);
    EXPECT_FALSE(profile.radiusAt(20001.0).has_value());
}

TEST(MeridianProfile, ArcLengthRoundTrips)
{
    const MeridianProfile profile = buildOneillProfile(OneillCylinderSpec{});
    for (const double z : {-15000.0, -9000.0, 0.0, 15000.0, 19000.0})
    {
        EXPECT_NEAR(profile.pointAt(profile.arcAt(z)).x, z, 1e-6);
    }
    // The floor normal points at the axis, the anti-sunward ramp's toward +z and the axis.
    const Vec2d floorNormal = profile.inwardNormalAt(profile.arcAt(0.0));
    EXPECT_NEAR(floorNormal.x, 0.0, 1e-12);
    EXPECT_NEAR(floorNormal.y, -1.0, 1e-12);
    const Vec2d rampNormal = profile.inwardNormalAt(profile.arcAt(-12000.0));
    EXPECT_GT(rampNormal.x, 0.0);
    EXPECT_LT(rampNormal.y, 0.0);
}

// ---- Geometry ----------------------------------------------------------------------------------

TEST(HabitatGeometry, RegionsAlternateAroundTheAxis)
{
    const HabitatGeometry geometry(OneillCylinderSpec{});
    EXPECT_EQ(geometry.regionAt(0.0, 0.0).kind, RegionKind::WINDOW);
    EXPECT_EQ(geometry.regionAt(0.0, 0.0).index, 0);
    EXPECT_EQ(geometry.regionAt(0.0, kPi).kind, RegionKind::LAND);
    EXPECT_EQ(geometry.regionAt(0.0, kPi).index, 1);
    EXPECT_EQ(geometry.regionAt(0.0, degreesToRadians(240.0)).kind, RegionKind::WINDOW);
    EXPECT_EQ(geometry.regionAt(0.0, degreesToRadians(240.0)).index, 2);
    EXPECT_EQ(geometry.regionAt(0.0, degreesToRadians(359.0)).index, 0);
    EXPECT_EQ(geometry.regionAt(-12000.0, 0.0).kind, RegionKind::ENDCAP);
    EXPECT_EQ(geometry.regionAt(30000.0, 0.0).kind, RegionKind::OUTSIDE);
    EXPECT_NEAR(geometry.landCenter(1), kPi, 1e-12);
}

TEST(HabitatGeometry, WindowsAreFlatGlassAndLandHasHills)
{
    const HabitatGeometry geometry(OneillCylinderSpec{});
    EXPECT_EQ(geometry.terrainHeight(0.0, 0.0), 0.0);
    EXPECT_EQ(geometry.groundRadius(0.0, 0.0).value_or(-1.0), 4000.0);
    // Land right next to the glass is flush with it.
    const double edge = geometry.windowHalfAngle() + (10.0 / 4000.0);
    EXPECT_EQ(geometry.terrainHeight(0.0, edge), 0.0);

    double highest = 0.0;
    for (int i = 0; i < 200; ++i)
    {
        const double z = -8000.0 + (80.0 * i);
        const double h = geometry.naturalHeight(z, kPi);
        EXPECT_GE(h, 0.0);
        EXPECT_LE(h, geometry.spec().terrain.hillHeightM + 1e-9);
        highest = std::max(highest, h);
        // Rivers and lakes cut into it, down to wading depth.
        EXPECT_GE(geometry.terrainHeight(z, kPi), kWaterLevelM - kWaterDepthM - 1e-9);
    }
    EXPECT_GT(highest, 5.0);
}

TEST(HabitatGeometry, GroundUpAndSlope)
{
    const HabitatGeometry geometry(flatIslandThree());
    expectNear(HabitatGeometry::localUp(Vec3d(4000.0, 0.0, 0.0)), Vec3d(-1.0, 0.0, 0.0), 1e-12);

    const GroundSample floor = geometry.ground(Vec3d(-3998.3, 0.0, 0.0));
    EXPECT_NEAR(floor.heightAboveGround, 1.7, 1e-9);
    expectNear(floor.normal, Vec3d(1.0, 0.0, 0.0), 1e-6);
    EXPECT_NEAR(floor.slopeRadians, 0.0, 1e-6);
    EXPECT_EQ(floor.region.kind, RegionKind::LAND);

    const double       rampZ = geometry.floorZMin() - 1000.0;
    const GroundSample ramp  = geometry.ground(Vec3d(0.0, -3000.0, rampZ));
    EXPECT_NEAR(ramp.slopeRadians, degreesToRadians(25.0), 1e-3);
    EXPECT_GT(ramp.normal.z, 0.0);  // faces into the habitat, away from the end
}

TEST(HabitatGeometry, WalkableRangeStopsAtTheEnds)
{
    const HabitatGeometry geometry(OneillCylinderSpec{});
    EXPECT_NEAR(geometry.walkableZMin(), -15999.0, 1e-9);
    EXPECT_NEAR(geometry.walkableZMax(), 19999.0, 1e-9);
}

TEST(HabitatGeometry, RaycastFindsTheFarSide)
{
    const HabitatGeometry geometry(flatIslandThree());
    // From the valley floor at theta = pi straight up through the axis: the far window at 8 km.
    const auto hit = geometry.raycast(Vec3d(-3998.3, 0.0, 0.0), Vec3d(1.0, 0.0, 0.0), 10000.0);
    EXPECT_NEAR(hit.value_or(-1.0), 7998.3, 0.01);
    EXPECT_FALSE(
        geometry.raycast(Vec3d(-3998.3, 0.0, 0.0), Vec3d(1.0, 0.0, 0.0), 100.0).has_value());
}

// ---- Mirrors and sunlight ----------------------------------------------------------------------

TEST(MirrorOptics, FortyFiveDegreesIsNoon)
{
    const Vec3d sun = apparentSunDirection(0.0, degreesToRadians(45.0));
    expectNear(sun, Vec3d(1.0, 0.0, 0.0), 1e-12);  // straight up for the valley at theta = pi
    EXPECT_NEAR(sunElevation(degreesToRadians(45.0)), kPi / 2.0, 1e-12);
}

TEST(MirrorOptics, TheSunMovesAlongTheAxisNotEastWest)
{
    const Vec3d afternoon = apparentSunDirection(0.0, degreesToRadians(60.0));
    EXPECT_NEAR(afternoon.y, 0.0, 1e-12);  // never sideways
    EXPECT_LT(afternoon.z, 0.0);           // toward the anti-sunward end
    EXPECT_NEAR(sunElevation(degreesToRadians(60.0)), degreesToRadians(60.0), 1e-12);
    EXPECT_GT(apparentSunDirection(0.0, degreesToRadians(30.0)).z, 0.0);  // toward the sunward end
}

TEST(MirrorOptics, BeamIsTheSunlightReflectedByTheMirror)
{
    for (const double alphaDeg : {10.0, 30.0, 45.0, 70.0, 85.0})
    {
        const double alpha  = degreesToRadians(alphaDeg);
        const double window = degreesToRadians(120.0);
        const Vec3d  normal = mirrorNormal(window, alpha);
        const Vec3d  sunlight(0.0, 0.0, -1.0);
        const Vec3d  reflected = sunlight - (2.0 * glm::dot(sunlight, normal) * normal);
        expectNear(reflected, beamTravelDirection(window, alpha), 1e-12);
        EXPECT_NEAR(glm::dot(normal, mirrorDirection(window, alpha)), 0.0, 1e-12);
        // The beam heads inward through its window, across the axis to the opposite valley.
        const Vec3d  beam       = beamTravelDirection(window, alpha);
        const double crossAngle = std::atan2(beam.y, beam.x);
        EXPECT_NEAR(std::abs(std::remainder(crossAngle - window, 2.0 * kPi)), kPi, 1e-9);
    }
}

TEST(MirrorOptics, DaylightFadesAtDuskAndNight)
{
    EXPECT_DOUBLE_EQ(daylightFactor(degreesToRadians(45.0)), 1.0);
    EXPECT_DOUBLE_EQ(daylightFactor(degreesToRadians(90.0)), 0.0);
    EXPECT_DOUBLE_EQ(daylightFactor(degreesToRadians(120.0)), 0.0);
    EXPECT_DOUBLE_EQ(daylightFactor(0.0), 0.0);
    EXPECT_GT(daylightFactor(degreesToRadians(88.0)), 0.0);
    EXPECT_LT(daylightFactor(degreesToRadians(88.0)), 1.0);

    const HabitatGeometry geometry(OneillCylinderSpec{});
    const auto            beams = sunBeams(geometry, degreesToRadians(60.0));
    ASSERT_EQ(beams.size(), 3U);
    EXPECT_NEAR(beams[0].intensity, 0.9, 1e-12);  // mirror reflectivity
}

// ---- Day schedule -----------------------------------------------------------------------------

TEST(DaySchedule, SunriseNoonSunsetAndMidnight)
{
    const DayScheduleSpec day;  // 14 h of daylight from 06:00, noon at 45, night at 105 degrees
    EXPECT_NEAR(scheduledMirrorAngleDeg(day, 6.0), 90.0, 1e-9);
    EXPECT_NEAR(scheduledMirrorAngleDeg(day, 13.0), 45.0, 1e-9);
    EXPECT_NEAR(scheduledMirrorAngleDeg(day, 20.0), 90.0, 1e-9);
    EXPECT_NEAR(sunsetHour(day), 20.0, 1e-12);
    EXPECT_NEAR(scheduledMirrorAngleDeg(day, 1.0), 105.0, 1e-9);   // halfway through the night
    EXPECT_NEAR(scheduledMirrorAngleDeg(day, 25.0), 105.0, 1e-9);  // hours wrap
    EXPECT_NEAR(scheduledMirrorAngleDeg(day, -23.0), 105.0, 1e-9);
}

TEST(DaySchedule, DaylightExactlyWhileTheMirrorsAreBelowNinety)
{
    const DayScheduleSpec day;
    for (int quarter = 0; quarter < 96; ++quarter)
    {
        const double hour    = quarter / 4.0;
        const double angle   = scheduledMirrorAngleDeg(day, hour);
        const bool   daytime = hour > 6.0 && hour < 20.0;
        EXPECT_EQ(daylightFactor(degreesToRadians(angle)) > 0.0, daytime) << hour;
    }
}

TEST(DaySchedule, IsContinuous)
{
    const DayScheduleSpec day{.enabled        = true,
                              .dayLengthHours = 10.0,
                              .sunriseHour    = 22.0,
                              .noonAngleDeg   = 60.0,
                              .nightAngleDeg  = 120.0};
    double                previous = scheduledMirrorAngleDeg(day, 0.0);
    for (int step = 1; step <= 2400; ++step)
    {
        const double hour  = step / 100.0;
        const double angle = scheduledMirrorAngleDeg(day, hour);
        EXPECT_LT(std::abs(angle - previous), 0.2) << hour;
        previous = angle;
    }
}

TEST(HabitatSpec, PartnerMirrorsMustClearEachOther)
{
    OneillCylinderSpec spec;
    EXPECT_GE(spec.partner.separationM, minimumPartnerSeparation(spec));
    spec.partner.separationM = 20000.0;
    EXPECT_FALSE(validate(spec).empty());
    spec.partner.enabled = false;
    EXPECT_TRUE(validate(spec).empty());
}

}  // namespace
