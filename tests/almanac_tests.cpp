#include "StarshipSimulator/core/almanac.h"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/math.h"

namespace
{

using namespace StarshipSimulator;  // NOLINT(google-build-using-namespace): test brevity

const HabitatGeometry& island()
{
    static const HabitatGeometry kGeometry{HabitatSpec{}};
    return kGeometry;
}

AlmanacState islandState()
{
    AlmanacState state;
    state.geometry = &island();
    state.metrics  = computeMetrics(island().spec());
    state.eye      = Vec3d(island().radius() - 1.7, 0.0, 0.0);
    return state;
}

TEST(Almanac, DropsLandBehindTheSpinByTheKnownAmount)
{
    // The reference figure for Island Three: a 1.5 m hand drop lands 2.7 cm antispinward.
    const double drop = dropDeflection(island(), 1.5);
    EXPECT_LT(drop, 0.0) << "a dropped object should land behind the spin";
    EXPECT_NEAR(std::abs(drop), 0.027, 0.002);

    // Twice as high is more than twice as far: the drift goes as the cube of the fall time.
    const double higher = std::abs(dropDeflection(island(), 3.0));
    EXPECT_GT(higher, 2.5 * std::abs(drop));
}

TEST(Almanac, JumpsLandAheadOfTheSpin)
{
    // The other way round from a drop, and by (4/3) w v^3 / g^2.
    const double speed    = 3.5;
    const double jump     = jumpDeflection(island(), speed);
    const double omega    = island().omega();
    const double gravity  = island().gravityAt(island().radius());
    const double expected = (4.0 / 3.0) * omega * speed * speed * speed / (gravity * gravity);
    EXPECT_GT(jump, 0.0) << "a jump should land ahead of the spin";
    EXPECT_NEAR(jump, expected, 0.1 * expected);
}

TEST(Almanac, GravityFallsAwayTowardTheAxis)
{
    const double half = radiusForGravity(island(), 0.5);
    EXPECT_NEAR(island().gravityAt(half), 0.5 * island().gravityAt(island().radius()), 1e-9);
    EXPECT_EQ(radiusForGravity(island(), 0.0), 0.0);
}

TEST(Almanac, ReadsAsPagesOfFactsWithNoGapsInThem)
{
    const std::vector<AlmanacPage> pages = almanacPages(islandState());
    ASSERT_GE(pages.size(), 5U);
    for (const AlmanacPage& page : pages)
    {
        EXPECT_FALSE(page.title.empty());
        EXPECT_GT(page.story.size(), 80U) << page.title << " has no paragraph worth reading";
        EXPECT_FALSE(page.facts.empty()) << page.title << " has no numbers in it";
        for (const AlmanacFact& fact : page.facts)
        {
            EXPECT_FALSE(fact.label.empty());
            EXPECT_FALSE(fact.value.empty()) << page.title << ": " << fact.label;
            // Nothing half-formatted: no stray placeholders or "nan"/"inf" leaking through.
            for (const std::string* text : {&fact.value, &fact.note})
            {
                EXPECT_EQ(text->find("{}"), std::string::npos) << *text;
                EXPECT_EQ(text->find("nan"), std::string::npos) << *text;
                EXPECT_EQ(text->find("inf"), std::string::npos) << *text;
            }
        }
    }
}

TEST(Almanac, QuotesTheHabitatItIsGiven)
{
    const std::vector<AlmanacPage> pages  = almanacPages(islandState());
    const auto                     joined = [&pages] {
        std::string all;
        for (const AlmanacPage& page : pages)
        {
            all += page.title;
            for (const AlmanacFact& fact : page.facts)
            {
                all += fact.label + fact.value + fact.note;
            }
        }
        return all;
    }();
    EXPECT_NE(joined.find("8.0 km across"), std::string::npos)
        << "the size should be Island Three's";
    EXPECT_NE(joined.find("0.47"), std::string::npos) << "the spin should be 0.473 rpm";
    EXPECT_NE(joined.find("structural steel"), std::string::npos)
        << "and steel should be enough to build it";
    EXPECT_NE(joined.find("buildable with materials we have today"), std::string::npos);

    // A habitat whose rim moves fast enough to need materials we do not have says so instead.
    AlmanacState wild                 = islandState();
    wild.metrics                      = islandState().metrics;
    wild.metrics.material             = MaterialClass::FUTURE_MATERIALS;
    wild.metrics.hoopSpecificStrength = 9.8e6;  // a ring the size of a continent
    std::string impossible;
    for (const AlmanacPage& page : almanacPages(wild))
    {
        for (const AlmanacFact& fact : page.facts)
        {
            impossible += fact.value + fact.note;
        }
    }
    EXPECT_NE(impossible.find("do not exist yet"), std::string::npos);
    EXPECT_NE(impossible.find("9.800 MJ/kg"), std::string::npos);
}

TEST(Almanac, TellsKalpanaOnesOwnStory)
{
    HabitatSpec spec;
    spec.kind    = HabitatKind::KALPANA_CYLINDER;
    spec.radiusM = 250.0;
    spec.lengthM = 325.0;
    const HabitatGeometry geometry{spec};
    AlmanacState          state;
    state.geometry       = &geometry;
    state.metrics        = computeMetrics(spec);
    state.eye            = Vec3d(geometry.radius() - 1.7, 0.0, 0.0);
    state.mirrorAngleRad = degreesToRadians(50.0);
    std::string text;
    for (const AlmanacPage& page : almanacPages(state))
    {
        text += page.title + page.story;
        for (const AlmanacFact& fact : page.facts)
        {
            text += fact.label + fact.value + fact.note;
        }
    }
    EXPECT_NE(text.find("500 m across, 325 m long"), std::string::npos) << text;
    EXPECT_NE(text.find("glass end"), std::string::npos);
    EXPECT_NE(text.find("ecliptic"), std::string::npos);
    for (const char* island : {"valley", "eight kilometres", "Eight kilometres", "km^3"})
    {
        EXPECT_EQ(text.find(island), std::string::npos) << island;
    }
}

TEST(Almanac, WorksWithoutASkyOrAHabitatToDescribe)
{
    EXPECT_TRUE(almanacPages(AlmanacState{}).empty());  // nothing to say about nothing
    // The sky pages hold up while the star data is still loading.
    AlmanacState loading = islandState();
    loading.sky          = nullptr;
    EXPECT_FALSE(almanacPages(loading).empty());
}

}  // namespace
