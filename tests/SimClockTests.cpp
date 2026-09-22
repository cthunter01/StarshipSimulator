#include "StarshipSimulator/core/SimClock.h"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace
{

using StarshipSimulator::SimClock;

TEST(SimClock, RunsTwoFixedStepsPerSixtiethOfASecondAt120Hz)
{
    SimClock clock;
    EXPECT_EQ(clock.advance(1.0 / 60.0), 2);
    EXPECT_EQ(clock.stepCount(), 2);
    EXPECT_NEAR(clock.simulatedSeconds(), 1.0 / 60.0, 1e-12);
}

TEST(SimClock, CarriesLeftoverTimeToTheNextFrame)
{
    SimClock clock(0.01);
    EXPECT_EQ(clock.advance(0.006), 0);
    EXPECT_NEAR(clock.alpha(), 0.6, 1e-9);
    EXPECT_EQ(clock.advance(0.006), 1);
    EXPECT_NEAR(clock.alpha(), 0.2, 1e-9);
}

TEST(SimClock, PausedClockRunsNoStepsAndKeepsNoBacklog)
{
    SimClock clock(0.01);
    clock.setPaused(true);
    EXPECT_EQ(clock.advance(1.0), 0);
    clock.setPaused(false);
    EXPECT_EQ(clock.advance(0.005), 0);
}

TEST(SimClock, LongStallsAreClampedInsteadOfFastForwarded)
{
    SimClock clock(0.01, 5);
    EXPECT_EQ(clock.advance(10.0), 5);
    EXPECT_LE(clock.alpha(), 1.0);
    EXPECT_LE(clock.advance(0.001), 1);
}

TEST(SimClock, IgnoresInvalidTimes)
{
    SimClock clock;
    EXPECT_EQ(clock.advance(-1.0), 0);
    EXPECT_EQ(clock.advance(std::numeric_limits<double>::quiet_NaN()), 0);
    EXPECT_EQ(clock.advance(std::numeric_limits<double>::infinity()), 0);
    EXPECT_EQ(clock.stepCount(), 0);
}

TEST(SimClock, RejectsNonPositiveSteps)
{
    EXPECT_THROW(SimClock(0.0), std::invalid_argument);
    EXPECT_THROW(SimClock(0.01, 0), std::invalid_argument);
}

}  // namespace
