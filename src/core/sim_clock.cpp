#include "StarshipSimulator/core/sim_clock.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace StarshipSimulator
{

SimClock::SimClock(double stepSeconds, int maxStepsPerFrame)
  : stepSeconds_(stepSeconds), maxStepsPerFrame_(maxStepsPerFrame)
{
    if (!(stepSeconds > 0.0) || maxStepsPerFrame < 1)
    {
        throw std::invalid_argument(
            "SimClock needs a positive step and at least one step per frame");
    }
}

int SimClock::advance(double realSeconds)
{
    if (paused_ || !std::isfinite(realSeconds) || realSeconds <= 0.0)
    {
        return 0;
    }
    accumulator_ += realSeconds;
    const double available = std::floor(accumulator_ / stepSeconds_);
    const int steps = static_cast<int>(std::min(available, static_cast<double>(maxStepsPerFrame_)));
    accumulator_ -= static_cast<double>(steps) * stepSeconds_;
    // After a long stall (debugger, window drag) drop the backlog instead of fast-forwarding.
    accumulator_ = std::min(accumulator_, stepSeconds_);
    stepCount_ += steps;
    return steps;
}

double SimClock::simulatedSeconds() const
{
    return static_cast<double>(stepCount_) * stepSeconds_;
}

}  // namespace StarshipSimulator
