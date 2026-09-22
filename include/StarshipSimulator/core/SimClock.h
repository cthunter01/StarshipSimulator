#pragma once

#include <cstdint>

namespace StarshipSimulator
{

/// Turns variable real frame times into a whole number of fixed simulation steps, so physics is
/// deterministic and independent of frame rate. Leftover time carries over to the next frame.
class SimClock
{
public:
    static constexpr double kDefaultStepSeconds = 1.0 / 120.0;
    static constexpr int kDefaultMaxSteps = 12;  // per frame; beyond this the simulation slows down

    explicit SimClock(double stepSeconds      = kDefaultStepSeconds,
                      int    maxStepsPerFrame = kDefaultMaxSteps);

    /// Adds real elapsed time and returns how many fixed steps to run now. Returns 0 while paused.
    /// Negative or non-finite times count as zero.
    [[nodiscard]] int advance(double realSeconds);

    [[nodiscard]] double stepSeconds() const { return stepSeconds_; }
    /// How far the leftover time reaches into the next step, 0..1 (for interpolating rendering).
    [[nodiscard]] double       alpha() const { return accumulator_ / stepSeconds_; }
    [[nodiscard]] double       simulatedSeconds() const;
    [[nodiscard]] std::int64_t stepCount() const { return stepCount_; }
    [[nodiscard]] bool         paused() const { return paused_; }
    void                       setPaused(bool paused) { paused_ = paused; }

private:
    double       stepSeconds_;
    int          maxStepsPerFrame_;
    double       accumulator_ = 0.0;
    std::int64_t stepCount_   = 0;
    bool         paused_      = false;
};

}  // namespace StarshipSimulator
