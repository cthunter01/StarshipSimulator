#pragma once

#include <optional>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/math.h"

// Motion in a spinning habitat, described in the habitat's own rotating frame (spin axis +Z).
// Seen from outside (the inertial frame) a free-flying object moves in a straight line; seen from
// inside it feels two fictitious accelerations:
//   centrifugal  -w x (w x r) = w^2 (x, y, 0)   the "gravity" pushing outward to the floor
//   Coriolis     -2 w x v                       sideways drift of anything moving
namespace StarshipSimulator
{

struct BodyState
{
    Vec3d position{0.0};
    Vec3d velocity{0.0};
};

class RotatingFrame
{
public:
    explicit RotatingFrame(double omega) : omega_(omega) { }

    [[nodiscard]] double omega() const { return omega_; }
    [[nodiscard]] Vec3d  centrifugal(const Vec3d& position) const;
    [[nodiscard]] Vec3d  coriolis(const Vec3d& velocity) const;
    [[nodiscard]] Vec3d  acceleration(const BodyState& state) const;

    /// Converts to the inertial frame, given how far the habitat has turned (phase, radians).
    [[nodiscard]] BodyState toInertial(const BodyState& state, double phase) const;
    [[nodiscard]] BodyState fromInertial(const BodyState& state, double phase) const;

    /// Jacobi integral: energy in the rotating frame, conserved in free flight.
    [[nodiscard]] double jacobi(const BodyState& state) const;

private:
    double omega_;
};

/// Exact free flight: where a body is after t seconds with no forces but spin (straight line in the
/// inertial frame, transformed back).
[[nodiscard]] BodyState propagateFreeFlight(const RotatingFrame& frame, const BodyState& start,
                                            double seconds);

/// Same trajectory with the spin frozen: constant gravity g along -up, as on a planet. Used to show
/// how far spin deflects a throw.
[[nodiscard]] BodyState propagateUniformGravity(const BodyState& start, const Vec3d& up,
                                                double gravity, double seconds);

struct Impact
{
    double    time = 0.0;
    BodyState state;
};

/// When and where a free-flying body first touches the ground (exact trajectory, bisected).
[[nodiscard]] std::optional<Impact> predictImpact(const RotatingFrame&   frame,
                                                  const HabitatGeometry& geometry,
                                                  const BodyState& start, double maxSeconds);

/// Advances a free body by dt. With Coriolis (the physical case) the step is exact: a straight
/// line in the inertial frame, with any extra acceleration (thrust) held constant over the step.
/// Without Coriolis (comfort mode) only centrifugal gravity acts, integrated by velocity Verlet.
void stepFreeBody(BodyState& state, const RotatingFrame& frame, const Vec3d& extraAcceleration,
                  double dt, bool includeCoriolis = true);

}  // namespace StarshipSimulator
