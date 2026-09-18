#include "StarshipSimulator/core/physics/rotating_frame.h"

#include <cmath>
#include <optional>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kImpactStep       = 1.0 / 240.0;
constexpr int    kBisectIterations = 40;

Vec3d rotateZ(const Vec3d& v, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {(c * v.x) - (s * v.y), (s * v.x) + (c * v.y), v.z};
}

}  // namespace

Vec3d RotatingFrame::centrifugal(const Vec3d& position) const
{
    const double w2 = omega_ * omega_;
    return {w2 * position.x, w2 * position.y, 0.0};
}

Vec3d RotatingFrame::coriolis(const Vec3d& velocity) const
{
    // -2 w x v with w = (0, 0, omega)
    return {2.0 * omega_ * velocity.y, -2.0 * omega_ * velocity.x, 0.0};
}

Vec3d RotatingFrame::acceleration(const BodyState& state) const
{
    return centrifugal(state.position) + coriolis(state.velocity);
}

BodyState RotatingFrame::toInertial(const BodyState& state, double phase) const
{
    const Vec3d spin(-omega_ * state.position.y, omega_ * state.position.x, 0.0);  // w x r
    return {.position = rotateZ(state.position, phase),
            .velocity = rotateZ(state.velocity + spin, phase)};
}

BodyState RotatingFrame::fromInertial(const BodyState& state, double phase) const
{
    const Vec3d position = rotateZ(state.position, -phase);
    const Vec3d spin(-omega_ * position.y, omega_ * position.x, 0.0);
    return {.position = position, .velocity = rotateZ(state.velocity, -phase) - spin};
}

double RotatingFrame::jacobi(const BodyState& state) const
{
    const double r2 = (state.position.x * state.position.x) + (state.position.y * state.position.y);
    return (0.5 * glm::dot(state.velocity, state.velocity)) - (0.5 * omega_ * omega_ * r2);
}

BodyState propagateFreeFlight(const RotatingFrame& frame, const BodyState& start, double seconds)
{
    const BodyState inertial = frame.toInertial(start, 0.0);
    const BodyState moved{.position = inertial.position + (inertial.velocity * seconds),
                          .velocity = inertial.velocity};
    return frame.fromInertial(moved, frame.omega() * seconds);
}

BodyState propagateUniformGravity(const BodyState& start, const Vec3d& up, double gravity,
                                  double seconds)
{
    const Vec3d acceleration = -gravity * up;
    return {.position = start.position + (start.velocity * seconds) +
                        (0.5 * seconds * seconds * acceleration),
            .velocity = start.velocity + (acceleration * seconds)};
}

std::optional<Impact> predictImpact(const RotatingFrame& frame, const HabitatGeometry& geometry,
                                    const BodyState& start, double maxSeconds)
{
    const auto height = [&](double t) {
        return geometry.ground(propagateFreeFlight(frame, start, t).position).heightAboveGround;
    };
    const auto steps    = static_cast<int>(std::ceil(maxSeconds / kImpactStep));
    double     previous = 0.0;
    for (int step = 1; step <= steps; ++step)
    {
        const double t = static_cast<double>(step) * kImpactStep;
        if (height(t) <= 0.0)
        {
            double low  = previous;
            double high = t;
            for (int i = 0; i < kBisectIterations; ++i)
            {
                const double mid = 0.5 * (low + high);
                if (height(mid) > 0.0)
                {
                    low = mid;
                }
                else
                {
                    high = mid;
                }
            }
            return Impact{.time = high, .state = propagateFreeFlight(frame, start, high)};
        }
        previous = t;
    }
    return std::nullopt;
}

void stepFreeBody(BodyState& state, const RotatingFrame& frame, const Vec3d& extraAcceleration,
                  double dt, bool includeCoriolis)
{
    if (includeCoriolis)
    {
        // The frames coincide at the start of the step, so the acceleration carries over as is.
        const BodyState inertial = frame.toInertial(state, 0.0);
        const BodyState moved{.position = inertial.position + (inertial.velocity * dt) +
                                          (0.5 * dt * dt * extraAcceleration),
                              .velocity = inertial.velocity + (extraAcceleration * dt)};
        state = frame.fromInertial(moved, frame.omega() * dt);
        return;
    }
    Vec3d velocity =
        state.velocity + ((frame.centrifugal(state.position) + extraAcceleration) * (0.5 * dt));
    state.position += velocity * dt;
    velocity += (frame.centrifugal(state.position) + extraAcceleration) * (0.5 * dt);
    state.velocity = velocity;
}

}  // namespace StarshipSimulator
