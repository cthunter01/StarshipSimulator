#include "StarshipSimulator/core/procgen/birds.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/rng.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kMinHeightM = 14.0;  // above the ground, at the bottom of a flock's wheel
constexpr double kMaxHeightM = 110.0;
constexpr double kBeatsPerS  = 3.1;  // wing beats

/// One flock: where it wheels, how wide, how fast and how many.
struct Flock
{
    double z      = 0.0;
    double theta  = 0.0;
    double radius = 0.0;  // of the habitat, at the flock's height
    double wheelM = 60.0;
    double period = 40.0;  // seconds to go round once (from the flying speed)
    double spanM  = 0.9;
    double spread = 12.0;  // how far birds stray from the ring
    int    birds  = 10;
    double turn   = 1.0;  // +1 or -1
};

std::optional<Flock> flockIn(const HabitatGeometry& geometry, std::int64_t cellZ,
                             std::int64_t cellAround, double cellM, double cellArc,
                             std::uint64_t seed, const BirdSettings& settings, double drift)
{
    SplitMix64 rng(hashSeed(hashSeed(seed, static_cast<std::uint64_t>(cellZ)),
                            static_cast<std::uint64_t>(cellAround) + 0xB1D5ULL));
    if (rng.uniform() > settings.flockChance)
    {
        return std::nullopt;
    }
    Flock flock;
    flock.z     = ((static_cast<double>(cellZ) + rng.uniform()) * cellM) + drift;
    flock.theta = (static_cast<double>(cellAround) + rng.uniform()) * cellArc;
    const std::optional<double> ground = geometry.groundRadius(flock.z, flock.theta);
    if (!ground)
    {
        return std::nullopt;  // over a window, or off the end of the floor
    }
    const double height = rng.uniform(kMinHeightM, kMaxHeightM);
    flock.radius        = *ground - height;
    flock.wheelM        = rng.uniform(25.0, 120.0);
    // Birds fly at a bird's speed, so a wide ring simply takes longer to go round.
    flock.period = (2.0 * kPi * flock.wheelM) / rng.uniform(7.0, 16.0);
    flock.spanM  = rng.uniform(0.7, 2.1);
    flock.spread = 6.0 + (0.35 * flock.wheelM);
    flock.birds  = 4 + static_cast<int>(26.0 * rng.uniform() * rng.uniform());
    flock.turn   = rng.uniform() < 0.5 ? -1.0 : 1.0;
    return flock;
}

}  // namespace

std::vector<Bird> birdsNear(const HabitatGeometry& geometry, const Vec3d& camera, double seconds,
                            std::uint64_t seed, const BirdSettings& settings)
{
    std::vector<Bird> birds;
    const double      cellM = std::max(50.0, settings.flockCellM);
    const double      R     = geometry.radius();
    // The cells wrap exactly once around the habitat.
    const auto   around  = std::max<std::int64_t>(4, std::llround(2.0 * kPi * R / cellM));
    const double cellArc = (2.0 * kPi) / static_cast<double>(around);
    const double drift   = settings.windMS * seconds;

    const double cameraTheta = HabitatGeometry::angleOf(camera);
    const auto   zFrom =
        static_cast<std::int64_t>(std::floor((camera.z - settings.rangeM - drift) / cellM));
    const auto zTo =
        static_cast<std::int64_t>(std::ceil((camera.z + settings.rangeM - drift) / cellM));
    // How many cells around cover the range (the arc at the floor).
    const auto spread = std::max<std::int64_t>(
        1, static_cast<std::int64_t>(std::ceil(settings.rangeM / (R * cellArc))) + 1);
    const auto centre = static_cast<std::int64_t>(std::floor(cameraTheta / cellArc));

    const double rangeSq = settings.rangeM * settings.rangeM;
    for (std::int64_t cz = zFrom; cz <= zTo; ++cz)
    {
        for (std::int64_t step = -spread; step <= spread; ++step)
        {
            const std::int64_t         wrapped = (((centre + step) % around) + around) % around;
            const std::optional<Flock> flock =
                flockIn(geometry, cz, wrapped, cellM, cellArc, seed, settings, drift);
            if (!flock)
            {
                continue;
            }
            SplitMix64 rng(hashSeed(hashSeed(seed, static_cast<std::uint64_t>(cz)),
                                    static_cast<std::uint64_t>(wrapped) + 0x51D5ULL));
            for (int i = 0; i < flock->birds; ++i)
            {
                if (std::cmp_greater_equal(birds.size(), settings.maxBirds))
                {
                    return birds;
                }
                // Around the ring: evenly spaced, each bird a little off it.
                const double own   = rng.uniform();
                const double phase = (2.0 * kPi *
                                      ((flock->turn * seconds / flock->period) +
                                       (static_cast<double>(i) / flock->birds))) +
                                     (0.6 * own);
                const double wheel = flock->wheelM + (flock->spread * (own - 0.5));
                const double lift  = flock->spread * 0.5 * (rng.uniform() - 0.5);
                const double dz    = wheel * std::cos(phase);
                const double ds    = wheel * std::sin(phase);
                const double r     = flock->radius - lift;
                const double theta = flock->theta + (ds / std::max(r, 1.0));
                const double z     = flock->z + dz;
                const Vec3d  p(r * std::cos(theta), r * std::sin(theta), z);
                const Vec3d  offset = p - camera;
                if (glm::dot(offset, offset) > rangeSq)
                {
                    continue;
                }
                // Flying along the ring: the tangent of the circle, turned into the habitat frame.
                const Vec3d along(-std::sin(theta), std::cos(theta), 0.0);
                const Vec3d ahead = glm::normalize(
                    (along * std::cos(phase) - Vec3d(0.0, 0.0, 1.0) * std::sin(phase)) *
                    flock->turn);
                Bird bird;
                bird.position  = p;
                bird.forward   = ahead;
                bird.wingspanM = flock->spanM;
                bird.wingBeat  = std::sin(
                    2.0 * kPi * ((kBeatsPerS * seconds) + own + (0.17 * static_cast<double>(i))));
                birds.push_back(bird);
            }
        }
    }
    return birds;
}

}  // namespace StarshipSimulator
