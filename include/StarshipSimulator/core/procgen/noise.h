#pragma once

#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// Seeded 3D simplex noise in double precision (after Stefan Gustavson's reference implementation).
/// Sampling on the curved habitat surface in 3D makes the terrain seamless all the way around.
class SimplexNoise
{
public:
    explicit SimplexNoise(std::uint64_t seed);

    /// Smooth noise in about [-1, 1] with features roughly one unit apart.
    [[nodiscard]] double sample(const Vec3d& p) const;

    /// Fractal sum of octaves, normalized to about [-1, 1].
    [[nodiscard]] double fbm(const Vec3d& p, int octaves, double lacunarity = 2.0,
                             double gain = 0.5) const;

    /// Ridged fractal noise in [0, 1]: sharp crests, as for mountain ridges.
    [[nodiscard]] double ridged(const Vec3d& p, int octaves, double lacunarity = 2.0,
                                double gain = 0.5) const;

private:
    [[nodiscard]] int hash(int i, int j, int k) const;

    std::vector<int> permutation_;  // 512 entries: a shuffled 0..255, repeated
};

}  // namespace StarshipSimulator
