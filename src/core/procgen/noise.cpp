#include "StarshipSimulator/core/procgen/noise.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/rng.h"

namespace StarshipSimulator
{

namespace
{

constexpr int    kTableSize = 256;
constexpr double kSkew      = 1.0 / 3.0;
constexpr double kUnskew    = 1.0 / 6.0;
// Scales the sum of corner contributions (kernel radius^2 = 0.5, continuous) to about [-1, 1].
constexpr double kNormalization = 76.883;

/// Dot product of the offset with one of 12 gradient directions chosen by the hash (Perlin's
/// scheme).
double gradient(int hash, double x, double y, double z)
{
    const int    h = hash & 15;
    const double u = h < 8 ? x : y;
    double       v = z;
    if (h < 4)
    {
        v = y;
    }
    else if (h == 12 || h == 14)
    {
        v = x;
    }
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

/// Contribution of one simplex corner at the given offset.
double corner(int hash, double x, double y, double z)
{
    const double t = 0.5 - (x * x) - (y * y) - (z * z);
    if (t <= 0.0)
    {
        return 0.0;
    }
    const double t2 = t * t;
    return t2 * t2 * gradient(hash, x, y, z);
}

int floorToInt(double value)
{
    return static_cast<int>(std::floor(value));
}

}  // namespace

SimplexNoise::SimplexNoise(std::uint64_t seed) : permutation_(2 * std::size_t{kTableSize})
{
    std::vector<int> table(kTableSize);
    std::ranges::iota(table, 0);
    SplitMix64 random(seed);
    for (std::size_t i = kTableSize - 1; i > 0; --i)  // Fisher-Yates with our own RNG
    {
        const std::size_t j = random.next() % (i + 1);
        std::swap(table[i], table[j]);
    }
    for (std::size_t i = 0; i < permutation_.size(); ++i)
    {
        permutation_[i] = table[i % kTableSize];
    }
}

int SimplexNoise::hash(int i, int j, int k) const
{
    const auto index = [](int value) { return static_cast<std::size_t>(value & (kTableSize - 1)); };
    const int  a     = permutation_[index(k)];
    const int  b     = permutation_[index(j) + static_cast<std::size_t>(a)];
    return permutation_[index(i) + static_cast<std::size_t>(b)];
}

double SimplexNoise::sample(const Vec3d& p) const
{
    // Skew into the simplex grid and find the containing cell.
    const double skew = (p.x + p.y + p.z) * kSkew;
    const int    i    = floorToInt(p.x + skew);
    const int    j    = floorToInt(p.y + skew);
    const int    k    = floorToInt(p.z + skew);
    const double t    = static_cast<double>(i + j + k) * kUnskew;
    const double x0   = p.x - (static_cast<double>(i) - t);
    const double y0   = p.y - (static_cast<double>(j) - t);
    const double z0   = p.z - (static_cast<double>(k) - t);

    // Which of the six tetrahedra of the cube we are in.
    int i1 = 0;
    int j1 = 0;
    int k1 = 0;
    int i2 = 0;
    int j2 = 0;
    int k2 = 0;
    if (x0 >= y0)
    {
        if (y0 >= z0)
        {
            i1 = i2 = j2 = 1;
        }
        else if (x0 >= z0)
        {
            i1 = i2 = k2 = 1;
        }
        else
        {
            k1 = i2 = k2 = 1;
        }
    }
    else
    {
        if (y0 < z0)
        {
            k1 = j2 = k2 = 1;
        }
        else if (x0 < z0)
        {
            j1 = j2 = k2 = 1;
        }
        else
        {
            j1 = i2 = j2 = 1;
        }
    }

    const double x1 = x0 - i1 + kUnskew;
    const double y1 = y0 - j1 + kUnskew;
    const double z1 = z0 - k1 + kUnskew;
    const double x2 = x0 - i2 + (2.0 * kUnskew);
    const double y2 = y0 - j2 + (2.0 * kUnskew);
    const double z2 = z0 - k2 + (2.0 * kUnskew);
    const double x3 = x0 - 1.0 + (3.0 * kUnskew);
    const double y3 = y0 - 1.0 + (3.0 * kUnskew);
    const double z3 = z0 - 1.0 + (3.0 * kUnskew);

    const double sum = corner(hash(i, j, k), x0, y0, z0) +
                       corner(hash(i + i1, j + j1, k + k1), x1, y1, z1) +
                       corner(hash(i + i2, j + j2, k + k2), x2, y2, z2) +
                       corner(hash(i + 1, j + 1, k + 1), x3, y3, z3);
    return kNormalization * sum;
}

double SimplexNoise::fbm(const Vec3d& p, int octaves, double lacunarity, double gain) const
{
    double sum       = 0.0;
    double amplitude = 1.0;
    double total     = 0.0;
    double frequency = 1.0;
    for (int octave = 0; octave < octaves; ++octave)
    {
        // Offset each octave so their lattices don't line up at the origin.
        const Vec3d offset(17.3 * octave, -9.1 * octave, 31.7 * octave);
        sum += amplitude * sample((p * frequency) + offset);
        total += amplitude;
        amplitude *= gain;
        frequency *= lacunarity;
    }
    return total > 0.0 ? sum / total : 0.0;
}

double SimplexNoise::ridged(const Vec3d& p, int octaves, double lacunarity, double gain) const
{
    double sum       = 0.0;
    double amplitude = 1.0;
    double total     = 0.0;
    double frequency = 1.0;
    double weight    = 1.0;
    for (int octave = 0; octave < octaves; ++octave)
    {
        const Vec3d  offset(-23.9 * octave, 41.3 * octave, 7.7 * octave);
        const double ridge = 1.0 - std::abs(sample((p * frequency) + offset));
        const double value = ridge * ridge * weight;
        weight             = std::clamp(value * 2.0, 0.0, 1.0);  // detail follows the big ridges
        sum += amplitude * value;
        total += amplitude;
        amplitude *= gain;
        frequency *= lacunarity;
    }
    return total > 0.0 ? sum / total : 0.0;
}

}  // namespace StarshipSimulator
