#pragma once

#include <cstdint>

namespace StarshipSimulator
{

/// SplitMix64 random numbers. Small, fast, and bit-identical on every compiler and platform, which
/// the std:: distributions are not; shared habitat files must regenerate the same world everywhere.
class SplitMix64
{
public:
    explicit constexpr SplitMix64(std::uint64_t seed) : state_(seed) { }

    constexpr std::uint64_t next()
    {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state_;
        z               = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
        z               = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31U);
    }

    /// Uniform in [0, 1), 53 random bits.
    constexpr double uniform() { return static_cast<double>(next() >> 11U) * 0x1.0p-53; }

    /// Uniform in [low, high).
    constexpr double uniform(double low, double high) { return low + ((high - low) * uniform()); }

private:
    std::uint64_t state_;
};

/// Derives an independent seed from a seed and a value (chunk index, octave, feature id...).
[[nodiscard]] constexpr std::uint64_t hashSeed(std::uint64_t seed, std::uint64_t value)
{
    return SplitMix64(seed ^ (value * 0xD6E8FEB86659FD93ULL)).next();
}

}  // namespace StarshipSimulator
