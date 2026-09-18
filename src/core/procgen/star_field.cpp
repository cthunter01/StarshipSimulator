#include "StarshipSimulator/core/procgen/star_field.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/rng.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kBrightestMagnitude = -1.5;
constexpr double kFaintestMagnitude  = 6.5;

}  // namespace

Vec3f blackbodyColor(double kelvin)
{
    // Tanner Helland's fit of black-body colour (sRGB), converted to linear.
    const double t = std::clamp(kelvin, 1000.0, 40000.0) / 100.0;
    double       r = 255.0;
    double       g = 0.0;
    double       b = 255.0;
    if (t <= 66.0)
    {
        g = (99.4708025861 * std::log(t)) - 161.1195681661;
        b = t <= 19.0 ? 0.0 : (138.5177312231 * std::log(t - 10.0)) - 305.0447927307;
    }
    else
    {
        r = 329.698727446 * std::pow(t - 60.0, -0.1332047592);
        g = 288.1221695283 * std::pow(t - 60.0, -0.0755148492);
    }
    const auto linear = [](double srgb) {
        const double c = std::clamp(srgb, 0.0, 255.0) / 255.0;
        return c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4);
    };
    const Vec3d color(linear(r), linear(g), linear(b));
    return Vec3f(color / std::max({color.x, color.y, color.z}));
}

std::vector<GpuStar> generateStarField(std::uint64_t seed, std::size_t count)
{
    std::vector<GpuStar> stars;
    stars.reserve(count);
    SplitMix64 random(seed);
    for (std::size_t i = 0; i < count; ++i)
    {
        // Uniform direction on the sphere.
        const double z   = random.uniform(-1.0, 1.0);
        const double phi = random.uniform(0.0, 2.0 * kPi);
        const double s   = std::sqrt(1.0 - (z * z));
        const Vec3d  direction(s * std::cos(phi), s * std::sin(phi), z);

        // Star counts grow about 3x per magnitude: sample the magnitude accordingly.
        const double u    = random.uniform();
        const double span = kFaintestMagnitude - kBrightestMagnitude;
        const double magnitude =
            kFaintestMagnitude +
            (std::log(1.0 - (u * (1.0 - std::pow(3.0, -span)))) / std::log(3.0));
        const double flux = std::pow(10.0, -0.4 * magnitude);

        const double kelvin = 3000.0 + (9000.0 * std::pow(random.uniform(), 1.6));
        const Vec3f  color  = blackbodyColor(kelvin);
        const double size   = 1.4 + (0.9 * std::clamp(2.0 - (0.4 * magnitude), 0.0, 3.0));

        stars.push_back({.direction = Vec4f(Vec3f(direction), static_cast<float>(size)),
                         .color     = Vec4f(color * static_cast<float>(flux), 0.0F)});
    }
    return stars;
}

}  // namespace StarshipSimulator
