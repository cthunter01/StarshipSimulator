#include "StarshipSimulator/core/gpu_abi/color_grade.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr Vec3d kLuma(0.2126, 0.7152, 0.0722);

/// Rotates a colour's hue by `radians` about the grey axis (Rodrigues' rotation).
Vec3d rotateHue(const Vec3d& c, double radians)
{
    const Vec3d  axis = glm::normalize(Vec3d(1.0));
    const double cosA = std::cos(radians);
    const double sinA = std::sin(radians);
    return (c * cosA) + (glm::cross(axis, c) * sinA) + (axis * glm::dot(axis, c) * (1.0 - cosA));
}

/// Hue in degrees (0 red, 120 green, 240 blue).
double hueDegrees(const Vec3d& c)
{
    const double top    = std::max({c.x, c.y, c.z});
    const double bottom = std::min({c.x, c.y, c.z});
    const double range  = top - bottom;
    if (range <= 1e-9)
    {
        return 0.0;
    }
    double hue = 0.0;
    if (top == c.x)
    {
        hue = std::fmod((c.y - c.z) / range, 6.0);
    }
    else if (top == c.y)
    {
        hue = ((c.z - c.x) / range) + 2.0;
    }
    else
    {
        hue = ((c.x - c.y) / range) + 4.0;
    }
    return std::fmod((hue * 60.0) + 360.0, 360.0);
}

}  // namespace

Vec3d gradeColor(const Vec3d& display, const GradeSettings& settings)
{
    Vec3d c = glm::clamp(display, 0.0, 1.0);

    // Greens lean toward yellow, most for saturated mid greens; other hues stay.
    const double hue = hueDegrees(c);
    const double greenness =
        glm::smoothstep(60.0, 100.0, hue) * (1.0 - glm::smoothstep(150.0, 190.0, hue));
    const double chroma = std::max({c.x, c.y, c.z}) - std::min({c.x, c.y, c.z});
    // Rotating toward yellow is negative about the grey axis (red -> green -> blue is positive).
    c = rotateHue(c, -degreesToRadians(settings.greenShiftDeg) * greenness *
                         glm::smoothstep(0.0, 0.3, chroma));

    // Richer colour, easing off toward white so highlights do not clip into odd hues.
    const double luma = glm::dot(c, kLuma);
    c = Vec3d(luma) +
        ((c - Vec3d(luma)) * std::lerp(settings.saturation, 1.0, glm::smoothstep(0.7, 1.0, luma)));

    // Split toning: golden light, cool shade.
    c += settings.warmth * glm::smoothstep(0.35, 1.0, luma) * Vec3d(1.0, 0.45, -0.55);
    c += settings.coolShadows * (1.0 - glm::smoothstep(0.0, 0.45, luma)) * Vec3d(-0.4, -0.1, 1.0);

    // A gentle S-curve.
    c = glm::clamp(c, 0.0, 1.0);
    c = glm::mix(c, c * c * (3.0 - (2.0 * c)), settings.contrast);
    return glm::clamp(c, 0.0, 1.0);
}

std::vector<std::uint8_t> makeGradeLut(const GradeSettings& settings, std::uint32_t size)
{
    std::vector<std::uint8_t> lut;
    lut.reserve(static_cast<std::size_t>(size) * size * size * 4);
    const double step = 1.0 / static_cast<double>(std::max<std::uint32_t>(size - 1, 1));
    for (std::uint32_t b = 0; b < size; ++b)
    {
        for (std::uint32_t g = 0; g < size; ++g)
        {
            for (std::uint32_t r = 0; r < size; ++r)
            {
                const Vec3d graded = gradeColor(Vec3d(r, g, b) * step, settings);
                for (const double channel : {graded.x, graded.y, graded.z, 1.0})
                {
                    lut.push_back(static_cast<std::uint8_t>(std::lround(channel * 255.0)));
                }
            }
        }
    }
    return lut;
}

}  // namespace StarshipSimulator
