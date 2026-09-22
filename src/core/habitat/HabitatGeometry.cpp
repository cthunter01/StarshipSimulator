#include "StarshipSimulator/core/habitat/HabitatGeometry.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kWindowMarginM    = 30.0;   // flat walkway beside the glass
constexpr double kWindowBlendM     = 200.0;  // terrain fades in over this distance
constexpr double kHubBlendM        = 150.0;  // terrain fades out toward the hub
constexpr double kMountainBlendM   = 400.0;  // endcap relief fades in from the floor
constexpr double kEndMarginM       = 1.0;    // keep off the end walls
constexpr double kPoleRadiusM      = 5.0;    // dome poles: stop where the dome closes in
constexpr double kNormalStepM      = 0.5;
constexpr int    kHillOctaves      = 5;
constexpr int    kRidgeOctaves     = 5;
constexpr double kFloodplainReachM = 250.0;  // water shapes the land out to this distance

OneillCylinderSpec validated(const OneillCylinderSpec& spec)
{
    const auto problems = validate(spec);
    if (!problems.empty())
    {
        std::string message = "invalid habitat:";
        for (const std::string& problem : problems)
        {
            message += "\n  " + problem;
        }
        throw std::invalid_argument(message);
    }
    return spec;
}

double wrapAngle(double angle)
{
    const double wrapped = std::fmod(angle, 2.0 * kPi);
    return wrapped < 0.0 ? wrapped + (2.0 * kPi) : wrapped;
}

/// z where the profile radius shrinks to r, searching from one end toward the other.
double zWhereRadiusReaches(const MeridianProfile& profile, double r, bool fromEnd)
{
    const auto& points = profile.points();
    if (fromEnd)
    {
        for (std::size_t i = points.size() - 1; i > 0; --i)
        {
            const ProfilePoint& a = points[i - 1];
            const ProfilePoint& b = points[i];
            if (a.r >= r && b.r < r)
            {
                return std::lerp(a.z, b.z, (a.r - r) / (a.r - b.r));
            }
        }
        return profile.zMax();
    }
    for (std::size_t i = 0; i + 1 < points.size(); ++i)
    {
        const ProfilePoint& a = points[i];
        const ProfilePoint& b = points[i + 1];
        if (a.r < r && b.r >= r)
        {
            return std::lerp(a.z, b.z, (r - a.r) / (b.r - a.r));
        }
    }
    return profile.zMin();
}

}  // namespace

const char* regionKindName(RegionKind kind)
{
    switch (kind)
    {
        case RegionKind::LAND:
            return "valley";
        case RegionKind::WINDOW:
            return "window";
        case RegionKind::ENDCAP:
            return "endcap";
        case RegionKind::OUTSIDE:
            return "outside";
    }
    return "unknown";
}

HabitatGeometry::HabitatGeometry(const OneillCylinderSpec& spec)
  : spec_(validated(spec)),
    profile_(buildOneillProfile(spec_)),
    hills_(hashSeed(spec_.terrain.seed, 1)),
    ridges_(hashSeed(spec_.terrain.seed, 2)),
    omega_(spinRate(spec_.radiusM, spec_.surfaceGravityG)),
    floorZMin_((-spec_.lengthM / 2.0) + endcapDepthInside(spec_.antisunwardEndcap, spec_.radiusM)),
    floorZMax_((spec_.lengthM / 2.0) - endcapDepthInside(spec_.sunwardEndcap, spec_.radiusM)),
    landscape_(spec_.terrain, LandscapeFrame{.radiusM         = spec_.radiusM,
                                             .floorZMin       = floorZMin_,
                                             .floorZMax       = floorZMax_,
                                             .stripCount      = spec_.stripPairs,
                                             .stripAngle      = stripAngle(),
                                             .windowHalfAngle = windowHalfAngle()})
{
    walkableZMin_ =
        std::max(profile_.zMin() + kEndMarginM, zWhereRadiusReaches(profile_, kPoleRadiusM, false));
    walkableZMax_ =
        std::min(profile_.zMax() - kEndMarginM, zWhereRadiusReaches(profile_, kPoleRadiusM, true));
}

double HabitatGeometry::stripAngle() const
{
    return 2.0 * kPi / static_cast<double>(spec_.stripPairs);
}

double HabitatGeometry::windowHalfAngle() const
{
    return 0.5 * spec_.windowFraction * stripAngle();
}

double HabitatGeometry::landHalfAngle() const
{
    return 0.5 * (1.0 - spec_.windowFraction) * stripAngle();
}

double HabitatGeometry::windowCenter(int index) const
{
    return wrapAngle(static_cast<double>(index) * stripAngle());
}

double HabitatGeometry::landCenter(int index) const
{
    return wrapAngle((static_cast<double>(index) + 0.5) * stripAngle());
}

double HabitatGeometry::angleOf(const Vec3d& position)
{
    return wrapAngle(std::atan2(position.y, position.x));
}

double HabitatGeometry::angularDistance(double a, double b)
{
    return std::abs(std::remainder(a - b, 2.0 * kPi));
}

Vec3d HabitatGeometry::localUp(const Vec3d& position)
{
    const double r = std::hypot(position.x, position.y);
    if (r < 1e-9)
    {
        return {1.0, 0.0, 0.0};
    }
    return {-position.x / r, -position.y / r, 0.0};
}

Region HabitatGeometry::regionAt(double z, double theta) const
{
    if (z < profile_.zMin() || z > profile_.zMax())
    {
        return {.kind = RegionKind::OUTSIDE, .index = -1};
    }
    if (z < floorZMin_ || z > floorZMax_)
    {
        return {.kind = RegionKind::ENDCAP, .index = -1};
    }
    const double strip = stripAngle();
    // Strip index of the nearest window centre.
    const int nearest = static_cast<int>(std::lround(wrapAngle(theta) / strip)) % spec_.stripPairs;
    if (angularDistance(theta, windowCenter(nearest)) <= windowHalfAngle())
    {
        return {.kind = RegionKind::WINDOW, .index = nearest};
    }
    const int land = static_cast<int>(std::floor(wrapAngle(theta) / strip)) % spec_.stripPairs;
    return {.kind = RegionKind::LAND, .index = land};
}

double HabitatGeometry::distanceToWindow(double z, double theta, double radius) const
{
    const double strip = stripAngle();
    const int nearest  = static_cast<int>(std::lround(wrapAngle(theta) / strip)) % spec_.stripPairs;
    const double arc =
        std::max(0.0, angularDistance(theta, windowCenter(nearest)) - windowHalfAngle()) * radius;
    const double along = std::max({0.0, floorZMin_ - z, z - floorZMax_});
    return std::hypot(arc, along);
}

double HabitatGeometry::terrainHeight(double z, double theta) const
{
    return Landscape::shapeNearWater(naturalHeight(z, theta),
                                     landscape_.shoreDistance(z, theta, kFloodplainReachM));
}

double HabitatGeometry::naturalHeight(double z, double theta) const
{
    const std::optional<double> baseRadius = profile_.radiusAt(z);
    if (!baseRadius)
    {
        return 0.0;
    }
    const double radius     = *baseRadius;
    const double windowMask = glm::smoothstep(kWindowMarginM, kWindowMarginM + kWindowBlendM,
                                              distanceToWindow(z, theta, radius));
    const double hubRadius =
        std::min(spec_.antisunwardEndcap.hubRadiusM, spec_.sunwardEndcap.hubRadiusM);
    const double hubMask = glm::smoothstep(hubRadius, hubRadius + kHubBlendM, radius);
    const double mask    = windowMask * hubMask;
    if (mask <= 0.0)
    {
        return 0.0;
    }

    const TerrainSpec& terrain = spec_.terrain;
    const Vec3d        surface(radius * std::cos(theta), radius * std::sin(theta), z);
    const Vec3d        q = surface / terrain.featureSizeM;

    const double base  = 0.5 + (0.5 * hills_.fbm(q, kHillOctaves));
    const double hills = terrain.hillHeightM * base * base;

    const double beyondFloor    = std::max({0.0, floorZMin_ - z, z - floorZMax_});
    const double mountainWeight = glm::smoothstep(0.0, kMountainBlendM, beyondFloor);
    double       mountains      = 0.0;
    if (mountainWeight > 0.0)
    {
        mountains =
            terrain.mountainHeightM * mountainWeight * ridges_.ridged(q * 1.3, kRidgeOctaves);
    }
    return mask * (hills + mountains);
}

double HabitatGeometry::forestDensity(double z, double theta) const
{
    const std::optional<double> baseRadius = profile_.radiusAt(z);
    if (!baseRadius || regionAt(z, theta).kind == RegionKind::WINDOW)
    {
        return 0.0;
    }
    const double radius = *baseRadius;
    // Not on the walkways beside the windows, nor in or right beside the water (meadows there).
    const double walkway = glm::smoothstep(kWindowMarginM + 20.0, kWindowMarginM + 120.0,
                                           distanceToWindow(z, theta, radius));
    const double shore   = landscape_.shoreDistance(z, theta, kFloodplainReachM);
    const double water   = glm::smoothstep(15.0, 90.0, shore);
    // Woods thin out up the endcap mountains and stop near the hub.
    const double beyondFloor = std::max({0.0, floorZMin_ - z, z - floorZMax_});
    const double treeLine    = 1.0 - glm::smoothstep(0.35, 0.6, beyondFloor / spec_.radiusM);
    return landscape_.woodland(z, theta) * walkway * water * treeLine;
}

double HabitatGeometry::waterDepth(double z, double theta) const
{
    return std::max(0.0, kWaterLevelM - terrainHeight(z, theta));
}

std::optional<double> HabitatGeometry::groundRadius(double z, double theta) const
{
    const std::optional<double> baseRadius = profile_.radiusAt(z);
    if (!baseRadius)
    {
        return std::nullopt;
    }
    return std::max(0.0, *baseRadius - terrainHeight(z, theta));
}

Vec3d HabitatGeometry::surfacePoint(double z, double theta) const
{
    const double clamped = std::clamp(z, profile_.zMin(), profile_.zMax());
    const double r       = groundRadius(clamped, theta).value_or(0.0);
    return {r * std::cos(theta), r * std::sin(theta), clamped};
}

GroundSample HabitatGeometry::ground(const Vec3d& position) const
{
    GroundSample sample;
    const double theta       = angleOf(position);
    const double z           = std::clamp(position.z, profile_.zMin(), profile_.zMax());
    sample.region            = regionAt(position.z, theta);
    sample.up                = localUp(position);
    sample.groundRadius      = groundRadius(z, theta).value_or(0.0);
    sample.heightAboveGround = sample.groundRadius - std::hypot(position.x, position.y);

    // Surface normal from nearby ground points by central differences (d/dz x d/dtheta points into
    // the habitat), one-sided at the ends of the profile.
    const double zLow     = std::max(profile_.zMin(), z - kNormalStepM);
    const double zHigh    = std::min(profile_.zMax(), z + kNormalStepM);
    const double dtheta   = kNormalStepM / std::max(sample.groundRadius, 1.0);
    const Vec3d  alongZ   = surfacePoint(zHigh, theta) - surfacePoint(zLow, theta);
    const Vec3d  alongArc = surfacePoint(z, theta + dtheta) - surfacePoint(z, theta - dtheta);
    Vec3d        normal   = glm::cross(alongZ, alongArc);
    if (glm::dot(normal, normal) < 1e-18)
    {
        normal = sample.up;
    }
    sample.normal       = glm::normalize(normal);
    sample.slopeRadians = std::acos(std::clamp(glm::dot(sample.normal, sample.up), -1.0, 1.0));
    return sample;
}

std::optional<double> HabitatGeometry::raycast(const Vec3d& origin, const Vec3d& direction,
                                               double maxDistance) const
{
    const Vec3d dir      = glm::normalize(direction);
    const auto  heightAt = [&](double t) { return ground(origin + (dir * t)).heightAboveGround; };
    double      previous = 0.0;
    double      t        = 0.0;
    double      height   = heightAt(0.0);
    if (height <= 0.0)
    {
        return 0.0;
    }
    while (t < maxDistance)
    {
        // Height above ground bounds how far we can safely step on gentle terrain.
        previous = t;
        t        = std::min(maxDistance, t + std::max(0.5, 0.5 * height));
        height   = heightAt(t);
        if (height <= 0.0)
        {
            double low  = previous;
            double high = t;
            for (int i = 0; i < 40; ++i)
            {
                const double mid                   = 0.5 * (low + high);
                (heightAt(mid) > 0.0 ? low : high) = mid;
            }
            return high;
        }
        if (const double z = origin.z + (dir.z * t); z < profile_.zMin() || z > profile_.zMax())
        {
            return std::nullopt;  // left through an end
        }
    }
    return std::nullopt;
}

}  // namespace StarshipSimulator
