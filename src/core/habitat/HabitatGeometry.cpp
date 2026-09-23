#include "StarshipSimulator/core/habitat/HabitatGeometry.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/habitat/metrics.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kWindowMarginM = 30.0;   // flat walkway beside the glass
constexpr double kWindowBlendM  = 200.0;  // terrain fades in over this distance
// A Kalpana cylinder is a few hundred metres long: a narrower walkway along the foot of each glass
// end wall, and the land rising from it sooner.
constexpr double kEndWalkwayM      = 12.0;
constexpr double kEndBlendM        = 40.0;
constexpr double kHubBlendM        = 150.0;  // terrain fades out toward the hub
constexpr double kMountainBlendM   = 400.0;  // endcap relief fades in from the floor
constexpr double kEndMarginM       = 1.0;    // keep off the end walls
constexpr double kPoleRadiusM      = 5.0;    // dome poles: stop where the dome closes in
constexpr double kNormalStepM      = 0.5;
constexpr int    kHillOctaves      = 5;
constexpr int    kRidgeOctaves     = 5;
constexpr double kFloodplainReachM = 250.0;  // water shapes the land out to this distance
// A torus's walls are terraced from the land's edge (fading in over this much arc) up to where they
// stand this steep (fading out over the last few degrees).
constexpr double kTerraceBlendM = 6.0;
constexpr double kTerracesEnd   = degreesToRadians(72.0);
constexpr double kTerraceFade   = degreesToRadians(10.0);
// A sphere's floor rises away from the equator (a torus's from the tube's lowest line), and water
// lies level (at one radius): it keeps to where the floor has risen no more than this above it.
constexpr double kMaxWaterRiseM = 2.5;

HabitatSpec validated(const HabitatSpec& spec)
{
    auto problems = validate(spec);
    if (!habitatKindBuilt(spec.kind))
    {
        problems.push_back(std::format("a {} cannot be built yet", habitatKindName(spec.kind)));
    }
    if (!problems.empty())
    {
        std::string message = "invalid habitat:";
        for (const std::string& problem : problems)
        {
            message += "\n  " + problem;
        }
        throw std::invalid_argument(message);
    }
    return normalizedForKind(spec);
}

/// The z range of the land: an O'Neill or Kalpana cylinder's floor between its endcaps, a
/// sphere's band round the equator, or the bottom of a torus's tube.
std::pair<double, double> floorZRange(const HabitatSpec& spec)
{
    if (spec.kind == HabitatKind::BERNAL_SPHERE)
    {
        const double reach = spec.radiusM * std::sin(degreesToRadians(spec.sphere.landLatitudeDeg));
        return {-reach, reach};
    }
    if (spec.kind == HabitatKind::STANFORD_TORUS)
    {
        const double reach =
            spec.torus.tubeRadiusM * std::sin(degreesToRadians(spec.torus.landHalfAngleDeg));
        return {-reach, reach};
    }
    return {(-spec.lengthM / 2.0) + endcapDepthInside(spec.antisunwardEndcap, spec.radiusM),
            (spec.lengthM / 2.0) - endcapDepthInside(spec.sunwardEndcap, spec.radiusM)};
}

/// The band of land that runs round the axis, if the habitat's land is laid out that way.
std::optional<LandBand> aroundBand(const std::vector<LandBand>& bands)
{
    if (bands.size() == 1 && bands.front().axis == BandAxis::AROUND)
    {
        return bands.front();
    }
    return std::nullopt;
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

HabitatGeometry::HabitatGeometry(const HabitatSpec& spec)
  : spec_(validated(spec)),
    profile_(std::make_shared<const MeridianProfile>(buildFloorProfile(spec_))),
    hills_(hashSeed(spec_.terrain.seed, 1)),
    ridges_(hashSeed(spec_.terrain.seed, 2)),
    omega_(spinRate(spec_.radiusM, spec_.surfaceGravityG)),
    floorZMin_(floorZRange(spec_).first),
    floorZMax_(floorZRange(spec_).second),
    bands_(planLandBands(spec_, profile_, floorZMin_, floorZMax_)),
    enclosure_(spec_, profile_),
    landscape_(spec_.terrain, LandscapeFrame{.radiusM         = spec_.radiusM,
                                             .floorZMin       = floorZMin_,
                                             .floorZMax       = floorZMax_,
                                             .stripCount      = spec_.stripPairs,
                                             .stripAngle      = stripAngle(),
                                             .windowHalfAngle = windowHalfAngle(),
                                             .around          = aroundBand(bands_),
                                             .waterReachM     = waterReachAcrossM(),
                                             .sections = spec_.kind == HabitatKind::STANFORD_TORUS
                                                             ? spec_.torus.sections
                                                             : 0})
{
    walkableZMin_ = std::max(profile_->zMin() + kEndMarginM,
                             zWhereRadiusReaches(*profile_, kPoleRadiusM, false));
    walkableZMax_ = std::min(profile_->zMax() - kEndMarginM,
                             zWhereRadiusReaches(*profile_, kPoleRadiusM, true));
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
    if (z < profile_->zMin() || z > profile_->zMax())
    {
        return {.kind = RegionKind::OUTSIDE, .index = -1};
    }
    if (floorCurves())
    {
        // The band of land round the equator, and the polar slopes above it up to the windows (a
        // torus: the bottom of the tube, and its walls up to the ceiling).
        if (z < floorZMin_ || z > floorZMax_)
        {
            return {.kind = RegionKind::ENDCAP, .index = -1};
        }
        return {.kind = RegionKind::LAND, .index = 0};
    }
    if (spec_.kind != HabitatKind::ONEILL_CYLINDER)
    {
        // The whole floor is one band of land; the windows are the end walls.
        return {.kind = RegionKind::LAND, .index = 0};
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
    if (spec_.kind == HabitatKind::KALPANA_CYLINDER)
    {
        // The glass end walls.
        return std::max(0.0, std::min(z - profile_->zMin(), profile_->zMax() - z));
    }
    if (floorCurves())
    {
        // Along the floor to the rim of the nearer polar window (or where the ceiling begins).
        const double u = profile_->arcAt(z);
        return std::max(0.0, std::min(u, profile_->length() - u));
    }
    const double strip = stripAngle();
    const int nearest  = static_cast<int>(std::lround(wrapAngle(theta) / strip)) % spec_.stripPairs;
    const double arc =
        std::max(0.0, angularDistance(theta, windowCenter(nearest)) - windowHalfAngle()) * radius;
    const double along = std::max({0.0, floorZMin_ - z, z - floorZMax_});
    return std::hypot(arc, along);
}

double HabitatGeometry::floorRadiusAt(double z) const
{
    return profile_->radiusAt(std::clamp(z, profile_->zMin(), profile_->zMax())).value_or(0.0);
}

double HabitatGeometry::arcBeyondLand(double z) const
{
    const double u = profile_->arcAt(z);
    return std::max({0.0, profile_->arcAt(floorZMin_) - u, u - profile_->arcAt(floorZMax_)});
}

double HabitatGeometry::slopeArcM() const
{
    return profile_->length() - profile_->arcAt(floorZMax_);
}

double HabitatGeometry::maxWaterRiseM() const
{
    return floorCurves() ? kMaxWaterRiseM : 0.0;
}

std::optional<double> HabitatGeometry::waterReachAcrossM() const
{
    if (!floorCurves())
    {
        return std::nullopt;
    }
    // The floor curves across the band like a circle of this radius: the sphere's, or the tube's.
    const double radius = isSphere() ? spec_.radiusM : spec_.torus.tubeRadiusM;
    return radius * std::acos((radius - kMaxWaterRiseM) / radius);
}

double HabitatGeometry::spanAcrossM() const
{
    if (spec_.kind == HabitatKind::STANFORD_TORUS)
    {
        return 2.0 * spec_.torus.tubeRadiusM;  // up to the ceiling
    }
    return 2.0 * spec_.radiusM;
}

const LandBand& HabitatGeometry::band(int index) const
{
    return bands_.at(static_cast<std::size_t>(index));
}

bool HabitatGeometry::onLand(double z, double theta) const
{
    return regionAt(z, theta).kind == RegionKind::LAND;
}

double HabitatGeometry::waterLevelAt(double z) const
{
    // A level surface under spin gravity is a cylinder round the axis: kWaterLevelM below the floor
    // at the land's middle line, and deeper under the floor wherever the floor rises toward the
    // axis (on a sphere or in a torus's tube; a cylinder's floor is level).
    if (!floorCurves())
    {
        return kWaterLevelM;
    }
    return kWaterLevelM + (floorRadiusAt(z) - bands_.front().radiusM);
}

double HabitatGeometry::terrainHeight(double z, double theta) const
{
    return landscape_.shape(naturalHeight(z, theta),
                            landscape_.shoreDistance(z, theta, kFloodplainReachM), waterLevelAt(z));
}

double HabitatGeometry::naturalHeight(double z, double theta) const
{
    const std::optional<double> baseRadius = profile_->radiusAt(z);
    if (!baseRadius)
    {
        return 0.0;
    }
    const double radius = *baseRadius;
    const bool   oneill = spec_.kind == HabitatKind::ONEILL_CYLINDER;
    const double margin = oneill ? kWindowMarginM : kEndWalkwayM;
    const double blend  = oneill ? kWindowBlendM : kEndBlendM;
    const double windowMask =
        glm::smoothstep(margin, margin + blend, distanceToWindow(z, theta, radius));
    const double hubRadius =
        std::min(spec_.antisunwardEndcap.hubRadiusM, spec_.sunwardEndcap.hubRadiusM);
    // A sphere has no hub: its floor ends at the polar windows' rims (a torus's at the ceiling).
    const double hubMask =
        floorCurves() ? 1.0 : glm::smoothstep(hubRadius, hubRadius + kHubBlendM, radius);
    const double mask = windowMask * hubMask;
    if (mask <= 0.0)
    {
        return 0.0;
    }

    const TerrainSpec& terrain = spec_.terrain;
    const Vec3d        surface(radius * std::cos(theta), radius * std::sin(theta), z);
    const Vec3d        q = surface / terrain.featureSizeM;

    const double base  = 0.5 + (0.5 * hills_.fbm(q, kHillOctaves));
    const double hills = terrain.hillHeightM * base * base;

    if (spec_.kind == HabitatKind::STANFORD_TORUS)
    {
        return (mask * hills) + terraceHeight(z);
    }

    const double beyondFloor = std::max({0.0, floorZMin_ - z, z - floorZMax_});
    // On a sphere the polar slopes are short: the relief grows over the first half of them (and
    // likewise up the walls of a torus's tube).
    const double mountainWeight = floorCurves()
                                      ? glm::smoothstep(0.0, 0.5 * slopeArcM(), arcBeyondLand(z))
                                      : glm::smoothstep(0.0, kMountainBlendM, beyondFloor);
    double       mountains      = 0.0;
    if (mountainWeight > 0.0)
    {
        mountains =
            terrain.mountainHeightM * mountainWeight * ridges_.ridged(q * 1.3, kRidgeOctaves);
    }
    return mask * (hills + mountains);
}

double HabitatGeometry::terraceHeight(double z) const
{
    // A torus's walls are terraced: shelves at one radius, level under spin gravity, each a riser
    // of the mountain height above the last, up to where the wall stands upright.
    const double step = spec_.terrain.mountainHeightM;
    if (spec_.kind != HabitatKind::STANFORD_TORUS || step < 0.5)
    {
        return 0.0;
    }
    const std::optional<double> radius = profile_->radiusAt(z);
    if (!radius)
    {
        return 0.0;
    }
    const double across = std::asin(std::clamp(std::abs(z) / spec_.torus.tubeRadiusM, 0.0, 1.0));
    const double weight =
        glm::smoothstep(0.0, kTerraceBlendM, arcBeyondLand(z)) *
        (1.0 - glm::smoothstep(kTerracesEnd - kTerraceFade, kTerracesEnd, across));
    return weight * (*radius - (step * std::floor(*radius / step)));
}

double HabitatGeometry::forestDensity(double z, double theta) const
{
    const std::optional<double> baseRadius = profile_->radiusAt(z);
    if (!baseRadius || regionAt(z, theta).kind == RegionKind::WINDOW)
    {
        return 0.0;
    }
    const double radius = *baseRadius;
    // Not on the walkways beside the windows, nor in or right beside the water (meadows there).
    const bool   oneill  = spec_.kind == HabitatKind::ONEILL_CYLINDER;
    const double walkway = oneill ? glm::smoothstep(kWindowMarginM + 20.0, kWindowMarginM + 120.0,
                                                    distanceToWindow(z, theta, radius))
                                  : glm::smoothstep(kEndWalkwayM + 8.0, kEndWalkwayM + 30.0,
                                                    distanceToWindow(z, theta, radius));
    const double shore   = landscape_.shoreDistance(z, theta, kFloodplainReachM);
    const double water   = glm::smoothstep(15.0, 90.0, shore);
    // Woods thin out up the endcap mountains and stop near the hub.
    const double beyondFloor = std::max({0.0, floorZMin_ - z, z - floorZMax_});
    const double treeLine    = floorCurves()
                                   ? 1.0 - glm::smoothstep(0.3, 0.7, arcBeyondLand(z) / slopeArcM())
                                   : 1.0 - glm::smoothstep(0.35, 0.6, beyondFloor / spec_.radiusM);
    return landscape_.woodland(z, theta) * walkway * water * treeLine;
}

double HabitatGeometry::waterDepth(double z, double theta) const
{
    return std::max(0.0, waterLevelAt(z) - terrainHeight(z, theta));
}

std::optional<double> HabitatGeometry::groundRadius(double z, double theta) const
{
    const std::optional<double> baseRadius = profile_->radiusAt(z);
    if (!baseRadius)
    {
        return std::nullopt;
    }
    return std::max(0.0, *baseRadius - terrainHeight(z, theta));
}

Vec3d HabitatGeometry::surfacePoint(double z, double theta) const
{
    const double clamped = std::clamp(z, profile_->zMin(), profile_->zMax());
    const double r       = groundRadius(clamped, theta).value_or(0.0);
    return {r * std::cos(theta), r * std::sin(theta), clamped};
}

GroundSample HabitatGeometry::ground(const Vec3d& position) const
{
    GroundSample sample;
    const double theta       = angleOf(position);
    const double z           = std::clamp(position.z, profile_->zMin(), profile_->zMax());
    sample.region            = regionAt(position.z, theta);
    sample.up                = localUp(position);
    sample.groundRadius      = groundRadius(z, theta).value_or(0.0);
    sample.heightAboveGround = sample.groundRadius - std::hypot(position.x, position.y);

    // Surface normal from nearby ground points by central differences (d/dz x d/dtheta points into
    // the habitat), one-sided at the ends of the profile.
    const double zLow     = std::max(profile_->zMin(), z - kNormalStepM);
    const double zHigh    = std::min(profile_->zMax(), z + kNormalStepM);
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
        if (const double z = origin.z + (dir.z * t); z < profile_->zMin() || z > profile_->zMax())
        {
            return std::nullopt;  // left through an end
        }
    }
    return std::nullopt;
}

}  // namespace StarshipSimulator
