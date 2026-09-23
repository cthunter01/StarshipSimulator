#include "StarshipSimulator/core/habitat/Landscape.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/SimplexNoise.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kBankWidthM       = 5.0;   // from the waterline up to the floodplain
constexpr double kShelfM           = 12.0;  // from the waterline down to full depth
constexpr double kPlainHeightM     = 0.35;  // floodplain above the datum
constexpr double kMeanderWaveM     = 2600.0;
constexpr double kWoodsFeatureM    = 620.0;
constexpr int    kWoodsOctaves     = 4;
constexpr double kWoodsSoftness    = 0.06;  // noise range over which woods thin out
constexpr int    kCalibrationCount = 4096;
constexpr double kRiverTableStepM  = 4.0;

double wrapAngle(double angle)
{
    const double wrapped = std::fmod(angle, 2.0 * kPi);
    return wrapped < 0.0 ? wrapped + (2.0 * kPi) : wrapped;
}

double angularDistance(double a, double b)
{
    return std::abs(std::remainder(a - b, 2.0 * kPi));
}

}  // namespace

Landscape::Landscape(const TerrainSpec& terrain, const LandscapeFrame& frame)
  : frame_(frame), meander_(hashSeed(terrain.seed, 4)), woods_(hashSeed(terrain.seed, 3))
{
    if (frame.around)
    {
        planAround(terrain);
        return;
    }
    const double landHalfWidth = ((0.5 * frame.stripAngle) - frame.windowHalfAngle) * frame.radiusM;
    const double margin        = std::max(0.2 * landHalfWidth, 30.0);  // water keeps off walkways
    const double floorLength   = frame.floorZMax - frame.floorZMin;
    const double endMargin     = std::max(0.03 * floorLength, 2.0 * kShelfM);
    riverZMin_                 = frame.floorZMin + endMargin;
    riverZMax_                 = frame.floorZMax - endMargin;

    // Rivers swing across the middle of each valley, never closer to the windows than the margin.
    const double halfWidth = 0.5 * terrain.riverWidthM;
    meanderM_              = std::clamp(0.3 * landHalfWidth, 0.0, 700.0);
    meanderM_ =
        std::min(meanderM_, std::max(0.0, landHalfWidth - margin - halfWidth - kFloodplainM));
    if (halfWidth > 0.0 && landHalfWidth - margin - meanderM_ > halfWidth &&
        riverZMax_ > riverZMin_)
    {
        riverHalfWidthM_ = halfWidth;
    }

    // Tabulate the rivers' courses: the meander noise is too slow to evaluate per sample.
    if (hasRivers())
    {
        riverTableStep_ = kRiverTableStepM;
        const auto count =
            static_cast<std::size_t>(std::ceil((riverZMax_ - riverZMin_) / riverTableStep_)) + 1;
        riverTable_.resize(static_cast<std::size_t>(frame.stripCount));
        for (int valley = 0; valley < frame.stripCount; ++valley)
        {
            auto& table = riverTable_[static_cast<std::size_t>(valley)];
            table.resize(count);
            for (std::size_t i = 0; i < count; ++i)
            {
                const double z = riverZMin_ + (static_cast<double>(i) * riverTableStep_);
                const double slope =
                    (meanderAngle(valley, z + 2.0) - meanderAngle(valley, z - 2.0)) / 4.0 *
                    frame.radiusM;
                table[i] = Vec2d(meanderAngle(valley, z), slope);
            }
        }
    }

    // Lakes, spread along each valley on the river's course.
    const int count = terrain.lakesPerValley;
    for (int valley = 0; valley < frame.stripCount && count > 0; ++valley)
    {
        SplitMix64   random(hashSeed(terrain.seed, 100 + static_cast<std::uint64_t>(valley)));
        const double segment = (riverZMax_ - riverZMin_) / count;
        for (int i = 0; i < count; ++i)
        {
            Lake lake;
            lake.valley = valley;
            lake.z      = riverZMin_ + (segment * (i + random.uniform(0.2, 0.8)));
            lake.theta  = hasRivers() ? riverAngle(valley, lake.z)
                                      : ((valley + 0.5) * frame.stripAngle) +
                                            (random.uniform(-0.3, 0.3) * meanderM_ / frame.radiusM);
            const double offCentre =
                angularDistance(lake.theta, (valley + 0.5) * frame.stripAngle) * frame.radiusM;
            const double room = landHalfWidth - margin - offCentre;
            lake.halfWidthM   = std::min(terrain.lakeRadiusM * random.uniform(0.7, 1.2), room);
            lake.halfLengthM = std::min(lake.halfWidthM * random.uniform(1.3, 2.4), 0.45 * segment);
            if (lake.halfWidthM >= 5.0 && lake.halfLengthM >= 5.0)
            {
                lakes_.push_back(lake);
            }
        }
    }

    // Woods: the noise level above which about forestCover of the floor lies.
    std::vector<double> samples;
    samples.reserve(kCalibrationCount);
    SplitMix64 random(hashSeed(terrain.seed, 5));
    for (int i = 0; i < kCalibrationCount; ++i)
    {
        const double z     = random.uniform(frame.floorZMin, frame.floorZMax);
        const double theta = random.uniform(0.0, 2.0 * kPi);
        const Vec3d  p(frame.radiusM * std::cos(theta), frame.radiusM * std::sin(theta), z);
        samples.push_back(woods_.fbm(p / kWoodsFeatureM, kWoodsOctaves));
    }
    const double cover = std::clamp(terrain.forestCover, 0.0, 1.0);
    const auto   index = static_cast<std::size_t>(
        std::clamp((1.0 - cover) * (kCalibrationCount - 1), 0.0, kCalibrationCount - 1.0));
    std::ranges::nth_element(samples, samples.begin() + static_cast<std::ptrdiff_t>(index));
    woodsThreshold_ = cover <= 0.0 ? 10.0 : samples[index];
}

int Landscape::valleyAt(double theta) const
{
    const int valley = static_cast<int>(std::floor(wrapAngle(theta) / frame_.stripAngle));
    return std::clamp(valley, 0, frame_.stripCount - 1);
}

double Landscape::riverAngle(int valley, double z) const
{
    if (riverTable_.empty())
    {
        return meanderAngle(valley, z);
    }
    const auto&  table = riverTable_[static_cast<std::size_t>(valley)];
    const double t =
        std::clamp((z - riverZMin_) / riverTableStep_, 0.0, static_cast<double>(table.size() - 1));
    const auto i = std::min(static_cast<std::size_t>(t), table.size() - 2);
    return std::lerp(table[i].x, table[i + 1].x, t - static_cast<double>(i));
}

double Landscape::meanderAngle(int valley, double z) const
{
    const double v     = static_cast<double>(valley) * 7.3;
    const double swing = meander_.sample(Vec3d(z / kMeanderWaveM, v, 0.5)) +
                         (0.35 * meander_.sample(Vec3d(z / (0.37 * kMeanderWaveM), v, 4.5)));
    return ((valley + 0.5) * frame_.stripAngle) + (swing / 1.35 * meanderM_ / frame_.radiusM);
}

double Landscape::riverDistance(int valley, double z, double theta) const
{
    const double zc    = std::clamp(z, riverZMin_, riverZMax_);
    const double along = z - zc;  // beyond the river's ends: rounded ends
    const auto&  table = riverTable_[static_cast<std::size_t>(valley)];
    const double t     = (zc - riverZMin_) / riverTableStep_;
    const auto   i     = std::min(static_cast<std::size_t>(t), table.size() - 2);
    const Vec2d  here  = glm::mix(table[i], table[i + 1], t - static_cast<double>(i));
    // Distance across the channel, corrected for the channel's slant where it swings.
    const double across =
        angularDistance(theta, here.x) * frame_.radiusM / std::sqrt(1.0 + (here.y * here.y));
    return std::hypot(across, along) - riverHalfWidthM_;
}

double Landscape::lakeDistance(const Lake& lake, double z, double theta) const
{
    const double dz = (z - lake.z) / lake.halfLengthM;
    const double arc =
        std::remainder(theta - lake.theta, 2.0 * kPi) * frame_.radiusM / lake.halfWidthM;
    const double e = std::hypot(dz, arc);
    if (e > 1.5)
    {
        return (e - 1.0) * std::min(lake.halfLengthM, lake.halfWidthM);  // far: no need for detail
    }
    // An irregular shore: the radius wobbles with direction.
    const double direction = std::atan2(arc, dz);
    const double wobble =
        1.0 + (0.12 * meander_.sample(Vec3d(2.0 * std::cos(direction), 2.0 * std::sin(direction),
                                            lake.z / 997.0)));
    return (e - wobble) * std::min(lake.halfLengthM, lake.halfWidthM);
}

double Landscape::shoreDistance(double z, double theta, double far) const
{
    if (frame_.around)
    {
        return aroundShoreDistance(z, theta, far);
    }
    if (z < frame_.floorZMin - far || z > frame_.floorZMax + far)
    {
        return far;
    }
    const int valley = valleyAt(theta);
    double    best   = far;
    if (hasRivers())
    {
        best = std::min(best, riverDistance(valley, z, theta));
    }
    for (const Lake& lake : lakes_)
    {
        if (lake.valley == valley && std::abs(z - lake.z) < (lake.halfLengthM * 1.2) + far)
        {
            best = std::min(best, lakeDistance(lake, z, theta));
        }
    }
    return best;
}

double Landscape::woodland(double z, double theta) const
{
    const double radius = frame_.around
                              ? frame_.around->profile->radiusAt(z).value_or(frame_.radiusM)
                              : frame_.radiusM;
    const Vec3d  p(radius * std::cos(theta), radius * std::sin(theta), z);
    const double n = woods_.fbm(p / kWoodsFeatureM, kWoodsOctaves);
    return glm::smoothstep(woodsThreshold_ - kWoodsSoftness, woodsThreshold_ + kWoodsSoftness, n);
}

double Landscape::shapeNearWater(double natural, double shore, double floodplainM)
{
    if (shore >= floodplainM)
    {
        return natural;
    }
    const double nearWater =
        shore >= 0.0
            ? std::lerp(kWaterLevelM, kPlainHeightM, glm::smoothstep(0.0, kBankWidthM, shore))
            : kWaterLevelM - (kWaterDepthM * glm::smoothstep(0.0, kShelfM, -shore));
    const double weight = 1.0 - glm::smoothstep(kBankWidthM, floodplainM, shore);
    return std::lerp(natural, nearWater, weight);
}

// ---- A band running round the axis --------------------------------------------------------------
//
// Laid out on the band's plan (x across, y along), the river runs once round the habitat and back
// into itself, so its meander is noise sampled round a circle; the lakes lie along it. Everything
// scales with the band, which may be a few hundred metres wide rather than kilometres.

void Landscape::planAround(const TerrainSpec& terrain)
{
    if (!frame_.around)
    {
        return;
    }
    const LandBand& band      = *frame_.around;
    const double    length    = band.alongLengthM();
    const double    halfWidth = band.halfWidthM;
    const double    margin    = std::max(0.2 * halfWidth, 15.0);  // water keeps off the walkways
    const double    halfRiver = 0.5 * terrain.riverWidthM;
    floodplainM_              = std::min(kFloodplainM, 0.25 * halfWidth);
    meanderM_                 = std::clamp(0.3 * halfWidth, 0.0, 700.0);
    meanderM_ = std::min(meanderM_, std::max(0.0, halfWidth - margin - halfRiver - floodplainM_));
    if (halfRiver > 0.0 && halfWidth - margin - meanderM_ > halfRiver)
    {
        riverHalfWidthM_ = halfRiver;
        riverTableStep_  = kRiverTableStepM;
        const auto count = static_cast<std::size_t>(std::ceil(length / riverTableStep_));
        riverTableStep_  = length / static_cast<double>(count);  // a whole number of steps round
        auto& table      = riverTable_.emplace_back(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            const double along = band.alongMinM + (static_cast<double>(i) * riverTableStep_);
            const double slope = (aroundMeander(along + 2.0) - aroundMeander(along - 2.0)) / 4.0;
            table[i]           = Vec2d(aroundMeander(along), slope);
        }
    }

    // Lakes, spread round the band on the river's course.
    const int  lakes = terrain.lakesPerValley;
    SplitMix64 random(hashSeed(terrain.seed, 100));
    for (int i = 0; i < lakes; ++i)
    {
        const double segment = length / lakes;
        const double spot    = random.uniform(0.2, 0.8);
        const double along   = band.alongMinM + (segment * (i + spot));
        const double swing   = random.uniform(-0.3, 0.3);
        const double across  = hasRivers() ? riverAcross(along).x : swing * meanderM_;
        const double room    = halfWidth - margin - std::abs(across);
        const double width   = random.uniform(0.7, 1.2);
        const double stretch = random.uniform(1.3, 2.4);
        Lake         lake;
        lake.valley          = 0;
        lake.plan            = Vec2d(across, along);
        lake.halfWidthM      = std::min(terrain.lakeRadiusM * width, room);
        lake.halfLengthM     = std::min(lake.halfWidthM * stretch, 0.45 * segment);
        const SurfaceSpot at = band.toSurface(lake.plan);
        lake.z               = at.z;
        lake.theta           = at.theta;
        if (lake.halfWidthM >= 5.0 && lake.halfLengthM >= 5.0)
        {
            lakes_.push_back(lake);
        }
    }

    // Woods: the noise level above which about forestCover of the land lies.
    std::vector<double> samples;
    samples.reserve(kCalibrationCount);
    SplitMix64 woods(hashSeed(terrain.seed, 5));
    for (int i = 0; i < kCalibrationCount; ++i)
    {
        const double z     = woods.uniform(frame_.floorZMin, frame_.floorZMax);
        const double theta = woods.uniform(0.0, 2.0 * kPi);
        const double r     = band.profile->radiusAt(z).value_or(frame_.radiusM);
        const Vec3d  p(r * std::cos(theta), r * std::sin(theta), z);
        samples.push_back(woods_.fbm(p / kWoodsFeatureM, kWoodsOctaves));
    }
    const double cover = std::clamp(terrain.forestCover, 0.0, 1.0);
    const auto   index = static_cast<std::size_t>(
        std::clamp((1.0 - cover) * (kCalibrationCount - 1), 0.0, kCalibrationCount - 1.0));
    std::ranges::nth_element(samples, samples.begin() + static_cast<std::ptrdiff_t>(index));
    woodsThreshold_ = cover <= 0.0 ? 10.0 : samples[index];
}

double Landscape::aroundMeander(double alongM) const
{
    // Noise round a circle, so the river's course closes on itself after one turn.
    if (!frame_.around)
    {
        return 0.0;
    }
    const LandBand& band   = *frame_.around;
    const double    length = band.alongLengthM();
    const double    wave   = std::clamp(length / 4.0, 100.0, kMeanderWaveM);
    const double    ring   = length / (2.0 * kPi * wave);
    const double    phase  = 2.0 * kPi * (alongM - band.alongMinM) / length;
    const Vec2d     round(std::cos(phase), std::sin(phase));
    const double    swing = meander_.sample(Vec3d(round * ring, 0.5)) +
                            (0.35 * meander_.sample(Vec3d(round * (ring / 0.37), 4.5)));
    return swing / 1.35 * meanderM_;
}

Vec2d Landscape::riverAcross(double alongM) const
{
    if (riverTable_.empty() || !frame_.around)
    {
        return Vec2d(0.0);
    }
    const auto& table = riverTable_.front();
    const auto  count = static_cast<double>(table.size());
    double      t     = std::fmod((alongM - frame_.around->alongMinM) / riverTableStep_, count);
    t                 = t < 0.0 ? t + count : t;
    const auto i      = std::min(static_cast<std::size_t>(t), table.size() - 1);
    const auto next   = (i + 1) % table.size();
    return glm::mix(table[i], table[next], t - static_cast<double>(i));
}

double Landscape::aroundShoreDistance(double z, double theta, double far) const
{
    if (!frame_.around)
    {
        return far;
    }
    const LandBand& band = *frame_.around;
    const Vec2d     p    = band.toPlan(z, theta);
    if (std::abs(p.x) > band.halfWidthM + far)
    {
        return far;
    }
    double best = far;
    if (hasRivers())
    {
        const Vec2d  river  = riverAcross(p.y);
        const double across = std::abs(p.x - river.x) / std::sqrt(1.0 + (river.y * river.y));
        best                = std::min(best, across - riverHalfWidthM_);
    }
    const double length = band.alongLengthM();
    for (const Lake& lake : lakes_)
    {
        const double along = std::remainder(p.y - lake.plan.y, length);
        if (std::abs(along) > (lake.halfLengthM * 1.2) + far)
        {
            continue;
        }
        const double da   = along / lake.halfLengthM;
        const double dx   = (p.x - lake.plan.x) / lake.halfWidthM;
        const double e    = std::hypot(da, dx);
        const double size = std::min(lake.halfLengthM, lake.halfWidthM);
        if (e > 1.5)
        {
            best = std::min(best, (e - 1.0) * size);
            continue;
        }
        // An irregular shore, as in a valley's lakes.
        const double direction = std::atan2(dx, da);
        const double wobble =
            1.0 + (0.12 * meander_.sample(Vec3d(2.0 * std::cos(direction),
                                                2.0 * std::sin(direction), lake.plan.y / 997.0)));
        best = std::min(best, (e - wobble) * size);
    }
    return best;
}

}  // namespace StarshipSimulator
