#include "StarshipSimulator/core/procgen/settlements.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/habitat/habitat_geometry.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/habitat/landscape.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/trees.h"
#include "StarshipSimulator/core/rng.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kMainStreetHalfWidth  = 7.0;  // a boulevard, with a row of trees down each side
constexpr double kStreetHalfWidth      = 3.25;
constexpr double kCrossStreetHalfWidth = 3.0;
constexpr double kWaterMarginM         = 8.5;   // buildings keep off the river front path
constexpr double kWalkwayMarginM       = 60.0;  // and from the windows
constexpr double kMaxFootprintRiseM    = 3.0;   // most height difference under a building
constexpr double kPromenadeNearM       = 2.0;   // the river front path: this far from the water
constexpr double kPromenadeFarM        = 7.0;   // ... to this far
constexpr double kLampSpacingM         = 26.0;
constexpr double kLampRangeM           = 14.0;  // the pool of light under a street lamp
constexpr double kMapMarginM           = 24.0;
constexpr double kBuiltReachM          = 30.0;  // ground this near paving is gardens, not fields
constexpr double kMaxSwing             = 0.25;  // steepest the streets swing to follow the river
constexpr double kFarmRadiusM          = 45.0;  // farmyards: wild trees keep off
constexpr int    kFarmAttempts         = 24;

constexpr std::array<const char*, 28> kTownNames{
    "Stanford",   "Oberth",  "Tsiolkovsky", "Goddard", "Noordung",   "Bernal",    "Korolev",
    "Tereshkova", "Gagarin", "Leonov",      "Ride",    "Jemison",    "Johnson",   "Hamilton",
    "Chawla",     "Sagan",   "Clarke",      "Hohmann", "Kondratyuk", "Tsander",   "Vaughan",
    "Jackson",    "Glenn",   "Lovell",      "Guidice", "Davis",      "Princeton", "Heinlein"};

double wrapAngle(double angle)
{
    const double wrapped = std::fmod(angle, 2.0 * kPi);
    return wrapped < 0.0 ? wrapped + (2.0 * kPi) : wrapped;
}

Vec2d rotate90(const Vec2d& v)
{
    return {-v.y, v.x};
}

/// Distance from p to the segment a-b.
double segmentDistance(const Vec2d& p, const Vec2d& a, const Vec2d& b)
{
    const Vec2d  ab = b - a;
    const double t  = std::clamp(glm::dot(p - a, ab) / std::max(glm::dot(ab, ab), 1e-12), 0.0, 1.0);
    return glm::length(p - (a + (ab * t)));
}

/// Signed distance from p to a rectangle (centre, half extents, angle of its x axis).
double boxDistance(const Vec2d& p, const Vec2d& centre, const Vec2d& half, double angle)
{
    const Vec2d axis(std::cos(angle), std::sin(angle));
    const Vec2d local(glm::dot(p - centre, axis), glm::dot(p - centre, rotate90(axis)));
    const Vec2d d = glm::abs(local) - half;
    return glm::length(glm::max(d, Vec2d(0.0))) + std::min(std::max(d.x, d.y), 0.0);
}

/// The four corners of a footprint on the plan.
std::array<Vec2d, 4> corners(const Vec2d& centre, const Vec2d& half, double angle)
{
    const Vec2d x = Vec2d(std::cos(angle), std::sin(angle)) * half.x;
    const Vec2d y = rotate90(Vec2d(std::cos(angle), std::sin(angle))) * half.y;
    return {centre - x - y, centre + x - y, centre + x + y, centre - x + y};
}

/// Whether two footprints overlap (separating axes); touching is not overlapping.
bool footprintsOverlap(const Building& a, const Building& b)
{
    const auto ca = corners(a.centre, a.halfSize, a.angle);
    const auto cb = corners(b.centre, b.halfSize, b.angle);
    for (const double angle : {a.angle, a.angle + (0.5 * kPi), b.angle, b.angle + (0.5 * kPi)})
    {
        const Vec2d axis(std::cos(angle), std::sin(angle));
        double      aLow  = std::numeric_limits<double>::max();
        double      aHigh = std::numeric_limits<double>::lowest();
        double      bLow  = aLow;
        double      bHigh = aHigh;
        for (std::size_t i = 0; i < 4; ++i)
        {
            aLow  = std::min(aLow, glm::dot(ca.at(i), axis));
            aHigh = std::max(aHigh, glm::dot(ca.at(i), axis));
            bLow  = std::min(bLow, glm::dot(cb.at(i), axis));
            bHigh = std::max(bHigh, glm::dot(cb.at(i), axis));
        }
        if (aHigh <= bLow + 0.01 || bHigh <= aLow + 0.01)
        {
            return false;
        }
    }
    return true;
}

/// How many of from, from + step, from + 2 step, ... lie below `to` (step > 0).
int stepsBelow(double from, double to, double step)
{
    return to > from ? static_cast<int>(std::ceil((to - from) / step)) : 0;
}

std::uint8_t toByte(double value)
{
    return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0, 1.0) * 255.0));
}

/// Something paved on a town's ground: a street (a capsule) or a rectangle.
struct Paving
{
    bool         box = false;
    Vec2d        a{0.0};  // capsule: one end; box: centre
    Vec2d        b{0.0};  // capsule: other end; box: half extents
    double       halfWidth = 0.0;
    double       angle     = 0.0;
    std::uint8_t style     = 0;  // 0 street, 255 stone

    [[nodiscard]] double distance(const Vec2d& p) const
    {
        return box ? boxDistance(p, a, b, angle) : segmentDistance(p, a, b) - halfWidth;
    }
    [[nodiscard]] Vec2d reachMin() const
    {
        return box ? a - Vec2d(glm::length(b)) : glm::min(a, b) - Vec2d(halfWidth);
    }
    [[nodiscard]] Vec2d reachMax() const
    {
        return box ? a + Vec2d(glm::length(b)) : glm::max(a, b) + Vec2d(halfWidth);
    }
};

/// What planning needs to know about the habitat.
struct Site
{
    const HabitatGeometry* geometry  = nullptr;
    const TerrainGrid*     grid      = nullptr;
    const Landscape*       landscape = nullptr;
};

/// A town while it is being laid out: its plan's frame and outline.
struct TownFrame
{
    FloorPlane plane;
    int        valley = 0;
    Vec2d      along{0.0, 1.0};    // the main street's direction on the plan
    Vec2d      across{-1.0, 0.0};  // away from the river: always along turned a quarter left
    double     halfLength = 200.0;
    double     halfWidth  = 120.0;
    double     wobble1    = 0.0;  // phases of the outline's irregularity
    double     wobble2    = 0.0;
    double     riverSide  = 1.0;  // +1: the town lies at larger x than the river (on the plan)

    [[nodiscard]] Vec2d  toPlan(double u, double w) const { return (along * u) + (across * w); }
    [[nodiscard]] double u(const Vec2d& p) const { return glm::dot(p, along); }
    [[nodiscard]] double w(const Vec2d& p) const { return glm::dot(p, across); }
    /// 0 at the centre, 1 on the (irregular) outline.
    [[nodiscard]] double reach(const Vec2d& p) const
    {
        const double x   = u(p) / halfLength;
        const double y   = w(p) / halfWidth;
        const double phi = std::atan2(y, x);
        const double r   = std::hypot(x, y);
        return r / (1.0 + (0.12 * std::sin((3.0 * phi) + wobble1)) +
                    (0.06 * std::sin((5.0 * phi) + wobble2)));
    }
};

/// Tests a spot on a settlement's plan: on dry, walkable valley floor.
struct GroundCheck
{
    Site       site;
    FloorPlane plane;
    int        valley = 0;

    [[nodiscard]] bool dry(const Vec2d& p, double margin) const
    {
        const double z     = plane.z(p.y);
        const double theta = plane.theta(p.x);
        if (site.landscape->shoreDistance(z, theta, 100.0) < margin)
        {
            return false;
        }
        const Region region = site.geometry->regionAt(z, theta);
        return region.kind == RegionKind::Land && region.index == valley &&
               distanceToWindow(p) > kWalkwayMarginM;
    }

    [[nodiscard]] double distanceToWindow(const Vec2d& p) const
    {
        const HabitatGeometry& geometry = *site.geometry;
        const double           theta    = plane.theta(p.x);
        const double           centre   = geometry.landCenter(valley);
        const double off = std::abs(std::remainder(theta - centre, 2.0 * kPi)) * plane.radius;
        return (geometry.landHalfAngle() * plane.radius) - off;
    }

    [[nodiscard]] double height(const Vec2d& p) const
    {
        return site.grid->groundHeight(plane.z(p.y), plane.theta(p.x));
    }

    /// Floor level and foundation depth for a footprint, if the ground there can take it.
    [[nodiscard]] std::optional<Vec2d> footprint(const Vec2d& centre, const Vec2d& half,
                                                 double angle) const
    {
        double low  = std::numeric_limits<double>::max();
        double high = std::numeric_limits<double>::lowest();
        for (const Vec2d& corner : corners(centre, half, angle))
        {
            if (!dry(corner, kWaterMarginM))
            {
                return std::nullopt;
            }
            const double h = height(corner);
            low            = std::min(low, h);
            high           = std::max(high, h);
        }
        const double middle = height(centre);
        low                 = std::min(low, middle);
        high                = std::max(high, middle);
        if (high - low > kMaxFootprintRiseM)
        {
            return std::nullopt;
        }
        // The ground floor a step above the highest corner; the walls go well below the lowest,
        // so distant, coarser terrain never shows a gap under them.
        return Vec2d(high + 0.15, high + 0.15 - low + 2.0);
    }
};

// ---- Towns ------------------------------------------------------------------------------------

/// Where a town goes: beside the river, on alternate banks along the valley.
std::optional<TownFrame> placeTown(const Site& site, int valley, int index, int count,
                                   SplitMix64& random)
{
    const HabitatGeometry& geometry  = *site.geometry;
    const SettlementSpec&  spec      = geometry.spec().settlements;
    const double           R         = geometry.radius();
    const double           landHalf  = geometry.landHalfAngle() * R;
    const double           floorSpan = geometry.floorZMax() - geometry.floorZMin();

    TownFrame town;
    town.valley     = valley;
    town.halfLength = spec.townRadiusM * random.uniform(0.75, 1.25);
    town.halfWidth  = std::min(0.6 * town.halfLength, 0.32 * landHalf);
    town.halfLength = std::min({town.halfLength, 1.8 * town.halfWidth, 0.2 * floorSpan});
    town.wobble1    = random.uniform(0.0, 2.0 * kPi);
    town.wobble2    = random.uniform(0.0, 2.0 * kPi);
    if (town.halfWidth < 25.0)
    {
        return std::nullopt;
    }

    const double margin  = std::max(2.0 * town.halfLength, 0.06 * floorSpan);
    const double segment = (floorSpan - (2.0 * margin)) / count;
    if (segment <= 0.0)
    {
        return std::nullopt;
    }
    const double z =
        geometry.floorZMin() + margin + (segment * (index + 0.5 + random.uniform(-0.2, 0.2)));
    const Landscape& landscape  = *site.landscape;
    const double     side       = ((index + valley) % 2 == 0) ? 1.0 : -1.0;
    double           riverTheta = geometry.landCenter(valley);
    Vec2d            along(0.0, 1.0);
    if (landscape.hasRivers())
    {
        riverTheta = landscape.riverAngle(valley, z);
        const double slope =
            (landscape.riverAngle(valley, z + 10.0) - landscape.riverAngle(valley, z - 10.0)) /
            20.0 * R;
        along = glm::normalize(Vec2d(slope, 1.0));
    }
    // Beside the river: step away from it until the town's middle is well clear of the water.
    const double halfRiver = 0.5 * geometry.spec().terrain.riverWidthM;
    double       offset    = landscape.hasRivers() ? halfRiver + 20.0 + (0.55 * town.halfWidth)
                                                   : random.uniform(-0.3, 0.3) * landHalf;
    double       theta     = riverTheta + (side * offset / R);
    while (landscape.hasRivers() &&
           landscape.shoreDistance(z, theta, 400.0) < (0.45 * town.halfWidth) + 10.0 &&
           offset < landHalf - town.halfWidth)
    {
        offset += 15.0;
        theta = riverTheta + (side * offset / R);
    }
    town.plane     = FloorPlane{.z0 = z, .theta0 = wrapAngle(theta), .radius = R};
    town.riverSide = landscape.hasRivers() ? side : 1.0;
    // Across (away from the river) is always along turned a quarter left: the layout's houses and
    // wings rely on that handedness. So turn `along` round if need be.
    town.along  = glm::dot(rotate90(along), Vec2d(side, 0.0)) < 0.0 ? -along : along;
    town.across = rotate90(town.along);

    const GroundCheck check{.site = site, .plane = town.plane, .valley = valley};
    if (!check.dry(Vec2d(0.0), 10.0) ||
        check.distanceToWindow(Vec2d(0.0)) < town.halfWidth + kWalkwayMarginM)
    {
        return std::nullopt;
    }
    return town;
}

/// Everything a town is made of, collected while laying it out.
struct TownPlan
{
    std::vector<Paving>        paving;
    std::vector<Vec2d>         lamps;
    std::vector<Building>      buildings;
    std::vector<Furniture>     furniture;
    std::vector<Bridge>        bridges;
    std::vector<PropPlacement> props;
    std::vector<StandingTree>  trees;
    std::vector<Street>        streets;
};

class TownBuilder
{
public:
    TownBuilder(const Site& site, const TownFrame& town, std::size_t index, SplitMix64& random)
      : site_(site),
        town_(town),
        index_(index),
        random_(&random),
        check_{.site = site, .plane = town.plane, .valley = town.valley}
    {
    }

    TownPlan build()
    {
        layLattice();
        layStreets();
        const bool square = layOutSquare();
        for (int j = 0; j + 1 < static_cast<int>(us_.size()); ++j)
        {
            for (int k = 0; k + 1 < static_cast<int>(ws_.size()); ++k)
            {
                const bool isSquare = square && j == squareJ_ && k == squareK_;
                if (isBlock(j, k) && !isSquare)
                {
                    fillBlock(j, k);
                }
            }
        }
        layRiverFront();
        layBridge();
        return std::move(plan_);
    }

private:
    [[nodiscard]] Vec2d  at(double u, double w) const { return town_.toPlan(u, w); }
    [[nodiscard]] double uAt(int j) const { return us_[static_cast<std::size_t>(j)]; }
    [[nodiscard]] double wAt(int k) const { return ws_[static_cast<std::size_t>(k)]; }

    /// How far the long streets swing sideways at u, following the river's curve (straight
    /// through the middle of town, around the square).
    [[nodiscard]] double bend(double u) const
    {
        if (bend_.size() < 2)
        {
            return 0.0;
        }
        const auto next =
            std::ranges::lower_bound(bend_, u, {}, [](const Vec2d& e) { return e.x; });
        if (next == bend_.begin())
        {
            return bend_.front().y;
        }
        if (next == bend_.end())
        {
            return bend_.back().y;
        }
        const Vec2d& a = *(next - 1);
        const Vec2d& b = *next;
        return std::lerp(a.y, b.y, (u - a.x) / std::max(b.x - a.x, 1e-9));
    }

    /// A point on the plan at (u, w), with the long streets' swing.
    [[nodiscard]] Vec2d bent(double u, double w) const { return at(u, w + bend(u)); }

    /// The direction of the long streets at u (radians from the plan's x axis).
    [[nodiscard]] double streetAngle(double u) const
    {
        const Vec2d dir = glm::normalize(bent(u + 2.0, 0.0) - bent(u - 2.0, 0.0));
        return std::atan2(dir.y, dir.x);
    }

    void layBend()
    {
        const Landscape& landscape = *site_.landscape;
        const double     span      = (1.5 * town_.halfLength) + 100.0;
        const double     limit     = 0.35 * town_.halfWidth;
        if (landscape.hasRivers())
        {
            const int samples = stepsBelow(-span, span + 1e-9, 5.0);
            for (int i = 0; i < samples; ++i)
            {
                const double y = -span + (5.0 * i);
                const double z = town_.plane.z(y);
                const Vec2d  river(
                    std::remainder(landscape.riverAngle(town_.valley, z) - town_.plane.theta0,
                                   2.0 * kPi) *
                        town_.plane.radius,
                    y);
                bend_.emplace_back(town_.u(river), town_.w(river));
            }
            std::ranges::sort(bend_, {}, [](const Vec2d& e) { return e.x; });
            const double middle = bend(0.0);
            const double slope  = (bend(5.0) - bend(-5.0)) / 10.0;
            for (Vec2d& e : bend_)
            {
                e.y -= middle + (slope * e.x);
            }
            // Gentle: from the middle outward, never steeper than kMaxSwing, nor farther out
            // than `limit`, so houses along the streets keep square to them.
            const std::ptrdiff_t centre =
                std::ranges::lower_bound(bend_, 0.0, {}, [](const Vec2d& e) { return e.x; }) -
                bend_.begin();
            const auto          count = static_cast<std::ptrdiff_t>(bend_.size());
            std::vector<double> raw;
            raw.reserve(bend_.size());
            for (const Vec2d& e : bend_)
            {
                raw.push_back(e.y);
            }
            for (const std::ptrdiff_t step : {std::ptrdiff_t{1}, std::ptrdiff_t{-1}})
            {
                for (std::ptrdiff_t i = centre + step; i >= 0 && i < count; i += step)
                {
                    const auto   at     = static_cast<std::size_t>(i);
                    const auto   before = static_cast<std::size_t>(i - step);
                    const double du     = std::abs(bend_[at].x - bend_[before].x);
                    const double change =
                        std::clamp(raw[at] - raw[before], -kMaxSwing * du, kMaxSwing * du);
                    bend_[at].y = std::clamp(bend_[before].y + change, -limit, limit);
                }
            }
            return;
        }
        const double amplitude  = random_->uniform(-12.0, 12.0);
        const double wavelength = random_->uniform(120.0, 220.0);
        const int    samples    = stepsBelow(-span, span + 1e-9, 5.0);
        for (int i = 0; i < samples; ++i)
        {
            const double u = -span + (5.0 * i);
            bend_.emplace_back(u, amplitude * (1.0 - std::cos(u / wavelength)));
        }
    }

    void layLattice()
    {
        layBend();
        // Long streets (along the main street) at ws_, cross streets at us_; the square sits
        // between uAt(jSquare) and uAt(jSquare + 1), on the far side of the main street.
        const int           across = static_cast<int>(std::ceil(town_.halfWidth / 46.0)) + 1;
        const int           along  = static_cast<int>(std::ceil(town_.halfLength / 62.0)) + 1;
        std::vector<double> below;
        std::vector<double> above;
        double              w = 0.0;
        for (int k = 0; k < across; ++k)
        {
            w += random_->uniform(42.0, 54.0);
            above.push_back(w);
        }
        w = 0.0;
        for (int k = 0; k < across; ++k)
        {
            w -= random_->uniform(42.0, 54.0);
            below.push_back(w);
        }
        ws_.assign(below.rbegin(), below.rend());
        mainK_ = static_cast<int>(ws_.size());
        ws_.push_back(0.0);
        ws_.insert(ws_.end(), above.begin(), above.end());

        const double        squareHalf = random_->uniform(22.0, 30.0);
        std::vector<double> left;
        std::vector<double> right;
        double              u = squareHalf;
        right.push_back(u);
        for (int j = 0; j < along; ++j)
        {
            u += random_->uniform(55.0, 78.0);
            right.push_back(u);
        }
        u = -squareHalf;
        left.push_back(u);
        for (int j = 0; j < along; ++j)
        {
            u -= random_->uniform(55.0, 78.0);
            left.push_back(u);
        }
        us_.assign(left.rbegin(), left.rend());
        squareJ_ = static_cast<int>(us_.size()) - 1;
        us_.insert(us_.end(), right.begin(), right.end());
        squareK_ = mainK_;

        const std::size_t cells = (us_.size() - 1) * (ws_.size() - 1);
        blocks_.assign(cells, false);
        for (int j = 0; j + 1 < static_cast<int>(us_.size()); ++j)
        {
            for (int k = 0; k + 1 < static_cast<int>(ws_.size()); ++k)
            {
                const Vec2d centre = bent(0.5 * (uAt(j) + uAt(j + 1)), 0.5 * (wAt(k) + wAt(k + 1)));
                blocks_[cell(j, k)] = town_.reach(centre) < 1.0 && check_.dry(centre, 12.0);
            }
        }
    }

    [[nodiscard]] std::size_t cell(int j, int k) const
    {
        return (static_cast<std::size_t>(j) * (ws_.size() - 1)) + static_cast<std::size_t>(k);
    }

    [[nodiscard]] bool isBlock(int j, int k) const
    {
        return j >= 0 && k >= 0 && j + 1 < static_cast<int>(us_.size()) &&
               k + 1 < static_cast<int>(ws_.size()) && blocks_[cell(j, k)];
    }

    [[nodiscard]] double longHalfWidth(int k) const
    {
        return k == mainK_ ? kMainStreetHalfWidth : kStreetHalfWidth;
    }

    [[nodiscard]] bool hasLongStreet(int j, int k) const
    {
        return isBlock(j, k) || isBlock(j, k - 1);
    }

    [[nodiscard]] bool hasCrossStreet(int j, int k) const
    {
        return isBlock(j, k) || isBlock(j - 1, k);
    }

    /// Adds a street if it stays on dry land (at least `dryMargin` from the water).
    bool addStreet(const Vec2d& from, const Vec2d& to, double halfWidth, double dryMargin = -1.0)
    {
        const std::array<Vec2d, 2> line{from, to};
        return addStreet(line, halfWidth, dryMargin);
    }

    /// Adds a street along a polyline if it all stays on dry land.
    bool addStreet(std::span<const Vec2d> line, double halfWidth, double dryMargin = -1.0)
    {
        const double margin = dryMargin >= 0.0 ? dryMargin : halfWidth + 1.0;
        for (std::size_t i = 0; i + 1 < line.size(); ++i)
        {
            const double length = glm::distance(line[i], line[i + 1]);
            const int    steps  = std::max(1, static_cast<int>(length / 3.0));
            for (int s = 0; s <= steps; ++s)
            {
                if (!check_.dry(glm::mix(line[i], line[i + 1], static_cast<double>(s) / steps),
                                margin))
                {
                    return false;
                }
            }
        }
        for (std::size_t i = 0; i + 1 < line.size(); ++i)
        {
            plan_.paving.push_back(
                {.a = line[i], .b = line[i + 1], .halfWidth = halfWidth, .style = 0});
            plan_.streets.push_back({.from = line[i], .to = line[i + 1], .halfWidth = halfWidth});
        }
        return true;
    }

    /// Points along the long street k from u0 to u1, every few metres, following the swing.
    [[nodiscard]] std::vector<Vec2d> longStreetLine(double u0, double u1, int k) const
    {
        const int          pieces = std::max(1, static_cast<int>(std::ceil((u1 - u0) / 10.0)));
        std::vector<Vec2d> line;
        for (int i = 0; i <= pieces; ++i)
        {
            line.push_back(bent(std::lerp(u0, u1, static_cast<double>(i) / pieces), wAt(k)));
        }
        return line;
    }

    void layStreets()
    {
        const auto longs = static_cast<int>(ws_.size());
        const auto cross = static_cast<int>(us_.size());
        for (int k = 0; k < longs; ++k)
        {
            for (int j = 0; j + 1 < cross; ++j)
            {
                const std::vector<Vec2d> line = longStreetLine(uAt(j), uAt(j + 1), k);
                if (hasLongStreet(j, k) && addStreet(line, longHalfWidth(k)))
                {
                    lineStreet(line, longHalfWidth(k), k == mainK_);
                }
            }
        }
        for (int j = 0; j < cross; ++j)
        {
            for (int k = 0; k + 1 < longs; ++k)
            {
                const std::array<Vec2d, 2> line{bent(uAt(j), wAt(k)), bent(uAt(j), wAt(k + 1))};
                if (hasCrossStreet(j, k) && addStreet(line, kCrossStreetHalfWidth))
                {
                    lineStreet(line, kCrossStreetHalfWidth, false);
                }
            }
        }
    }

    /// Street lamps along a street, alternating sides; on the main street, rows of trees too.
    void lineStreet(std::span<const Vec2d> line, double halfWidth, bool boulevard)
    {
        // Walk the polyline by distance.
        const auto pointAt = [&](double t, double offset) {
            for (std::size_t i = 0; i + 1 < line.size(); ++i)
            {
                const double length = glm::distance(line[i], line[i + 1]);
                if (t <= length || i + 2 == line.size())
                {
                    const Vec2d dir = (line[i + 1] - line[i]) / std::max(length, 1e-9);
                    return line[i] + (dir * t) + (rotate90(dir) * offset);
                }
                t -= length;
            }
            return line.front();
        };
        double total = 0.0;
        for (std::size_t i = 0; i + 1 < line.size(); ++i)
        {
            total += glm::distance(line[i], line[i + 1]);
        }
        const int lamps = stepsBelow(0.5 * kLampSpacingM, total - 6.0, kLampSpacingM);
        for (int n = 0; n < lamps; ++n)
        {
            const double t = (0.5 * kLampSpacingM) + (n * kLampSpacingM);
            addLamp(pointAt(t, (n % 2 == 0 ? 1.0 : -1.0) * (halfWidth - 0.5)));
        }
        if (!boulevard)
        {
            return;
        }
        const int rows = stepsBelow(9.0, total - 9.0, 13.0);
        for (int i = 0; i < rows; ++i)
        {
            const double t = 9.0 + (13.0 * i);
            for (const double offset : {-4.6, 4.6})
            {
                addTree(pointAt(t, offset), TreeSpecies::Broadleaf, random_->uniform(8.0, 11.0));
            }
        }
    }

    void addLamp(const Vec2d& p)
    {
        for (const Street& street : plan_.streets)
        {
            // Not in the middle of a crossing street.
            if (segmentDistance(p, street.from, street.to) < street.halfWidth - 1.0)
            {
                return;
            }
        }
        if (!check_.dry(p, 1.0))
        {
            return;
        }
        plan_.lamps.push_back(p);
        plan_.furniture.push_back({.kind       = FurnitureKind::Lamp,
                                   .settlement = index_,
                                   .position   = p,
                                   .angle      = 0.0,
                                   .height     = check_.height(p)});
    }

    void addTree(const Vec2d& p, TreeSpecies species, double height)
    {
        if (!check_.dry(p, 2.0))
        {
            return;
        }
        plan_.trees.push_back({.settlement = index_,
                               .position   = town_.plane.point(p, check_.height(p)),
                               .species    = species,
                               .heightM    = height});
    }

    void addFurniture(FurnitureKind kind, const Vec2d& p, double angle)
    {
        if (check_.dry(p, 1.0))
        {
            plan_.furniture.push_back({.kind       = kind,
                                       .settlement = index_,
                                       .position   = p,
                                       .angle      = angle,
                                       .height     = check_.height(p)});
        }
    }

    void addProp(PropKind kind, const Vec2d& p, double angle, double lift = 0.0)
    {
        if (!check_.dry(p, 1.0))
        {
            return;
        }
        const Vec3d ground = town_.plane.point(p, check_.height(p) + lift);
        plan_.props.push_back({.kind        = kind,
                               .position    = ground,
                               .orientation = floorOrientation(ground, -angle),
                               .tint        = static_cast<float>(random_->uniform())});
    }

    /// Places a building if the ground takes it. Returns its index in the plan.
    std::optional<std::size_t> addBuilding(Building building)
    {
        const auto level = check_.footprint(building.centre, building.halfSize, building.angle);
        if (!level)
        {
            return std::nullopt;
        }
        const auto around = corners(building.centre, building.halfSize, building.angle);
        for (const Vec2d& corner : around)
        {
            if (town_.reach(corner) > 1.15)
            {
                return std::nullopt;
            }
        }
        // Clear of the streets: corners, edge middles and the centre.
        for (std::size_t i = 0; i < around.size(); ++i)
        {
            const std::array<Vec2d, 2> points{around.at(i),
                                              0.5 * (around.at(i) + around.at((i + 1) % 4))};
            for (const Vec2d& p : points)
            {
                if (onStreet(p, 0.2))
                {
                    return std::nullopt;
                }
            }
        }
        if (onStreet(building.centre, std::min(building.halfSize.x, building.halfSize.y)))
        {
            return std::nullopt;
        }
        building.settlement  = index_;
        building.floorHeight = level->x;
        building.foundation  = level->y;
        building.seed        = static_cast<std::uint32_t>(random_->next());
        plan_.buildings.push_back(building);
        plan_.paving.push_back({.box   = true,
                                .a     = building.centre,
                                .b     = building.halfSize + Vec2d(0.6),
                                .angle = building.angle,
                                .style = 255});
        return plan_.buildings.size() - 1;
    }

    [[nodiscard]] bool onStreet(const Vec2d& p, double margin) const
    {
        return std::ranges::any_of(plan_.streets, [&](const Street& street) {
            return segmentDistance(p, street.from, street.to) < street.halfWidth + margin;
        });
    }

    [[nodiscard]] Building house(const Vec2d& centre, const Vec2d& half, double angle, double reach,
                                 bool shop)
    {
        Building b;
        b.use      = shop ? BuildingUse::Shop : BuildingUse::House;
        b.centre   = centre;
        b.halfSize = half;
        b.angle    = angle;
        // Taller toward the middle of town.
        const double tall = 1.0 - std::clamp(reach, 0.0, 1.0);
        b.storeys         = std::clamp(
            1 + static_cast<int>(std::floor((tall * 2.6) + random_->uniform(0.0, 1.2))), 1, 4);
        const double pick = random_->uniform();
        if (pick < 0.1)
        {
            b.roof = RoofKind::Flat;
        }
        else
        {
            b.roof =
                half.x > 1.35 * half.y || half.y > 1.35 * half.x ? RoofKind::Gable : RoofKind::Hip;
        }
        b.roofPitchDeg  = random_->uniform(22.0, 32.0);
        b.wallColour    = static_cast<std::uint8_t>(random_->next() % 8);
        b.roofColour    = static_cast<std::uint8_t>(random_->next() % 4);
        b.shutterColour = static_cast<std::uint8_t>(random_->next() % 5);
        b.frontSide     = 0;
        return b;
    }

    /// The part of a block left for building: inside its streets (u along, w across).
    struct BlockArea
    {
        double u0 = 0.0;
        double u1 = 0.0;
        double w0 = 0.0;
        double w1 = 0.0;
    };

    [[nodiscard]] bool overlapsOthers(const Building& b, std::size_t first) const
    {
        return std::any_of(plan_.buildings.begin() + static_cast<std::ptrdiff_t>(first),
                           plan_.buildings.end(),
                           [&](const Building& other) { return footprintsOverlap(b, other); });
    }

    /// Houses around a block, facing the streets; gardens (and their trees) in the middle.
    void fillBlock(int j, int k)
    {
        const BlockArea area{
            .u0 = uAt(j) + (hasCrossStreet(j, k) ? kCrossStreetHalfWidth : 0.0) + 2.5,
            .u1 = uAt(j + 1) - (hasCrossStreet(j + 1, k) ? kCrossStreetHalfWidth : 0.0) - 2.5,
            .w0 = wAt(k) + (hasLongStreet(j, k) ? longHalfWidth(k) : 0.0) + 1.0,
            .w1 = wAt(k + 1) - (hasLongStreet(j, k + 1) ? longHalfWidth(k + 1) : 0.0) - 1.0};
        if (area.u1 - area.u0 < 16.0 || area.w1 - area.w0 < 16.0)
        {
            return;
        }
        const double reach =
            town_.reach(bent(0.5 * (area.u0 + area.u1), 0.5 * (area.w0 + area.w1)));
        const bool   terrace = reach < 0.45;  // row houses in the middle of town
        const double depth =
            std::min(random_->uniform(13.0, 17.0), (0.5 * (area.w1 - area.w0)) - 2.0);
        const std::size_t first = plan_.buildings.size();
        for (const bool upper : {false, true})
        {
            if (hasLongStreet(j, upper ? k + 1 : k))
            {
                rowOfHouses(area, upper, terrace, depth, reach, (upper ? k + 1 : k) == mainK_,
                            first);
            }
        }
        for (const bool right : {false, true})
        {
            if (hasCrossStreet(right ? j + 1 : j, k))
            {
                endHouse(area, right, terrace, depth, reach, first);
            }
        }
        plantGardens(area, first);
        leaveCrates(first);
    }

    /// Houses along one long side of a block, facing the street.
    void rowOfHouses(const BlockArea& area, bool upper, bool terrace, double depth, double reach,
                     bool main, std::size_t first)
    {
        double u = area.u0;
        while (u < area.u1 - 7.0)
        {
            const double frontage =
                terrace ? random_->uniform(7.5, 10.0) : random_->uniform(10.0, 15.0);
            const double end     = std::min(u + frontage, area.u1);
            const double gap     = terrace ? 0.0 : random_->uniform(1.5, 4.0);
            const double setback = terrace ? 0.0 : random_->uniform(0.5, 3.0);
            const double deep    = std::min(random_->uniform(8.0, 12.0), depth - setback);
            const double width   = end - u - gap;
            const double middle  = 0.5 * (u + end - gap);
            u                    = end;
            // Now and then a garden instead of a house.
            if (width < 6.0 || deep < 6.0 || (!terrace && random_->uniform() < 0.1))
            {
                continue;
            }
            const double w =
                upper ? area.w1 - setback - (0.5 * deep) : area.w0 + setback + (0.5 * deep);
            const double   angle = streetAngle(middle) + (upper ? kPi : 0.0);
            const Building b = house(bent(middle, w), Vec2d(0.5 * width, 0.5 * deep), angle, reach,
                                     main && reach < 0.7 && random_->uniform() < 0.7);
            if (overlapsOthers(b, first) || !addBuilding(b))
            {
                continue;
            }
            if (!terrace && random_->uniform() < 0.2)
            {
                addWing(plan_.buildings.back(), first);
            }
        }
    }

    /// A lower wing behind a house, making an L.
    void addWing(const Building& main, std::size_t first)
    {
        const Vec2d  x(std::cos(main.angle), std::sin(main.angle));
        const Vec2d  y    = rotate90(x);
        const double deep = random_->uniform(4.0, 6.0);
        const double side = random_->uniform() < 0.5 ? -1.0 : 1.0;
        Building     wing = main;
        wing.halfSize     = Vec2d(std::min((0.3 * main.halfSize.x) + 1.2, 3.5), 0.5 * deep);
        wing.centre       = main.centre + (y * (main.halfSize.y + wing.halfSize.y)) +
                            (x * side * (main.halfSize.x - wing.halfSize.x));
        wing.storeys      = std::max(1, main.storeys - 1);
        wing.use          = BuildingUse::House;
        wing.roof         = wing.roof == RoofKind::Flat ? RoofKind::Flat : RoofKind::Hip;
        wing.frontSide    = -1;  // no door
        if (!overlapsOthers(wing, first))
        {
            addBuilding(wing);
        }
    }

    /// A house at one end of a block, facing the cross street, between the rows.
    void endHouse(const BlockArea& area, bool right, bool terrace, double depth, double reach,
                  std::size_t first)
    {
        const double spanW = (area.w1 - depth) - (area.w0 + depth);
        if (spanW < 9.0)
        {
            return;
        }
        const double deep    = std::min(random_->uniform(8.0, 11.0), 0.3 * (area.u1 - area.u0));
        const double setback = terrace ? 0.0 : random_->uniform(0.5, 2.5);
        const double u =
            right ? area.u1 - setback - (0.5 * deep) : area.u0 + setback + (0.5 * deep);
        const double   side = std::atan2(town_.across.y, town_.across.x);
        const Building b    = house(bent(u, 0.5 * (area.w0 + area.w1)),
                                    Vec2d(0.5 * std::min(spanW - 1.0, 14.0), 0.5 * deep),
                                    right ? side : side + kPi, reach, false);
        if (!overlapsOthers(b, first))
        {
            addBuilding(b);
        }
    }

    /// Fruit trees in the gardens, clear of the houses.
    void plantGardens(const BlockArea& area, std::size_t first)
    {
        const int trees = static_cast<int>(random_->uniform(0.0, 3.5));
        for (int t = 0; t < trees; ++t)
        {
            const Vec2d p = bent(random_->uniform(area.u0 + 4.0, area.u1 - 4.0),
                                 random_->uniform(area.w0 + 4.0, area.w1 - 4.0));
            const bool  clear =
                std::none_of(plan_.buildings.begin() + static_cast<std::ptrdiff_t>(first),
                             plan_.buildings.end(), [&](const Building& b) {
                                 return boxDistance(p, b.centre, b.halfSize, b.angle) < 3.5;
                             });
            if (clear)
            {
                addTree(p, TreeSpecies::Broadleaf, random_->uniform(5.0, 8.5));
            }
        }
    }

    /// A few crates and barrels by the doors.
    void leaveCrates(std::size_t first)
    {
        for (std::size_t i = first; i < plan_.buildings.size(); ++i)
        {
            const Building& b = plan_.buildings[i];
            if (b.frontSide != 0 || random_->uniform() >= 0.14)
            {
                continue;
            }
            const Vec2d x = Vec2d(std::cos(b.angle), std::sin(b.angle));
            const Vec2d front =
                b.centre - (rotate90(x) * (b.halfSize.y + 0.8)) + (x * (b.halfSize.x - 1.0));
            if (random_->uniform() < 0.5)
            {
                addProp(PropKind::Crate, front, b.angle);
                if (random_->uniform() < 0.5)
                {
                    addProp(PropKind::Crate, front, b.angle + 0.3, 0.6);
                }
            }
            else
            {
                addProp(PropKind::Barrel, front, 0.0);
                addProp(PropKind::Barrel, front - (x * 0.7), 0.0);
            }
        }
    }

    /// The town square: a fountain, a hall with a bell tower, market stalls and a cafe.
    bool layOutSquare()
    {
        const int j = squareJ_;
        const int k = squareK_;
        if (!isBlock(j, k))
        {
            return false;
        }
        const double u0    = uAt(j) + kCrossStreetHalfWidth;
        const double u1    = uAt(j + 1) - kCrossStreetHalfWidth;
        const double w0    = wAt(k) + kMainStreetHalfWidth;
        const double w1    = wAt(k + 1) - (hasLongStreet(j, k + 1) ? kStreetHalfWidth : 0.0);
        const double angle = std::atan2(town_.along.y, town_.along.x);
        const double uMid  = 0.5 * (u0 + u1);
        const double hallD = 12.0;
        const double open  = w1 - w0 - hallD - 2.0;  // the square itself, in front of the hall
        if (u1 - u0 < 30.0 || open < 18.0)
        {
            return false;
        }
        const double wSquare = w0 + (0.5 * open);
        plan_.paving.push_back({.box   = true,
                                .a     = at(uMid, wSquare),
                                .b     = Vec2d((0.5 * (u1 - u0)) + 1.0, (0.5 * open) + 1.0),
                                .angle = angle,
                                .style = 255});

        // The hall along the back, its bell tower at one end.
        const double hallW = std::min(26.0, u1 - u0 - 10.0);
        Building     hall  = house(at(uMid - 2.5, w1 - 1.0 - (0.5 * hallD)),
                                   Vec2d(0.5 * hallW, 0.5 * hallD), angle, 0.0, false);
        hall.use           = BuildingUse::Hall;
        hall.storeys       = 2;
        hall.storeyHeightM = 4.2;
        hall.roof          = RoofKind::Hip;
        hall.wallColour    = 0;
        hall.shutterColour = 0;
        addBuilding(hall);
        Building tower      = hall;
        tower.use           = BuildingUse::Tower;
        tower.centre        = at(uMid - 2.5 + (0.5 * hallW) + 3.2, w1 - 1.0 - 3.0);
        tower.halfSize      = Vec2d(2.6);
        tower.storeys       = static_cast<int>(random_->uniform(5.0, 7.0));
        tower.storeyHeightM = 3.4;
        tower.roof          = RoofKind::Pyramid;
        tower.roofPitchDeg  = 55.0;
        addBuilding(tower);

        // The fountain in the middle, benches and lamps around it.
        const Vec2d centre = at(uMid, wSquare);
        addFurniture(FurnitureKind::Fountain, centre, angle);
        for (int i = 0; i < 4; ++i)
        {
            const double a = angle + (0.25 * kPi) + (0.5 * kPi * i);
            addFurniture(FurnitureKind::Bench, centre + (Vec2d(std::cos(a), std::sin(a)) * 7.5),
                         a + (0.5 * kPi));
            const double b = a + (0.25 * kPi);
            addLamp(centre + (Vec2d(std::cos(b), std::sin(b)) * 9.0));
        }
        // Planters and trees in the corners, lamps along the edges.
        for (const double su : {-1.0, 1.0})
        {
            for (const double sw : {-1.0, 1.0})
            {
                const Vec2d corner = at(uMid + (su * ((0.5 * (u1 - u0)) - 4.0)),
                                        wSquare + (sw * ((0.5 * open) - 4.0)));
                addTree(corner, TreeSpecies::Broadleaf, random_->uniform(9.0, 12.0));
                addFurniture(FurnitureKind::Planter,
                             at(uMid + (su * ((0.5 * (u1 - u0)) - 9.0)),
                                wSquare + (sw * ((0.5 * open) - 2.0))),
                             angle);
                addLamp(at(uMid + (su * ((0.5 * (u1 - u0)) - 1.0)), wSquare + (sw * 3.0)));
            }
        }
        // Market stalls on one side, the cafe on the other.
        for (int i = 0; i < 3; ++i)
        {
            const Vec2d stall = at(u0 + 3.0, wSquare - 7.0 + (7.0 * i));
            addFurniture(FurnitureKind::Stall, stall, angle + (0.5 * kPi));
            addProp(PropKind::Crate, stall + (town_.along * 1.8), angle);
        }
        for (int i = 0; i < 4; ++i)
        {
            const Vec2d table =
                at(u1 - 4.0 - (random_->uniform(0.0, 1.0)), wSquare - 7.5 + (5.0 * i));
            addProp(PropKind::Table, table, 0.0);
            const int chairs = 2 + static_cast<int>(random_->uniform(0.0, 2.0));
            for (int c = 0; c < chairs; ++c)
            {
                const double a = (2.0 * kPi * c / chairs) + random_->uniform(-0.3, 0.3);
                addProp(PropKind::Chair, table + (Vec2d(std::cos(a), std::sin(a)) * 0.75),
                        a + (0.5 * kPi));
            }
        }
        addProp(PropKind::Ball, centre + (town_.across * 5.0) + (town_.along * 4.0), 0.0);
        return true;
    }

    /// The river front: a stone footpath along the bank, with lamps, benches and poplars.
    void layRiverFront()
    {
        const Landscape& landscape = *site_.landscape;
        if (!landscape.hasRivers())
        {
            return;
        }
        const int stations = stepsBelow(-town_.halfLength, town_.halfLength + 1e-9, 12.0);
        for (int n = 0; n < stations; ++n)
        {
            // Walk from the town toward the river until the bank.
            const double u     = -town_.halfLength + (12.0 * n);
            const Vec2d  start = at(u, 0.0);
            if (town_.reach(start) > 1.0)
            {
                continue;
            }
            std::optional<Vec2d> bank;
            const int            steps = stepsBelow(0.0, 3.0 * town_.halfWidth, 1.0);
            for (int i = 0; i < steps; ++i)
            {
                const Vec2d  p = at(u, -static_cast<double>(i));
                const double shore =
                    landscape.shoreDistance(town_.plane.z(p.y), town_.plane.theta(p.x), 50.0);
                if (shore < 4.5)
                {
                    bank = p;
                    break;
                }
            }
            if (!bank || town_.reach(*bank) > 1.1)
            {
                continue;
            }
            const Vec2d inland = town_.across;
            if (n % 2 == 0)
            {
                addLamp(*bank + (inland * 2.3));
            }
            if (n % 4 == 1)
            {
                addFurniture(FurnitureKind::Bench, *bank + (inland * 0.2),
                             std::atan2(-inland.y, -inland.x) + (0.5 * kPi));
            }
            if (n % 2 == 1)
            {
                addTree(*bank + (inland * 6.5), TreeSpecies::Poplar, random_->uniform(14.0, 19.0));
            }
        }
    }

    /// A stone bridge over the river, carrying the cross street beside the square.
    void layBridge()
    {
        const Landscape& landscape = *site_.landscape;
        if (!landscape.hasRivers())
        {
            return;
        }
        const int    j     = std::min(squareJ_ + 1, static_cast<int>(us_.size()) - 1);
        const double u     = uAt(j);
        const auto   shore = [&](double w) {
            const Vec2d p = at(u, w);
            return landscape.shoreDistance(town_.plane.z(p.y), town_.plane.theta(p.x), 60.0);
        };
        // Walk from the main street toward the river, across it, and up the far bank.
        std::optional<double> nearBank;
        std::optional<double> farBank;
        const double          limit = -(3.0 * town_.halfWidth) - 300.0;
        const int             steps = stepsBelow(limit, bend(u), 0.5);
        for (int i = 0; i < steps; ++i)
        {
            const double w = bend(u) - (0.5 * i);
            const double s = shore(w);
            if (!nearBank && s < 1.0)
            {
                nearBank = w + 2.0;
            }
            if (nearBank && s > 2.0 && w < *nearBank - 8.0)
            {
                farBank = w - 1.0;
                break;
            }
        }
        if (!nearBank || !farBank || *nearBank - *farBank > 110.0)
        {
            return;
        }
        // The cross street runs down as far as the town's blocks; a lane continues to the bridge.
        int k = mainK_;
        while (k > 0 && hasCrossStreet(j, k - 1))
        {
            --k;
        }
        const double streetEnd = wAt(k) + bend(u);
        const Vec2d  from      = at(u, *nearBank);
        const Vec2d  to        = at(u, *farBank);
        if (streetEnd > *nearBank + 1.0)
        {
            const Vec2d approach = at(u, streetEnd);
            if (!clearOfBuildings(approach, from, kCrossStreetHalfWidth + 1.0) ||
                !addStreet(approach, from, kCrossStreetHalfWidth, 0.5))
            {
                return;
            }
        }
        if (clearOfBuildings(to, to - (town_.across * 40.0), kCrossStreetHalfWidth + 1.0))
        {
            addStreet(to, to - (town_.across * 40.0), kCrossStreetHalfWidth, 0.5);
        }
        plan_.bridges.push_back({.settlement = index_,
                                 .from       = from,
                                 .to         = to,
                                 .fromHeight = check_.height(from),
                                 .toHeight   = check_.height(to),
                                 .halfWidth  = kCrossStreetHalfWidth,
                                 .rise = std::min(2.2, (0.05 * glm::distance(from, to)) + 0.8)});
    }

    [[nodiscard]] bool clearOfBuildings(const Vec2d& from, const Vec2d& to, double halfWidth) const
    {
        return std::ranges::none_of(plan_.buildings, [&](const Building& b) {
            return std::ranges::any_of(
                       corners(b.centre, b.halfSize, b.angle),
                       [&](const Vec2d& c) { return segmentDistance(c, from, to) < halfWidth; }) ||
                   segmentDistance(b.centre, from, to) <
                       halfWidth + std::min(b.halfSize.x, b.halfSize.y);
        });
    }

    Site                site_;
    TownFrame           town_;
    std::size_t         index_;
    SplitMix64*         random_;
    GroundCheck         check_;
    std::vector<double> us_;
    std::vector<double> ws_;
    std::vector<bool>   blocks_;
    std::vector<Vec2d>  bend_;  // (u, swing of the long streets), sorted by u
    int                 mainK_   = 0;
    int                 squareJ_ = 0;
    int                 squareK_ = 0;
    TownPlan            plan_;
};

/// Paints a town's ground map: paving, built-up ground, the light of its lamps.
class GroundPainter
{
public:
    GroundPainter(const Site& site, const TownFrame& town, const TownPlan& plan)
      : site_(site), town_(town), plan_(&plan)
    {
        frame();
        const std::size_t count = static_cast<std::size_t>(map_.width) * map_.height;
        distance_.assign(count, static_cast<float>(kBuiltReachM));
        style_.assign(count, 0);
        light_.assign(count, 0.0F);
    }

    GroundMap paint()
    {
        for (const Paving& paving : plan_->paving)
        {
            pave(paving);
        }
        for (const Vec2d& lamp : plan_->lamps)
        {
            light(lamp);
        }
        map_.texels.resize(distance_.size() * 4);
        for (std::uint32_t y = 0; y < map_.height; ++y)
        {
            for (std::uint32_t x = 0; x < map_.width; ++x)
            {
                finish(x, y);
            }
        }
        return std::move(map_);
    }

private:
    /// The map's extent: the town's outline and everything paved, with a margin.
    void frame()
    {
        Vec2d low(std::numeric_limits<double>::max());
        Vec2d high(std::numeric_limits<double>::lowest());
        for (const double su : {-1.0, 1.0})
        {
            for (const double sw : {-1.0, 1.0})
            {
                const Vec2d p =
                    town_.toPlan(su * 1.2 * town_.halfLength, sw * 1.2 * town_.halfWidth);
                low  = glm::min(low, p);
                high = glm::max(high, p);
            }
        }
        for (const Paving& paving : plan_->paving)
        {
            low  = glm::min(low, paving.reachMin());
            high = glm::max(high, paving.reachMax());
        }
        for (const Bridge& bridge : plan_->bridges)
        {
            low  = glm::min(low, glm::min(bridge.from, bridge.to));
            high = glm::max(high, glm::max(bridge.from, bridge.to));
        }
        low -= Vec2d(kMapMarginM);
        high += Vec2d(kMapMarginM);
        map_.origin = glm::floor(low);
        map_.texelM = 1.0;
        map_.width  = static_cast<std::uint32_t>(std::ceil(high.x - map_.origin.x));
        map_.height = static_cast<std::uint32_t>(std::ceil(high.y - map_.origin.y));
    }

    [[nodiscard]] Vec2d texel(std::uint32_t x, std::uint32_t y) const
    {
        return map_.origin + ((Vec2d(x, y) + 0.5) * map_.texelM);
    }
    [[nodiscard]] std::size_t index(std::uint32_t x, std::uint32_t y) const
    {
        return (static_cast<std::size_t>(y) * map_.width) + x;
    }
    /// The texels covering plan positions from `low` to `high`, as [x0, x1] x [y0, y1].
    [[nodiscard]] std::array<std::uint32_t, 4> span(const Vec2d& low, const Vec2d& high) const
    {
        const auto x = [&](double v) {
            return static_cast<std::uint32_t>(std::clamp(v - map_.origin.x, 0.0, map_.width - 1.0));
        };
        const auto y = [&](double v) {
            return static_cast<std::uint32_t>(
                std::clamp(v - map_.origin.y, 0.0, map_.height - 1.0));
        };
        return {x(low.x), x(high.x), y(low.y), y(high.y)};
    }

    /// The nearest paving (and its style), out to the built-up reach.
    void pave(const Paving& paving)
    {
        const auto [x0, x1, y0, y1] =
            span(paving.reachMin() - Vec2d(kBuiltReachM), paving.reachMax() + Vec2d(kBuiltReachM));
        for (std::uint32_t y = y0; y <= y1; ++y)
        {
            for (std::uint32_t x = x0; x <= x1; ++x)
            {
                const auto d = static_cast<float>(paving.distance(texel(x, y)));
                if (d < distance_[index(x, y)])
                {
                    distance_[index(x, y)] = d;
                    style_[index(x, y)]    = paving.style;
                }
            }
        }
    }

    /// A pool of light around a lamp.
    void light(const Vec2d& lamp)
    {
        const auto [x0, x1, y0, y1] = span(lamp - Vec2d(kLampRangeM), lamp + Vec2d(kLampRangeM));
        for (std::uint32_t y = y0; y <= y1; ++y)
        {
            for (std::uint32_t x = x0; x <= x1; ++x)
            {
                const Vec2d d = texel(x, y) - lamp;
                light_[index(x, y)] +=
                    static_cast<float>(std::exp(-glm::dot(d, d) / (2.0 * 4.5 * 4.5)));
            }
        }
    }

    /// One texel: the river front path, no paving under water, then encoded.
    void finish(std::uint32_t x, std::uint32_t y)
    {
        const Landscape& landscape = *site_.landscape;
        const Vec2d      p         = texel(x, y);
        const double     reach     = town_.reach(p);
        const double     range     = 0.5 * kGroundMapDistanceRange;
        const auto       nearest   = static_cast<double>(distance_[index(x, y)]);
        double           d         = nearest;
        std::uint8_t     kind      = style_[index(x, y)];
        if (landscape.hasRivers() && reach < 1.25)
        {
            const double z     = town_.plane.z(p.y);
            const double shore = landscape.shoreDistance(z, town_.plane.theta(p.x), 20.0);
            const double river =
                std::remainder(landscape.riverAngle(town_.valley, z) - town_.plane.theta0,
                               2.0 * kPi) *
                town_.plane.radius;
            const double path = std::max(kPromenadeNearM - shore, shore - kPromenadeFarM);
            if ((p.x - river) * town_.riverSide > 0.0 && reach < 1.1 && path < d)
            {
                d    = path;
                kind = 255;
            }
            d = std::max(d, 1.0 - shore);
        }
        const double built =
            std::max(1.0 - glm::smoothstep(0.95, 1.2, reach),
                     1.0 - glm::smoothstep(0.4 * kBuiltReachM, kBuiltReachM, nearest));
        const std::size_t at = index(x, y) * 4;
        map_.texels[at]      = toByte((d + range) / (2.0 * range));
        map_.texels[at + 1]  = kind;
        map_.texels[at + 2]  = toByte(built);
        map_.texels[at + 3]  = toByte(0.9 * static_cast<double>(light_[index(x, y)]));
    }

    Site                      site_;
    TownFrame                 town_;
    const TownPlan*           plan_;
    GroundMap                 map_;
    std::vector<float>        distance_;
    std::vector<std::uint8_t> style_;
    std::vector<float>        light_;
};

// ---- Farms ------------------------------------------------------------------------------------

std::optional<Settlement> planFarm(const Site& site, int valley, std::size_t index,
                                   const std::vector<Settlement>& others, SplitMix64& random,
                                   std::vector<Building>&      buildings,
                                   std::vector<PropPlacement>& props,
                                   std::vector<StandingTree>&  trees)
{
    const HabitatGeometry& geometry = *site.geometry;
    const double           R        = geometry.radius();
    const double           landHalf = geometry.landHalfAngle() * R;
    const double margin = std::min(800.0, 0.1 * (geometry.floorZMax() - geometry.floorZMin()));
    for (int attempt = 0; attempt < kFarmAttempts; ++attempt)
    {
        const double z =
            random.uniform(geometry.floorZMin() + margin, geometry.floorZMax() - margin);
        const double theta =
            geometry.landCenter(valley) + (random.uniform(-0.8, 0.8) * landHalf / R);
        const FloorPlane  plane{.z0 = z, .theta0 = wrapAngle(theta), .radius = R};
        const GroundCheck check{.site = site, .plane = plane, .valley = valley};
        const Vec3d       here    = plane.point(Vec2d(0.0), 0.0);
        const bool        crowded = std::ranges::any_of(others, [&](const Settlement& other) {
            const double room = other.kind == SettlementKind::Town ? other.radiusM + 250.0 : 400.0;
            return glm::distance(other.plane.point(Vec2d(0.0), 0.0), here) < room;
        });
        if (crowded || !check.dry(Vec2d(0.0), 70.0) || check.distanceToWindow(Vec2d(0.0)) < 150.0 ||
            site.landscape->woodland(z, theta) > 0.35)
        {
            continue;
        }
        const double angle = random.uniform(0.0, 2.0 * kPi);
        const Vec2d  dir(std::cos(angle), std::sin(angle));
        Building     house;
        house.use           = BuildingUse::Farmhouse;
        house.settlement    = index;
        house.centre        = Vec2d(0.0);
        house.halfSize      = Vec2d(random.uniform(5.5, 6.5), random.uniform(4.0, 4.6));
        house.angle         = angle;
        house.storeys       = 2;
        house.roof          = RoofKind::Gable;
        house.roofPitchDeg  = random.uniform(30.0, 38.0);
        house.wallColour    = static_cast<std::uint8_t>(random.next() % 3);
        house.roofColour    = static_cast<std::uint8_t>(random.next() % 4);
        house.shutterColour = static_cast<std::uint8_t>(1 + (random.next() % 4));
        house.seed          = static_cast<std::uint32_t>(random.next());
        Building barn       = house;
        barn.use            = BuildingUse::Barn;
        barn.centre =
            (dir * random.uniform(20.0, 26.0)) + (rotate90(dir) * random.uniform(-6.0, 6.0));
        barn.halfSize         = Vec2d(random.uniform(9.0, 12.0), random.uniform(5.5, 6.5));
        barn.angle            = angle + (0.5 * kPi);
        barn.storeys          = 1;
        barn.storeyHeightM    = random.uniform(5.5, 6.5);
        barn.roofPitchDeg     = random.uniform(35.0, 42.0);
        barn.wallColour       = static_cast<std::uint8_t>(random.next() % 2);
        barn.shutterColour    = 0;
        barn.seed             = static_cast<std::uint32_t>(random.next());
        const auto houseLevel = check.footprint(house.centre, house.halfSize, house.angle);
        const auto barnLevel  = check.footprint(barn.centre, barn.halfSize, barn.angle);
        if (!houseLevel || !barnLevel)
        {
            continue;
        }
        house.floorHeight = houseLevel->x;
        house.foundation  = houseLevel->y;
        barn.floorHeight  = barnLevel->x;
        barn.foundation   = barnLevel->y;

        Settlement farm;
        farm.kind          = SettlementKind::Farm;
        farm.name          = "a farmstead";
        farm.valley        = valley;
        farm.plane         = plane;
        farm.radiusM       = kFarmRadiusM;
        farm.boundsMin     = Vec2d(-kFarmRadiusM);
        farm.boundsMax     = Vec2d(kFarmRadiusM);
        farm.buildingCount = 2;
        buildings.push_back(house);
        buildings.push_back(barn);

        // Hay bales by the barn, two trees by the house.
        const Vec2d barnX(std::cos(barn.angle), std::sin(barn.angle));
        const Vec2d yard  = barn.centre - (rotate90(barnX) * (barn.halfSize.y + 3.5));
        const int   bales = 3 + static_cast<int>(random.uniform(0.0, 4.0));
        for (int b = 0; b < bales; ++b)
        {
            const Vec2d p = yard + (barnX * (1.3 * (b - (0.5 * bales)))) +
                            (rotate90(barnX) * random.uniform(-0.3, 0.3));
            if (check.dry(p, 2.0))
            {
                const Vec3d ground = plane.point(p, check.height(p));
                props.push_back({.kind        = PropKind::HayBale,
                                 .position    = ground,
                                 .orientation = floorOrientation(ground),
                                 .tint        = static_cast<float>(random.uniform())});
            }
        }
        for (const double side : {-1.0, 1.0})
        {
            const Vec2d p = (dir * -9.0) + (rotate90(dir) * (side * 7.0));
            if (check.dry(p, 2.0))
            {
                trees.push_back({.settlement = index,
                                 .position   = plane.point(p, check.height(p)),
                                 .species    = TreeSpecies::Broadleaf,
                                 .heightM    = random.uniform(9.0, 13.0)});
            }
        }
        return farm;
    }
    return std::nullopt;
}

/// Grows the plan bounds of a settlement to cover its buildings.
void fitBounds(Settlement& place, std::span<const Building> buildings)
{
    for (const Building& b : buildings)
    {
        for (const Vec2d& corner : corners(b.centre, b.halfSize + Vec2d(2.0), b.angle))
        {
            place.boundsMin = glm::min(place.boundsMin, corner);
            place.boundsMax = glm::max(place.boundsMax, corner);
        }
    }
}

}  // namespace

Vec3d FloorPlane::point(const Vec2d& p, double height) const
{
    const double t = theta(p.x);
    const double r = radius - height;
    return {r * std::cos(t), r * std::sin(t), z(p.y)};
}

Vec2d FloorPlane::toPlan(double z, double theta) const
{
    return {std::remainder(theta - theta0, 2.0 * kPi) * radius, z - z0};
}

Vec4d GroundMap::sample(const Vec2d& p) const
{
    if (width < 2 || height < 2)
    {
        return Vec4d(0.0);
    }
    const Vec2d t = ((p - origin) / texelM) - 0.5;
    if (t.x < 0.0 || t.y < 0.0 || t.x > width - 1.0 || t.y > height - 1.0)
    {
        return Vec4d(0.0);
    }
    const auto  x0 = std::min(static_cast<std::uint32_t>(t.x), width - 2);
    const auto  y0 = std::min(static_cast<std::uint32_t>(t.y), height - 2);
    const Vec2d f  = t - Vec2d(x0, y0);
    const auto  at = [&](std::uint32_t x, std::uint32_t y) {
        const std::size_t offset = ((static_cast<std::size_t>(y) * width) + x) * 4;
        return Vec4d(texels[offset], texels[offset + 1], texels[offset + 2], texels[offset + 3]) /
               255.0;
    };
    return glm::mix(glm::mix(at(x0, y0), at(x0 + 1, y0), f.x),
                    glm::mix(at(x0, y0 + 1), at(x0 + 1, y0 + 1), f.x), f.y);
}

double GroundMap::pavingDistance(const Vec2d& p) const
{
    const Vec2d t = (p - origin) / texelM;
    if (t.x < 0.0 || t.y < 0.0 || t.x > width || t.y > height)
    {
        return 0.5 * kGroundMapDistanceRange;
    }
    return (sample(p).x - 0.5) * kGroundMapDistanceRange;
}

bool Settlements::keepsTreesOff(double z, double theta) const
{
    return std::ranges::any_of(places, [&](const Settlement& place) {
        const Vec2d p = place.plane.toPlan(z, theta);
        if (p.x < place.boundsMin.x || p.y < place.boundsMin.y || p.x > place.boundsMax.x ||
            p.y > place.boundsMax.y)
        {
            return false;
        }
        if (place.kind == SettlementKind::Farm)
        {
            return glm::length(p) < place.radiusM;
        }
        return place.ground.sample(p).z > 0.1 || place.ground.pavingDistance(p) < 3.0;
    });
}

const Settlement* Settlements::townAt(double z, double theta) const
{
    for (const Settlement& place : places)
    {
        if (place.kind != SettlementKind::Town)
        {
            continue;
        }
        const Vec2d p = place.plane.toPlan(z, theta);
        if (p.x >= place.boundsMin.x && p.y >= place.boundsMin.y && p.x <= place.boundsMax.x &&
            p.y <= place.boundsMax.y)
        {
            return &place;
        }
    }
    return nullptr;
}

std::size_t Settlements::townCount() const
{
    return static_cast<std::size_t>(
        std::ranges::count(places, SettlementKind::Town, &Settlement::kind));
}

Settlements planSettlements(const HabitatGeometry& geometry, const TerrainGrid& grid)
{
    const Site site{.geometry = &geometry, .grid = &grid, .landscape = &geometry.landscape()};
    const SettlementSpec& spec = geometry.spec().settlements;
    SplitMix64            random(hashSeed(geometry.spec().terrain.seed, 0x70E5));
    Settlements           out;

    // Names, shuffled.
    std::vector<const char*> names(kTownNames.begin(), kTownNames.end());
    for (std::size_t i = names.size() - 1; i > 0; --i)
    {
        const std::uint64_t pick = random.next() % (i + 1);
        std::swap(names[i], names[pick]);
    }

    for (int valley = 0; valley < geometry.stripCount(); ++valley)
    {
        for (int i = 0; i < spec.townsPerValley; ++i)
        {
            SplitMix64 townRandom(hashSeed(random.next(), static_cast<std::uint64_t>(i)));
            const auto frame = placeTown(site, valley, i, spec.townsPerValley, townRandom);
            if (!frame)
            {
                continue;
            }
            const std::size_t index = out.places.size();
            TownBuilder       builder(site, *frame, index, townRandom);
            TownPlan          plan = builder.build();
            if (plan.buildings.size() < 3)
            {
                continue;
            }
            Settlement town;
            town.kind      = SettlementKind::Town;
            town.name      = names[index % names.size()];
            town.valley    = valley;
            town.plane     = frame->plane;
            town.radiusM   = frame->halfLength;
            town.streets   = plan.streets;
            town.ground    = GroundPainter(site, *frame, plan).paint();
            town.boundsMin = town.ground.origin;
            town.boundsMax = town.ground.origin +
                             (Vec2d(town.ground.width, town.ground.height) * town.ground.texelM);
            town.buildingCount = plan.buildings.size();
            out.places.push_back(std::move(town));
            out.buildings.insert(out.buildings.end(), plan.buildings.begin(), plan.buildings.end());
            out.furniture.insert(out.furniture.end(), plan.furniture.begin(), plan.furniture.end());
            out.bridges.insert(out.bridges.end(), plan.bridges.begin(), plan.bridges.end());
            out.props.insert(out.props.end(), plan.props.begin(), plan.props.end());
            out.trees.insert(out.trees.end(), plan.trees.begin(), plan.trees.end());
        }
    }
    for (int valley = 0; valley < geometry.stripCount(); ++valley)
    {
        for (int i = 0; i < spec.farmsPerValley; ++i)
        {
            SplitMix64        farmRandom(hashSeed(random.next(), 0xFA53));
            const std::size_t first = out.buildings.size();
            auto farm = planFarm(site, valley, out.places.size(), out.places, farmRandom,
                                 out.buildings, out.props, out.trees);
            if (farm)
            {
                fitBounds(*farm, std::span(out.buildings).subspan(first));
                out.places.push_back(std::move(*farm));
            }
        }
    }
    return out;
}

void stampSettlements(TerrainGrid& grid, const Settlements& settlements)
{
    const TerrainGridLayout& layout = grid.layout;
    for (const Settlement& place : settlements.places)
    {
        // The cover map's texels (two grid cells each) over the settlement's plan.
        const Vec2d a =
            grid.cellAt(place.plane.z(place.boundsMin.y), place.plane.theta(place.boundsMin.x));
        const Vec2d b =
            grid.cellAt(place.plane.z(place.boundsMax.y), place.plane.theta(place.boundsMax.x));
        const auto   rowFrom = static_cast<std::int64_t>(std::floor(std::min(a.y, b.y) / 2.0)) - 1;
        const auto   rowTo   = static_cast<std::int64_t>(std::ceil(std::max(a.y, b.y) / 2.0)) + 1;
        const double columnSpan = std::remainder(b.x - a.x, static_cast<double>(layout.columns));
        const auto   columnFrom =
            static_cast<std::int64_t>(std::floor(std::min(a.x, a.x + columnSpan) / 2.0)) - 1;
        const auto columnTo =
            static_cast<std::int64_t>(std::ceil(std::max(a.x, a.x + columnSpan) / 2.0)) + 1;
        for (std::int64_t row = std::max<std::int64_t>(rowFrom, 0);
             row <= std::min<std::int64_t>(rowTo, grid.coverRows - 1); ++row)
        {
            const double z =
                static_cast<double>(grid.profile[static_cast<std::size_t>(
                                                     std::min<std::int64_t>(2 * row, layout.cells))]
                                        .x);
            for (std::int64_t c = columnFrom; c <= columnTo; ++c)
            {
                const auto column = static_cast<std::uint32_t>(
                    ((c % grid.coverColumns) + grid.coverColumns) % grid.coverColumns);
                const double      theta = layout.theta(2.0 * column);
                const std::size_t at =
                    ((static_cast<std::size_t>(row) * grid.coverColumns) + column) * 4;
                if (settlements.keepsTreesOff(z, theta))
                {
                    grid.cover[at] = 0;
                }
                if (place.kind == SettlementKind::Town)
                {
                    grid.cover[at + 2] = 255;
                }
            }
        }
    }
}

void addStandingTrees(TreeLayer& layer, const Settlements& settlements)
{
    constexpr double kCrownMargin = 25.0;
    for (std::size_t place = 0; place < settlements.places.size(); ++place)
    {
        std::array<std::vector<TreeInstance>, kTreeSpeciesCount> bySpecies;
        const Vec3d origin = settlements.places[place].plane.point(Vec2d(0.0), 0.0);
        Vec3d       low(std::numeric_limits<double>::max());
        Vec3d       high(std::numeric_limits<double>::lowest());
        SplitMix64  random(hashSeed(0x7EE, place));
        for (const StandingTree& tree : settlements.trees)
        {
            if (tree.settlement != place)
            {
                continue;
            }
            const Vec3d local = tree.position - origin;
            low               = glm::min(low, local);
            high              = glm::max(high, local);
            bySpecies.at(static_cast<std::size_t>(tree.species))
                .push_back({.position = Vec3f(local),
                            .packed   = packTree(tree.heightM, random.uniform(), tree.species,
                                                 random.uniform())});
        }
        if (low.x > high.x)
        {
            continue;
        }
        TreeTile tile;
        tile.origin    = origin;
        tile.boundsMin = Vec3f(low - Vec3d(kCrownMargin));
        tile.boundsMax = Vec3f(high + Vec3d(kCrownMargin));
        tile.first     = static_cast<std::uint32_t>(layer.instances.size());
        for (std::size_t s = 0; s < kTreeSpeciesCount; ++s)
        {
            tile.counts.at(s) = static_cast<std::uint32_t>(bySpecies.at(s).size());
            layer.instances.insert(layer.instances.end(), bySpecies.at(s).begin(),
                                   bySpecies.at(s).end());
        }
        layer.tiles.push_back(tile);
    }
}

}  // namespace StarshipSimulator
