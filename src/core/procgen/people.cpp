#include "StarshipSimulator/core/procgen/people.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/SplitMix64.h"
#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/land_layout.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/procgen/mesh.h"
#include "StarshipSimulator/core/procgen/settlements.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"

namespace StarshipSimulator
{

namespace
{

constexpr double kPaceM     = 0.78;  // how far one stride carries you
constexpr double kEdgeM     = 0.9;   // how far from a street's middle people walk
constexpr double kBenchSitM = 0.45;  // along a bench, from its middle

/// A person's build: a little shorter or taller, with their own colours.
void dress(Person& person, SplitMix64& rng)
{
    person.heightM = rng.uniform(1.58, 1.92);
    person.clothes = static_cast<std::uint8_t>(rng.uniform(0.0, kPersonClothes - 1e-9));
    person.skin    = static_cast<std::uint8_t>(rng.uniform(0.0, kPersonSkins - 1e-9));
}

/// Stands a person on the ground at a plan position, facing a plan direction.
Person standing(const Settlement& place, const TerrainGrid& grid, const Vec2d& at,
                const Vec2d& facing)
{
    Person            person;
    const SurfaceSpot spot   = place.plane.surface(at);
    const double      theta  = spot.theta;
    const double      height = grid.groundHeight(spot.z, theta);
    person.position          = place.plane.point(at, height);
    const Vec2d aim = glm::length(facing) > 1e-9 ? glm::normalize(facing) : Vec2d(0.0, 1.0);
    if (place.plane.axis == BandAxis::AROUND)
    {
        // The plan's y runs around the axis, its x across it.
        const Vec3d along  = place.plane.alongDirection(at);
        const Vec3d across = glm::cross(HabitatGeometry::localUp(person.position), along);
        person.forward     = glm::normalize((across * aim.x) + (along * aim.y));
        return person;
    }
    // The plan's x runs around the habitat (spinward), its y along the axis.
    const Vec3d around(-std::sin(theta), std::cos(theta), 0.0);
    const Vec3d along(0.0, 0.0, 1.0);
    person.forward = glm::normalize((around * aim.x) + (along * aim.y));
    return person;
}

/// Someone walking up and down one street, at their own pace and on their own side.
Person walker(const Settlement& place, const TerrainGrid& grid, const Street& street,
              std::uint64_t seed, double seconds)
{
    SplitMix64   rng(seed);
    const Vec2d  line   = street.to - street.from;
    const double length = glm::length(line);
    const Vec2d  along  = length > 1e-6 ? line / length : Vec2d(0.0, 1.0);
    const Vec2d  across(-along.y, along.x);

    const double speed = rng.uniform(1.05, 1.65);
    const double side  = rng.uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0;
    const double edge  = side * std::min(kEdgeM, std::max(0.4, street.halfWidth - 0.7));
    // Back and forth: a triangle wave over the street, each walker starting somewhere else on it.
    const double period = (2.0 * length) / speed;
    const double phase  = std::fmod((seconds / period) + rng.uniform(0.0, 1.0), 1.0);
    const double there  = phase < 0.5 ? phase * 2.0 : (1.0 - phase) * 2.0;
    const double facing = phase < 0.5 ? 1.0 : -1.0;

    // Walking on the left going one way and the right coming back keeps them apart.
    const Vec2d at     = street.from + (along * (there * length)) + (across * (edge * facing));
    Person      person = standing(place, grid, at, along * facing);
    dress(person, rng);
    person.doing   = Activity::WALKING;
    person.speedMS = speed;
    person.gait    = std::fmod(((there * length) / kPaceM) + rng.uniform(0.0, 1.0), 1.0);
    return person;
}

/// Whether a person is near enough to draw, and inside the crowd's share that is out and about.
bool outAndAbout(std::uint64_t seed, double busy)
{
    return SplitMix64(seed).uniform() < busy;
}

/// Everyone walking the streets of one settlement.
void walkers(const Settlement& place, const TerrainGrid& grid, std::uint64_t placeSeed,
             double seconds, const CrowdSettings& settings,
             const std::function<void(const Person&)>& add)
{
    for (std::size_t s = 0; s < place.streets.size(); ++s)
    {
        const Street& street = place.streets[s];
        const double  length = glm::distance(street.from, street.to);
        const auto    count =
            static_cast<int>(std::max(1.0, std::round(length / settings.perStreetM)));
        const std::uint64_t streetSeed = hashSeed(placeSeed, s);
        for (int i = 0; i < count; ++i)
        {
            const std::uint64_t who = hashSeed(streetSeed, static_cast<std::uint64_t>(i));
            if (outAndAbout(who, settings.busy))
            {
                add(walker(place, grid, street, hashSeed(who, 1), seconds));
            }
        }
    }
}

/// Whoever is sitting on a bench, or standing about a fountain or a stall.
void aboutTheFurniture(const Settlements& settlements, const TerrainGrid& grid, std::size_t index,
                       std::uint64_t seed, const CrowdSettings& settings,
                       const std::function<void(const Person&)>& add)
{
    const Furniture&  item  = settlements.furniture[index];
    const Settlement& place = settlements.places[item.settlement];
    const bool        bench = item.kind == FurnitureKind::BENCH;
    if (!bench && item.kind != FurnitureKind::FOUNTAIN && item.kind != FurnitureKind::STALL)
    {
        return;
    }
    SplitMix64 rng(hashSeed(hashSeed(seed, index + 0x5EA7U), 3));
    if (rng.uniform() > settings.busy * (bench ? 0.55 : 0.8))
    {
        return;
    }
    const Vec2d facing(std::sin(item.angle), std::cos(item.angle));
    const Vec2d across(facing.y, -facing.x);
    if (bench)
    {
        // Along the bench, a little off its middle, facing out from its back.
        const Vec2d at     = item.position + (across * rng.uniform(-kBenchSitM, kBenchSitM));
        Person      person = standing(place, grid, at, facing);
        dress(person, rng);
        person.doing = Activity::SITTING;
        add(person);
        return;
    }
    // Standing about: turned toward whatever they are looking at, shifting their weight.
    const bool fountain = item.kind == FurnitureKind::FOUNTAIN;
    const int  knot     = static_cast<int>(rng.uniform(1.0, fountain ? 3.99 : 2.99));
    for (int i = 0; i < knot; ++i)
    {
        const double angle = rng.uniform(0.0, 2.0 * kPi);
        // Clear of the fountain's basin, or up at the counter of a stall.
        const double reach  = fountain ? rng.uniform(2.9, 4.2) : rng.uniform(1.3, 2.1);
        const Vec2d  at     = item.position + (Vec2d(std::cos(angle), std::sin(angle)) * reach);
        Person       person = standing(place, grid, at, item.position - at);
        dress(person, rng);
        person.doing = Activity::STANDING;
        person.gait  = rng.uniform(0.0, 1.0);
        add(person);
    }
}

}  // namespace

std::vector<Person> peopleNear(const Settlements& settlements, const TerrainGrid& grid,
                               const Vec3d& camera, double seconds, std::uint64_t seed,
                               const CrowdSettings& settings)
{
    std::vector<Person> people;
    const double        rangeSq = settings.rangeM * settings.rangeM;
    const auto          add     = [&](const Person& person) {
        const Vec3d offset = person.position - camera;
        if (std::cmp_less(people.size(), settings.maxPeople) && glm::dot(offset, offset) <= rangeSq)
        {
            people.push_back(person);
        }
    };

    for (std::size_t index = 0; index < settlements.places.size(); ++index)
    {
        const Settlement& place = settlements.places[index];
        // Skip whole settlements that are out of range (their centre plus their reach).
        const Vec3d centre = place.plane.point(Vec2d(0.0), 0.0);
        if (glm::distance(centre, camera) > settings.rangeM + place.radiusM)
        {
            continue;
        }
        walkers(place, grid, hashSeed(seed, index + 0x9EU), seconds, settings, add);
    }
    for (std::size_t f = 0; f < settlements.furniture.size(); ++f)
    {
        aboutTheFurniture(settlements, grid, f, seed, settings, add);
    }
    return people;
}

CpuMesh buildPersonMesh()
{
    using namespace person_part;  // NOLINT(google-build-using-namespace): the part names, just here
    CpuMesh    person;
    const auto put = [&person](const CpuMesh& part, const Vec3d& at, const Vec3d& scale) {
        appendMesh(person, part, glm::translate(Mat4d(1.0), at) * glm::scale(Mat4d(1.0), scale));
    };

    // A body of rounded boxes: nothing is detailed, but the proportions have to be right or it
    // reads as a doll rather than a person.
    const CpuMesh head  = makeSphere(1.0F, 10, kHead);
    const CpuMesh hair  = makeSphere(1.0F, 10, kHair);
    const CpuMesh torso = makeBox(Vec3f(1.0F), kTorso);
    put(head, Vec3d(0.0, 1.62, 0.01), Vec3d(0.093, 0.115, 0.098));
    put(hair, Vec3d(0.0, 1.655, -0.006), Vec3d(0.099, 0.10, 0.1));
    put(torso, Vec3d(0.0, 1.35, 0.0), Vec3d(0.168, 0.09, 0.098));  // shoulders
    put(torso, Vec3d(0.0, 1.17, 0.0), Vec3d(0.150, 0.20, 0.100));  // chest
    put(torso, Vec3d(0.0, 0.97, 0.0), Vec3d(0.140, 0.13, 0.092));  // waist and hips
    put(torso, Vec3d(0.0, 1.47, 0.0), Vec3d(0.058, 0.06, 0.058));  // neck

    for (int side = 0; side < 2; ++side)
    {
        const double x    = side == 0 ? -1.0 : 1.0;
        const auto   arm  = makeBox(Vec3f(1.0F), side == 0 ? kArmL : kArmR);
        const auto   high = makeBox(Vec3f(1.0F), side == 0 ? kThighL : kThighR);
        const auto   low  = makeBox(Vec3f(1.0F), side == 0 ? kShinL : kShinR);
        // Arms hang from the shoulder; the mesh reaches from the shoulder down to the hand.
        put(arm, Vec3d(x * 0.196, 1.12, 0.0), Vec3d(0.044, 0.30, 0.050));
        put(high, Vec3d(x * 0.083, 0.70, 0.0), Vec3d(0.073, 0.22, 0.082));
        put(low, Vec3d(x * 0.083, 0.25, 0.0), Vec3d(0.060, 0.23, 0.070));
        put(low, Vec3d(x * 0.083, 0.035, 0.03), Vec3d(0.062, 0.035, 0.105));  // the foot
    }
    return person;
}

}  // namespace StarshipSimulator
