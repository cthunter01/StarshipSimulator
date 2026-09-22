#include "StarshipSimulator/physics/PhysicsWorld.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/habitat/Landscape.h"
#include "StarshipSimulator/core/log.h"
#include "StarshipSimulator/core/math.h"
#include "StarshipSimulator/core/physics/CharacterMover.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/people.h"
#include "StarshipSimulator/core/procgen/props.h"
#include "StarshipSimulator/core/procgen/terrain_grid.h"
#include "StarshipSimulator/core/procgen/transit.h"
#include "StarshipSimulator/core/procgen/trees.h"
#include "jolt.h"

namespace StarshipSimulator
{

namespace
{

constexpr std::uint32_t kTileCells         = 32;    // terrain collision tiles: 32 x 32 grid cells
constexpr double        kTileKeepSeconds   = 8.0;   // unused tiles are dropped after this long...
constexpr std::size_t   kTileCacheSize     = 48;    // ... once there are more than this many
constexpr double        kGroundReachM      = 60.0;  // terrain matters only this close to the ground
constexpr double        kPropTerrainM      = 15.0;  // terrain kept solid around each moving prop
constexpr double        kCharacterTerrainM = 40.0;
constexpr double        kFallThroughM      = 2.0;  // a prop this far under the ground is put back
constexpr JPH::uint     kMaxBodies         = 65536;
constexpr JPH::uint     kMaxBodyPairs      = 65536;
constexpr JPH::uint     kMaxContacts       = 16384;
constexpr std::size_t   kTempMemory        = std::size_t{16} * 1024 * 1024;
constexpr float         kTerrainFriction   = 0.8F;
constexpr float         kBuiltFriction     = 0.6F;
constexpr float         kWaterDrag         = 0.6F;  // linear drag in water
constexpr double        kTreeReachM        = 50.0;  // trunks are solid this near the player
constexpr double        kTreeCheckSeconds  = 0.25;  // how often to look for tree tiles to load
constexpr float         kWaterAngularDrag  = 0.05F;

namespace layers
{
constexpr JPH::ObjectLayer kStatic = 0;
constexpr JPH::ObjectLayer kMoving = 1;
}  // namespace layers

// ---- Conversions --------------------------------------------------------------------------------

JPH::RVec3 toJolt(const Vec3d& v)
{
    return {v.x, v.y, v.z};
}

JPH::Vec3 toJoltF(const Vec3d& v)
{
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

JPH::Vec3 toJoltF(const Vec3f& v)
{
    return {v.x, v.y, v.z};
}

JPH::Quat toJolt(const Quatd& q)
{
    return JPH::Quat(static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z),
                     static_cast<float>(q.w))
        .Normalized();
}

Vec3d fromJolt(JPH::RVec3Arg v)
{
    return {v.GetX(), v.GetY(), v.GetZ()};
}

Vec3d fromJoltF(JPH::Vec3Arg v)
{
    return {static_cast<double>(v.GetX()), static_cast<double>(v.GetY()),
            static_cast<double>(v.GetZ())};
}

Quatd fromJolt(JPH::QuatArg q)
{
    return {static_cast<double>(q.GetW()), static_cast<double>(q.GetX()),
            static_cast<double>(q.GetY()), static_cast<double>(q.GetZ())};
}

// ---- Jolt's process-wide setup -----------------------------------------------------------------

// Jolt reports problems through a printf-style callback; its format string alone says enough.
// NOLINTNEXTLINE(cert-dcl50-cpp,cppcoreguidelines-pro-type-vararg,modernize-avoid-variadic-functions)
void trace(const char* format, ...)
{
    log::warn("Jolt: {}", format);
}

#ifdef JPH_ENABLE_ASSERTS
bool assertFailed(const char* expression, const char* message, const char* file, JPH::uint line)
{
    log::error("Jolt assertion failed: {} ({}) at {}:{}", expression,
               message != nullptr ? message : "", file, line);
    return true;  // stop in the debugger
}
#endif

struct JoltUsers
{
    std::mutex mutex;
    int        count = 0;
};

JoltUsers& joltUsers()
{
    static JoltUsers s_users;
    return s_users;
}

/// Jolt's allocator, factory and type registry, set up while any world exists.
class JoltRuntime
{
public:
    JoltRuntime()
    {
        JoltUsers&             users = joltUsers();
        const std::scoped_lock lock(users.mutex);
        if (users.count++ == 0)
        {
            JPH::RegisterDefaultAllocator();
            JPH::Trace = &trace;
            JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = &assertFailed;)
            // Jolt's own global, deleted when the last world goes.
            // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
            JPH::Factory::sInstance = new JPH::Factory();
            JPH::RegisterTypes();
        }
    }
    ~JoltRuntime()
    {
        JoltUsers&             users = joltUsers();
        const std::scoped_lock lock(users.mutex);
        if (--users.count == 0)
        {
            JPH::UnregisterTypes();
            delete JPH::Factory::sInstance;  // NOLINT(cppcoreguidelines-owning-memory)
            JPH::Factory::sInstance = nullptr;
        }
    }
    JoltRuntime(const JoltRuntime&)            = delete;
    JoltRuntime& operator=(const JoltRuntime&) = delete;
    JoltRuntime(JoltRuntime&&)                 = delete;
    JoltRuntime& operator=(JoltRuntime&&)      = delete;
};

// ---- Layers: static things only meet moving ones -----------------------------------------------

class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
{
public:
    [[nodiscard]] JPH::uint            GetNumBroadPhaseLayers() const override { return 2; }
    [[nodiscard]] JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
    {
        return JPH::BroadPhaseLayer(static_cast<JPH::BroadPhaseLayer::Type>(layer));
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    [[nodiscard]] const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
    {
        return layer.GetValue() == layers::kStatic ? "static" : "moving";
    }
#endif
};

class ObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    [[nodiscard]] bool ShouldCollide(JPH::ObjectLayer     layer,
                                     JPH::BroadPhaseLayer broadPhase) const override
    {
        return layer == layers::kMoving || broadPhase.GetValue() == layers::kMoving;
    }
};

class ObjectPairs final : public JPH::ObjectLayerPairFilter
{
public:
    [[nodiscard]] bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override
    {
        return a == layers::kMoving || b == layers::kMoving;
    }
};

// ---- Shapes -------------------------------------------------------------------------------------

JPH::RefConst<JPH::Shape> createShape(const JPH::ShapeSettings& settings)
{
    const JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
    {
        throw std::runtime_error(std::format("Jolt cannot make a shape: {}", result.GetError()));
    }
    return result.Get();
}

/// Convex radius for a box or cylinder: rounded edges, but never more than it can hold.
float convexRadius(float smallestHalfExtent)
{
    return std::min(JPH::cDefaultConvexRadius, 0.5F * smallestHalfExtent);
}

JPH::RefConst<JPH::Shape> createPart(const PropPart& part)
{
    const Vec3f half = part.halfExtents;
    switch (part.shape)
    {
        case PropPart::Shape::SPHERE:
            return createShape(JPH::SphereShapeSettings(half.x));
        case PropPart::Shape::CYLINDER:
            return createShape(
                JPH::CylinderShapeSettings(half.y, half.x, convexRadius(std::min(half.x, half.y))));
        case PropPart::Shape::BOX:
            break;
    }
    return createShape(
        JPH::BoxShapeSettings(toJoltF(half), convexRadius(std::min({half.x, half.y, half.z}))));
}

JPH::RefConst<JPH::Shape> createPropShape(PropKind kind)
{
    const PropInfo& info = propInfo(kind);
    if (info.parts.size() == 1)
    {
        const PropPart& part = info.parts.front();
        return createShape(JPH::RotatedTranslatedShapeSettings(
            toJoltF(part.centre), JPH::Quat::sIdentity(), createPart(part)));
    }
    JPH::StaticCompoundShapeSettings compound;
    for (const PropPart& part : info.parts)
    {
        compound.AddShape(toJoltF(part.centre), JPH::Quat::sIdentity(), createPart(part));
    }
    return createShape(compound);
}

// ---- The spinning frame ------------------------------------------------------------------------

struct PropBody
{
    JPH::BodyID id;
    float       buoyancy = 1.0F;
};

/// Before each step, gives every moving body the fictitious forces of the rotating frame, and the
/// lift and drag of the water it is in. The Coriolis force turns velocities about the axis at -2w
/// (exactly, so it adds no energy); the centrifugal acceleration w^2 r is a kick. Bodies just
/// launched get half a kick first, which makes the steps leapfrog integration: thrown things follow
/// their exact paths to second order.
class SpinFrame final : public JPH::PhysicsStepListener
{
public:
    SpinFrame(const HabitatGeometry& geometry, const std::vector<PropBody>& props)
      : geometry_(&geometry), props_(&props)
    {
    }

    void launched(JPH::BodyID id) { launched_.push_back(id); }

    void OnStep(const JPH::PhysicsStepListenerContext& context) override
    {
        const JPH::PhysicsSystem&           system = *context.mPhysicsSystem;
        const JPH::BodyLockInterfaceNoLock& locks  = system.GetBodyLockInterfaceNoLock();
        const JPH::uint    count = system.GetNumActiveBodies(JPH::EBodyType::RigidBody);
        const JPH::BodyID* ids   = system.GetActiveBodiesUnsafe(JPH::EBodyType::RigidBody);
        const auto         dt    = static_cast<double>(context.mDeltaTime);
        for (const JPH::BodyID id : std::span(ids, count))
        {
            const JPH::BodyLockWrite lock(locks, id);
            if (!lock.Succeeded() || !lock.GetBody().IsDynamic())
            {
                continue;
            }
            const bool first = std::ranges::find(launched_, id) != launched_.end();
            accelerate(lock.GetBody(), first ? 0.5 * dt : dt);
            applyWater(lock.GetBody(), context.mDeltaTime);
        }
        launched_.clear();
    }

private:
    void accelerate(JPH::Body& body, double dt) const
    {
        const double w    = geometry_->omega();
        const Vec3d  x    = fromJolt(body.GetCenterOfMassPosition());
        const Vec3d  v    = fromJoltF(body.GetLinearVelocity());
        const double turn = -2.0 * w * dt;
        const double c    = std::cos(turn);
        const double s    = std::sin(turn);
        const Vec3d  turned((c * v.x) - (s * v.y), (s * v.x) + (c * v.y), v.z);
        const Vec3d  centrifugal(w * w * x.x, w * w * x.y, 0.0);
        body.SetLinearVelocityClamped(toJoltF(turned + (centrifugal * dt)));
    }

    void applyWater(JPH::Body& body, float dt) const
    {
        const std::uint64_t index = body.GetUserData();
        if (index == 0 || index > props_->size())
        {
            return;
        }
        const Vec3d  x     = fromJolt(body.GetCenterOfMassPosition());
        const double theta = HabitatGeometry::angleOf(x);
        if (geometry_->waterDepth(x.z, theta) <= 0.0)
        {
            return;
        }
        const double floor   = geometry_->profile().radiusAt(x.z).value_or(geometry_->radius());
        const Vec3d  up      = HabitatGeometry::localUp(x);
        const Vec3d  surface = (-up * (floor - kWaterLevelM)) + Vec3d(0.0, 0.0, x.z);
        const double g       = geometry_->gravityAt(std::hypot(x.x, x.y));
        body.ApplyBuoyancyImpulse(toJolt(surface), toJoltF(up), (*props_)[index - 1].buoyancy,
                                  kWaterDrag, kWaterAngularDrag, JPH::Vec3::sZero(),
                                  toJoltF(-up * g), dt);
    }

    const HabitatGeometry*       geometry_;
    const std::vector<PropBody>* props_;
    std::vector<JPH::BodyID>     launched_;
};

// ---- Terrain tiles -----------------------------------------------------------------------------

/// The terrain grid as collision meshes, built in tiles where they are needed and dropped when
/// they have not been for a while.
class TerrainTiles
{
public:
    TerrainTiles(const HabitatGeometry& geometry, const TerrainGrid& grid,
                 JPH::BodyInterface& bodies)
      : geometry_(&geometry),
        grid_(&grid),
        bodies_(&bodies),
        tilesAround_(grid.layout.columns / kTileCells),
        tilesAlong_((grid.layout.cells + kTileCells - 1) / kTileCells)
    {
    }

    /// Makes the terrain within `radius` of a point solid, if the point is near the ground.
    void require(const Vec3d& point, double radius, double now)
    {
        const TerrainGridLayout& layout  = grid_->layout;
        const MeridianProfile&   profile = geometry_->profile();
        const double             z       = std::clamp(point.z, profile.zMin(), profile.zMax());
        const double             theta   = HabitatGeometry::angleOf(point);
        const double             r       = std::hypot(point.x, point.y);
        const double ground = geometry_->groundRadius(z, theta).value_or(geometry_->radius());
        if (ground - r > kGroundReachM + radius)
        {
            return;  // high above the ground
        }
        const double column      = theta / (2.0 * kPi) * layout.columns;
        const double row         = profile.arcAt(z) / layout.cellU;
        const double columnReach = std::min(
            radius / (layout.cellArcM * std::max(r, 50.0) / layout.radiusM), 0.5 * layout.columns);
        const double rowReach = radius / layout.cellU;
        const auto   firstRow =
            static_cast<std::int64_t>(std::floor(std::max(0.0, row - rowReach) / kTileCells));
        const auto lastRow =
            std::min(static_cast<std::int64_t>(std::floor((row + rowReach) / kTileCells)),
                     static_cast<std::int64_t>(tilesAlong_) - 1);
        const auto firstColumn =
            static_cast<std::int64_t>(std::floor((column - columnReach) / kTileCells));
        const auto lastColumn =
            static_cast<std::int64_t>(std::floor((column + columnReach) / kTileCells));
        const auto around = static_cast<std::int64_t>(tilesAround_);
        for (std::int64_t tileRow = firstRow; tileRow <= lastRow; ++tileRow)
        {
            for (std::int64_t c = firstColumn; c <= lastColumn; ++c)
            {
                const auto tileColumn =
                    static_cast<std::uint32_t>(((c % around) + around) % around);
                const std::uint64_t key = (static_cast<std::uint64_t>(tileRow) << 32U) | tileColumn;
                auto                found = tiles_.find(key);
                if (found == tiles_.end())
                {
                    const JPH::BodyID id = build(tileColumn, static_cast<std::uint32_t>(tileRow));
                    found = tiles_.emplace(key, Tile{.id = id, .lastUsed = now}).first;
                }
                found->second.lastUsed = now;
            }
        }
    }

    /// Drops tiles that have not been needed for a while, once there are many.
    void trim(double now)
    {
        if (tiles_.size() <= kTileCacheSize)
        {
            return;
        }
        std::erase_if(tiles_, [&](const auto& entry) {
            if (now - entry.second.lastUsed < kTileKeepSeconds)
            {
                return false;
            }
            bodies_->RemoveBody(entry.second.id);
            bodies_->DestroyBody(entry.second.id);
            return true;
        });
    }

    [[nodiscard]] std::size_t count() const { return tiles_.size(); }

private:
    struct Tile
    {
        JPH::BodyID id;
        double      lastUsed = 0.0;
    };

    [[nodiscard]] JPH::BodyID build(std::uint32_t tileColumn, std::uint32_t tileRow) const
    {
        const TerrainGridLayout& layout = grid_->layout;
        const std::uint32_t      c0     = tileColumn * kTileCells;
        const std::uint32_t      r0     = tileRow * kTileCells;
        const std::uint32_t      r1     = std::min(r0 + kTileCells, layout.cells);
        const std::uint32_t      across = kTileCells + 1;

        // The tile's origin: its middle, on the profile surface.
        const Vec4d  middle(grid_->profile[(r0 + r1) / 2]);
        const double middleTheta = layout.theta(c0 + (0.5 * kTileCells));
        const Vec3d  origin(middle.y * std::cos(middleTheta), middle.y * std::sin(middleTheta),
                            middle.x);

        JPH::VertexList vertices;
        vertices.reserve(static_cast<std::size_t>(across) * (r1 - r0 + 1));
        for (std::uint32_t row = r0; row <= r1; ++row)
        {
            const Vec4d at(grid_->profile[row]);
            for (std::uint32_t column = c0; column <= c0 + kTileCells; ++column)
            {
                const double theta  = layout.theta(column);
                const double radius = at.y - grid_->height(column % layout.columns, row);
                const Vec3d  p =
                    Vec3d(radius * std::cos(theta), radius * std::sin(theta), at.x) - origin;
                vertices.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y),
                                      static_cast<float>(p.z));
            }
        }
        // The same triangles as the terrain the GPU draws (landscape patches), facing the axis.
        JPH::IndexedTriangleList triangles;
        triangles.reserve(static_cast<std::size_t>(2) * kTileCells * (r1 - r0));
        for (std::uint32_t row = 0; row < r1 - r0; ++row)
        {
            for (std::uint32_t column = 0; column < kTileCells; ++column)
            {
                const std::uint32_t a = (row * across) + column;
                const std::uint32_t b = a + 1;
                const std::uint32_t c = a + across;
                const std::uint32_t d = c + 1;
                triangles.emplace_back(a, c, b);
                triangles.emplace_back(b, c, d);
            }
        }
        const JPH::MeshShapeSettings mesh(std::move(vertices), std::move(triangles));
        mesh.SetEmbedded();
        JPH::BodyCreationSettings body(createShape(mesh), toJolt(origin), JPH::Quat::sIdentity(),
                                       JPH::EMotionType::Static, layers::kStatic);
        body.mFriction = kTerrainFriction;
        return bodies_->CreateAndAddBody(body, JPH::EActivation::DontActivate);
    }

    const HabitatGeometry*                  geometry_;
    const TerrainGrid*                      grid_;
    JPH::BodyInterface*                     bodies_;
    std::uint32_t                           tilesAround_;
    std::uint32_t                           tilesAlong_;
    std::unordered_map<std::uint64_t, Tile> tiles_;
};

// ---- Tree trunks --------------------------------------------------------------------------------

/// A tree tile's trunks as one static body: an upright cylinder per tree, about as thick as the
/// drawn trunk and as tall as the bare part below the crown.
JPH::BodyID buildTreeTile(JPH::BodyInterface& bodies, const TreeLayer& layer, const TreeTile& tile)
{
    std::size_t count = 0;
    for (const std::uint32_t n : tile.counts)
    {
        count += n;
    }
    if (count == 0)
    {
        return {};
    }
    JPH::StaticCompoundShapeSettings trunks;
    JPH::RefConst<JPH::Shape>        single;
    JPH::Vec3                        singlePosition = JPH::Vec3::sZero();
    JPH::Quat                        singleRotation = JPH::Quat::sIdentity();
    for (std::size_t i = tile.first; i < tile.first + count; ++i)
    {
        const TreeInstance& tree    = layer.instances[i];
        const double        height  = static_cast<double>(tree.packed & 0xFFFU) * 0.1;
        const auto          species = static_cast<TreeSpecies>((tree.packed >> 20U) & 0xFU);
        const double        width  = 0.85 + (0.3 * static_cast<double>(tree.packed >> 24U) / 255.0);
        const double        base   = species == TreeSpecies::BROADLEAF ? 0.035 : 0.03;
        const auto          radius = static_cast<float>(std::max(0.1, 0.8 * base * width * height));
        const auto          half   = static_cast<float>(0.25 * height);
        const Vec3d         up     = HabitatGeometry::localUp(tile.origin + Vec3d(tree.position));
        const JPH::Quat     rotation = JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), toJoltF(up));
        const JPH::Vec3 centre = toJoltF(Vec3d(tree.position) + (up * static_cast<double>(half)));
        const JPH::RefConst<JPH::Shape> cylinder = createShape(
            JPH::CylinderShapeSettings(half, radius, convexRadius(std::min(half, radius))));
        if (count == 1)
        {
            single         = cylinder;
            singlePosition = centre;
            singleRotation = rotation;
        }
        else
        {
            trunks.AddShape(centre, rotation, cylinder);
        }
    }
    const JPH::RefConst<JPH::Shape> shape = count == 1
                                                ? createShape(JPH::RotatedTranslatedShapeSettings(
                                                      singlePosition, singleRotation, single))
                                                : createShape(trunks);
    JPH::BodyCreationSettings       body(shape, toJolt(tile.origin), JPH::Quat::sIdentity(),
                                         JPH::EMotionType::Static, layers::kStatic);
    body.mFriction = kBuiltFriction;
    return bodies.CreateAndAddBody(body, JPH::EActivation::DontActivate);
}

// ---- The character -----------------------------------------------------------------------------

/// The player's body: Jolt's virtual character (a capsule moved by sweeps, not simulated), stood
/// upright along the local "up" every step, with a kinematic inner body so props bump into it.
class JoltCharacter final : public CharacterMover
{
public:
    JoltCharacter(JPH::PhysicsSystem& system, JPH::TempAllocator& temp, TerrainTiles& tiles,
                  const double& clock, const PhysicsSettings& settings)
      : system_(&system),
        temp_(&temp),
        tiles_(&tiles),
        clock_(&clock),
        stepUp_(settings.stepUpM),
        stepDown_(settings.stepDownM)
    {
        const auto radius     = static_cast<float>(settings.characterRadius);
        const auto halfHeight = static_cast<float>(0.5 * settings.characterHeight);
        // A capsule with its origin at the feet.
        const JPH::RefConst<JPH::Shape> capsule = createShape(JPH::RotatedTranslatedShapeSettings(
            JPH::Vec3(0.0F, halfHeight, 0.0F), JPH::Quat::sIdentity(),
            createShape(JPH::CapsuleShapeSettings(halfHeight - radius, radius))));
        const JPH::RefConst<JPH::Shape> inner   = createShape(JPH::RotatedTranslatedShapeSettings(
            JPH::Vec3(0.0F, halfHeight, 0.0F), JPH::Quat::sIdentity(),
            createShape(JPH::CapsuleShapeSettings(halfHeight - radius, 0.9F * radius))));

        JPH::CharacterVirtualSettings character;
        character.SetEmbedded();
        character.mShape            = capsule;
        character.mInnerBodyShape   = inner;
        character.mInnerBodyLayer   = layers::kMoving;
        character.mMaxSlopeAngle    = static_cast<float>(degreesToRadians(settings.maxSlopeDeg));
        character.mMass             = static_cast<float>(settings.characterMassKg);
        character.mMaxStrength      = static_cast<float>(settings.pushStrengthN);
        character.mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(), -radius);
        character.mEnhancedInternalEdgeRemoval = true;
        body_ = std::make_unique<JPH::CharacterVirtual>(&character, JPH::RVec3::sZero(),
                                                        JPH::Quat::sIdentity(), 0, &system);
    }

    [[nodiscard]] JPH::BodyID innerBody() const { return body_->GetInnerBodyID(); }

    [[nodiscard]] CharacterMove move(const Vec3d& feet, const Vec3d& velocity, const Vec3d& up,
                                     double gravity, double dt, MoveMode mode) override
    {
        tiles_->require(feet, kCharacterTerrainM + (glm::length(velocity) * dt), *clock_);
        const JPH::Vec3 joltUp = toJoltF(up);
        body_->SetUp(joltUp);
        body_->SetRotation(JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), joltUp));
        if (glm::distance(fromJolt(body_->GetPosition()), feet) > 1e-3)
        {
            body_->SetPosition(toJolt(feet));
        }
        // Walking presses on the ground (a step's worth of falling), which is what keeps the
        // character in contact with it.
        const Vec3d press = mode == MoveMode::WALK ? up * (-gravity * dt) : Vec3d(0.0);
        // Standing on something that is moving (a tram, a lift) carries you along with it.
        const Vec3d carried =
            mode == MoveMode::WALK &&
                    body_->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround
                ? fromJoltF(body_->GetGroundVelocity())
                : Vec3d(0.0);
        body_->SetLinearVelocity(toJoltF(velocity + press + carried));

        const JPH::Vec3        pull = joltUp * static_cast<float>(-gravity);
        const auto             bp   = system_->GetDefaultBroadPhaseLayerFilter(layers::kMoving);
        const auto             obj  = system_->GetDefaultLayerFilter(layers::kMoving);
        const JPH::BodyFilter  bodies;
        const JPH::ShapeFilter shapes;
        if (mode == MoveMode::WALK)
        {
            JPH::CharacterVirtual::ExtendedUpdateSettings walk;
            walk.mStickToFloorStepDown = joltUp * static_cast<float>(-stepDown_);
            walk.mWalkStairsStepUp     = joltUp * static_cast<float>(stepUp_);
            body_->ExtendedUpdate(static_cast<float>(dt), pull, walk, bp, obj, bodies, shapes,
                                  *temp_);
        }
        else
        {
            body_->Update(static_cast<float>(dt), pull, bp, obj, bodies, shapes, *temp_);
        }

        CharacterMove moved;
        moved.feet      = fromJolt(body_->GetPosition());
        moved.velocity  = fromJoltF(body_->GetLinearVelocity());
        moved.supported = body_->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround;
        if (mode == MoveMode::WALK)
        {
            moved.velocity -= press + carried;  // what you are doing, not what is carrying you
        }
        moved.blocked = glm::distance(moved.velocity, velocity) > 1e-3;
        return moved;
    }

private:
    JPH::PhysicsSystem*                    system_;
    JPH::TempAllocator*                    temp_;
    TerrainTiles*                          tiles_;
    const double*                          clock_;
    double                                 stepUp_;
    double                                 stepDown_;
    std::unique_ptr<JPH::CharacterVirtual> body_;
};

}  // namespace

// ---- The world ---------------------------------------------------------------------------------

struct PhysicsWorld::Impl
{
    Impl(std::shared_ptr<const HabitatGeometry> habitat, std::shared_ptr<const TerrainGrid> grid,
         const PhysicsSettings& settings)
      : geometry(std::move(habitat)),
        terrain(std::move(grid)),
        temp(std::make_unique<JPH::TempAllocatorImpl>(kTempMemory)),
        jobs(std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, static_cast<int>(settings.threads))),
        system(std::make_unique<JPH::PhysicsSystem>()),
        spin(*geometry, propBodies),
        terrainRadius(settings.terrainRadiusM)
    {
        system->Init(kMaxBodies, 0, kMaxBodyPairs, kMaxContacts, broadPhaseLayers,
                     objectVsBroadPhase, objectPairs);
        system->SetGravity(JPH::Vec3::sZero());  // the spinning frame supplies it (SpinFrame)
        system->AddStepListener(&spin);
        tiles     = std::make_unique<TerrainTiles>(*geometry, *terrain, system->GetBodyInterface());
        character = std::make_unique<JoltCharacter>(*system, *temp, *tiles, seconds, settings);
    }
    ~Impl() { system->RemoveStepListener(&spin); }
    Impl(const Impl&)            = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&)                 = delete;
    Impl& operator=(Impl&&)      = delete;

    [[nodiscard]] JPH::BodyInterface& bodies() const { return system->GetBodyInterface(); }

    JPH::RefConst<JPH::Shape> propShape(PropKind kind)
    {
        auto& shape = propShapes.at(static_cast<std::size_t>(kind));
        if (shape == nullptr)
        {
            shape = createPropShape(kind);
        }
        return shape;
    }

    /// Makes the trunks of the tree tiles near a point solid, and drops the ones long unused.
    void requireTrees(const Vec3d& focus)
    {
        if (!trees || seconds - treesChecked < kTreeCheckSeconds)
        {
            return;
        }
        treesChecked = seconds;
        for (std::size_t i = 0; i < trees->tiles.size(); ++i)
        {
            const TreeTile& tile = trees->tiles[i];
            const Vec3d     low  = tile.origin + Vec3d(tile.boundsMin);
            const Vec3d     high = tile.origin + Vec3d(tile.boundsMax);
            if (glm::length(glm::max(glm::max(low - focus, focus - high), Vec3d(0.0))) >
                kTreeReachM)
            {
                continue;
            }
            auto [entry, added] = treeTiles.try_emplace(i);
            if (added)
            {
                entry->second.id = buildTreeTile(bodies(), *trees, tile);
            }
            entry->second.lastUsed = seconds;
        }
        std::erase_if(treeTiles, [&](const auto& entry) {
            if (seconds - entry.second.lastUsed < kTileKeepSeconds)
            {
                return false;
            }
            if (!entry.second.id.IsInvalid())
            {
                bodies().RemoveBody(entry.second.id);
                bodies().DestroyBody(entry.second.id);
            }
            return true;
        });
    }

    /// Reads back where the moving props went, and catches any that fell through the ground.
    void readBack()
    {
        for (const std::size_t index : awake)
        {
            props[index].awake = false;
        }
        awake.clear();
        const JPH::uint    count = system->GetNumActiveBodies(JPH::EBodyType::RigidBody);
        const JPH::BodyID* ids   = system->GetActiveBodiesUnsafe(JPH::EBodyType::RigidBody);
        // Copied: moving a prop back up below can change the list.
        const std::span<const JPH::BodyID> list(ids, count);
        const std::vector<JPH::BodyID>     active(list.begin(), list.end());
        JPH::BodyInterface&                bodyInterface = bodies();
        for (const JPH::BodyID id : active)
        {
            const std::uint64_t user = bodyInterface.GetUserData(id);
            if (user == 0 || user > props.size())
            {
                continue;
            }
            PropState& prop     = props[user - 1];
            JPH::RVec3 position = JPH::RVec3::sZero();
            JPH::Quat  rotation = JPH::Quat::sIdentity();
            bodyInterface.GetPositionAndRotation(id, position, rotation);
            prop.position    = fromJolt(position);
            prop.orientation = fromJolt(rotation);
            prop.velocity    = fromJoltF(bodyInterface.GetLinearVelocity(id));
            prop.awake       = true;
            awake.push_back(user - 1);

            const double theta = HabitatGeometry::angleOf(prop.position);
            const auto   ground =
                geometry->groundRadius(prop.position.z, theta).value_or(geometry->radius());
            if (std::hypot(prop.position.x, prop.position.y) > ground + kFallThroughM)
            {
                const Vec3d up = HabitatGeometry::localUp(prop.position);
                prop.position += up * (std::hypot(prop.position.x, prop.position.y) - ground + 0.5);
                bodyInterface.SetPositionAndRotation(id, toJolt(prop.position), rotation,
                                                     JPH::EActivation::Activate);
                bodyInterface.SetLinearVelocity(id, JPH::Vec3::sZero());
            }
        }
    }

    // Order matters: members are destroyed bottom-up, and Jolt's globals must outlive the rest.
    JoltRuntime                                           runtime;
    std::shared_ptr<const HabitatGeometry>                geometry;
    std::shared_ptr<const TerrainGrid>                    terrain;
    BroadPhaseLayers                                      broadPhaseLayers;
    ObjectVsBroadPhase                                    objectVsBroadPhase;
    ObjectPairs                                           objectPairs;
    std::unique_ptr<JPH::TempAllocatorImpl>               temp;
    std::unique_ptr<JPH::JobSystemThreadPool>             jobs;
    std::unique_ptr<JPH::PhysicsSystem>                   system;
    std::vector<PropBody>                                 propBodies;
    SpinFrame                                             spin;
    std::unique_ptr<TerrainTiles>                         tiles;
    std::unique_ptr<JoltCharacter>                        character;
    std::array<JPH::RefConst<JPH::Shape>, kPropKindCount> propShapes;
    std::vector<PropState>                                props;
    std::vector<std::size_t>                              awake;
    struct TreeTileBody
    {
        JPH::BodyID id;
        double      lastUsed = 0.0;
    };
    /// A pool of kinematic bodies kept where moving things are, so you bump into them and ride
    /// on them: people near you, and the trams. They push you gently rather than being shoved.
    struct Movers
    {
        std::vector<JPH::BodyID> bodies;
        std::vector<Vec3d>       positions;  // where each of the used ones should be
        std::vector<Quatd>       orientations;
        std::size_t              used = 0;
    };
    Movers crowd;
    Movers trams;

    /// Grows a pool to hold what it has been given, and parks the bodies it does not need.
    void fitPool(Movers& pool, const JPH::RefConst<JPH::Shape>& shape, float friction,
                 std::size_t limit) const
    {
        while (pool.bodies.size() < std::min(pool.positions.size(), limit))
        {
            JPH::BodyCreationSettings settings(shape, JPH::RVec3(0.0, 0.0, 1.0e7),
                                               JPH::Quat::sIdentity(), JPH::EMotionType::Kinematic,
                                               layers::kMoving);
            settings.mFriction = friction;
            const JPH::BodyID id =
                bodies().CreateAndAddBody(settings, JPH::EActivation::DontActivate);
            if (id.IsInvalid())
            {
                break;
            }
            pool.bodies.push_back(id);
        }
        pool.used = std::min(pool.bodies.size(), pool.positions.size());
        for (std::size_t i = pool.used; i < pool.bodies.size(); ++i)
        {
            bodies().DeactivateBody(pool.bodies[i]);
        }
    }

    /// A person's capsule, and a tram car's box: one shape each, shared by the whole pool.
    JPH::RefConst<JPH::Shape> personShape()
    {
        if (personShapeCache == nullptr)
        {
            personShapeCache = createShape(JPH::CapsuleShapeSettings(0.55F, 0.26F));
        }
        return personShapeCache;
    }
    JPH::RefConst<JPH::Shape> tramShape()
    {
        if (tramShapeCache == nullptr)
        {
            tramShapeCache = createShape(JPH::BoxShapeSettings(
                JPH::Vec3(1.25F, static_cast<float>(0.5 * kTramBodyHeightM), 6.2F), 0.05F));
        }
        return tramShapeCache;
    }
    JPH::RefConst<JPH::Shape>                     personShapeCache;
    JPH::RefConst<JPH::Shape>                     tramShapeCache;
    std::shared_ptr<const TreeLayer>              trees;
    std::unordered_map<std::size_t, TreeTileBody> treeTiles;
    double                                        treesChecked  = -1.0e9;
    std::size_t                                   staticCount   = 0;
    double                                        seconds       = 0.0;
    double                                        terrainRadius = 100.0;
};

PhysicsWorld::PhysicsWorld(std::shared_ptr<const HabitatGeometry> geometry,
                           std::shared_ptr<const TerrainGrid>     terrain,
                           const PhysicsSettings&                 settings)
  : impl_(std::make_unique<Impl>(std::move(geometry), std::move(terrain), settings))
{
}

PhysicsWorld::~PhysicsWorld() = default;

void PhysicsWorld::addColliders(const StaticColliders& colliders)
{
    JPH::BodyInterface&      bodies = impl_->bodies();
    std::vector<JPH::BodyID> added;
    const auto               create = [&](const JPH::BodyCreationSettings& settings) {
        if (const JPH::Body* body = bodies.CreateBody(settings); body != nullptr)
        {
            added.push_back(body->GetID());
        }
    };
    for (const StaticBox& box : colliders.boxes)
    {
        const Vec3f               half(box.halfExtents);
        JPH::BodyCreationSettings settings(
            createShape(JPH::BoxShapeSettings(toJoltF(half),
                                              convexRadius(std::min({half.x, half.y, half.z})))),
            toJolt(box.centre), toJolt(box.orientation), JPH::EMotionType::Static, layers::kStatic);
        settings.mFriction = kBuiltFriction;
        create(settings);
    }
    for (const StaticHull& hull : colliders.hulls)
    {
        JPH::Array<JPH::Vec3> points;
        points.reserve(hull.points.size());
        for (const Vec3f& point : hull.points)
        {
            points.push_back(toJoltF(point));
        }
        JPH::BodyCreationSettings settings(createShape(JPH::ConvexHullShapeSettings(points)),
                                           toJolt(hull.origin), toJolt(hull.orientation),
                                           JPH::EMotionType::Static, layers::kStatic);
        settings.mFriction = kBuiltFriction;
        create(settings);
    }
    if (added.empty())
    {
        return;
    }
    const auto  count = static_cast<int>(added.size());
    auto* const state = bodies.AddBodiesPrepare(added.data(), count);
    bodies.AddBodiesFinalize(added.data(), count, state, JPH::EActivation::DontActivate);
    impl_->system->OptimizeBroadPhase();
    impl_->staticCount += added.size();
}

std::size_t PhysicsWorld::addProp(const PropPlacement& placement, const Vec3d& velocity, bool awake)
{
    const PropInfo&           info = propInfo(placement.kind);
    JPH::BodyCreationSettings settings(impl_->propShape(placement.kind), toJolt(placement.position),
                                       toJolt(placement.orientation), JPH::EMotionType::Dynamic,
                                       layers::kMoving);
    settings.mFriction                     = info.friction;
    settings.mRestitution                  = info.restitution;
    settings.mLinearDamping                = 0.0F;  // air drag is negligible for these
    settings.mAngularDamping               = info.angularDamping;
    settings.mOverrideMassProperties       = JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = info.massKg;
    settings.mLinearVelocity               = toJoltF(velocity);
    settings.mUserData                     = impl_->props.size() + 1;
    const bool        moving               = glm::length(velocity) > 0.0;
    const JPH::BodyID id                   = impl_->bodies().CreateAndAddBody(
        settings, moving || awake ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (id.IsInvalid())
    {
        log::warn("The physics world is full: a {} was left out", propKindName(placement.kind));
    }
    else if (moving)
    {
        impl_->spin.launched(id);
    }
    impl_->propBodies.push_back({.id = id, .buoyancy = info.buoyancy});
    impl_->props.push_back({.kind        = placement.kind,
                            .position    = placement.position,
                            .orientation = placement.orientation,
                            .velocity    = velocity,
                            .tint        = placement.tint,
                            .awake       = false,
                            .removed     = id.IsInvalid()});
    const std::size_t index = impl_->props.size() - 1;
    if ((moving || awake) && !id.IsInvalid())
    {
        impl_->props.back().awake = true;  // so the terrain under it is ready for its first step
        impl_->awake.push_back(index);
    }
    return index;
}

void PhysicsWorld::removeProp(std::size_t index)
{
    PropState& prop = impl_->props.at(index);
    if (prop.removed)
    {
        return;
    }
    const JPH::BodyID id = impl_->propBodies.at(index).id;
    impl_->bodies().RemoveBody(id);
    impl_->bodies().DestroyBody(id);
    prop.removed = true;
    prop.awake   = false;
    std::erase(impl_->awake, index);
}

void PhysicsWorld::push(std::size_t index, const Vec3d& impulse, const Vec3d& at)
{
    if (impl_->props.at(index).removed)
    {
        return;
    }
    impl_->bodies().AddImpulse(impl_->propBodies.at(index).id, toJoltF(impulse), toJolt(at));
}

void PhysicsWorld::setPeople(std::span<const Person> people, const Vec3d& focus, double radiusM)
{
    constexpr std::size_t kMaxSolidPeople = 48;

    Impl&        world   = *impl_;
    const double reachSq = radiusM * radiusM;
    world.crowd.positions.clear();
    world.crowd.orientations.clear();
    for (const Person& person : people)
    {
        if (world.crowd.positions.size() >= kMaxSolidPeople)
        {
            break;
        }
        const Vec3d offset = person.position - focus;
        if (glm::dot(offset, offset) <= reachSq)
        {
            // The capsule stands on their feet, so its middle is half their height up.
            const Vec3d up = HabitatGeometry::localUp(person.position);
            world.crowd.positions.push_back(person.position + (up * (0.5 * person.heightM)));
            world.crowd.orientations.push_back(floorOrientation(person.position));
        }
    }
    world.fitPool(world.crowd, world.personShape(), 0.6F, kMaxSolidPeople);
}

void PhysicsWorld::setTrams(std::span<const Tram> trams, const Vec3d& focus, double radiusM)
{
    constexpr std::size_t kMaxSolidTrams = 6;

    Impl&        world   = *impl_;
    const double reachSq = radiusM * radiusM;
    world.trams.positions.clear();
    world.trams.orientations.clear();
    for (const Tram& tram : trams)
    {
        if (world.trams.positions.size() >= kMaxSolidTrams)
        {
            break;
        }
        const Vec3d offset = tram.position - focus;
        if (glm::dot(offset, offset) > reachSq)
        {
            continue;
        }
        // The body is the car: its middle is half its height above the rails.
        const Vec3d up    = HabitatGeometry::localUp(tram.position);
        const Vec3d ahead = glm::normalize(tram.forward - (up * glm::dot(tram.forward, up)));
        const Vec3d side  = glm::cross(up, ahead);
        world.trams.positions.push_back(tram.position + (up * (0.5 * kTramBodyHeightM)));
        world.trams.orientations.push_back(glm::normalize(glm::quat_cast(Mat3d(side, up, ahead))));
    }
    world.fitPool(world.trams, world.tramShape(), 0.9F, kMaxSolidTrams);
}

void PhysicsWorld::step(double dt, const Vec3d& focus)
{
    if (!(dt > 0.0))
    {
        return;
    }
    Impl& world = *impl_;
    // Walk the kinematic bodies to where the people and the trams are now.
    JPH::BodyInterface& bodies = world.bodies();
    for (Impl::Movers* pool : {&world.crowd, &world.trams})
    {
        for (std::size_t i = 0; i < pool->used; ++i)
        {
            const JPH::RVec3 at   = toJolt(pool->positions[i]);
            const JPH::Quat  turn = toJolt(pool->orientations[i]);
            if (!bodies.IsActive(pool->bodies[i]))
            {
                bodies.SetPositionAndRotation(pool->bodies[i], at, turn,
                                              JPH::EActivation::Activate);
                continue;
            }
            bodies.MoveKinematic(pool->bodies[i], at, turn, static_cast<float>(dt));
        }
    }
    world.seconds += dt;
    world.tiles->require(focus, world.terrainRadius, world.seconds);
    world.requireTrees(focus);
    for (const std::size_t index : world.awake)
    {
        const PropState& prop = world.props[index];
        world.tiles->require(prop.position, kPropTerrainM + (glm::length(prop.velocity) * dt),
                             world.seconds);
    }
    world.system->Update(static_cast<float>(dt), 1, world.temp.get(), world.jobs.get());
    world.readBack();
    world.tiles->trim(world.seconds);
}

std::span<const PropState> PhysicsWorld::props() const
{
    return impl_->props;
}

std::size_t PhysicsWorld::awakeProps() const
{
    return impl_->awake.size();
}

std::size_t PhysicsWorld::terrainTiles() const
{
    return impl_->tiles->count();
}

void PhysicsWorld::setTrees(std::shared_ptr<const TreeLayer> trees)
{
    impl_->trees        = std::move(trees);
    impl_->treesChecked = -1.0e9;
}

std::size_t PhysicsWorld::treeTiles() const
{
    return impl_->treeTiles.size();
}

std::size_t PhysicsWorld::staticBodies() const
{
    return impl_->staticCount;
}

std::optional<RayHit> PhysicsWorld::raycast(const Vec3d& origin, const Vec3d& direction,
                                            double maxDistance) const
{
    const Vec3d                       along = glm::normalize(direction) * maxDistance;
    const JPH::RRayCast               ray{toJolt(origin), toJoltF(along)};
    JPH::RayCastResult                hit;
    const JPH::IgnoreSingleBodyFilter notMe(impl_->character->innerBody());
    if (!impl_->system->GetNarrowPhaseQuery().CastRay(ray, hit, {}, {}, notMe))
    {
        return std::nullopt;
    }
    RayHit result;
    result.distance = static_cast<double>(hit.mFraction) * maxDistance;
    result.point    = origin + (glm::normalize(direction) * result.distance);
    const JPH::BodyLockRead lock(impl_->system->GetBodyLockInterface(), hit.mBodyID);
    if (lock.Succeeded())
    {
        const JPH::Body& body = lock.GetBody();
        result.normal =
            fromJoltF(body.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, toJolt(result.point)));
        const std::uint64_t user = body.GetUserData();
        if (user > 0 && user <= impl_->props.size())
        {
            result.prop = user - 1;
        }
    }
    return result;
}

CharacterMover& PhysicsWorld::character()
{
    return *impl_->character;
}

}  // namespace StarshipSimulator
