#include "StarshipSimulator/core/habitat/Enclosure.h"

#include <cmath>
#include <memory>
#include <optional>
#include <utility>

#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

Enclosure::Enclosure(const HabitatSpec& spec, std::shared_ptr<const MeridianProfile> floor)
  : floor_(std::move(floor)),
    kind_(spec.kind),
    radiusM_(spec.radiusM),
    headroomM_(StarshipSimulator::headroomM(spec))
{
}

bool Enclosure::contains(const Vec3d& point) const
{
    if (kind_ == HabitatKind::BERNAL_SPHERE)
    {
        return glm::length(point) < radiusM_;
    }
    const std::optional<double> floorRadius = floor_->radiusAt(point.z);
    return floorRadius && std::hypot(point.x, point.y) < *floorRadius;
}

}  // namespace StarshipSimulator
