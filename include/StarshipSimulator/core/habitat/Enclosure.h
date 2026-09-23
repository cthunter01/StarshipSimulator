#pragma once

#include <memory>

#include "StarshipSimulator/core/habitat/MeridianProfile.h"
#include "StarshipSimulator/core/habitat/habitat_spec.h"
#include "StarshipSimulator/core/math.h"

namespace StarshipSimulator
{

/// The habitat's air: the space inside it where a person, a thrown ball or a camera can be. In a
/// cylinder or a sphere that is everything above the floor, up to and across the axis; a torus
/// adds a ceiling (M8d) and an open ring its walls (M10).
class Enclosure
{
public:
    Enclosure(const HabitatSpec& spec, std::shared_ptr<const MeridianProfile> floor);

    /// Whether a point is inside the air: above the floor (its datum, before any terrain) and
    /// within the habitat's length.
    [[nodiscard]] bool contains(const Vec3d& point) const;

    /// How high the air goes above the floor before it meets the far side or the ceiling.
    [[nodiscard]] double headroomM() const { return headroomM_; }

private:
    std::shared_ptr<const MeridianProfile> floor_;
    double                                 headroomM_ = 0.0;
};

}  // namespace StarshipSimulator
