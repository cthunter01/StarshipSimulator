#pragma once

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/procgen/mesh.h"

namespace StarshipSimulator
{

/// The outside of a habitat as seen from afar (the partner cylinder): the cylinder wall with its
/// window strips (material::kGlass), the end walls and domes (material::kMetal). In the habitat's
/// own frame, outward normals, counter-clockwise front faces seen from outside. uv: x = arc around
/// the hull (m), y = z (m).
[[nodiscard]] CpuMesh buildHullMesh(const HabitatGeometry& geometry);

/// Where the counter-rotating partner cylinder is in our spinning habitat frame: the transform from
/// its own frame into ours. Both have turned by spinPhase since phase 0 (in opposite directions);
/// at phase 0 the partner lies along our +X, separationM away (axis to axis), axes parallel.
[[nodiscard]] Mat4d partnerTransform(double separationM, double spinPhase);

}  // namespace StarshipSimulator
