#pragma once

#include <vector>

#include "StarshipSimulator/core/habitat/HabitatGeometry.h"
#include "StarshipSimulator/core/physics/colliders.h"
#include "StarshipSimulator/core/procgen/habitat_mesher.h"

namespace StarshipSimulator
{

/// A Stanford torus's ceiling, spokes and hub: everything inside it that is not floor, with the
/// ceiling's windows as glass, and its hull as seen from outside (through the windows, the rest of
/// the wheel). Chunks as buildHabitatMeshes makes them, the inside's metal and glass facing into
/// the air and the hull facing out, gathered into a few big ones (the solid parts and the glass
/// round each spoke, and the hub) so the whole of it takes a dozen draws; none for the other kinds.
[[nodiscard]] std::vector<MeshChunk> buildEnclosureMeshes(const HabitatGeometry& geometry,
                                                          double                 cellSizeM);

/// The inside of a torus's ceiling, spokes and hub among a habitat's chunks, as surfaces to collide
/// with (the hull, outside, is left out): you stand on the hub's floor, and a ball thrown up hits
/// the ceiling. Empty for the other kinds.
[[nodiscard]] StaticColliders enclosureColliders(const HabitatGeometry&        geometry,
                                                 const std::vector<MeshChunk>& chunks);

}  // namespace StarshipSimulator
