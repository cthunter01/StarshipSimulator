// The parts of a person and what they are doing: must match StarshipSimulator::personPart and
// StarshipSimulator::Activity (core/procgen/people.h).
const uint PART_HEAD = 0u, PART_HAIR = 1u, PART_TORSO = 2u;
const uint PART_ARM_L = 3u, PART_ARM_R = 4u;
const uint PART_THIGH_L = 5u, PART_THIGH_R = 6u;
const uint PART_SHIN_L = 7u, PART_SHIN_R = 8u;

const uint PERSON_WALKING = 0u, PERSON_STANDING = 1u, PERSON_SITTING = 2u;

// Where the joints are in the mesh's own frame (metres), as in core/procgen/people.h.
const float PERSON_HIP = 0.92, PERSON_KNEE = 0.48, PERSON_SHOULDER = 1.42;
