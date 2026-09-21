#version 450

// People, instanced: one body mesh, bent at the hips, knees and shoulders by this shader, so a
// crowd costs one draw call and no animation data. The mesh stands 1.75 m tall facing +z with its
// feet at the origin; each instance scales it, stands it on the ground and turns it to face the way
// it is walking.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "habitat.glsl"
#include "people.glsl"

// Must match StarshipSimulator::Vertex.
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;
layout(location = 3) in uint inMaterial;
// Must match StarshipSimulator::PersonInstance.
layout(location = 4) in vec3  inOffset;   // where their feet are, relative to the camera
layout(location = 5) in vec3  inForward;  // the way they face (habitat frame)
layout(location = 6) in float inGait;     // 0..1 through a stride
layout(location = 7) in float inSpeed;    // m/s
layout(location = 8) in float inScale;    // how tall they are, over 1.75 m
layout(location = 9) in uint  inLook;     // clothes | skin << 8 | activity << 16

layout(location = 0) out vec3 outNormal;
layout(location = 1) flat out uint outMaterial;
layout(location = 2) out vec3 outCameraRelative;
layout(location = 3) flat out uint outLook;
layout(location = 4) out vec2 outUv;

// Turns a point about the x axis of the body, through a joint.
vec3 bend(vec3 p, float pivotY, float angle)
{
    vec3  v = p - vec3(0.0, pivotY, 0.0);
    float c = cos(angle);
    float s = sin(angle);
    return vec3(p.x, pivotY + (c * v.y) - (s * v.z), (s * v.y) + (c * v.z));
}

vec3 bendNormal(vec3 n, float angle)
{
    float c = cos(angle);
    float s = sin(angle);
    return vec3(n.x, (c * n.y) - (s * n.z), (s * n.y) + (c * n.z));
}

void main()
{
    uint  activity = (inLook >> 16) & 0xFFu;
    vec3  local    = inPosition;
    vec3  normal   = inNormal;
    float turn     = 0.0;   // this part's own bend
    float carry    = 0.0;   // plus its parent's (the thigh, for a shin)

    if (activity == PERSON_SITTING)
    {
        // Thighs forward and level, shins down: the body settles to bench height.
        if (inMaterial == PART_THIGH_L || inMaterial == PART_THIGH_R)
        {
            turn = -1.53;
        }
        else if (inMaterial == PART_SHIN_L || inMaterial == PART_SHIN_R)
        {
            turn  = 1.53;
            carry = -1.53;
        }
        else if (inMaterial == PART_ARM_L || inMaterial == PART_ARM_R)
        {
            turn = -0.35;
        }
    }
    else
    {
        // A stride: the legs swing about the hips, the knees fold as they come through, and the
        // arms swing the other way. Standing still, all that is left is a slow shift of weight.
        float swing  = sin(2.0 * PI * inGait);
        float reach  = activity == PERSON_WALKING ? clamp(0.18 + (0.22 * inSpeed), 0.0, 0.62) : 0.0;
        float idle   = activity == PERSON_WALKING ? 0.0 : 0.05 * sin(2.0 * PI * inGait * 0.17);
        float side   = inMaterial == PART_THIGH_L || inMaterial == PART_SHIN_L ||
                             inMaterial == PART_ARM_L
                           ? 1.0
                           : -1.0;
        if (inMaterial == PART_THIGH_L || inMaterial == PART_THIGH_R)
        {
            turn = (reach * swing * side) + idle;
        }
        else if (inMaterial == PART_SHIN_L || inMaterial == PART_SHIN_R)
        {
            carry = (reach * swing * side) + idle;
            // Knees only fold one way, and most as the leg swings back through.
            turn = 1.35 * max(0.0, -swing * side) * reach;
        }
        else if (inMaterial == PART_ARM_L || inMaterial == PART_ARM_R)
        {
            turn = (-0.7 * reach * swing * side) + (0.6 * idle);
        }
    }

    if (turn != 0.0 || carry != 0.0)
    {
        float pivot = (inMaterial == PART_ARM_L || inMaterial == PART_ARM_R) ? PERSON_SHOULDER
                      : (inMaterial == PART_SHIN_L || inMaterial == PART_SHIN_R) ? PERSON_KNEE
                                                                                 : PERSON_HIP;
        local  = bend(local, pivot, turn);
        normal = bendNormal(normal, turn);
        if (carry != 0.0)
        {
            local  = bend(local, PERSON_HIP, carry);
            normal = bendNormal(normal, carry);
        }
    }
    if (activity == PERSON_SITTING)
    {
        local.y -= PERSON_HIP - PERSON_KNEE;  // sit down on the seat
    }
    else if (activity == PERSON_WALKING)
    {
        local.y -= 0.018 * (1.0 - cos(4.0 * PI * inGait));  // the body rises and falls as they walk
    }
    local *= inScale;

    // Stand them up: their own up is toward the axis, and they face along the ground.
    vec3 up    = localUp(inOffset + frame.cameraPosition.xyz);
    vec3 ahead = normalize(inForward - (up * dot(inForward, up)));
    vec3 right   = cross(up, ahead);  // right-handed: mirroring it would flip every face
    mat3 frameOf = mat3(right, up, ahead);

    outCameraRelative = inOffset + (frameOf * local);
    outNormal         = frameOf * normal;
    outMaterial       = inMaterial;
    outLook           = inLook;
    outUv             = inUv;
    gl_Position       = frame.viewProjection * vec4(outCameraRelative, 1.0);
}
