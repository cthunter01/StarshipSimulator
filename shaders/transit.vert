#version 450

// The tramway: stretches of track, each placed at its own origin, and the trams running on it,
// turned to face the way they are going. One instance apiece.
#define UNIFORM_SET 1
#include "frame.glsl"

// Must match StarshipSimulator::Vertex.
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;
layout(location = 3) in uint inMaterial;
// Must match StarshipSimulator::TransitInstance.
layout(location = 4) in vec3 inOffset;  // relative to the camera
layout(location = 5) in uint inTint;
layout(location = 6) in vec4 inRotation;

layout(location = 0) out vec3 outNormal;
layout(location = 1) out vec2 outUv;
layout(location = 2) flat out uint outMaterial;
layout(location = 3) out vec3 outCameraRelative;
layout(location = 4) flat out uint outTint;
layout(location = 5) out vec3 outLocal;

vec3 rotate(vec4 q, vec3 v) { return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v); }

void main()
{
    vec3 relative     = inOffset + rotate(inRotation, inPosition);
    outNormal         = rotate(inRotation, inNormal);
    outUv             = inUv;
    outMaterial       = inMaterial;
    outCameraRelative = relative;
    outTint           = inTint;
    outLocal          = inPosition;
    gl_Position       = frame.viewProjection * vec4(relative, 1.0);
}
