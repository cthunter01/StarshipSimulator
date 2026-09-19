#version 450

// Loose props (balls, crates, barrels, bales, cafe furniture), instanced: each one placed and
// turned (a quaternion) where the physics has it now.
#define UNIFORM_SET 1
#include "frame.glsl"

// Must match StarshipSimulator::Vertex.
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;
layout(location = 3) in uint inMaterial;
// Must match StarshipSimulator::PropInstance.
layout(location = 4) in vec3 inOffset;  // relative to the camera
layout(location = 5) in uint inTint;
layout(location = 6) in vec4 inRotation;

layout(location = 0) out vec3 outNormal;
layout(location = 1) out vec2 outUv;
layout(location = 2) flat out uint outMaterial;
layout(location = 3) out vec3 outCameraRelative;
layout(location = 4) flat out float outTint;
layout(location = 5) out vec3 outLocal;  // in the prop's own frame, for its patterns

vec3 rotate(vec4 q, vec3 v) { return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v); }

void main()
{
    vec3 relative     = inOffset + rotate(inRotation, inPosition);
    outNormal         = rotate(inRotation, inNormal);
    outUv             = inUv;
    outMaterial       = inMaterial;
    outCameraRelative = relative;
    outTint           = float(inTint) / 255.0;
    outLocal          = inPosition;
    gl_Position       = frame.viewProjection * vec4(relative, 1.0);
}
