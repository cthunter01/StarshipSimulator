#version 450

// A bird: a body, a tail and two wings, six vertices worked out here. One instance per bird, so
// nothing but the instance list is uploaded.
#define UNIFORM_SET 1
#include "frame.glsl"

layout(location = 0) in vec3  inPosition;  // per instance: relative to the camera
layout(location = 1) in float inWingspan;
layout(location = 2) in vec3  inForward;
layout(location = 3) in float inWingBeat;

layout(location = 0) out vec3 outCameraRelative;
layout(location = 1) out vec3 outNormal;

void main()
{
    // The bird's own frame: ahead, and up toward the spin axis.
    vec3 here    = inPosition + frame.cameraPosition.xyz;
    vec3 up      = length(here.xy) > 1e-3 ? vec3(-normalize(here.xy), 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 ahead   = normalize(inForward);
    vec3 side    = normalize(cross(ahead, up));
    vec3 overhead = cross(side, ahead);

    // Two triangles: body -> left tip -> tail, and body -> tail -> right tip.
    int  corner = gl_VertexIndex % 3;
    bool right  = gl_VertexIndex >= 3;
    vec3 local;
    if (corner == 0)
    {
        local = vec3(0.0, 0.02, 0.30);  // the body, a little above the wing line
    }
    else if ((corner == 1) == right)
    {
        local = vec3(0.0, 0.0, -0.40);  // the tail
    }
    else
    {
        // A wing tip, swept back and bent up or down by the beat.
        float bend = 0.42 * inWingBeat;
        local      = vec3(right ? 0.5 : -0.5, bend, -0.06);
    }
    // Never smaller than about half a degree across: a bird two fields away is a pixel wide and
    // would flicker in and out, and a flock you cannot see is not worth drawing.
    float span  = max(inWingspan, 0.008 * length(inPosition));
    vec3  offset = ((side * local.x) + (overhead * local.y) + (ahead * local.z)) * span;

    outCameraRelative = inPosition + offset;
    outNormal         = overhead;
    gl_Position       = frame.viewProjection * vec4(outCameraRelative, 1.0);
}
