#version 450

// The window strips: glass panes in a structural lattice. Drawn twice with different blending:
// first multiplying what lies beyond (space, mirrors) by the air's and the glass's transmittance,
// then adding the light scattered by the habitat's air and the lattice itself.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "noise.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec2 inUv;  // x: arc around the hull (m), y: z (m)
layout(location = 3) in vec3 inCameraRelative;

layout(std140, set = 3, binding = 2) uniform Glass
{
    vec4 mode;  // x: 0 = transmittance pass, 1 = emission pass
}
glass;

layout(location = 0) out vec4 outColor;

const float GLASS_TRANSMITTANCE = 0.92;

void main()
{
    vec3 p    = inCameraRelative + frame.cameraPosition.xyz;
    Haze haze = aerialPerspective(frame.cameraPosition.xyz, p);

    // 8 m panes in 10 cm frames, 40 cm ribs every 80 m.
    float frames = gridLines(inUv / 8.0, 0.0125);
    float ribs   = gridLines(inUv / 80.0, 0.005);
    float solid  = max(frames, ribs);

    if (glass.mode.x < 0.5)
    {
        outColor = vec4(haze.transmittance * (GLASS_TRANSMITTANCE * (1.0 - solid)), 1.0);
    }
    else
    {
        vec3 n     = normalize(inNormal);
        vec3 metal = vec3(0.30) * (sunlight(p, n) + ambientLight(p, n));
        outColor   = vec4(haze.inscatter + metal * solid * haze.transmittance, 1.0);
    }
}
