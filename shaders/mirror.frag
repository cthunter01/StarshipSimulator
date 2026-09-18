#version 450

// Mirror surfaces seen through the windows: the reflective side shows the Sun and dark space, the
// back side is bare structure.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "noise.glsl"

layout(location = 0) in vec3 inCameraRelative;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;

layout(location = 0) out vec4 outColor;

const float REFLECTIVITY = 0.9;

// What the mirror reflects: the Sun (with a little glare from dust and imperfections) and space.
vec3 skyRadiance(vec3 direction)
{
    float cosAngle = dot(direction, habitat.sun.xyz);
    float disk     = smoothstep(cos(habitat.sun.w * 1.05), cos(habitat.sun.w * 0.95), cosAngle);
    float glare    = pow(max(cosAngle, 0.0), 3000.0) * 3.0 + pow(max(cosAngle, 0.0), 200.0) * 0.15;
    return habitat.sunColor.rgb * (disk * 120.0 + glare);
}

void main()
{
    vec3 view   = normalize(inCameraRelative);
    vec3 normal = normalize(inNormal);
    // Panels: the mirror is built from segments with thin seams.
    float seams = gridLines(inUv * vec2(8.0, 40.0), 0.01);

    vec3 color;
    if (dot(view, normal) < 0.0)
    {
        // Dust and panel imperfections scatter a little sunlight, so the mirror reads as a surface.
        vec3 scatter = habitat.sunColor.rgb * 0.004 * max(dot(normal, habitat.sun.xyz), 0.0) *
                       (0.8 + 0.4 * valueNoise(inUv * vec2(24.0, 120.0)));
        color = mix(REFLECTIVITY * skyRadiance(reflect(view, normal)) + scatter, vec3(0.02), seams);
    }
    else
    {
        // The back faces away from the Sun: dark truss work, faintly lit by the habitat's glow.
        color = vec3(0.012) * (0.7 + 0.6 * valueNoise(inUv * vec2(40.0, 400.0))) + vec3(0.02) * seams;
    }
    outColor = vec4(min(color, vec3(200.0)), 1.0);  // keep fp16 and MSAA resolve well-behaved
}
