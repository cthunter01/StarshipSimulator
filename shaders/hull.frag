#version 450

// The partner cylinder seen from outside: structure lit by the Sun where it faces it (the sunward
// dome), and window strips showing the valleys inside, bright by day and speckled with the lights
// of towns at night.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "noise.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec2 inUv;  // x: arc around the hull (m), y: z (m)
layout(location = 2) flat in uint inMaterial;
layout(location = 3) in vec3 inCameraRelative;

layout(location = 0) out vec4 outColor;

const uint MATERIAL_GLASS = 2u;

// Average colours of the far valley and the sky glow inside, seen from outside through the glass.
const vec3 VALLEY    = vec3(0.075, 0.120, 0.045);
const vec3 AIR_GLOW  = vec3(0.020, 0.035, 0.070);
const vec3 TOWNLIGHT = vec3(1.000, 0.700, 0.400);

void main()
{
    vec3  n        = normalize(inNormal);
    float daylight = habitat.atmosphere.w;
    vec3  sun      = habitat.sunColor.rgb;

    vec3 color;
    if (inMaterial == MATERIAL_GLASS)
    {
        // Fields and woods in patches, lit through the mirrors, behind the air's blue glow.
        float patches = 0.7 + 0.6 * fbm2(inUv / 1500.0);
        vec3  inside  = (VALLEY * patches + AIR_GLOW) * sun * daylight;
        // Towns: a scatter of warm lights, only noticeable once the mirrors have closed.
        vec2  cell    = floor(inUv / 90.0);
        float town    = step(0.93, hash12(cell)) * smoothstep(0.55, 0.75, fbm2(inUv / 2500.0 + 3.0));
        vec3  lights  = TOWNLIGHT * 0.01 * town * (1.0 - daylight);
        // The window lattice: ribs every 80 m.
        float ribs    = gridLines(inUv / 80.0, 0.02);
        color         = mix(inside + lights, vec3(0.01), ribs * 0.6);
    }
    else
    {
        // Hull plating: sunlight where it faces the Sun, a faint glow from the partner's lit
        // windows and Earth elsewhere.
        float plates = gridLines(inUv / 200.0, 0.01);
        float facing = max(dot(n, habitat.sun.xyz), 0.0);
        vec3  albedo = vec3(0.32, 0.31, 0.30) * (0.85 + 0.3 * valueNoise(inUv / 400.0)) * (1.0 - 0.4 * plates);
        color        = albedo * (sun * facing + vec3(0.003));
    }
    outColor = vec4(color, 1.0);
}
