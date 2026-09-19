#version 450

// Props: wooden crates and barrels, footballs, hay bales, cafe chairs and tables, lit like the
// buildings around them.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "landscape.glsl"
#include "noise.glsl"
#define SHADOW_UNIFORM 3
#define SHADOW_SAMPLER 3
#include "shadow.glsl"
#include "lit.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec2 inUv;
layout(location = 2) flat in uint inMaterial;
layout(location = 3) in vec3 inCameraRelative;
layout(location = 4) flat in float inTint;
layout(location = 5) in vec3 inLocal;

layout(set = 2, binding = 0) uniform sampler2D heightMap;
layout(set = 2, binding = 1) uniform sampler2D profileMap;
layout(set = 2, binding = 2) uniform sampler2D arcMap;

layout(location = 0) out vec4 outColor;

// Must match StarshipSimulator::propMaterial.
const uint WOOD = 0u, STAVES = 1u, METAL = 2u, BALL = 3u, STRAW = 4u, TABLE_TOP = 5u;

const vec3 BALLS[4] = vec3[](vec3(0.55, 0.06, 0.04), vec3(0.06, 0.12, 0.45), vec3(0.60, 0.45, 0.04),
                             vec3(0.06, 0.35, 0.10));

vec3 albedo()
{
    vec2 uv = inUv;
    if (inMaterial == WOOD)
    {
        // Planks, with a darker frame around each face.
        float edge  = max(abs(uv.x - 0.5), abs(uv.y - 0.5));
        float plank = step(0.08, fract(uv.y * 5.0));
        vec3  wood  = vec3(0.30, 0.19, 0.10) * (0.85 + 0.2 * inTint);
        return wood * mix(0.7, 1.0, plank) * mix(1.0, 0.7, step(0.42, edge));
    }
    if (inMaterial == STAVES)
    {
        float hoop  = 1.0 - step(0.035, min(min(abs(uv.y - 0.12), abs(uv.y - 0.88)), abs(uv.y - 0.5)));
        float stave = step(0.06, fract(uv.x * 22.0));
        vec3  wood  = vec3(0.26, 0.15, 0.07) * mix(0.75, 1.0, stave);
        return mix(wood, vec3(0.08, 0.08, 0.08), hoop);
    }
    if (inMaterial == BALL)
    {
        // Panels of colour and white, like a beach ball.
        int   pick   = int(inTint * 3.99);
        float panel  = step(0.5, fract(atan(inLocal.z, inLocal.x) / (2.0 * PI) * 6.0));
        return mix(vec3(0.60, 0.58, 0.55), BALLS[pick], panel);
    }
    if (inMaterial == STRAW)
    {
        float fibre = valueNoise(inLocal.xz * 30.0 + inLocal.y * 4.0);
        return vec3(0.46, 0.36, 0.14) * (0.75 + 0.4 * fibre);
    }
    if (inMaterial == TABLE_TOP)
    {
        return vec3(0.55, 0.54, 0.50);
    }
    return vec3(0.05, 0.06, 0.05);  // painted metal
}

void main()
{
    vec3 p     = inCameraRelative + frame.cameraPosition.xyz;
    vec3 n     = normalize(inNormal);
    vec3 light = surfaceLight(heightMap, profileMap, arcMap, p, n, inCameraRelative);
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(albedo() * light * haze.transmittance + haze.inscatter, 1.0);
}
