#version 450

// The tramway: ballast and steel rails, concrete piers and platforms, and the trams themselves,
// painted with a window band that lights up at night.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "landscape.glsl"
#include "noise.glsl"
#define SHADOW_UNIFORM 3
#define SHADOW_SAMPLER 3
#include "shadow.glsl"
#include "clouds.glsl"
#include "lit.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec2 inUv;
layout(location = 2) flat in uint inMaterial;
layout(location = 3) in vec3 inCameraRelative;
layout(location = 4) flat in uint inTint;
layout(location = 5) in vec3 inLocal;

layout(set = 2, binding = 0) uniform sampler2D heightMap;
layout(set = 2, binding = 1) uniform sampler2D profileMap;
layout(set = 2, binding = 2) uniform sampler2D arcMap;
layout(set = 2, binding = 4) uniform sampler2D cloudMap;

layout(location = 0) out vec4 outColor;

// Must match StarshipSimulator::transit_material.
const uint BALLAST = 0u, RAIL = 1u, PIER = 2u, PLATFORM = 3u;
const uint BODY = 4u, GLASS = 5u, ROOF = 6u, SKIRT = 7u, DECK = 8u;

const vec3 LIVERY[4] = vec3[](vec3(0.42, 0.13, 0.09), vec3(0.10, 0.26, 0.34),
                              vec3(0.30, 0.34, 0.12), vec3(0.40, 0.33, 0.10));

void main()
{
    vec3  p       = inCameraRelative + frame.cameraPosition.xyz;
    vec3  n       = normalize(inNormal);
    vec3  albedo  = vec3(0.2);
    vec3  glow    = vec3(0.0);
    float night   = lightsOn();

    if (inMaterial == BALLAST)
    {
        // Crushed stone, with a sleeper showing through every 60 cm along the track (which runs
        // along the habitat's axis, so the mesh's own z measures it).
        float sleeper = 1.0 - step(0.34, fract(inLocal.z / 0.62));
        albedo = mix(vec3(0.165, 0.160, 0.150), vec3(0.075, 0.055, 0.038), sleeper) *
                 (0.8 + 0.4 * valueNoise(inLocal.xz * 6.0));
    }
    else if (inMaterial == RAIL)
    {
        // Worn steel: the rail head is polished bright where the wheels run.
        albedo = mix(vec3(0.085, 0.075, 0.070), vec3(0.46, 0.47, 0.49),
                     step(0.6, dot(n, localUp(p))));
    }
    else if (inMaterial == DECK)
    {
        // A girder deck: sleepers along the top, painted steel down the sides.
        float sleeper = 1.0 - step(0.34, fract(inLocal.z / 0.62));
        vec3  timber  = mix(vec3(0.105, 0.100, 0.095), vec3(0.075, 0.055, 0.038), sleeper);
        albedo        = mix(vec3(0.085, 0.090, 0.088), timber, step(0.6, dot(n, localUp(p))));
    }
    else if (inMaterial == PIER || inMaterial == PLATFORM)
    {
        albedo = vec3(0.30, 0.29, 0.275) * (0.9 + 0.2 * valueNoise(inUv * 8.0));
    }
    else if (inMaterial == GLASS)
    {
        // The window band: dark by day, warmly lit from inside once the mirrors close.
        albedo = vec3(0.035, 0.040, 0.045);
        float pane = step(0.06, fract(inUv.x * 12.0));
        glow = vec3(1.0, 0.86, 0.62) * (0.9 * night * pane);
        albedo = mix(vec3(0.10, 0.10, 0.11), albedo, pane);
    }
    else if (inMaterial == ROOF)
    {
        albedo = vec3(0.30, 0.30, 0.29);
    }
    else if (inMaterial == SKIRT)
    {
        albedo = vec3(0.055, 0.055, 0.060);
    }
    else
    {
        // The body: the line's livery above a cream band, and a headlight at each end.
        vec3  paint = LIVERY[inTint % 4u];
        float band  = smoothstep(0.30, 0.34, inUv.y) * (1.0 - smoothstep(0.62, 0.66, inUv.y));
        albedo      = mix(paint, vec3(0.62, 0.58, 0.50), band);
        float lamp  = step(0.82, abs(normalize(inLocal).z)) * step(0.55, inLocal.y);
        glow        = vec3(1.0, 0.94, 0.80) * (lamp * mix(0.35, 1.6, night));
    }

    vec3 light = surfaceLight(heightMap, profileMap, arcMap, cloudMap, p, n, inCameraRelative);
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(((albedo * light) + glow) * haze.transmittance + haze.inscatter, 1.0);
}
