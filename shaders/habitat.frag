#version 450

// The habitat's land: valley farmland, forests, endcap slopes. Lit by the sun beams from the mirrors
// and by the glow of the far side overhead, seen through the habitat's own air.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "noise.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec2 inUv;  // x: arc around the hull (m), y: distance along the profile (m)
layout(location = 2) flat in uint inMaterial;
layout(location = 3) in vec3 inCameraRelative;

layout(location = 0) out vec4 outColor;

const uint MATERIAL_VALLEY = 0u;
const uint MATERIAL_ENDCAP = 1u;

// Linear albedos, in the warm palette of the 1970s NASA habitat paintings.
const vec3 MEADOW  = vec3(0.105, 0.200, 0.050);
const vec3 SPROUTS = vec3(0.180, 0.270, 0.060);
const vec3 WHEAT   = vec3(0.420, 0.320, 0.110);
const vec3 SOIL    = vec3(0.170, 0.105, 0.055);
const vec3 ORCHARD = vec3(0.075, 0.150, 0.045);
const vec3 BLOSSOM = vec3(0.330, 0.250, 0.300);
const vec3 FOREST  = vec3(0.040, 0.085, 0.030);
const vec3 HEDGE   = vec3(0.035, 0.070, 0.025);
const vec3 STONE   = vec3(0.360, 0.340, 0.300);
const vec3 ROCK    = vec3(0.170, 0.150, 0.130);
const vec3 METAL   = vec3(0.450, 0.460, 0.480);

vec3 cropColor(float pick)
{
    if (pick < 0.30) return MEADOW;
    if (pick < 0.50) return SPROUTS;
    if (pick < 0.72) return WHEAT;
    if (pick < 0.84) return SOIL;
    if (pick < 0.97) return ORCHARD;
    return BLOSSOM;
}

// Fades out detail of the given size (m) once a pixel covers more than about half of it.
float detail(float footprint, float size) { return 1.0 - smoothstep(0.25 * size, 0.6 * size, footprint); }

vec3 valleyAlbedo(vec2 uv, vec3 p)
{
    float footprint = length(fwidth(uv));

    // Patchwork fields, their grid gently warped so it doesn't look machine made.
    vec2  warped = uv + 60.0 * vec2(fbm2(uv / 900.0), fbm2(uv / 900.0 + 7.3));
    vec2  cell   = warped / vec2(170.0, 240.0);
    vec2  id     = floor(cell);
    float crop   = hash12(id);
    vec3  field  = cropColor(crop) * (0.85 + 0.3 * valueNoise(uv / 18.0));

    // Close up: grass tufts, and furrows or crop rows running along each field.
    field *= 1.0 + 0.18 * detail(footprint, 3.0) * (valueNoise(uv / 1.3) - 0.5);
    field *= 1.0 + 0.12 * detail(footprint, 0.6) * (valueNoise(uv / 0.35) - 0.5);
    if (crop >= 0.30 && crop < 0.84)
    {
        float rows = 0.5 + 0.5 * sin(warped.x * (2.0 * PI / 1.1));
        field *= 1.0 - 0.25 * detail(footprint, 1.1) * rows;
    }

    // Hedgerows along field edges, fading out where they would be smaller than a pixel.
    float hedge = gridLines(cell, 0.03);
    vec3  color = mix(field, HEDGE, hedge * 0.8);

    // Woods and copses.
    float woods  = fbm2(uv / 650.0 + vec2(3.1, 1.7));
    float forest = smoothstep(0.58, 0.62, woods);
    float canopy = 0.8 + 0.4 * mix(0.5, valueNoise(uv / 9.0), detail(footprint, 9.0));
    color        = mix(color, FOREST * canopy, forest);

    // A stone promenade along the windows, where the terrain is flat.
    return mix(STONE, color, smoothstep(22.0, 32.0, distanceToWindow(p)));
}

vec3 endcapAlbedo(vec3 p, vec3 n)
{
    float footprint = length(fwidth(p));
    float slope     = 1.0 - dot(n, localUp(p));  // 0 flat, 1 vertical
    vec3  grass     = mix(MEADOW, SPROUTS, valueNoise3(p / 120.0));
    float woods     = smoothstep(0.50, 0.56, fbm3(p / 500.0));
    vec3  color     = mix(grass, FOREST, woods);
    // Terraced gardens on the slopes: alternate green and stone bands every 40 m of height.
    float terrace   = detail(footprint, 40.0) * smoothstep(0.35, 0.65, fract(length(p.xy) / 40.0));
    color           = mix(color, color * 0.75 + STONE * 0.1, terrace * 0.5);
    float rockMask  = smoothstep(0.28, 0.40, slope + 0.12 * valueNoise3(p / 60.0));
    float grain     = mix(0.5, valueNoise3(p / 15.0), detail(footprint, 15.0));
    return mix(color, ROCK * (0.8 + 0.4 * grain), rockMask);
}

void main()
{
    vec3 p = inCameraRelative + frame.cameraPosition.xyz;
    vec3 n = normalize(inNormal);

    vec3 albedo;
    if (inMaterial == MATERIAL_VALLEY)
    {
        albedo = valleyAlbedo(inUv, p);
    }
    else if (inMaterial == MATERIAL_ENDCAP)
    {
        albedo = endcapAlbedo(p, n);
    }
    else
    {
        albedo = METAL;
    }

    vec3 color = albedo * (sunlight(p, n) + ambientLight(p, n));
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(color * haze.transmittance + haze.inscatter, 1.0);
}
