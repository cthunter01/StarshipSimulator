#version 450

// Rivers and lakes: the water surface over the bed drawn by the landscape pass. Drawn twice: first
// multiplying the bed by the light that gets through the surface and the water, then adding the
// reflection of the habitat overhead, the sun's glints and the light the water itself scatters.
// The surface is a level surface of spin gravity: a cylinder around the axis.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "landscape.glsl"
#include "noise.glsl"

layout(location = 0) in vec2 inCell;
layout(location = 1) in vec3 inCameraRelative;

layout(set = 2, binding = 0) uniform sampler2D heightMap;
layout(set = 2, binding = 1) uniform sampler2D profileMap;  // z, radius, inward normal (z, r)

layout(location = 0) out vec4 outColor;

const vec3 ABSORPTION = vec3(0.60, 0.16, 0.10);     // per metre: clear, faintly green water
const vec3 SCATTERING = vec3(0.010, 0.026, 0.024);  // light scattered back out by the water

// Small waves: the slope of two layers of noise drifting across the surface (uv in metres).
vec2 rippleSlope(vec2 uv, float t)
{
    const float e = 0.05;
    vec2        a = uv / 2.3 + vec2(0.13 * t, 0.31 * t);
    vec2        b = uv / 0.7 - vec2(0.27 * t, 0.11 * t);
    float       h = valueNoise(a) + 0.35 * valueNoise(b);
    float       x = valueNoise(a + vec2(e, 0.0)) + 0.35 * valueNoise(b + vec2(e / 0.7 * 2.3, 0.0));
    float       y = valueNoise(a + vec2(0.0, e)) + 0.35 * valueNoise(b + vec2(0.0, e / 0.7 * 2.3));
    return vec2(x - h, y - h) / (e * 2.3);
}

void main()
{
    float depth = landscape.heights.z - decodeHeight(textureLod(heightMap, heightUv(inCell), 0.0).r);
    if (depth <= 0.0)
    {
        discard;  // dry land (the ground here is above the water level)
    }
    vec4  profile = textureLod(profileMap, profileUv(inCell.y), 0.0);
    float theta   = inCell.x * landscape.grid.w;
    vec3  radial  = vec3(cos(theta), sin(theta), 0.0);
    vec3  around  = vec3(-sin(theta), cos(theta), 0.0);
    vec3  along   = vec3(0.0, 0.0, -profile.w) + radial * profile.z;  // down the profile
    vec3  level   = vec3(0.0, 0.0, profile.z) + radial * profile.w;   // faces the axis
    vec3  p       = inCameraRelative + frame.cameraPosition.xyz;

    // Ripples fade with distance so the far water stays calm instead of shimmering.
    vec2  uv       = vec2(theta * landscape.heights.w, inCell.y * landscape.grid.z);
    float distance = length(inCameraRelative);
    vec2  slope    = rippleSlope(uv, frame.time.x) * 0.06 * (1.0 - smoothstep(60.0, 600.0, distance));
    vec3  n        = normalize(level - slope.x * around - slope.y * along);

    vec3  view     = inCameraRelative / max(distance, 1e-3);
    float cosView  = clamp(dot(-view, n), 0.0, 1.0);
    float fresnel  = 0.02 + 0.98 * pow(1.0 - cosView, 5.0);
    // Down to the bed and back up: the slant path through the water.
    float path     = depth * (1.0 + 1.0 / max(dot(-view, level), 0.15));
    vec3  through  = exp(-ABSORPTION * path) * (1.0 - fresnel);
    Haze  haze     = aerialPerspective(frame.cameraPosition.xyz, p);

    if (landscape.mode.y < 0.5)
    {
        outColor = vec4(through, 1.0);  // the bed, dimmed and tinted
        return;
    }

    // What the surface reflects: the far side of the habitat seen through the air overhead,
    // hazier toward the horizon, and the sun images.
    vec3  r       = reflect(view, n);
    // The reflected ray crosses the habitat to the far side (or runs off toward an endcap): the land
    // there, dimmed and tinted by the air on the way.
    float a       = dot(r.xy, r.xy);
    float b       = dot(p.xy, r.xy);
    float c       = dot(p.xy, p.xy) - habitat.shape.x * habitat.shape.x;
    float across  = a > 1e-6 ? (-b + sqrt(max(b * b - a * c, 0.0))) / a : 20000.0;
    float reach   = min(across, 20000.0);
    vec3  airT    = exp(-extinction() * reach * 0.85);  // the air is a little thinner overhead
    vec3  farSide = habitat.ambientUp.rgb * 0.45 + vec3(0.002);
    vec3  sky     = farSide * airT + airLight() * (1.0 - airT);
    vec3  glints   = vec3(0.0);
    for (int i = 0; i < beamCount(); ++i)
    {
        vec3  towardSun = beamDirection(p, i);
        float intensity = habitat.beams[i].w;
        if (intensity > 0.0)
        {
            float aligned = pow(max(dot(r, towardSun), 0.0), 900.0) * 80.0;
            glints += habitat.sunColor.rgb * intensity * aligned * beamAperture(p, towardSun, i);
        }
    }
    vec3 lit     = sunlight(p, level) + ambientLight(p, level);
    vec3 scatter = SCATTERING * lit * (1.0 - exp(-ABSORPTION * path)) * (1.0 - fresnel);
    vec3 surface = (sky + glints) * fresnel + scatter;
    // The first pass also dimmed the haze in front of the bed: put that part back.
    outColor = vec4(min(surface * haze.transmittance + haze.inscatter * (1.0 - through), vec3(200.0)),
                    1.0);
}
