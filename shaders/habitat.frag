#version 450

// Meshes that are part of the habitat's inside but not its terrain: flat end walls and the hub
// caps, a torus's ceiling, spokes and hub. Lit like the land (landscape.frag), seen through the
// habitat's own air; and a torus's hull, seen from inside through its windows.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "noise.glsl"
#include "terrain_colors.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec2 inUv;  // x: arc around the hull (m), y: distance along the profile (m)
layout(location = 2) flat in uint inMaterial;
layout(location = 3) in vec3 inCameraRelative;

layout(location = 0) out vec4 outColor;

const uint MATERIAL_VALLEY = 0u;
const uint MATERIAL_ENDCAP = 1u;
const uint MATERIAL_HULL   = 4u;

void main()
{
    vec3 p = inCameraRelative + frame.cameraPosition.xyz;
    vec3 n = normalize(inNormal);
    if (inMaterial == MATERIAL_HULL)
    {
        // The outside, seen through the windows: grey lunar soil in the shield, lit by the Sun
        // itself rather than the mirrors, and faintly by the Earth.
        const vec3 REGOLITH = vec3(0.13, 0.12, 0.11);
        vec3 albedo = REGOLITH * (0.8 + 0.4 * valueNoise(inUv / 23.0));
        vec3 color  = albedo * (habitat.sunColor.rgb * max(dot(n, habitat.sun.xyz), 0.0) + vec3(0.003));
        Haze haze   = aerialPerspective(frame.cameraPosition.xyz, p);
        outColor    = vec4(color * haze.transmittance + haze.inscatter, 1.0);
        return;
    }

    vec3 albedo;
    if (inMaterial == MATERIAL_VALLEY)
    {
        albedo = valleyAlbedo(inUv, p, 0.0, 0.0);
    }
    else if (inMaterial == MATERIAL_ENDCAP)
    {
        albedo = endcapAlbedo(p, n, 0.0);
    }
    else
    {
        albedo = METAL;
    }

    vec3  lamps   = vec3(0.0);
    vec3  glow    = vec3(0.0);
    float outside = 1.0;  // how much of the habitat's glow reaches here
    if (torusTube())
    {
        // A torus's ceiling, spokes and hub: panels with dark seams. The spokes and the hub have
        // no windows: bands of lamps round them light them.
        const vec3 LAMPS     = vec3(1.0, 0.8, 0.55);
        float      footprint = length(fwidth(inUv));
        albedo *= (0.85 + 0.15 * valueNoise(inUv / 7.0)) *
                  (1.0 - 0.5 * detail(footprint, 0.4) * gridLines(inUv / 6.0, 0.02));
        float t = length(p.xy) - habitat.light.z;
        if (t * t + p.z * p.z > (habitat.band.w + 1.0) * (habitat.band.w + 1.0))
        {
            float strip = 1.0 - smoothstep(0.025, 0.04, abs(fract(inUv.y / 18.0) - 0.5));
            lamps       = LAMPS * 0.1;
            glow        = LAMPS * 1.5 * strip;
            outside     = 0.2;
        }
    }

    vec3 color = albedo * (sunlight(p, n) + ambientLight(p, n) * outside + lamps) + glow;
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(color * haze.transmittance + haze.inscatter, 1.0);
}
