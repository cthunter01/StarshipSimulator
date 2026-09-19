#version 450

// Meshes that are part of the habitat's inside but not its terrain: flat end walls and the hub
// caps. Lit like the land (landscape.frag), seen through the habitat's own air.
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

void main()
{
    vec3 p = inCameraRelative + frame.cameraPosition.xyz;
    vec3 n = normalize(inNormal);

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

    vec3 color = albedo * (sunlight(p, n) + ambientLight(p, n));
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(color * haze.transmittance + haze.inscatter, 1.0);
}
