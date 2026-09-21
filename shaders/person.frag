#version 450

// People: skin, hair and clothes, lit like everything else in the valley (the hills' shadows, the
// trees' shadow map, the cloud deck overhead, and the lamps at night).
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
#include "people.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) flat in uint inMaterial;
layout(location = 2) in vec3 inCameraRelative;
layout(location = 3) flat in uint inLook;
layout(location = 4) in vec2 inUv;

layout(set = 2, binding = 0) uniform sampler2D heightMap;
layout(set = 2, binding = 1) uniform sampler2D profileMap;
layout(set = 2, binding = 2) uniform sampler2D arcMap;
layout(set = 2, binding = 4) uniform sampler2D cloudMap;

layout(location = 0) out vec4 outColor;

// Linear albedos. Must be kPersonClothes and kPersonSkins long (core/procgen/people.h).
const vec3 CLOTHES[12] =
    vec3[](vec3(0.32, 0.10, 0.09), vec3(0.09, 0.14, 0.30), vec3(0.42, 0.38, 0.30),
           vec3(0.10, 0.22, 0.14), vec3(0.45, 0.32, 0.08), vec3(0.16, 0.16, 0.19),
           vec3(0.50, 0.46, 0.42), vec3(0.28, 0.14, 0.26), vec3(0.08, 0.26, 0.28),
           vec3(0.46, 0.24, 0.10), vec3(0.20, 0.24, 0.34), vec3(0.38, 0.14, 0.16));
const vec3 SKINS[6] = vec3[](vec3(0.42, 0.28, 0.20), vec3(0.30, 0.19, 0.13),
                             vec3(0.19, 0.11, 0.07), vec3(0.50, 0.36, 0.28),
                             vec3(0.12, 0.07, 0.05), vec3(0.38, 0.24, 0.16));
const vec3 HAIRS[4] = vec3[](vec3(0.035, 0.025, 0.020), vec3(0.10, 0.06, 0.03),
                             vec3(0.22, 0.16, 0.07), vec3(0.28, 0.27, 0.26));

vec3 albedo()
{
    uint clothes = inLook & 0xFFu;
    uint skin    = (inLook >> 8) & 0xFFu;
    if (inMaterial == PART_HEAD || inMaterial == PART_ARM_L || inMaterial == PART_ARM_R)
    {
        vec3 bare = SKINS[skin % 6u];
        // Sleeves: the arms are covered down to somewhere between the elbow and the wrist.
        if (inMaterial != PART_HEAD)
        {
            float sleeve = 0.35 + (0.45 * fract(float(clothes) * 0.37));
            bare = mix(CLOTHES[clothes % 12u], bare, step(sleeve, 1.0 - inUv.y));
        }
        return bare;
    }
    if (inMaterial == PART_HAIR)
    {
        return HAIRS[(skin + clothes) % 4u];
    }
    if (inMaterial == PART_THIGH_L || inMaterial == PART_THIGH_R || inMaterial == PART_SHIN_L ||
        inMaterial == PART_SHIN_R)
    {
        // Trousers and skirts in a second colour, shoes at the bottom.
        vec3 legs = CLOTHES[(clothes + 5u) % 12u];
        return mix(vec3(0.04, 0.035, 0.03), legs, step(0.12, inUv.y));
    }
    return CLOTHES[clothes % 12u];
}

void main()
{
    vec3 p     = inCameraRelative + frame.cameraPosition.xyz;
    vec3 n     = normalize(inNormal);
    vec3 light = surfaceLight(heightMap, profileMap, arcMap, cloudMap, p, n, inCameraRelative);
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(albedo() * light * haze.transmittance + haze.inscatter, 1.0);
}
