#version 450

// Trees: rounded crowns lit by the mirrors' sun, softly (light filters through leaves), darker
// inside and toward the bottom, seen through the habitat's air.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#define SHADOW_UNIFORM 2
#define SHADOW_SAMPLER 0
#include "shadow.glsl"
#include "clouds.glsl"

layout(set = 2, binding = 1) uniform sampler2D cloudMap;

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec3 inCameraRelative;
layout(location = 2) flat in uint inMaterial;
layout(location = 3) flat in vec4 inTree;
layout(location = 4) in float inHeight;

layout(location = 0) out vec4 outColor;

const vec3 BARK = vec3(0.10, 0.075, 0.05);

vec3 leafColor(float species, float tint)
{
    if (species < 1.5 && species >= 0.5)
    {
        // Conifers keep their needles: only a little darker in the cold half of the year.
        vec3 needles = mix(vec3(0.028, 0.060, 0.030), vec3(0.048, 0.088, 0.040), tint);
        return needles * (1.0 - (0.15 * habitat.season.y));
    }
    vec3 green = species < 0.5 ? mix(vec3(0.050, 0.100, 0.028), vec3(0.090, 0.150, 0.036), tint)
                               : mix(vec3(0.065, 0.125, 0.034), vec3(0.100, 0.160, 0.050), tint);
    // Broadleaves and poplars: gold and russet in autumn, pale blossom in spring. Each tree turns
    // at its own pace, so a wood is never all one colour.
    float turn    = clamp(habitat.season.y * (0.25 + (1.5 * tint)), 0.0, 1.0);
    vec3  autumn  = mix(vec3(0.360, 0.190, 0.030), vec3(0.480, 0.280, 0.045), tint);
    vec3  blossom = mix(vec3(0.340, 0.235, 0.250), vec3(0.420, 0.320, 0.330), tint);
    return mix(mix(green, autumn, turn), blossom, habitat.season.z * 0.8 * step(species, 0.5));
}

void main()
{
    // Cross-fade between detail levels and out at the far end (interleaved gradient noise).
    float dither = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    if (dither < inTree.z || dither >= inTree.w)
    {
        discard;
    }

    vec3 p = inCameraRelative + frame.cameraPosition.xyz;
    vec3 n = normalize(inNormal);
    vec3 color;
    if (inMaterial == 1u)
    {
        vec3  albedo = leafColor(inTree.x, inTree.y);
        float inside = mix(0.45, 1.0, smoothstep(0.35, 0.95, inHeight));
        // Light through the leaves: some of it reaches faces turned away from the sun.
        vec3 light = vec3(0.0);
        for (int i = 0; i < beamCount(); ++i)
        {
            vec3 beam = beamLight(p, n, i) * 0.85 + beamLight(p, -n, i) * 0.2;
            if (max(beam.r, max(beam.g, beam.b)) > 0.0)
            {
                light += beam * treeShadow(inCameraRelative, n, i) *
                         cloudShade(cloudMap, p, beamDirection(p, i));
            }
        }
        color = albedo * (light + ambientLight(p, n)) * inside;
    }
    else
    {
        vec3 light = vec3(0.0);
        for (int i = 0; i < beamCount(); ++i)
        {
            light += beamLight(p, n, i) * treeShadow(inCameraRelative, n, i) *
                     cloudShade(cloudMap, p, beamDirection(p, i));
        }
        color = BARK * (light + ambientLight(p, n)) * 0.7;
    }
    Haze haze = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor  = vec4(color * haze.transmittance + haze.inscatter, 1.0);
}
