#version 450

// Trees: rounded crowns lit by the mirrors' sun, softly (light filters through leaves), darker
// inside and toward the bottom, seen through the habitat's air.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#define SHADOW_UNIFORM 2
#define SHADOW_SAMPLER 0
#include "shadow.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec3 inCameraRelative;
layout(location = 2) flat in uint inMaterial;
layout(location = 3) flat in vec4 inTree;
layout(location = 4) in float inHeight;

layout(location = 0) out vec4 outColor;

const vec3 BARK = vec3(0.10, 0.075, 0.05);

vec3 leafColor(float species, float tint)
{
    if (species < 0.5)
    {
        return mix(vec3(0.050, 0.100, 0.028), vec3(0.090, 0.150, 0.036), tint);  // broadleaf
    }
    if (species < 1.5)
    {
        return mix(vec3(0.028, 0.060, 0.030), vec3(0.048, 0.088, 0.040), tint);  // conifer
    }
    return mix(vec3(0.065, 0.125, 0.034), vec3(0.100, 0.160, 0.050), tint);  // poplar
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
        for (int i = 0; i < stripCount(); ++i)
        {
            vec3 beam = beamLight(p, n, i) * 0.85 + beamLight(p, -n, i) * 0.2;
            if (max(beam.r, max(beam.g, beam.b)) > 0.0)
            {
                light += beam * treeShadow(inCameraRelative, n, i);
            }
        }
        color = albedo * (light + ambientLight(p, n)) * inside;
    }
    else
    {
        vec3 light = vec3(0.0);
        for (int i = 0; i < stripCount(); ++i)
        {
            light += beamLight(p, n, i) * treeShadow(inCameraRelative, n, i);
        }
        color = BARK * (light + ambientLight(p, n)) * 0.7;
    }
    Haze haze = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor  = vec4(color * haze.transmittance + haze.inscatter, 1.0);
}
