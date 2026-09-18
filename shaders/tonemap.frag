#version 450

// Maps the linear HDR scene to the display: exposure, tone mapping, then sRGB encoding.
#include "color.glsl"

layout(location = 1) in vec2 inUv;

layout(set = 2, binding = 0) uniform sampler2D hdrScene;

// Must match StarshipSimulator::gpu::TonemapUniforms.
layout(std140, set = 3, binding = 0) uniform Tonemap
{
    vec4 params;  // x: exposure, y: 1 = encode sRGB (the target is UNORM)
}
tonemap;

layout(location = 0) out vec4 outColor;

void main()
{
    vec3 hdr = texture(hdrScene, inUv).rgb * tonemap.params.x;
    vec3 ldr = clamp(toneMapPbrNeutral(hdr), 0.0, 1.0);
    if (tonemap.params.y > 0.5)
    {
        ldr = linearToSrgb(ldr);
    }
    outColor = vec4(ldr, 1.0);
}
