#version 450

// Maps the linear HDR scene to the display: exposure, tone mapping, then sRGB encoding.
#include "color.glsl"

layout(location = 1) in vec2 inUv;

layout(set = 2, binding = 0) uniform sampler2D hdrScene;
// Display colour -> graded colour: a size^3 table with its blue slices side by side.
layout(set = 2, binding = 1) uniform sampler2D gradeLut;

// Must match StarshipSimulator::gpu::TonemapUniforms.
layout(std140, set = 3, binding = 0) uniform Tonemap
{
    vec4 params;  // x: exposure, y: 1 = encode sRGB (the target is UNORM), z: grade strength
}
tonemap;

layout(location = 0) out vec4 outColor;

void main()
{
    vec3 hdr = texture(hdrScene, inUv).rgb * tonemap.params.x;
    vec3 ldr = clamp(toneMapPbrNeutral(hdr), 0.0, 1.0);
    // The grade works on display (sRGB-encoded) colour.
    vec3  display = linearToSrgb(ldr);
    float size    = float(textureSize(gradeLut, 0).y);
    vec3  cell    = display * (size - 1.0);
    float slice   = min(floor(cell.b), size - 2.0);
    vec2  uv      = vec2(cell.r + 0.5 + slice * size, cell.g + 0.5) / vec2(size * size, size);
    vec3  graded  = mix(texture(gradeLut, uv).rgb, texture(gradeLut, uv + vec2(1.0 / size, 0.0)).rgb,
                        cell.b - slice);
    display       = mix(display, graded, tonemap.params.z);
    outColor      = vec4(tonemap.params.y > 0.5 ? display : srgbToLinear(display), 1.0);
}
