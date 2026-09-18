#version 450

// Stars as small camera-facing quads, 6 vertices per star, read from a storage buffer. Directions are
// fixed in the inertial frame and turned into the spinning habitat frame each frame.
#define UNIFORM_SET 1
#include "frame.glsl"

// Must match StarshipSimulator::GpuStar: two vec4 per star.
layout(std430, set = 0, binding = 0) readonly buffer Stars
{
    vec4 data[];
}
stars;

// Must match StarshipSimulator::gpu::SkyUniforms.
layout(std140, set = 1, binding = 1) uniform Sky
{
    mat4 habitatFromInertial;
    vec4 params;  // x: brightness
}
sky;

layout(location = 0) out vec3 outColor;
layout(location = 1) out vec2 outCorner;

void main()
{
    int  star      = gl_VertexIndex / 6;
    int  corner    = gl_VertexIndex % 6;
    vec2 offset    = vec2(corner == 1 || corner == 2 || corner == 4 ? 1.0 : -1.0,
                          corner == 2 || corner == 4 || corner == 5 ? 1.0 : -1.0);
    vec4 direction = stars.data[2 * star];
    vec4 color     = stars.data[2 * star + 1];

    vec3 habitatDirection = (sky.habitatFromInertial * vec4(direction.xyz, 0.0)).xyz;
    vec4 clip             = frame.viewProjection * vec4(habitatDirection, 0.0);  // at infinity
    clip.xy += offset * direction.w * frame.viewport.zw * clip.w;

    gl_Position = clip;
    outColor    = color.rgb * sky.params.x;
    outCorner   = offset;
}
