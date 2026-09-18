#version 450

// The catalog stars, read from a storage buffer. Directions are fixed in the inertial frame and
// turned into the spinning habitat frame each frame.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "sky.glsl"
#include "star_sprite.glsl"

// Must match StarshipSimulator::GpuStar: two vec4 per star.
layout(std430, set = 0, binding = 0) readonly buffer Stars
{
    vec4 data[];
}
stars;

layout(location = 0) out vec3 outColor;
layout(location = 1) out vec2 outCorner;

void main()
{
    int    star   = gl_VertexIndex / 6;
    Sprite sprite = starSprite(stars.data[2 * star], gl_VertexIndex);
    gl_Position   = sprite.clip;
    outColor      = stars.data[2 * star + 1].rgb * sky.params.x;
    outCorner     = sprite.corner;
}
