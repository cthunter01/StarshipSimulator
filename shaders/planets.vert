#version 450

// The planets as points of light, like stars but moving: their data arrives as uniforms each frame.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "sky.glsl"
#include "star_sprite.glsl"

// Must match StarshipSimulator::gpu::PlanetUniforms.
layout(std140, set = 1, binding = 2) uniform Planets
{
    vec4 stars[16];  // per planet: direction + size, colour (as StarshipSimulator::GpuStar)
    vec4 count;      // x: number of planets
}
planets;

layout(location = 0) out vec3 outColor;
layout(location = 1) out vec2 outCorner;

void main()
{
    int planet = gl_VertexIndex / 6;
    if (planet >= int(planets.count.x + 0.5))
    {
        gl_Position = vec4(0.0);  // degenerate: nothing drawn
        outColor    = vec3(0.0);
        outCorner   = vec2(0.0);
        return;
    }
    Sprite sprite = starSprite(planets.stars[2 * planet], gl_VertexIndex);
    gl_Position   = sprite.clip;
    outColor      = planets.stars[2 * planet + 1].rgb * sky.params.x;
    outCorner     = sprite.corner;
}
