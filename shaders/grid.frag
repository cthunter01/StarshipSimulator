#version 450

// M0 test scene: an infinite ground plane at z = 0 with a multi-scale metric grid under a sky gradient.
// The camera position arrives modulo 10 km (computed in double on the CPU), so lines stay sharp even
// 1000 km from the origin.
#define FRAME_SET 3
#include "frame.glsl"

layout(location = 0) in vec2 inNdc;
layout(location = 0) out vec4 outColor;

// Coverage (0..1) of grid lines of the given spacing and world-space width, filtered so that lines
// thinner than a pixel fade to their average coverage instead of aliasing ("pristine grid" by
// Ben Golus: https://bgolus.medium.com/the-best-darn-grid-shader-yet-727f9278b9d8).
float gridLines(vec2 coord, float spacing, float lineWidth)
{
    vec2 uv          = coord / spacing;
    vec4 uvDDXY      = vec4(dFdx(uv), dFdy(uv));
    vec2 uvDeriv     = vec2(length(uvDDXY.xz), length(uvDDXY.yw));
    vec2 targetWidth = vec2(lineWidth / spacing);  // full line width in cells
    vec2 drawWidth   = clamp(targetWidth, uvDeriv, vec2(0.5));
    vec2 lineAA      = max(uvDeriv, vec2(1e-6)) * 1.5;
    vec2 gridUV      = 1.0 - abs(fract(uv) * 2.0 - 1.0);
    vec2 grid2       = smoothstep(drawWidth + lineAA, drawWidth - lineAA, gridUV);
    grid2 *= clamp(targetWidth / drawWidth, 0.0, 1.0);
    grid2 = mix(grid2, targetWidth, clamp(uvDeriv * 2.0 - 1.0, 0.0, 1.0));
    return mix(grid2.x, 1.0, grid2.y);
}

void main()
{
    vec3  dir       = viewRayDirection(inNdc);
    float camHeight = frame.gridOrigin.z;

    // Ray/plane intersection. Computed for every pixel so derivatives stay valid near the horizon.
    float t         = camHeight / max(abs(dir.z), 1e-6);
    bool  hitsPlane = dir.z * camHeight < 0.0;
    vec3  hit       = dir * t;
    vec2  coord     = hit.xy + frame.gridOrigin.xy;

    // Sky: deep blue overhead fading to a pale horizon.
    float elevation = clamp(dir.z, -1.0, 1.0);
    vec3  zenith    = vec3(0.015, 0.03, 0.09);
    vec3  horizon   = vec3(0.35, 0.45, 0.6);
    vec3  sky       = mix(horizon, zenith, pow(clamp(elevation, 0.0, 1.0), 0.45));

    if (!hitsPlane)
    {
        outColor     = vec4(sky, 1.0);
        gl_FragDepth = 0.0;  // infinitely far (reverse-Z)
        return;
    }

    vec3 ground = vec3(0.035, 0.04, 0.045);
    // Line widths are in metres, like paint on the ground: thin lines fade out with distance.
    ground += vec3(0.10) * gridLines(coord, 1.0, 0.015);               // 1 m grid
    ground += vec3(0.20) * gridLines(coord, 10.0, 0.05);               // 10 m
    ground += vec3(0.50, 0.45, 0.30) * gridLines(coord, 100.0, 0.2);   // 100 m
    ground += vec3(1.00, 0.60, 0.25) * gridLines(coord, 1000.0, 0.3);  // 1 km
    ground += vec3(0.35, 0.90, 1.20) * gridLines(coord, 10000.0, 0.6); // 10 km

    // Aerial haze toward the horizon.
    float haze = 1.0 - exp(-t / 15000.0);
    outColor     = vec4(mix(ground, horizon, haze), 1.0);
    gl_FragDepth = clamp(depthOf(hit), 0.0, 1.0);
}
