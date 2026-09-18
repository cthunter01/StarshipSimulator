#version 450

// Earth and the Moon as spheres lit by the Sun: phases, textures from NASA, Earth's city lights
// on its night side and the thin blue shell of its atmosphere.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "body.glsl"

layout(location = 0) in vec3 inView;

layout(set = 2, binding = 0) uniform sampler2D dayMap;    // albedo (sRGB texture), equirectangular
layout(set = 2, binding = 1) uniform sampler2D nightMap;  // night lights (sRGB texture)

layout(location = 0) out vec4 outColor;  // premultiplied alpha

const float PI              = 3.14159265358979;
const vec3  SKY_BLUE        = vec3(0.30, 0.55, 1.00);
const vec3  CITY_LIGHT_TINT = vec3(1.00, 0.72, 0.42);

// Samples an equirectangular map at a body-fixed direction without a seam where longitude wraps.
vec3 sampleMap(sampler2D map, vec3 local)
{
    float lon = atan(local.y, local.x);
    float lat = asin(clamp(local.z, -1.0, 1.0));
    vec2  uv  = vec2(0.5 + lon / (2.0 * PI), 0.5 - lat / PI);
    // Derivatives from a longitude that wraps elsewhere, whichever is smaller (Tarini).
    vec2  uvAlt = vec2(fract(uv.x + 0.5) - 0.5, uv.y);
    vec2  dx    = dFdx(uv);
    vec2  dy    = dFdy(uv);
    vec2  dxAlt = dFdx(uvAlt);
    vec2  dyAlt = dFdy(uvAlt);
    if (abs(dxAlt.x) + abs(dyAlt.x) < abs(dx.x) + abs(dy.x))
    {
        dx = dxAlt;
        dy = dyAlt;
    }
    return textureGrad(map, uv, dx, dy).rgb;
}

void main()
{
    vec3  v          = normalize(inView);
    vec3  d          = normalize(body.direction.xyz);
    vec3  sun        = normalize(body.towardSun.xyz);
    float r          = sin(body.direction.w);  // sphere radius at unit distance
    float pixelAngle = max(length(fwidth(v)), 1e-7);

    // Closest approach of the view ray to the centre, and the edge coverage for antialiasing.
    float along    = dot(v, d);
    vec3  closest  = v * along - d;
    float miss     = length(closest);
    float coverage = clamp((r - miss) / pixelAngle + 0.5, 0.0, 1.0);

    // The atmosphere seen edge-on around the disk, glowing where the Sun lights it.
    vec3 glow = vec3(0.0);
    if (body.params.x > 0.0)
    {
        float altitude = max(miss - r, 0.0) / r;  // body radii above the surface
        vec3  rimDir   = miss > 1e-9 ? closest / miss : sun;
        float lit      = smoothstep(-0.25, 0.35, dot(rimDir, sun));
        glow = SKY_BLUE * body.sunlight.rgb * 0.03 * lit * exp(-altitude / (0.25 * ATMOSPHERE_REACH)) *
               step(miss, r * (1.0 + ATMOSPHERE_REACH));
    }

    vec3 surface = vec3(0.0);
    if (coverage > 0.0)
    {
        float b = along;
        float c = 1.0 - r * r;
        float t = b - sqrt(max(b * b - c, 0.0));
        vec3  n = normalize(v * t - d);  // surface normal, habitat frame

        vec3  local  = mat3(body.bodyFromHabitat) * n;
        float facing = dot(n, sun);
        vec3  albedo = sampleMap(dayMap, local) * body.sunlight.a;
        surface      = albedo * (body.sunlight.rgb * max(facing, 0.0) + body.params.z);

        if (body.params.x > 0.0)
        {
            // Air scatters blue light near the limb on the day side; cities glow at night.
            float limb = pow(1.0 - clamp(dot(n, -v), 0.0, 1.0), 3.0);
            surface += SKY_BLUE * body.sunlight.rgb * 0.03 * limb * smoothstep(-0.15, 0.3, facing);
            float night = smoothstep(0.05, -0.12, facing);
            surface += sampleMap(nightMap, local) * CITY_LIGHT_TINT * body.params.y * night;
        }
    }
    outColor = vec4(surface * coverage + glow * (1.0 - coverage), coverage);
}
