#version 450

// The Milky Way and the faint stars behind the catalog stars: NASA's all-sky map (equirectangular in
// right ascension and declination, RA 0h at the centre increasing to the left), drawn behind
// everything else.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "sky.glsl"

layout(location = 0) in vec2 inNdc;

layout(set = 2, binding = 0) uniform sampler2D milkyWay;

layout(location = 0) out vec4 outColor;

const float PI = 3.14159265358979;

void main()
{
    // The view ray through this pixel (a point on the near plane, relative to the camera).
    vec4 near = frame.inverseViewProjection * vec4(inNdc, 1.0, 1.0);
    vec3 view = normalize(near.xyz / near.w);
    vec3 eqj  = transpose(mat3(sky.habitatFromInertial)) * view;

    float ra  = atan(eqj.y, eqj.x);
    float dec = asin(clamp(eqj.z, -1.0, 1.0));
    vec2  uv  = vec2(0.5 - ra / (2.0 * PI), 0.5 - dec / PI);

    // Texture derivatives jump where RA wraps, so pick the mip level from the pixel's angular
    // size instead.
    float pixelAngle = max(length(fwidth(view)), 1e-7);
    float texelAngle = PI / float(textureSize(milkyWay, 0).y);
    float lod        = max(log2(pixelAngle / texelAngle), 0.0);

    outColor = vec4(textureLod(milkyWay, uv, lod).rgb * sky.params.y, 1.0);
}
