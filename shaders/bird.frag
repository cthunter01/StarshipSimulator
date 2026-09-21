#version 450

// Birds: dark against the bright far side, with a little light on their backs.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"

layout(location = 0) in vec3 inCameraRelative;
layout(location = 1) in vec3 inNormal;

layout(location = 0) out vec4 outColor;

const vec3 FEATHER = vec3(0.075, 0.072, 0.070);

void main()
{
    vec3 p = inCameraRelative + frame.cameraPosition.xyz;
    vec3 n = normalize(inNormal);
    if (!gl_FrontFacing)
    {
        n = -n;
    }
    vec3 color = FEATHER * (sunlight(p, n) + ambientLight(p, n));
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(color * haze.transmittance + haze.inscatter, 1.0);
}
