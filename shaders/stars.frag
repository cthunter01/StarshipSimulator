#version 450

layout(location = 0) in vec3 inColor;
layout(location = 1) in vec2 inCorner;

layout(location = 0) out vec4 outColor;

void main()
{
    float falloff = exp(-4.5 * dot(inCorner, inCorner));
    outColor      = vec4(inColor * falloff, 1.0);
}
