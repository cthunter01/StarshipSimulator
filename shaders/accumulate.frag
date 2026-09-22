#version 450

// Long exposure: the scene copied into a target that keeps the brightest it has ever been at each
// pixel. Everything that stays put stays as it is; anything bright that moves -- the stars as the
// habitat turns, a tram's headlight, a lit window going by -- draws a trail behind it.
layout(location = 0) in vec2 inNdc;
layout(location = 1) in vec2 inUv;

layout(set = 2, binding = 0) uniform sampler2D scene;

layout(location = 0) out vec4 outColor;

void main() { outColor = vec4(texture(scene, inUv).rgb, 1.0); }
