#version 450

// Vertex layout must match StarshipSimulator::Vertex (include/StarshipSimulator/core/procgen/mesh.h).
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;
layout(location = 3) in uint inMaterial;

// Must match StarshipSimulator::gpu::DrawUniforms.
layout(std140, set = 1, binding = 0) uniform Draw
{
    mat4 modelViewProjection;  // includes the camera-relative translation
    mat4 model;                // rotation and scale, for normals
}
draw;

layout(location = 0) out vec3 outNormal;
layout(location = 1) out vec2 outUv;
layout(location = 2) flat out uint outMaterial;

void main()
{
    gl_Position = draw.modelViewProjection * vec4(inPosition, 1.0);
    outNormal   = mat3(draw.model) * inNormal;
    outUv       = inUv;
    outMaterial = inMaterial;
}
