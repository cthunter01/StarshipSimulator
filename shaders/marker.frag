#version 450

// Simple lit, outlined boxes for test markers.
layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec2 inUv;

// Must match StarshipSimulator::gpu::MaterialUniforms.
layout(std140, set = 3, binding = 0) uniform Material
{
    vec4 color;     // linear RGB
    vec4 emission;  // linear RGB
}
material;

layout(location = 0) out vec4 outColor;

void main()
{
    vec3  normal  = normalize(inNormal);
    vec3  sun     = normalize(vec3(0.4, 0.3, 0.85));
    float diffuse = max(dot(normal, sun), 0.0);
    float ambient = 0.25 + 0.15 * normal.z;

    // Darken face edges so boxes read clearly at any size.
    vec2  edge    = min(inUv, 1.0 - inUv);
    float outline = 1.0 - smoothstep(0.0, 0.05, min(edge.x, edge.y));

    vec3 lit = material.color.rgb * (2.0 * diffuse + ambient) * (1.0 - 0.6 * outline);
    outColor = vec4(lit + material.emission.rgb, 1.0);
}
