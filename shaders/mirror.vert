#version 450

// The external mirrors, one flat quad per window, built from the habitat uniforms: hinged at the
// anti-sunward end of the window and opened by the mirror angle toward the Sun. Drawn for our
// habitat and for its partner, placed by the Placement transform.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "habitat.glsl"

// Must match StarshipSimulator::gpu::PlacementUniforms: which habitat's mirrors (ours, or the
// partner's) and where, relative to the camera.
layout(std140, set = 1, binding = 2) uniform Placement
{
    mat4 model;  // habitat frame -> camera-relative
}
placement;

layout(location = 0) out vec3 outCameraRelative;
layout(location = 1) out vec3 outNormal;  // reflective face
layout(location = 2) out vec2 outUv;

void main()
{
    int  mirrorIndex = gl_VertexIndex / 6;
    int  corner      = gl_VertexIndex % 6;
    vec2 uv          = vec2(corner == 1 || corner == 2 || corner == 4 ? 1.0 : 0.0,
                            corner == 2 || corner == 4 || corner == 5 ? 1.0 : 0.0);
    if (mirrorIndex >= stripCount())
    {
        gl_Position = vec4(0.0);  // degenerate: nothing drawn
        return;
    }
    float angle   = float(mirrorIndex) * habitat.strips.x;
    float alpha   = habitat.mirror.x;
    vec3  outward = vec3(cos(angle), sin(angle), 0.0);
    vec3  tangent = vec3(-sin(angle), cos(angle), 0.0);
    vec3  along   = sin(alpha) * outward + cos(alpha) * vec3(0.0, 0.0, 1.0);
    vec3  hinge   = habitat.shape.x * outward + vec3(0.0, 0.0, habitat.mirror.w);
    vec3  world   = hinge + (uv.x * 2.0 - 1.0) * habitat.mirror.z * tangent + uv.y * habitat.mirror.y * along;

    outCameraRelative = (placement.model * vec4(world, 1.0)).xyz;
    outNormal         = mat3(placement.model) * (-cos(alpha) * outward + sin(alpha) * vec3(0.0, 0.0, 1.0));
    outUv             = uv;
    gl_Position       = frame.viewProjection * vec4(outCameraRelative, 1.0);
}
