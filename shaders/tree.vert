#version 450

// Trees, instanced: one mesh per species and detail level, each instance placed on its tile,
// standing toward the axis, turned and scaled, its crown swaying a little in the breeze.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "habitat.glsl"

// Must match StarshipSimulator::Vertex.
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;
layout(location = 3) in uint inMaterial;
// Must match StarshipSimulator::TreeInstance.
layout(location = 4) in vec3 inTree;  // relative to the tile's origin
layout(location = 5) in uint inPacked;

// Must match StarshipSimulator::gpu::TreeDrawUniforms.
layout(std140, set = 1, binding = 2) uniform TreeDraw
{
    vec4 origin;  // xyz: the tile's origin relative to the camera
    vec4 lod;     // x: 1 = detailed mesh, y: detail end (m), z: blend (m), w: far end (m)
    vec4 fade;    // x: fade-out length (m)
}
draw;

layout(location = 0) out vec3 outNormal;
layout(location = 1) out vec3 outCameraRelative;
layout(location = 2) flat out uint outMaterial;
layout(location = 3) flat out vec4 outTree;  // x: species, y: tint, z: keep from, w: keep below
layout(location = 4) out float outHeight;    // 0 at the foot, 1 at the top

void main()
{
    float height  = float(inPacked & 0xFFFu) * 0.1;
    float turn    = float((inPacked >> 12u) & 0xFFu) / 255.0 * 2.0 * PI;
    float species = float((inPacked >> 20u) & 0xFu);
    float tint    = float(inPacked >> 24u) / 255.0;

    vec3  base  = draw.origin.xyz + inTree;
    vec3  up    = localUp(base + frame.cameraPosition.xyz);
    vec3  east  = normalize(cross(up, vec3(0.0, 0.0, 1.0)));
    vec3  x     = cos(turn) * east + sin(turn) * cross(east, up);
    vec3  z     = cross(x, up);
    float width = 0.85 + 0.3 * tint;

    vec3 local = inPosition * vec3(width, 1.0, width) * height;
    if (inMaterial == 1u)
    {
        // The crown sways, more toward the top.
        float phase = dot(base, vec3(0.13, 0.17, 0.11));
        local.x += sin(frame.time.x * 1.1 + phase) * 0.012 * inPosition.y * inPosition.y * height;
    }
    vec3 relative = base + local.x * x + local.y * up + local.z * z;

    // Detail levels cross-fade by dithering: keep a fragment when its dither value lies in
    // [keepFrom, keepBelow).
    float distance = length(base);
    float blend    = smoothstep(draw.lod.y - draw.lod.z, draw.lod.y, distance);
    float fadeOut  = 1.0 - smoothstep(draw.lod.w - draw.fade.x, draw.lod.w, distance);
    vec2  keep     = draw.lod.x > 0.5 ? vec2(0.0, 1.0 - blend) : vec2(1.0 - blend, fadeOut);

    outNormal         = inNormal.x * x + inNormal.y * up + inNormal.z * z;
    outCameraRelative = relative;
    outMaterial       = inMaterial;
    outTree           = vec4(species, tint, keep);
    outHeight         = inPosition.y;
    gl_Position       = frame.viewProjection * vec4(relative, 1.0);
}
