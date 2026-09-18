// Per-frame camera data. Must match StarshipSimulator::gpu::FrameUniforms
// (include/StarshipSimulator/core/gpu_abi/uniforms.h).
// SDL_GPU puts uniform buffers in set 1 (vertex) or set 3 (fragment); define FRAME_SET before including.
#ifndef FRAME_SET
#error "Define FRAME_SET (1 in vertex shaders, 3 in fragment shaders) before including frame.glsl"
#endif

layout(std140, set = FRAME_SET, binding = 0) uniform Frame
{
    mat4 viewProjection;         // projection * rotation-only view; positions are camera-relative
    mat4 inverseViewProjection;  // NDC -> camera-relative position
    vec4 gridOrigin;             // xy: camera xy modulo 10 km, z: camera height above z = 0, w: near
    vec4 viewport;               // xy: size in pixels, zw: 1 / size
    vec4 sunDirection;           // xyz: direction toward the sun
}
frame;

// Camera-relative direction of the view ray through a point in normalized device coordinates.
vec3 viewRayDirection(vec2 ndc)
{
    vec4 nearPoint = frame.inverseViewProjection * vec4(ndc, 1.0, 1.0);  // reverse-Z: near = 1
    return normalize(nearPoint.xyz / nearPoint.w);
}

// Reverse-Z depth (1 at the near plane, 0 at infinity) of a camera-relative position.
float depthOf(vec3 cameraRelative)
{
    vec4 clip = frame.viewProjection * vec4(cameraRelative, 1.0);
    return clip.z / clip.w;
}
