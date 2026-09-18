// Per-frame camera data. Must match StarshipSimulator::gpu::FrameUniforms
// (include/StarshipSimulator/core/gpu_abi/uniforms.h).
// SDL_GPU puts uniform buffers in set 1 (vertex) or set 3 (fragment); define UNIFORM_SET before
// including. Frame is always uniform slot 0.
#ifndef UNIFORM_SET
#error "Define UNIFORM_SET (1 in vertex shaders, 3 in fragment shaders) before including frame.glsl"
#endif

layout(std140, set = UNIFORM_SET, binding = 0) uniform Frame
{
    mat4 viewProjection;         // projection * rotation-only view; positions are camera-relative
    mat4 inverseViewProjection;  // NDC -> camera-relative position
    vec4 cameraPosition;         // xyz: camera in the habitat frame, w: near plane (m)
    vec4 viewport;               // xy: size in pixels, zw: 1 / size
}
frame;
