// The sky: uniform slot 1. Must match StarshipSimulator::gpu::SkyUniforms. Include frame.glsl first.
// Fixed directions (stars, the Milky Way, the planets) are in the J2000 equatorial frame (EQJ).

layout(std140, set = UNIFORM_SET, binding = 1) uniform Sky
{
    mat4 habitatFromInertial;  // EQJ -> the spinning habitat frame
    vec4 params;               // x: star brightness, y: Milky Way brightness
}
sky;
