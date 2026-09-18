// Earth or the Moon: uniform slot 1. Must match StarshipSimulator::gpu::BodyUniforms.

layout(std140, set = UNIFORM_SET, binding = 1) uniform Body
{
    vec4 direction;  // xyz: toward the body (habitat frame), w: angular radius (rad)
    vec4 towardSun;  // xyz: from the body toward the Sun (habitat frame)
    vec4 sunlight;   // rgb: sunlight on the body, a: albedo scale
    vec4 params;     // x: 1 = has an atmosphere, y: night lights, z: ambient (earthshine)
    mat4 bodyFromHabitat;  // habitat directions -> body-fixed (x: prime meridian, z: north)
}
body;

// How far beyond the disk the atmosphere's glow reaches, in body radii.
const float ATMOSPHERE_REACH = 0.05;
