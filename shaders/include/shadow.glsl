// The trees' shadow map: an orthographic depth map along one mirror's beam, around the camera.
// Must match StarshipSimulator::gpu::ShadowUniforms. Define UNIFORM_SET (3), SHADOW_UNIFORM (its
// uniform slot) and SHADOW_SAMPLER (its fragment sampler binding) before including.

layout(std140, set = UNIFORM_SET, binding = SHADOW_UNIFORM) uniform Shadow
{
    mat4 lightFromCameraRelative;  // camera-relative position -> shadow clip space (z 0..1)
    vec4 params;                   // x: 1 = in use, y: window whose beam, z: texel (m), w: bias
}
shadow;

layout(set = 2, binding = SHADOW_SAMPLER) uniform sampler2D shadowMap;

// How much of beam `beam` gets past the trees to a point (camera-relative, with its normal):
// 1 = fully lit. Outside the map's box, and for other beams, nothing is in the way.
float treeShadow(vec3 cameraRelative, vec3 normal, int beam)
{
    if (shadow.params.x < 0.5 || beam != int(shadow.params.y + 0.5))
    {
        return 1.0;
    }
    vec4  c    = shadow.lightFromCameraRelative * vec4(cameraRelative + normal * (2.0 * shadow.params.z), 1.0);
    float edge = max(abs(c.x), abs(c.y));
    if (edge >= 1.0 || c.z <= 0.0 || c.z >= 1.0)
    {
        return 1.0;
    }
    // 4 x 4 depth comparisons weighted like bilinear filtering: smooth edges over three texels.
    vec2  size  = vec2(textureSize(shadowMap, 0));
    vec2  at    = vec2(c.x * 0.5 + 0.5, 0.5 - c.y * 0.5) * size - 0.5;
    vec2  f     = fract(at);
    vec2  base  = floor(at) + 0.5;
    float depth = c.z - shadow.params.w;
    float lit   = 0.0;
    for (int y = -1; y <= 2; ++y)
    {
        float wy = y == -1 ? 1.0 - f.y : (y == 2 ? f.y : 1.0);
        for (int x = -1; x <= 2; ++x)
        {
            float wx   = x == -1 ? 1.0 - f.x : (x == 2 ? f.x : 1.0);
            float seen = depth <= texture(shadowMap, (base + vec2(x, y)) / size).r ? 1.0 : 0.0;
            lit += seen * wx * wy;
        }
    }
    return mix(lit / 9.0, 1.0, smoothstep(0.8, 1.0, edge));
}
