// Points of light (stars, planets) as small camera-facing quads at infinity, 6 vertices each.
// Include frame.glsl and sky.glsl first.

struct Sprite
{
    vec4 clip;
    vec2 corner;  // -1..1 across the quad
};

// direction: xyz unit vector in EQJ, w: size in pixels.
Sprite starSprite(vec4 direction, int vertex)
{
    int  corner = vertex % 6;
    vec2 offset = vec2(corner == 1 || corner == 2 || corner == 4 ? 1.0 : -1.0,
                       corner == 2 || corner == 4 || corner == 5 ? 1.0 : -1.0);

    vec3 habitatDirection = (sky.habitatFromInertial * vec4(direction.xyz, 0.0)).xyz;
    vec4 clip             = frame.viewProjection * vec4(habitatDirection, 0.0);  // at infinity
    clip.xy += offset * direction.w * frame.viewport.zw * clip.w;
    return Sprite(clip, offset);
}
