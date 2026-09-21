#version 450

// Rain, seen from inside it: layers of streaks at a few distances from the camera, so they slide
// past each other as you walk. In a spinning habitat rain does not fall straight down: Coriolis
// pushes a falling drop antispinward until drag balances it, a few degrees off the vertical.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "noise.glsl"

layout(location = 0) in vec2 inNdc;

layout(location = 0) out vec4 outColor;

const int   LAYERS    = 3;
const float FALL_MS   = 9.0;   // terminal speed of a raindrop
const float STREAK_M  = 0.9;   // how long the eye smears one
const float SPACING_M = 2.2;   // between drops down a column

// One layer of drops around the point p: columns across the fall, drops spaced along it.
float streaks(vec3 p, vec3 fall, vec3 side, vec3 other, float acrossM, float seconds, float seed)
{
    vec2  across = vec2(dot(p, side), dot(p, other)) / acrossM;
    vec2  id     = floor(across);
    vec2  centre = vec2(hash12(id + seed), hash12(id + seed + 3.7)) - 0.5;
    float radius = length(fract(across) - 0.5 - centre) * acrossM;
    float core   = 1.0 - smoothstep(0.006, 0.024, radius);
    if (core <= 0.0)
    {
        return 0.0;
    }
    float along = ((dot(p, fall) - (seconds * FALL_MS)) / SPACING_M) + hash12(id + seed + 9.1);
    float phase = fract(along);
    return core * (1.0 - smoothstep(0.0, STREAK_M / SPACING_M, phase));
}

void main()
{
    float rain = habitat.weather.x;
    if (rain <= 0.01)
    {
        discard;
    }
    vec4 far  = frame.inverseViewProjection * vec4(inNdc, 1.0, 1.0);
    vec3 view = normalize(far.xyz / far.w);
    vec3 eye  = frame.cameraPosition.xyz;

    // The frame the rain falls in: down (away from the axis), tilted antispinward, and two
    // directions across it.
    vec3  up        = localUp(eye);
    float radius    = max(length(eye.xy), 1.0);
    vec3  antispin  = vec3(eye.y, -eye.x, 0.0) / radius;
    vec3  fall      = normalize(-up + (0.09 * antispin));
    vec3  side      = normalize(cross(fall, vec3(0.0, 0.0, 1.0)));
    vec3  other     = cross(fall, side);

    // Under the deck only, and thinner right under it (the drops have not spread out yet).
    float depth = clamp((radius - habitat.cloud.y) / 150.0, 0.0, 1.0);
    if (depth <= 0.0)
    {
        discard;
    }

    float seen = 0.0;
    float near = -1.0;
    for (int layer = 0; layer < LAYERS; ++layer)
    {
        float distance = 2.2 * pow(3.0, float(layer));
        float acrossM  = 0.13 * distance;  // wider columns further off: about as many on screen
        float drop     = streaks(eye + (view * distance), fall, side, other, acrossM,
                                 frame.time.x, 11.0 * float(layer));
        if (drop > 0.0)
        {
            seen = max(seen, drop * (1.0 - (0.22 * float(layer))));
            near = near < 0.0 ? distance : near;
        }
    }
    float alpha = seen * depth * smoothstep(0.02, 0.5, rain) * 0.9;
    if (alpha < 0.004)
    {
        discard;
    }
    // A drop is a lens: it shows the bright sky, wherever it is in front of.
    vec3 color   = mistLight() * 1.6;
    vec4 clip    = frame.viewProjection * vec4(view * near, 1.0);
    gl_FragDepth = clip.z / clip.w;
    outColor     = vec4(color * alpha, alpha);
}
