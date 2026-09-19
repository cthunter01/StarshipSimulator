// Light arriving at a surface in the habitat: every mirror's beam, shadowed by the hills
// (terrainShadow, landscape.glsl) and by trees and buildings (treeShadow, shadow.glsl), plus the
// glow of the far side overhead. Include habitat.glsl, landscape.glsl and shadow.glsl first.

vec3 surfaceLight(sampler2D heights, sampler2D profiles, sampler2D arcs, vec3 p, vec3 n,
                  vec3 cameraRelative)
{
    vec3 direct = vec3(0.0);
    for (int i = 0; i < stripCount(); ++i)
    {
        vec3 beam = beamLight(p, n, i);
        if (max(beam.r, max(beam.g, beam.b)) > 0.0)
        {
            beam *= terrainShadow(heights, profiles, arcs, p, habitat.beams[i].xyz,
                                  0.5 + 0.004 * length(cameraRelative)) *
                    treeShadow(cameraRelative, n, i);
        }
        direct += beam;
    }
    return direct + ambientLight(p, n);
}

// How far the lights are on: people switch them on as the mirrors close for the night.
float lightsOn() { return 1.0 - smoothstep(0.04, 0.3, habitat.atmosphere.w); }

// Warm lamplight.
const vec3 LAMPLIGHT = vec3(1.0, 0.72, 0.42);
