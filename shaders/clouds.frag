#version 450

// The cloud deck over the valleys: a shell of cloud a few hundred metres up, ray-marched through
// the habitat's own air. Looking up you see its flat base; from the axis you look down on its
// tops; from the ground you also see the deck on the far side, hanging under the land overhead.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "clouds.glsl"

// Drawn on the stand-in shell (cloud_shell.vert): the depth test against the land runs before this
// shader, so pixels where the land is in front never march.
layout(early_fragment_tests) in;

layout(location = 0) in vec3 inCameraRelative;

layout(set = 2, binding = 0) uniform sampler2D cloudMap;

layout(location = 0) out vec4 outColor;

const float EXTINCTION = 0.0055;  // per metre at full density
const int   NEAR_STEPS = 40;      // for the first stretch of deck a ray meets
const int   FAR_STEPS  = 10;      // and for the far side's, kilometres away and mostly hidden
const float MAX_RANGE  = 14000.0;
const float MIN_STEP   = 35.0;  // metres: no finer, however near the cloud is

// Where a ray from the camera meets the cylinder of radius r about the axis.
bool cylinder(vec3 origin, vec3 direction, float r, out float near, out float far)
{
    float a    = dot(direction.xy, direction.xy);
    float b    = dot(origin.xy, direction.xy);
    float c    = dot(origin.xy, origin.xy) - (r * r);
    float disc = (b * b) - (a * c);
    if (a < 1e-9 || disc <= 0.0)
    {
        return false;
    }
    float root = sqrt(disc);
    near       = (-b - root) / a;
    far        = (-b + root) / a;
    return true;
}

// Which beams light a stretch of the deck, and how strongly: worked out at the two ends of the
// stretch and blended between them, rather than at every step (the windows' edges pass slowly
// across a cloud, and the aperture test is not cheap).
struct SpanLight
{
    vec3 from;       // the beams' light where the stretch starts
    vec3 to;         // and where it ends
    vec3 towardSun;  // the brightest beam's direction
};

vec3 beamsAt(vec3 p, inout float best, inout vec3 towardSun)
{
    vec3 light = vec3(0.0);
    for (int i = 0; i < stripCount(); ++i)
    {
        float intensity = habitat.beams[i].w;
        if (intensity <= 0.0)
        {
            continue;
        }
        float strength = intensity * beamAperture(p, habitat.beams[i].xyz, i);
        light += habitat.sunColor.rgb * strength;
        if (strength > best)
        {
            best      = strength;
            towardSun = habitat.beams[i].xyz;
        }
    }
    return light;
}

SpanLight spanLight(vec3 from, vec3 to)
{
    SpanLight lit  = SpanLight(vec3(0.0), vec3(0.0), vec3(0.0));
    float     best = 0.0;
    lit.from       = beamsAt(from, best, lit.towardSun);
    lit.to         = beamsAt(to, best, lit.towardSun);
    return lit;
}

// The light scattered out of the deck at a point: sunlight from above, dimmed by the cloud it has
// come through, plus the glow of the far side and of the sunlit land below. The depth below the
// deck's top does most of the work (bright tops, grey bases); one look along the beam adds the
// shadows the taller heaps throw across the deck.
vec3 cloudLight(vec3 p, float height, float lod, SpanLight lit, float along)
{
    vec3 light = vec3(0.0);
    vec3 beams = mix(lit.from, lit.to, along);
    if (dot(beams, beams) > 0.0)
    {
        // Near clouds get the shadows of their neighbours; far off they would not show.
        float beside =
            lod < 2.5 ? cloudCoverage(cloudMap, p + (lit.towardSun * 260.0), lod + 1.0) : 0.5;
        light = beams * exp(-(2.6 * (1.0 - height)) - (0.9 * beside));
    }
    // Ambient: the bright far side above, the land below.
    vec3 ambient = mix(habitat.ambientDown.rgb * 0.9, habitat.ambientUp.rgb * 1.3, height);
    return (light * 0.5) + ambient;
}

void main()
{
    vec3 view = normalize(inCameraRelative);
    vec3 eye  = frame.cameraPosition.xyz;
    if (habitat.cloud.z <= 0.001)
    {
        discard;
    }

    // The shell between the deck's base and top, in up to two stretches along the ray: the near
    // side of the habitat and, beyond the axis, the far side.
    float baseNear = 0.0;
    float baseFar  = 0.0;
    float lowest   = habitat.cloud.y + (0.1 * (habitat.cloud.y - habitat.cloud.x));  // sagging base
    if (!cylinder(eye, view, lowest, baseNear, baseFar) || baseFar <= 0.0)
    {
        discard;
    }
    float topNear = 0.0;
    float topFar  = 0.0;
    bool  through = cylinder(eye, view, habitat.cloud.x, topNear, topFar) && topFar > 0.0;
    vec2  spans[2];
    int   count = 0;
    if (through)
    {
        spans[count++] = vec2(baseNear, topNear);
        spans[count++] = vec2(topFar, baseFar);
    }
    else
    {
        spans[count++] = vec2(baseNear, baseFar);
    }

    // The map's texels are this many metres apart around the habitat; pick a mip from how far
    // away the cloud is, so the distant deck does not shimmer.
    float texelM     = (2.0 * PI * habitat.shape.x) / float(textureSize(cloudMap, 0).x);
    float pixelAngle = max(length(fwidth(view)), 1e-7);

    vec3  scattered    = vec3(0.0);
    float transmitted  = 1.0;
    float depthT       = -1.0;
    float weightedT    = 0.0;
    float weight       = 0.0;
    bool first = true;
    for (int span = 0; span < count; ++span)
    {
        float from = max(spans[span].x, 0.0);
        float to   = min(spans[span].y, MAX_RANGE);
        if (to <= from || transmitted < 0.02)
        {
            continue;
        }
        // Fine steps near the camera, coarse further off, never fewer than a few (a stretch that
        // grazes the deck can be kilometres long). The first stretch met gets most of the budget:
        // the far side's deck is seen through it, small and hazy.
        int budget = first ? NEAR_STEPS : FAR_STEPS;
        first      = false;
        int       steps = clamp(int((to - from) / max(MIN_STEP, 0.015 * from)), 5, budget);
        float     step  = (to - from) / float(steps);
        SpanLight lit   = spanLight(eye + (view * from), eye + (view * to));
        // Dither the first step so the layers do not band. Plain hashed noise, not an ordered
        // pattern: a regular dither shows up as hatching across a still picture.
        uint  hash   = (uint(gl_FragCoord.x) * 1973u) + (uint(gl_FragCoord.y) * 9277u) + 26699u;
        hash         = (hash ^ (hash >> 15)) * 2246822519u;
        hash         = (hash ^ (hash >> 13)) * 3266489917u;
        float jitter = float(hash >> 8) * (1.0 / 16777216.0);
        for (int i = 0; i < steps; ++i)
        {
            float t = from + (step * (float(i) + jitter));
            vec3  p = eye + (view * t);
            float lod = max(log2(max(t * pixelAngle, 1.0) / texelM), 0.0);
            float height;
            float density = cloudDensity(cloudMap, p, lod, height);
            if (density <= 0.001)
            {
                continue;
            }
            float taken = 1.0 - exp(-EXTINCTION * density * step);
            scattered += transmitted * taken *
                         cloudLight(p, height, lod + 1.0, lit, (t - from) / (to - from));
            transmitted *= 1.0 - taken;
            weightedT += transmitted * taken * t;
            weight += transmitted * taken;
            if (depthT < 0.0)
            {
                depthT = t;
            }
            if (transmitted < 0.02)
            {
                break;
            }
        }
    }
    float alpha = 1.0 - transmitted;
    if (alpha < 0.004 || depthT < 0.0)
    {
        discard;
    }

    // Seen through the habitat's air, like everything else.
    float mean  = weight > 0.0 ? weightedT / weight : depthT;
    Haze  haze  = aerialPerspective(eye, eye + (view * mean));
    vec3  color = (scattered * haze.transmittance) + (haze.inscatter * alpha);
    outColor    = vec4(color, alpha);
}
