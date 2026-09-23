// The deck of cloud over the valleys: a map wrapped once round the habitat and once along it
// (StarshipSimulator::CloudMap), with the cover and the wind's drift from the habitat uniforms.
// The deck lies between the radii habitat.cloud.x (its top, nearest the axis) and habitat.cloud.y
// (its base). Include habitat.glsl first.

// Where the cloud map is read for a point in the habitat, drifting with the wind.
vec2 cloudUv(vec3 p)
{
    float along = max(habitat.strips.w - habitat.strips.z, 1.0);
    return vec2((atan(p.y, p.x) + habitat.season.w) / (2.0 * PI),
                (p.z + habitat.weather.w - habitat.strips.z) / along);
}

// Cover from a texel of the map: the big shapes, roughened by the detail.
float coverFrom(vec2 m, float z)
{
    float shape = m.x + 0.35 * (m.y - 0.5);
    float open  = 1.0 - habitat.cloud.z;
    // The clouds thin out toward the ends of the land, where the endcaps (or the poles) are.
    float fade = habitat.band.z;
    float ends = smoothstep(habitat.shape.y - fade, habitat.shape.y + 1.5 * fade, z) *
                 (1.0 - smoothstep(habitat.shape.z - 1.5 * fade, habitat.shape.z + fade, z));
    return smoothstep(open - 0.02, open + 0.09, shape) * ends;
}

// How much cloud stands over a point, 0..1, whatever its height: what a sunbeam has to pass.
float cloudCoverage(sampler2D cloudMap, vec3 p, float lod)
{
    if (habitat.cloud.z <= 0.001)
    {
        return 0.0;
    }
    return coverFrom(textureLod(cloudMap, cloudUv(p), lod).rg, p.z);
}

// The density of cloud at a point in the deck (0 outside it). `top` comes back as how high the
// sample sits in the deck (0 at the base, 1 at the top), for shading.
float cloudDensity(sampler2D cloudMap, vec3 p, float lod, out float height)
{
    float radius = length(p.xy);
    float deck   = max(habitat.cloud.y - habitat.cloud.x, 1.0);
    if (radius > habitat.cloud.y + (0.1 * deck) || radius < habitat.cloud.x)
    {
        height = 0.0;
        return 0.0;
    }
    vec2 m = textureLod(cloudMap, cloudUv(p), lod).rg;
    // The base is not a perfect cylinder: it rises and falls by a tenth of the deck, so its far
    // horizon is ragged rather than a drawn line.
    float lift = 0.2 * deck * (m.y - 0.5);
    height     = clamp(((habitat.cloud.y + lift) - radius) / deck, 0.0, 1.0);  // 0 base, 1 top
    if (height <= 0.0 || height >= 1.0)
    {
        return 0.0;
    }
    float cover = coverFrom(m, p.z);
    if (cover <= 0.0)
    {
        return 0.0;
    }
    // Flat bases and piled-up tops: the thicker the cloud, the higher it climbs.
    float detail = textureLod(cloudMap, cloudUv(p) * 4.0, lod + 2.0).g;  // 4x finer: two mips
    float top    = clamp(0.25 + (0.85 * cover) + (0.25 * (detail - 0.5)), 0.15, 1.0);
    float shape  = smoothstep(0.0, 0.07, height) * (1.0 - smoothstep(top - 0.3, top, height));
    return cover * shape;
}

// How much of a sunbeam reaches a point under the deck: one look at the cloud where the beam
// crosses the middle of it.
float cloudShade(sampler2D cloudMap, vec3 p, vec3 towardSun)
{
    if (habitat.cloud.z <= 0.001)
    {
        return 1.0;
    }
    float middle = 0.5 * (habitat.cloud.x + habitat.cloud.y);
    if (length(p.xy) < middle)
    {
        return 1.0;  // already above the deck
    }
    // Where the beam crosses the cylinder of that radius (toward the axis, so the far root).
    float a = dot(towardSun.xy, towardSun.xy);
    float b = dot(p.xy, towardSun.xy);
    float c = dot(p.xy, p.xy) - (middle * middle);
    float disc = (b * b) - (a * c);
    if (a < 1e-8 || disc <= 0.0)
    {
        return 1.0;
    }
    float t = (-b + sqrt(disc)) / a;
    if (t <= 0.0)
    {
        return 1.0;
    }
    // A wide sample: the deck is thick, so its shadow edges are soft.
    float cover = cloudCoverage(cloudMap, p + (towardSun * t), 2.0);
    return 1.0 - (0.88 * cover);
}
