// The land's colours, in the warm palette of the 1970s NASA habitat paintings. Include habitat.glsl
// and noise.glsl first.

// Linear albedos.
const vec3 MEADOW  = vec3(0.105, 0.200, 0.050);
const vec3 SPROUTS = vec3(0.180, 0.270, 0.060);
const vec3 WHEAT   = vec3(0.420, 0.320, 0.110);
const vec3 SOIL    = vec3(0.170, 0.105, 0.055);
const vec3 ORCHARD = vec3(0.075, 0.150, 0.045);
const vec3 BLOSSOM = vec3(0.330, 0.250, 0.300);
const vec3 FOREST  = vec3(0.040, 0.085, 0.030);
const vec3 HEDGE   = vec3(0.035, 0.070, 0.025);
const vec3 STONE   = vec3(0.360, 0.340, 0.300);
const vec3 ROCK    = vec3(0.170, 0.150, 0.130);
const vec3 METAL   = vec3(0.450, 0.460, 0.480);
const vec3 LUSH    = vec3(0.070, 0.150, 0.038);
const vec3 SAND    = vec3(0.230, 0.205, 0.150);
const vec3 SILT    = vec3(0.090, 0.085, 0.055);

vec3 cropColor(float pick)
{
    if (pick < 0.30) return MEADOW;
    if (pick < 0.50) return SPROUTS;
    if (pick < 0.72) return WHEAT;
    if (pick < 0.84) return SOIL;
    if (pick < 0.97) return ORCHARD;
    return BLOSSOM;
}

// Fades out detail of the given size (m) once a pixel covers more than about half of it.
float detail(float footprint, float size) { return 1.0 - smoothstep(0.25 * size, 0.6 * size, footprint); }

// woods and wetness (lush meadows near water) come from the land-cover map, 0..1.
vec3 valleyAlbedo(vec2 uv, vec3 p, float woods, float wetness)
{
    float footprint = length(fwidth(uv));

    // Patchwork fields, their grid gently warped so it doesn't look machine made.
    vec2  warped = uv + 60.0 * vec2(fbm2(uv / 900.0), fbm2(uv / 900.0 + 7.3));
    vec2  cell   = warped / vec2(170.0, 240.0);
    vec2  id     = floor(cell);
    float crop   = hash12(id);
    vec3  field  = cropColor(crop) * (0.85 + 0.3 * valueNoise(uv / 18.0));

    // Close up: grass tufts, and furrows or crop rows running along each field.
    field *= 1.0 + 0.18 * detail(footprint, 3.0) * (valueNoise(uv / 1.3) - 0.5);
    field *= 1.0 + 0.12 * detail(footprint, 0.6) * (valueNoise(uv / 0.35) - 0.5);
    if (crop >= 0.30 && crop < 0.84)
    {
        float rows = 0.5 + 0.5 * sin(warped.x * (2.0 * PI / 1.1));
        field *= 1.0 - 0.25 * detail(footprint, 1.1) * rows;
    }

    // Hedgerows along field edges, fading out where they would be smaller than a pixel.
    float hedge = gridLines(cell, 0.03);
    vec3  color = mix(field, HEDGE, hedge * 0.8);

    // Meadows beside the rivers and lakes, left unploughed.
    float meadow = smoothstep(0.45, 0.8, wetness + 0.15 * valueNoise(uv / 70.0));
    vec3  grass  = LUSH * (0.75 + 0.5 * valueNoise(uv / 23.0)) * (0.9 + 0.2 * valueNoise(uv / 4.1));
    color        = mix(color, mix(grass, MEADOW, 0.4 * valueNoise(uv / 140.0)), meadow);

    // Close up: clumps of grass and scattered wildflowers in the meadows and pastures.
    float near   = detail(footprint, 0.5);
    color       *= 1.0 + 0.3 * near * (valueNoise(uv / 0.45) - 0.5);
    vec2  flower = floor(uv / 0.35);
    vec2  centre = vec2(hash12(flower + 3.3), hash12(flower + 5.9)) * 0.25 + 0.05;  // in the cell
    float petal  = 1.0 - smoothstep(0.018, 0.03, length(uv - flower * 0.35 - centre));
    float bloom  = step(0.93, hash12(flower)) * petal * near * max(meadow, step(crop, 0.30));
    vec3  petals = hash12(flower + 7.1) < 0.5 ? vec3(0.55, 0.52, 0.40) : vec3(0.50, 0.38, 0.08);
    color        = mix(color, petals, bloom * 0.8);

    // Woods and copses: tree tops, clumped.
    float forest = smoothstep(0.3, 0.7, woods);
    float canopy = 0.8 + 0.4 * mix(0.5, valueNoise(uv / 9.0), detail(footprint, 9.0));
    color        = mix(color, FOREST * canopy, forest);

    // A stone promenade along the windows, where the terrain is flat.
    return mix(STONE, color, smoothstep(22.0, 32.0, distanceToWindow(p)));
}

vec3 endcapAlbedo(vec3 p, vec3 n, float woods)
{
    float footprint = length(fwidth(p));
    float slope     = 1.0 - dot(n, localUp(p));  // 0 flat, 1 vertical
    vec3  grass     = mix(MEADOW, SPROUTS, valueNoise3(p / 120.0));
    vec3  color     = mix(grass, FOREST, smoothstep(0.3, 0.7, woods));
    // Terraced gardens on the slopes: alternate green and stone bands every 40 m of height.
    float terrace   = detail(footprint, 40.0) * smoothstep(0.35, 0.65, fract(length(p.xy) / 40.0));
    color           = mix(color, color * 0.75 + STONE * 0.1, terrace * 0.5);
    float rockMask  = smoothstep(0.28, 0.40, slope + 0.12 * valueNoise3(p / 60.0));
    float grain     = mix(0.5, valueNoise3(p / 15.0), detail(footprint, 15.0));
    return mix(color, ROCK * (0.8 + 0.4 * grain), rockMask);
}

// Shores and river beds: sand at the waterline, silt deeper down. depth: below the water level (m).
vec3 shoreAlbedo(vec3 land, vec2 uv, float depth)
{
    vec3 bed = mix(SAND, SILT, smoothstep(0.05, 0.7, depth)) * (0.85 + 0.3 * valueNoise(uv / 3.0));
    return mix(land, bed, smoothstep(-0.18, 0.02, depth));
}
