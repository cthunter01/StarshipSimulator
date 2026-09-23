// The habitat's shape, lighting and air: uniform slot 1. Must match
// StarshipSimulator::gpu::HabitatUniforms. Include frame.glsl first.
// Habitat frame: spin axis +Z, the Sun toward +Z; window i is centred on angle i * stripAngle.

layout(std140, set = UNIFORM_SET, binding = 1) uniform Habitat
{
    vec4 shape;     // x: hull radius, y: floor z min, z: floor z max, w: window half-angle
    vec4 strips;    // x: strip angle, y: strip count, z: habitat z min, w: habitat z max
    vec4 beams[6];  // xyz: toward the sun image seen through window i, w: intensity
    vec4 sunColor;  // rgb: full sunlight
    vec4 ambientUp;
    vec4 ambientDown;
    vec4 atmosphere;  // x: density falloff k (1/m^2), y: pressure / 1 atm, z: haze, w: daylight
    vec4 mirror;      // x: opening angle, y: mirror length, z: half width, w: hinge z
    vec4 sun;         // xyz: direction to the Sun, w: angular radius
    vec4 cloud;    // x: radius of the cloud deck's top, y: its base, z: cover, w: light let through
    vec4 weather;  // x: rain, y: mist, z: wetness of the ground, w: cloud drift along the axis (m)
    vec4 season;   // x: fresh green, y: autumn gold, z: blossom, w: how far the clouds have turned
    vec4 light;    // x: 1 when the sun images are points (beams' xyz), 0 directions; y: how many
    vec4 band;     // x: 1 when the land runs round the axis, y: the round's length (m)
}
habitat;

const float PI = 3.14159265358979;

int stripCount() { return int(habitat.strips.y + 0.5); }

// Land running round the axis: its arc coordinate starts again after one turn, so the patterns
// drawn on it have to wrap too.
bool landRunsRound() { return habitat.band.x > 0.5; }

// Sun images at points: the light through a windowless cylinder's glass end caps.
bool pointImages() { return habitat.light.x > 0.5; }

// How many sun images light the habitat: one per window strip, or one per glass end cap.
int beamCount() { return pointImages() ? int(habitat.light.y + 0.5) : stripCount(); }

// The direction toward sun image i, seen from p.
vec3 beamDirection(vec3 p, int i)
{
    return pointImages() ? normalize(habitat.beams[i].xyz - p) : habitat.beams[i].xyz;
}

// |a - b| wrapped to [0, pi].
float angularDistance(float a, float b) { return abs(mod(a - b + PI, 2.0 * PI) - PI); }

// Distance (m) around the hull from p to the nearest window strip (0 over a window).
float distanceToWindow(vec3 p)
{
    if (habitat.strips.y < 0.5)
    {
        return 1e6;  // no window strips along the hull
    }
    float phase = mod(atan(p.y, p.x) + habitat.shape.w, habitat.strips.x);  // window: [0, 2 hw)
    float edge  = min(phase - 2.0 * habitat.shape.w, habitat.strips.x - phase);
    return max(edge, 0.0) * habitat.shape.x;
}

// Up (toward the spin axis) at a position.
vec3 localUp(vec3 p)
{
    float r = length(p.xy);
    return r > 1e-3 ? vec3(-p.xy / r, 0.0) : vec3(1.0, 0.0, 0.0);
}

// ---- Air ---------------------------------------------------------------------------------------
// Isothermal air in spin gravity thins toward the axis: density ~ exp(-k (R^2 - r^2)).
// Rayleigh scattering (blue) plus a little Mie haze (white), per metre at floor density.
const vec3 RAYLEIGH = vec3(5.8e-6, 13.5e-6, 33.1e-6);
const vec3 MIE      = vec3(3.0e-6);

float airDensity(vec3 p)
{
    float R = habitat.shape.x;
    return exp(-habitat.atmosphere.x * (R * R - dot(p.xy, p.xy)));
}

// Density-weighted length (m) of the segment a -> b (midpoint rule).
float densityPath(vec3 a, vec3 b, int samples)
{
    float sum = 0.0;
    for (int i = 0; i < samples; ++i)
    {
        sum += airDensity(mix(a, b, (float(i) + 0.5) / float(samples)));
    }
    return sum * length(b - a) / float(samples);
}

vec3 extinction() { return (RAYLEIGH + MIE) * habitat.atmosphere.y * habitat.atmosphere.z; }

// ---- Mist and rain -----------------------------------------------------------------------------
// Water in the air: mist lying in the lowest tens of metres, and rain filling everything under the
// cloud deck. Grey, so it whitens what it hides instead of colouring it.
const vec3 WATER_HAZE = vec3(2.6e-3);  // per metre at full mist

float mistDensity(vec3 p)
{
    float radius  = length(p.xy);
    float above   = max(habitat.shape.x - radius, 0.0);  // roughly metres above the floor
    float lying   = habitat.weather.y * exp(-above / 40.0);
    float falling = 0.30 * habitat.weather.x *
                    smoothstep(habitat.cloud.y - 150.0, habitat.cloud.y + 50.0, radius);
    return lying + falling;
}

// Mist-weighted length (m) of the segment a -> b (midpoint rule).
float mistPath(vec3 a, vec3 b, int samples)
{
    if (habitat.weather.x + habitat.weather.y <= 0.0)
    {
        return 0.0;
    }
    float sum = 0.0;
    for (int i = 0; i < samples; ++i)
    {
        sum += mistDensity(mix(a, b, (float(i) + 0.5) / float(samples)));
    }
    return sum * length(b - a) / float(samples);
}

// Radiance of the sunlit air itself (light scattered toward the viewer).
// Radiance of the sunlit air itself (light scattered toward the viewer), averaged over directions.
vec3 airLight() { return habitat.sunColor.rgb * habitat.atmosphere.w * 0.42 + vec3(0.0004, 0.0005, 0.0009); }

// How the air's glow depends on the direction you look: Rayleigh scattering (blue, gentle) and a
// forward Mie peak (white) around each sun image, relative to the average over all directions.
const float MIE_G = 0.76;

vec3 phaseWeight(vec3 view, vec3 from)
{
    vec3  weight = vec3(0.0);
    float lit    = 0.0;
    vec3  beta   = RAYLEIGH + MIE;
    for (int i = 0; i < beamCount(); ++i)
    {
        float intensity = habitat.beams[i].w;
        if (intensity <= 0.0)
        {
            continue;
        }
        float mu       = dot(view, beamDirection(from, i));
        float rayleigh = 0.75 * (1.0 + mu * mu);
        float mie      = (1.0 - MIE_G * MIE_G) / pow(1.0 + MIE_G * MIE_G - 2.0 * MIE_G * mu, 1.5);
        weight += intensity * (RAYLEIGH * rayleigh + MIE * mie) / beta;
        lit += intensity;
    }
    return lit > 0.0 ? weight / lit : vec3(1.0);
}

struct Haze
{
    vec3 transmittance;
    vec3 inscatter;
};

// The mist's own glow: the whole sky lights it, so it has no direction and no colour of its own.
vec3 mistLight() { return habitat.sunColor.rgb * habitat.atmosphere.w * 0.30 + vec3(0.0006); }

// Aerial perspective between the camera and a point inside the habitat.
Haze aerialPerspective(vec3 camera, vec3 point)
{
    vec3  air           = extinction() * densityPath(camera, point, 8);
    float water         = WATER_HAZE.g * mistPath(camera, point, 8);
    vec3  transmittance = exp(-(air + vec3(water)));
    vec3  view          = normalize(point - camera);
    float grey          = water / max(water + air.g, 1e-7);
    vec3  glow          = mix(airLight() * phaseWeight(view, camera), mistLight(), grey);
    return Haze(transmittance, glow * (1.0 - transmittance));
}

// ---- Sunlight through the windows --------------------------------------------------------------
// Fraction of beam i reaching point p: the ray toward the sun image must leave through window i,
// between the mirror's hinge and the end of the window. (Terrain and tree shadows are separate.)
float beamAperture(vec3 p, vec3 towardSun, int i)
{
    if (pointImages())
    {
        return 1.0;  // an image on the axis beyond the glass shines through it from anywhere inside
    }
    float R = habitat.shape.x;
    float a = dot(towardSun.xy, towardSun.xy);
    if (a < 1e-8)
    {
        return 0.0;
    }
    float b    = dot(p.xy, towardSun.xy);
    float c    = dot(p.xy, p.xy) - R * R;
    float disc = b * b - a * c;
    if (disc < 0.0)
    {
        return 0.0;
    }
    vec3  exitPoint = p + towardSun * ((-b + sqrt(disc)) / a);
    float off       = angularDistance(atan(exitPoint.y, exitPoint.x), float(i) * habitat.strips.x);
    float across    = 1.0 - smoothstep(habitat.shape.w - 0.004, habitat.shape.w, off);
    float along     = smoothstep(habitat.shape.y, habitat.shape.y + 40.0, exitPoint.z) *
                  (1.0 - smoothstep(habitat.shape.z - 40.0, habitat.shape.z, exitPoint.z));
    return across * along;
}

// How far a beam has come through the habitat's air to reach p: from the far side, or from the
// glass end cap it came in through.
float beamPathLength(vec3 p, vec3 towardSun)
{
    if (pointImages())
    {
        float glass = towardSun.z > 0.0 ? habitat.strips.w : habitat.strips.z;
        return clamp((glass - p.z) / max(abs(towardSun.z), 1e-3) * sign(towardSun.z), 0.0, 6000.0);
    }
    return 6000.0;
}

// Direct sunlight from beam i arriving at p with normal n, before terrain shadows (zero where the
// beam cannot reach p through its window).
vec3 beamLight(vec3 p, vec3 n, int i)
{
    vec3  towardSun = beamDirection(p, i);
    float intensity = habitat.beams[i].w;
    float facing    = dot(n, towardSun);
    if (intensity <= 0.0 || facing <= 0.0)
    {
        return vec3(0.0);
    }
    float aperture = beamAperture(p, towardSun, i);
    if (aperture <= 0.0)
    {
        return vec3(0.0);
    }
    // Light crossing the habitat's air picks up a warm tint on long paths.
    vec3 beamTransmittance = exp(-extinction() * densityPath(p, p + towardSun * beamPathLength(p, towardSun), 4));
    return habitat.sunColor.rgb * (intensity * facing * aperture) * beamTransmittance;
}

// Direct sunlight arriving at p with normal n, from all the mirrors (no terrain shadows).
vec3 sunlight(vec3 p, vec3 n)
{
    vec3 light = vec3(0.0);
    for (int i = 0; i < beamCount(); ++i)
    {
        light += beamLight(p, n, i);
    }
    return light;
}

// Light from the far side of the habitat overhead and bounced from the ground.
vec3 ambientLight(vec3 p, vec3 n)
{
    float upness = dot(n, localUp(p)) * 0.5 + 0.5;
    // At night Earthlight through the windows and the glow of towns across the habitat keep the
    // land just visible.
    const vec3 NIGHT_GLOW = vec3(0.0020, 0.0021, 0.0026);
    return mix(habitat.ambientDown.rgb, habitat.ambientUp.rgb, upness) + NIGHT_GLOW;
}
