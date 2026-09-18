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
}
habitat;

const float PI = 3.14159265358979;

int stripCount() { return int(habitat.strips.y + 0.5); }

// |a - b| wrapped to [0, pi].
float angularDistance(float a, float b) { return abs(mod(a - b + PI, 2.0 * PI) - PI); }

// Distance (m) around the hull from p to the nearest window strip (0 over a window).
float distanceToWindow(vec3 p)
{
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

// Radiance of the sunlit air itself (light scattered toward the viewer).
vec3 airLight() { return habitat.sunColor.rgb * habitat.atmosphere.w * 0.42 + vec3(0.0004, 0.0005, 0.0009); }

struct Haze
{
    vec3 transmittance;
    vec3 inscatter;
};

// Aerial perspective between the camera and a point inside the habitat.
Haze aerialPerspective(vec3 camera, vec3 point)
{
    vec3 transmittance = exp(-extinction() * densityPath(camera, point, 8));
    return Haze(transmittance, airLight() * (1.0 - transmittance));
}

// ---- Sunlight through the windows --------------------------------------------------------------
// Fraction of beam i reaching point p: the ray toward the sun image must leave through window i,
// between the mirror's hinge and the end of the window. (Terrain shadows are not modelled yet.)
float beamAperture(vec3 p, vec3 towardSun, int i)
{
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

// Direct sunlight arriving at p with normal n, from all the mirrors.
vec3 sunlight(vec3 p, vec3 n)
{
    vec3 light = vec3(0.0);
    for (int i = 0; i < stripCount(); ++i)
    {
        vec3  towardSun = habitat.beams[i].xyz;
        float intensity = habitat.beams[i].w;
        float facing    = dot(n, towardSun);
        if (intensity <= 0.0 || facing <= 0.0)
        {
            continue;
        }
        float aperture = beamAperture(p, towardSun, i);
        if (aperture <= 0.0)
        {
            continue;
        }
        // Light crossing the habitat's air picks up a warm tint on long paths.
        vec3 beamTransmittance = exp(-extinction() * densityPath(p, p + towardSun * 6000.0, 4));
        light += habitat.sunColor.rgb * (intensity * facing * aperture) * beamTransmittance;
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
