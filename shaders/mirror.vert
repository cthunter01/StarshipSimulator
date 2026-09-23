#version 450

// The external mirrors, one flat quad per window, built from the habitat uniforms: hinged at the
// anti-sunward end of the window and opened by the mirror angle toward the Sun. Drawn for our
// habitat and for its partner, placed by the Placement transform.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "habitat.glsl"

// Must match StarshipSimulator::gpu::PlacementUniforms: which habitat's mirrors (ours, or the
// partner's) and where, relative to the camera.
layout(std140, set = 1, binding = 2) uniform Placement
{
    mat4 model;  // habitat frame -> camera-relative
}
placement;

layout(location = 0) out vec3 outCameraRelative;
layout(location = 1) out vec3 outNormal;  // reflective face
layout(location = 2) out vec2 outUv;

// A windowless cylinder's mirrors: eight petals hinged round the rim of each glass end, flared out
// like a trumpet. Sunlight arrives from the side, square to the axis; a petal flared 45 degrees
// would turn it straight along the axis, and one flared further sends it in at an angle, down onto
// the land. So by day the flare is 45 degrees plus half the light's elevation; at night they fold
// flat, out of the way of the glass, and the ends show the stars.
//
// A Bernal sphere's axis points at the Sun, so the light arrives along the axis instead: petals
// round each polar window's rim, flared out from the axis by half the light's elevation, turn it
// in across the axis and through the glass; at night they fold flat.
const int PETALS = 8;

void endCapPetal(int index, vec2 uv)
{
    if (index >= 2 * PETALS)
    {
        gl_Position = vec4(0.0);
        return;
    }
    float alpha     = habitat.mirror.x;
    float twilight  = radians(4.0);
    float daylight  = smoothstep(0.0, twilight, alpha) * (1.0 - smoothstep(PI / 2.0 - twilight, PI / 2.0, alpha));
    float elevation = clamp(atan(sin(2.0 * alpha), abs(cos(2.0 * alpha))), radians(15.0), radians(40.0));
    float flare     = mix(PI / 2.0, PI / 4.0 + 0.5 * elevation, daylight);
    if (sphericalHull())
    {
        // The window's latitude from its rim, and the elevation the light is held to (see
        // polarWindowElevation in mirror_optics.cpp).
        float lowest = 0.5 * acos(habitat.light.z / habitat.light.w) + radians(3.0);
        float high   = max(radians(45.0), lowest + radians(5.0));
        elevation    = clamp(atan(sin(2.0 * alpha), abs(cos(2.0 * alpha))), lowest, high);
        flare        = mix(PI / 2.0, 0.5 * elevation, daylight);
    }

    float side    = index < PETALS ? 1.0 : -1.0;  // the +z end, then the -z end
    float glass   = side > 0.0 ? habitat.strips.w : habitat.strips.z;
    float phi     = (float(index % PETALS) + 0.5) * (2.0 * PI / float(PETALS));
    float radius  = sphericalHull() ? habitat.light.z : habitat.shape.x;
    float reach   = 0.9 * radius;
    vec3  outward = vec3(cos(phi), sin(phi), 0.0);
    vec3  tangent = vec3(-sin(phi), cos(phi), 0.0);
    vec3  axis    = vec3(0.0, 0.0, side);
    vec3  along   = sin(flare) * outward + cos(flare) * axis;
    // Widening outward, so neighbours meet edge to edge round the cone (with a hand's breadth
    // between).
    float halfWidth = sin(PI / float(PETALS)) * 0.97 * (radius + uv.y * reach * sin(flare));
    vec3  hinge = radius * outward + vec3(0.0, 0.0, glass + side * 0.5);
    vec3  world = hinge + (uv.x * 2.0 - 1.0) * halfWidth * tangent + uv.y * reach * along;

    outCameraRelative = (placement.model * vec4(world, 1.0)).xyz;
    // The reflective face looks outward and back toward the habitat: into the Sun and the glass.
    // A sphere's look in toward the axis and out along it, into the Sun.
    vec3 face   = sphericalHull() ? sin(flare) * axis - cos(flare) * outward
                                  : cos(flare) * outward - sin(flare) * axis;
    outNormal   = mat3(placement.model) * face;
    outUv       = uv;
    gl_Position = frame.viewProjection * vec4(outCameraRelative, 1.0);
}

// A Stanford torus's mirrors. Over the hub, not turning with the wheel, a round mirror at 45
// degrees faces the Sun (square to the axis) and sends its light down the axis; round the hub a
// ring of mirrors at 45 degrees throws it out along every radius, through the windows in the
// tube's ceiling. Leaning further, they tilt the light across the tube; at night they fold flat.
const int PRIMARY_SLICES    = 8;
const int FACETS_PER_SECTOR = 4;  // round the hub between two spokes

void torusMirror(int index, vec2 uv)
{
    float centre = habitat.light.z;  // the axis to the tube's middle
    float hub    = habitat.mirror.z;
    int   spokes = max(int(habitat.mirror.w + 0.5), 1);
    vec3  world;
    vec3  face;
    if (index < PRIMARY_SLICES)
    {
        // A slice of the round mirror, from its middle to its rim (two corners at the middle).
        vec3  sun    = normalize(vec3(habitat.sun.xy, 0.0) + vec3(1e-6, 0.0, 0.0));
        vec3  normal = normalize(sun - vec3(0.0, 0.0, 1.0));  // turns the sunlight down the axis
        vec3  across = normalize(cross(vec3(0.0, 0.0, 1.0), sun));
        vec3  up     = cross(normal, across);
        float angle  = (float(index) + uv.x) * (2.0 * PI / float(PRIMARY_SLICES));
        world = vec3(0.0, 0.0, 0.85 * centre) + uv.y * 0.3 * centre * (cos(angle) * across + sin(angle) * up);
        face  = normal;
    }
    else
    {
        int facet = index - PRIMARY_SLICES;
        if (facet >= spokes * FACETS_PER_SECTOR)
        {
            gl_Position = vec4(0.0);
            return;
        }
        // Between the spokes, leaving room for them.
        float sector = 2.0 * PI / float(spokes);
        float step   = sector / float(FACETS_PER_SECTOR + 1);
        float angle  = float(facet / FACETS_PER_SECTOR) * sector + step * float(facet % FACETS_PER_SECTOR + 1);
        float theta  = angle + (uv.x * 2.0 - 1.0) * 0.45 * step;
        // Leaning 45 degrees plus half the light's tilt, from face-up: the light coming down the
        // axis goes out along the radius, tilted as much down the other way.
        float alpha    = habitat.mirror.x;
        float twilight = radians(4.0);
        float daylight = smoothstep(0.0, twilight, alpha) * (1.0 - smoothstep(PI / 2.0 - twilight, PI / 2.0, alpha));
        float lean     = mix(0.0, PI / 4.0 + 0.5 * habitat.mirror.y, daylight);
        float span     = 2.0 * habitat.band.w;
        float middle   = hub + 15.0 + 0.5 * span * cos(PI / 4.0);
        vec3  outward  = vec3(cos(theta), sin(theta), 0.0);
        float along    = (uv.y - 0.5) * span;
        world = outward * (middle + along * cos(lean)) - vec3(0.0, 0.0, along * sin(lean));
        face  = outward * sin(lean) + vec3(0.0, 0.0, cos(lean));
        uv.y += 2.0;  // mirror.frag: these see the Sun in the mirror over the hub, up the axis
    }
    outCameraRelative = (placement.model * vec4(world, 1.0)).xyz;
    outNormal         = mat3(placement.model) * face;
    outUv             = uv;
    gl_Position       = frame.viewProjection * vec4(outCameraRelative, 1.0);
}

void main()
{
    int  mirrorIndex = gl_VertexIndex / 6;
    int  corner      = gl_VertexIndex % 6;
    vec2 uv          = vec2(corner == 1 || corner == 2 || corner == 4 ? 1.0 : 0.0,
                            corner == 2 || corner == 4 || corner == 5 ? 1.0 : 0.0);
    if (pointImages())
    {
        endCapPetal(mirrorIndex, uv);
        return;
    }
    if (hubLight())
    {
        torusMirror(mirrorIndex, uv);
        return;
    }
    if (mirrorIndex >= stripCount())
    {
        gl_Position = vec4(0.0);  // degenerate: nothing drawn
        return;
    }
    float angle   = float(mirrorIndex) * habitat.strips.x;
    float alpha   = habitat.mirror.x;
    vec3  outward = vec3(cos(angle), sin(angle), 0.0);
    vec3  tangent = vec3(-sin(angle), cos(angle), 0.0);
    vec3  along   = sin(alpha) * outward + cos(alpha) * vec3(0.0, 0.0, 1.0);
    vec3  hinge   = habitat.shape.x * outward + vec3(0.0, 0.0, habitat.mirror.w);
    vec3  world   = hinge + (uv.x * 2.0 - 1.0) * habitat.mirror.z * tangent + uv.y * habitat.mirror.y * along;

    outCameraRelative = (placement.model * vec4(world, 1.0)).xyz;
    outNormal         = mat3(placement.model) * (-cos(alpha) * outward + sin(alpha) * vec3(0.0, 0.0, 1.0));
    outUv             = uv;
    gl_Position       = frame.viewProjection * vec4(outCameraRelative, 1.0);
}
