// The level-of-detail terrain: uniform slot 2. Must match StarshipSimulator::gpu::LandscapeUniforms.
// Include frame.glsl and habitat.glsl first. The terrain is a height field on a grid over the
// habitat's surface: cell (column, row) lies at theta = column * grid.w and profile arc length
// u = row * grid.z; heights are measured toward the axis from the profile surface.

layout(std140, set = UNIFORM_SET, binding = 2) uniform Landscape
{
    vec4 grid;      // x: columns, y: rows, z: metres per row, w: 2 pi / columns
    vec4 heights;   // x: height at texel value 0, y: at 1, z: water level, w: floor radius
    vec4 morph[12];  // per level: x start (m), y end, z 1 / (end - start)
    vec4 mode;       // x: 0 = ground, 1 = water surface; y: water pass (0 dims, 1 adds)
    vec4 extent;     // x, y: z range of the profile (for arcByZ); z: 1 when the water lies at the
                     // radius w below a floor that is not level (a sphere), else 0
}
landscape;

// The water's surface on a row of the grid, measured like the heights: heights.z on a level floor;
// where the floor rises away from the water (a sphere) the level surface is a cylinder of radius
// extent.w, deeper under the floor wherever the floor is nearer the axis.
float waterLevel(vec4 profile)
{
    return landscape.heights.z + landscape.extent.z * (profile.y - landscape.extent.w);
}

float decodeHeight(float unorm) { return mix(landscape.heights.x, landscape.heights.y, unorm); }
vec2  heightUv(vec2 cell) { return (cell + 0.5) / landscape.grid.xy; }
vec2  profileUv(float row) { return vec2((row + 0.5) / landscape.grid.y, 0.5); }

const float SUN_ANGULAR_RADIUS = 0.0047;

// Soft terrain shadow toward the sun image along towardSun, from a point on the ground: marches
// the ray through the height field (in steps growing with distance) and keeps the smallest
// clearance over the ground relative to the width of the sun's penumbra there. 1 = fully lit.
float terrainShadow(sampler2D heightMap, sampler2D profileMap, sampler2D arcMap, vec3 p,
                    vec3 towardSun, float start)
{
    float lit = 1.0;
    float t   = start;
    for (int k = 0; k < 26 && t < 6000.0; ++k)
    {
        vec3 q = p + towardSun * t;
        if (q.z <= landscape.extent.x || q.z >= landscape.extent.y)
        {
            break;  // out through an end of the habitat
        }
        if (torusTube() && !inTube(q))
        {
            break;  // up through a torus's ceiling (which shades it or not: beamAperture)
        }
        float u       = textureLod(arcMap, vec2((q.z - landscape.extent.x) /
                                                    (landscape.extent.y - landscape.extent.x),
                                                0.5), 0.0).r;
        float row     = u / landscape.grid.z;
        float column  = atan(q.y, q.x) / landscape.grid.w;
        float groundR = textureLod(profileMap, profileUv(row), 0.0).y -
                        decodeHeight(textureLod(heightMap, heightUv(vec2(column, row)), 0.0).r);
        float clearance = groundR - length(q.xy);  // above the ground (toward the axis) when > 0
        lit = min(lit, clamp(0.5 + clearance / (2.0 * t * SUN_ANGULAR_RADIUS + 0.3), 0.0, 1.0));
        if (lit <= 0.0)
        {
            break;
        }
        t = t * 1.35 + 0.5;
    }
    return lit;
}
