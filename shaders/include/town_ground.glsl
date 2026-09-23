// The towns' ground maps (streets, squares, gardens, lamplight), packed in one atlas. Must match
// StarshipSimulator::gpu::GroundAtlas. Define TOWN_ATLAS_BINDING (a fragment sampler) and
// TOWN_RECORDS_BINDING (a fragment storage buffer) before including.

layout(set = 2, binding = TOWN_ATLAS_BINDING) uniform sampler2D townAtlas;
layout(std430, set = 2, binding = TOWN_RECORDS_BINDING) readonly buffer TownRecords
{
    vec4 records[];
}
towns;

// What the ground is at (z, theta), u metres along the floor profile: x the signed distance to
// paving (m, negative on it), y its style (0 street, 1 stone), z how built up (gardens rather than
// fields), w lamplight. Outside the towns: (4, 0, 0, 0). dx and dy: the screen derivatives of
// (theta * floor radius, z, u), taken by the caller in uniform control flow (the lookups here are
// not).
vec4 townGround(float z, float theta, float u, vec3 dx, vec3 dy)
{
    int count = int(towns.records[0].x + 0.5);
    for (int i = 0; i < count; ++i)
    {
        vec4 origin = towns.records[1 + 3 * i];
        vec4 area   = towns.records[2 + 3 * i];
        float around = mod(theta - origin.y + PI, 2.0 * PI) - PI;
        // A plan along a valley is (around, z); one running round the axis (w set) is (toward -z
        // along the floor, around), placed by its arc length u0 in x.
        bool roundBand = origin.w > 0.5;
        vec2 p  = roundBand ? vec2(origin.x - u, around * origin.z) : vec2(around * origin.z, z - origin.x);
        vec2 gx = roundBand ? vec2(-dx.z, dx.x) : dx.xy;
        vec2 gy = roundBand ? vec2(-dy.z, dy.x) : dy.xy;
        if (p.x < area.x || p.y < area.y || p.x > area.z || p.y > area.w)
        {
            continue;
        }
        vec4 atlas = towns.records[3 + 3 * i];
        vec2 uv    = atlas.xy + (p - area.xy) * atlas.zw;
        vec4 t     = textureGrad(townAtlas, uv, gx * atlas.zw, gy * atlas.zw);
        return vec4((t.x - 0.5) * 8.0, t.y, t.z, t.w);
    }
    return vec4(4.0, 0.0, 0.0, 0.0);
}
