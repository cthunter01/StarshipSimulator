#version 450

// The terrain (or, in water mode, the water surface at its level) as square patches of one grid mesh, placed and scaled per instance by the
// level-of-detail quadtree (CDLOD). Vertices take their height from the height map and slide onto
// the next coarser grid as they near the edge of their level's distance band, so nothing pops.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "habitat.glsl"
#include "landscape.glsl"

layout(location = 0) in vec2 inGrid;  // 0..quads on each side

layout(set = 0, binding = 0) uniform sampler2D heightMap;
layout(set = 0, binding = 1) uniform sampler2D profileMap;  // z, radius, inward normal (z, r)

// Must match StarshipSimulator::TerrainPatch as uploaded by GpuLandscape: column, row, level.
layout(std430, set = 0, binding = 2) readonly buffer Patches
{
    vec4 data[];
}
patches;

layout(location = 0) out vec2 outCell;
layout(location = 1) out vec3 outCameraRelative;

vec3 groundPosition(vec2 cell)
{
    vec4  profile = textureLod(profileMap, profileUv(cell.y), 0.0);
    float h       = landscape.mode.x > 0.5 ? landscape.heights.z  // the water surface
                                           : decodeHeight(textureLod(heightMap, heightUv(cell), 0.0).r);
    float theta   = cell.x * landscape.grid.w;
    float r       = profile.y - h;
    return vec3(r * cos(theta), r * sin(theta), profile.x);
}

void main()
{
    vec4  tile    = patches.data[gl_InstanceIndex];
    float spacing = exp2(tile.z);  // cells per quad
    vec4  morph   = landscape.morph[int(tile.z + 0.5)];

    vec2  cell = tile.xy + inGrid * spacing;
    float k    = clamp((distance(groundPosition(cell), frame.cameraPosition.xyz) - morph.x) * morph.z,
                       0.0, 1.0);
    // Odd vertices slide onto their even neighbours: at k = 1 this is the coarser level's grid.
    cell = tile.xy + (inGrid - mod(inGrid, 2.0) * k) * spacing;

    outCell           = cell;
    outCameraRelative = groundPosition(cell) - frame.cameraPosition.xyz;
    gl_Position       = frame.viewProjection * vec4(outCameraRelative, 1.0);
}
