#version 450

// The habitat's land: valley farmland, meadows, woods, river banks and endcap slopes. Lit by the
// sun beams from the mirrors and by the glow of the far side overhead, seen through the habitat's
// own air. Normals come from the height map per pixel, filtered with distance.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "landscape.glsl"
#include "noise.glsl"
#include "terrain_colors.glsl"
#define SHADOW_UNIFORM 3
#define SHADOW_SAMPLER 4
#include "shadow.glsl"
#include "clouds.glsl"
#include "lit.glsl"
#define TOWN_ATLAS_BINDING 5
#define TOWN_RECORDS_BINDING 7
#include "town_ground.glsl"

layout(location = 0) in vec2 inCell;
layout(location = 1) in vec3 inCameraRelative;

layout(set = 2, binding = 0) uniform sampler2D heightMap;
layout(set = 2, binding = 1) uniform sampler2D profileMap;  // z, radius, inward normal (z, r)
layout(set = 2, binding = 2) uniform sampler2D coverMap;    // woods, wetness, near a town
layout(set = 2, binding = 3) uniform sampler2D arcMap;      // profile arc length u by z
layout(set = 2, binding = 6) uniform sampler2D cloudMap;   // the cloud deck's cover

layout(location = 0) out vec4 outColor;

float heightAt(vec2 cell, float lod) { return decodeHeight(textureLod(heightMap, heightUv(cell), lod).r); }

void main()
{
    vec4  profile = textureLod(profileMap, profileUv(inCell.y), 0.0);
    float theta   = inCell.x * landscape.grid.w;
    bool  onFloor = profile.x >= habitat.shape.y && profile.x <= habitat.shape.z;
    vec3  p       = inCameraRelative + frame.cameraPosition.xyz;
    if (onFloor && distanceToWindow(p) <= 0.0)
    {
        discard;  // window glass: the glass pass draws it
    }

    // The surface S(column, row) = (r(u) - h) * radial + z(u) * axis, differentiated per cell.
    float footprint = max(length(dFdx(inCell)), length(dFdy(inCell)));
    float lod       = max(log2(footprint), 0.0);
    float step      = exp2(lod);  // continuous, so the filtering has no visible bands
    float h         = heightAt(inCell, 0.0);
    float dhdc = (heightAt(inCell + vec2(step, 0.0), lod) - heightAt(inCell - vec2(step, 0.0), lod)) /
                 (2.0 * step);
    float dhdr = (heightAt(inCell + vec2(0.0, step), lod) - heightAt(inCell - vec2(0.0, step), lod)) /
                 (2.0 * step);
    vec3  radial   = vec3(cos(theta), sin(theta), 0.0);
    vec3  around   = vec3(-sin(theta), cos(theta), 0.0);
    vec2  tangent  = vec2(-profile.w, profile.z);  // (dz/du, dr/du)
    float cellU    = landscape.grid.z;
    vec3  alongRow = radial * (tangent.y * cellU - dhdr) + vec3(0.0, 0.0, tangent.x * cellU);
    vec3  alongCol = around * ((profile.y - h) * landscape.grid.w) - radial * dhdc;
    vec3  n        = normalize(cross(alongRow, alongCol));

    vec4  cover = texture(coverMap, heightUv(inCell));
    vec2  uv    = vec2(theta * landscape.heights.w, inCell.y * cellU);  // metres: around, along
    vec3  albedo = onFloor ? valleyAlbedo(uv, p, cover.r, cover.g)
                           : (torusTube() ? tubeWallAlbedo(p, n, cover.r) : endcapAlbedo(p, n, cover.r));
    // Towns: streets, squares and gardens (only looked up where the cover map marks a town).
    vec3 plan      = vec3(theta * landscape.heights.w, profile.x, uv.y);  // around, z, arc
    vec3 planDx    = dFdx(plan);
    vec3 planDy    = dFdy(plan);
    float metres   = length(fwidth(uv));
    vec4 town      = vec4(4.0, 0.0, 0.0, 0.0);
    if (onFloor && cover.b > 0.0)
    {
        town   = townGround(profile.x, theta, uv.y, planDx, planDy);
        albedo = townAlbedo(albedo, uv, town, metres);
    }
    albedo = shoreAlbedo(albedo, uv, waterLevel(profile) - h);

    // Sunlight, shadowed by hills and mountains. The march starts from the height field's own
    // surface (the patch mesh can differ from it by metres far away).
    vec3 ground = vec3(radial.xy * (profile.y - h), profile.x);
    vec3 direct = vec3(0.0);
    for (int i = 0; i < beamCount(); ++i)
    {
        vec3 beam = beamLight(p, n, i);
        if (max(beam.r, max(beam.g, beam.b)) > 0.0)
        {
            beam *= terrainShadow(heightMap, profileMap, arcMap, ground, beamDirection(ground, i),
                                  0.5 + 0.004 * length(inCameraRelative)) *
                    treeShadow(inCameraRelative, n, i) *
                    cloudShade(cloudMap, ground, beamDirection(ground, i));
        }
        direct += beam;
    }
    vec3 lamps = LAMPLIGHT * 0.015 * town.w * lightsOn();  // pools of light under street lamps
    vec3 color = albedo * (direct + ambientLight(p, n) + lamps);
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(color * haze.transmittance + haze.inscatter, 1.0);
}
