#version 450

// The towns and farms: plastered houses in whitewash and ochre under terracotta roofs, windows
// with shutters, doors and shop fronts drawn from each wall's window bays (the mesh carries only
// the walls), stone bridges, street lamps and fountains. At night windows and lamps glow.
#define UNIFORM_SET 3
#include "frame.glsl"
#include "habitat.glsl"
#include "landscape.glsl"
#include "noise.glsl"
#define SHADOW_UNIFORM 3
#define SHADOW_SAMPLER 3
#include "shadow.glsl"
#include "clouds.glsl"
#include "lit.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec2 inUv;
layout(location = 2) flat in uint inMaterial;
layout(location = 3) in vec3 inCameraRelative;

layout(set = 2, binding = 0) uniform sampler2D heightMap;
layout(set = 2, binding = 1) uniform sampler2D profileMap;
layout(set = 2, binding = 2) uniform sampler2D arcMap;
layout(set = 2, binding = 4) uniform sampler2D cloudMap;

layout(location = 0) out vec4 outColor;

// Must match StarshipSimulator::building_material and packFacade().
const uint WALL = 0u, ROOF_TILES = 1u, FLAT_ROOF = 2u, STONE = 3u, WOOD = 4u, METAL = 5u,
           LAMP_GLASS = 6u, AWNING = 7u, WATER = 8u, SOFFIT = 9u, FOLIAGE = 10u;
const uint STYLE_HOUSE = 0u, STYLE_TOWER = 1u, STYLE_BARN = 2u, STYLE_HALL = 3u;
const float BAY = 3.2;  // kWindowBayM

// Linear albedos: the Mediterranean palette of the habitat paintings.
const vec3 WALLS[8] = vec3[](vec3(0.56, 0.54, 0.50), vec3(0.56, 0.47, 0.33), vec3(0.52, 0.35, 0.15),
                             vec3(0.56, 0.40, 0.34), vec3(0.40, 0.45, 0.48), vec3(0.50, 0.43, 0.31),
                             vec3(0.47, 0.26, 0.16), vec3(0.43, 0.47, 0.35));
const vec3 ROOFS[4] = vec3[](vec3(0.34, 0.12, 0.06), vec3(0.40, 0.17, 0.07), vec3(0.25, 0.11, 0.06),
                             vec3(0.12, 0.13, 0.15));
const vec3 SHUTTERS[5] = vec3[](vec3(0.0), vec3(0.07, 0.18, 0.09), vec3(0.06, 0.12, 0.26),
                                vec3(0.18, 0.09, 0.05), vec3(0.05, 0.20, 0.20));
const vec3 AWNINGS[6] = vec3[](vec3(0.45, 0.05, 0.04), vec3(0.06, 0.25, 0.08), vec3(0.06, 0.10, 0.32),
                               vec3(0.55, 0.38, 0.04), vec3(0.55, 0.18, 0.03), vec3(0.25, 0.08, 0.30));
const vec3 BLOSSOMS[4] = vec3[](vec3(0.55, 0.06, 0.06), vec3(0.60, 0.48, 0.05), vec3(0.35, 0.10, 0.45),
                                vec3(0.62, 0.60, 0.58));
const vec3 BARNS[2] = vec3[](vec3(0.30, 0.06, 0.04), vec3(0.26, 0.18, 0.11));
const vec3 WOOD_COLOR  = vec3(0.20, 0.12, 0.065);
const vec3 STONE_COLOR = vec3(0.40, 0.37, 0.32);
const vec3 GLASS       = vec3(0.012, 0.015, 0.02);
const vec3 FRAME       = vec3(0.60, 0.58, 0.54);

float storeyHeight(uint style)
{
    return style == STYLE_TOWER ? 3.4 : (style == STYLE_BARN ? 6.0 : (style == STYLE_HALL ? 4.2 : 3.0));
}

// Inside a rectangle (x0, y0) - (x1, y1), anti-aliased over w.
float box(vec2 p, vec4 r, vec2 w)
{
    vec2 inside = smoothstep(r.xy - w, r.xy + w, p) * (1.0 - smoothstep(r.zw - w, r.zw + w, p));
    return inside.x * inside.y;
}

struct Look
{
    vec3 albedo;
    vec3 emission;  // light of its own (lit windows, lamps) and reflections of the sky
};

// Glass: dark, reflecting the bright far side at grazing angles.
vec3 glassReflection(vec3 n, vec3 view)
{
    float fresnel = 0.04 + 0.96 * pow(1.0 - abs(dot(n, view)), 5.0);
    return habitat.ambientUp.rgb * 1.5 * fresnel;
}

Look wall(uint material, vec3 n, vec3 view)
{
    uint  colour  = (material >> 4u) & 15u;
    uint  windows = (material >> 8u) & 63u;
    bool  door    = ((material >> 14u) & 1u) != 0u;
    bool  shop    = ((material >> 15u) & 1u) != 0u;
    uint  style   = (material >> 16u) & 3u;
    uint  shutter = (material >> 18u) & 7u;
    float seed    = float(material >> 21u);

    vec2  uv      = inUv;  // x: window bays along the wall, y: metres above the ground floor
    vec2  metres  = vec2(uv.x * BAY, uv.y);
    vec2  aa      = max(fwidth(metres), vec2(0.004));
    float sH      = storeyHeight(style);
    float storey  = floor(uv.y / sH);
    float fy      = uv.y - storey * sH;
    float bay     = floor(uv.x);
    vec2  inBay   = vec2((fract(uv.x) - 0.5) * BAY, fy);  // metres from the bay's middle line

    vec3 base = style == STYLE_BARN ? BARNS[min(colour, 1u)] : WALLS[min(colour, 7u)];
    // Plaster: gently mottled, darker near the ground.
    base *= 0.9 + 0.14 * valueNoise(metres / 2.3) + 0.05 * valueNoise(metres / 0.35);
    base *= mix(0.78, 1.0, smoothstep(0.0, 1.2, uv.y));
    Look look = Look(base, vec3(0.0));
    if (uv.y < 0.35)
    {
        look.albedo = STONE_COLOR * 0.7 * (0.9 + 0.2 * valueNoise(metres / 0.5));  // plinth
    }
    if (style == STYLE_BARN)
    {
        // Upright planks, and big doors in the middle of the front.
        look.albedo *= 0.85 + 0.15 * step(0.12, fract(metres.x / 0.3));
        if (door)
        {
            float d = box(metres, vec4(-1.75, 0.0, 1.75, 4.0), aa);
            look.albedo = mix(look.albedo, WOOD_COLOR * (0.8 + 0.2 * step(0.1, fract(metres.x / 0.25))), d);
        }
        return look;
    }

    bool  bays   = bay >= 0.0 && bay < float(windows) && uv.y >= 0.0;
    float lit     = hash12(vec2(bay * 7.0 + storey, seed + storey * 3.1));
    float night   = lightsOn();
    vec3  glass   = GLASS;                       // dark: an albedo
    vec3  reflect = glassReflection(n, view);    // light: the sky seen in it
    if (!bays)
    {
        return look;
    }
    bool  ground = storey < 0.5;
    bool  middle = bay == floor(float(windows) * 0.5);
    if (door && ground && middle)
    {
        vec4  r      = style == STYLE_HALL ? vec4(-1.0, 0.0, 1.0, 3.2) : vec4(-0.6, 0.0, 0.6, 2.3);
        float border = box(inBay, r + vec4(-0.1, 0.0, 0.1, 0.1), aa);
        float leaf   = box(inBay, r, aa);
        look.albedo  = mix(look.albedo, FRAME * 0.8, border);
        vec3 panels  = WOOD_COLOR * (0.8 + 0.2 * step(0.1, fract((inBay.y + 0.2) / 0.6)));
        look.albedo  = mix(look.albedo, panels, leaf);
        return look;
    }
    if (style == STYLE_TOWER)
    {
        float top = float(material >> 21u);  // towers keep their storey count in the seed bits
        if (storey >= top - 1.5)
        {
            // The belfry: an arched opening.
            float arch = max(box(inBay, vec4(-0.8, 0.8, 0.8, 2.3), aa),
                             1.0 - smoothstep(0.8 - aa.x, 0.8 + aa.x, length(inBay - vec2(0.0, 2.3))));
            look.albedo    = mix(look.albedo, vec3(0.01), arch * step(0.0, inBay.y - 0.8));
        }
        else if (storey >= top - 2.5)
        {
            // The clock.
            float r     = length(inBay - vec2(0.0, 1.7));
            float face  = 1.0 - smoothstep(0.75 - aa.x, 0.75 + aa.x, r);
            float rim   = face * smoothstep(0.62, 0.66, r);
            float t     = frame.time.x;
            vec2  d     = inBay - vec2(0.0, 1.7);
            float hands = 0.0;
            for (int h = 0; h < 2; ++h)
            {
                float a   = (h == 0 ? t / 43200.0 : t / 3600.0) * 2.0 * PI;
                vec2  dir = vec2(sin(a), cos(a));
                float len = h == 0 ? 0.4 : 0.6;
                float s   = clamp(dot(d, dir), 0.0, len);
                hands     = max(hands, 1.0 - smoothstep(0.03, 0.05, length(d - dir * s)));
            }
            look.albedo = mix(look.albedo, mix(vec3(0.6, 0.58, 0.52), vec3(0.03), max(rim, hands)), face);
        }
        else
        {
            float slit  = box(inBay, vec4(-0.2, 1.0, 0.2, 2.2), aa);
            look.albedo = mix(look.albedo, glass, slit);
            look.emission += slit * reflect;
        }
        return look;
    }
    if (shop && ground)
    {
        // A shop window across the bay, lit in the evening.
        float border = box(inBay, vec4(-1.3, 0.35, 1.3, 2.7), aa);
        float pane   = box(inBay, vec4(-1.2, 0.45, 1.2, 2.6), aa);
        look.albedo  = mix(look.albedo, WOOD_COLOR * 1.2, border);
        look.albedo = mix(look.albedo, glass, pane);
        look.emission += pane * (reflect + LAMPLIGHT * 0.022 * night * (0.6 + 0.4 * lit));
        return look;
    }
    // A window: a sill, a frame, the glass, and shutters beside it.
    float tall   = style == STYLE_HALL ? 3.4 : 2.25;
    vec4  r      = vec4(-0.6, 0.9, 0.6, tall);
    float border = box(inBay, r + vec4(-0.09, -0.12, 0.09, 0.09), aa);
    float pane   = box(inBay, r, aa);
    if (style == STYLE_HALL)
    {
        // Round-headed windows.
        vec2  c   = inBay - vec2(0.0, tall);
        float top = (1.0 - smoothstep(0.6 - aa.x, 0.6 + aa.x, length(c))) * step(0.0, c.y);
        pane      = max(pane, top);
        border    = max(border, (1.0 - smoothstep(0.69 - aa.x, 0.69 + aa.x, length(c))) * step(0.0, c.y));
    }
    float bars   = pane * max(1.0 - smoothstep(0.02, 0.04, abs(inBay.x)),
                              1.0 - smoothstep(0.02, 0.04, abs(inBay.y - (r.y + tall) * 0.5)));
    look.albedo  = mix(look.albedo, FRAME, border);
    look.albedo  = mix(look.albedo, glass, pane);
    look.albedo  = mix(look.albedo, FRAME, bars);
    vec3  inside  = LAMPLIGHT * 0.014 * night * step(lit, 0.45) * (0.5 + 0.5 * hash12(vec2(seed, bay)));
    look.emission += (pane - bars) * (reflect + inside);
    if (shutter > 0u && style == STYLE_HOUSE)
    {
        float leaves = box(vec2(abs(inBay.x), inBay.y), vec4(0.72, 0.9, 1.25, tall), aa);
        vec3  wood   = SHUTTERS[min(shutter, 4u)] * (0.75 + 0.25 * step(0.25, fract(inBay.y / 0.12)));
        look.albedo  = mix(look.albedo, wood, leaves);
    }
    return look;
}

Look roofTiles(uint material)
{
    uint  colour = (material >> 4u) & 15u;
    vec2  uv     = inUv;  // metres along the eave, and up the slope
    float fade   = 1.0 - smoothstep(0.08, 0.2, length(fwidth(uv)));
    float row    = floor(uv.y / 0.32);
    float across = (uv.x + row * 0.11) / 0.24;
    float tile   = hash12(vec2(floor(across), row));
    vec3  base   = ROOFS[min(colour, 3u)] * (0.82 + 0.3 * tile * fade + 0.1 * valueNoise(uv / 3.0));
    float curve  = 0.8 + 0.2 * sin(fract(across) * PI);
    float lap    = mix(1.0, 0.65, (1.0 - smoothstep(0.0, 0.12, fract(uv.y / 0.32))) * fade);
    return Look(base * mix(1.0, curve * lap, fade), vec3(0.0));
}

Look stone(uint material)
{
    vec2  uv   = inUv;
    float fade = 1.0 - smoothstep(0.08, 0.3, length(fwidth(uv)));
    bool  deck = ((material >> 4u) & 15u) == 1u;
    vec2  size = deck ? vec2(0.8, 0.5) : vec2(0.9, 0.45);
    float row  = floor(uv.y / size.y);
    vec2  cell = vec2((uv.x + row * 0.37) / size.x, uv.y / size.y);
    float mortar = max(1.0 - smoothstep(0.0, 0.05, fract(cell.x)), 1.0 - smoothstep(0.0, 0.08, fract(cell.y)));
    vec3  base = STONE_COLOR * (deck ? 0.85 : 1.0) * (0.85 + 0.25 * hash12(floor(cell)) * fade);
    return Look(base * mix(1.0, 0.75, mortar * fade), vec3(0.0));
}

void main()
{
    vec3 p        = inCameraRelative + frame.cameraPosition.xyz;
    vec3 n        = normalize(inNormal);
    vec3 view     = normalize(-inCameraRelative);
    uint material = inMaterial;
    uint surface  = material & 15u;
    float night   = lightsOn();

    Look look;
    if (surface == WALL)
    {
        look = wall(material, n, view);
    }
    else if (surface == ROOF_TILES)
    {
        look = roofTiles(material);
    }
    else if (surface == STONE)
    {
        look = stone(material);
    }
    else if (surface == FLAT_ROOF)
    {
        look = Look(vec3(0.30, 0.28, 0.25) * (0.85 + 0.25 * valueNoise(inUv / 1.7)), vec3(0.0));
    }
    else if (surface == WOOD)
    {
        look = Look(WOOD_COLOR * (0.85 + 0.25 * step(0.15, fract(inUv.x / 0.14))), vec3(0.0));
    }
    else if (surface == METAL)
    {
        look = Look(vec3(0.03, 0.05, 0.04), vec3(0.0));
    }
    else if (surface == LAMP_GLASS)
    {
        look = Look(vec3(0.45, 0.44, 0.40), LAMPLIGHT * 0.45 * night);
    }
    else if (surface == AWNING)
    {
        uint colour = (material >> 4u) & 15u;
        float white = step(0.5, fract(inUv.x / 0.6));
        look = Look(mix(AWNINGS[min(colour, 5u)], vec3(0.55, 0.53, 0.48), white), vec3(0.0));
    }
    else if (surface == WATER)
    {
        float ripple = valueNoise(inUv * 3.0 + frame.time.x * 0.7);
        look = Look(vec3(0.02, 0.045, 0.06), glassReflection(n, view) * (0.7 + 0.6 * ripple));
    }
    else if (surface == SOFFIT)
    {
        uint colour = (material >> 4u) & 15u;
        look = Look(WALLS[min(colour, 7u)] * 0.45, vec3(0.0));
    }
    else
    {
        // Flowers in a planter.
        uint  colour  = (material >> 4u) & 15u;
        float blossom = step(0.62, valueNoise(p.xy * 5.0 + p.z * 3.0));
        look = Look(mix(vec3(0.05, 0.12, 0.03), BLOSSOMS[min(colour, 3u)], blossom), vec3(0.0));
    }

    vec3 light = surfaceLight(heightMap, profileMap, arcMap, cloudMap, p, n, inCameraRelative);
    vec3 color = look.albedo * light + look.emission;
    Haze haze  = aerialPerspective(frame.cameraPosition.xyz, p);
    outColor   = vec4(color * haze.transmittance + haze.inscatter, 1.0);
}
