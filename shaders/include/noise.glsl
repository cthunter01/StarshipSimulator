// Small hash and value-noise helpers for procedural surface detail.

float hash12(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

float hash13(vec3 p3)
{
    p3 = fract(p3 * 0.1031);
    p3 += dot(p3, p3.zyx + 31.32);
    return fract((p3.x + p3.y) * p3.z);
}

float valueNoise(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash12(i), hash12(i + vec2(1.0, 0.0)), u.x),
               mix(hash12(i + vec2(0.0, 1.0)), hash12(i + vec2(1.0, 1.0)), u.x), u.y);
}

float valueNoise3(vec3 p)
{
    vec3 i = floor(p);
    vec3 f = fract(p);
    vec3 u = f * f * (3.0 - 2.0 * f);
    float a = mix(hash13(i), hash13(i + vec3(1, 0, 0)), u.x);
    float b = mix(hash13(i + vec3(0, 1, 0)), hash13(i + vec3(1, 1, 0)), u.x);
    float c = mix(hash13(i + vec3(0, 0, 1)), hash13(i + vec3(1, 0, 1)), u.x);
    float d = mix(hash13(i + vec3(0, 1, 1)), hash13(i + vec3(1, 1, 1)), u.x);
    return mix(mix(a, b, u.y), mix(c, d, u.y), u.z);
}

float fbm2(vec2 p)
{
    float sum = 0.0;
    float amplitude = 0.5;
    for (int i = 0; i < 4; ++i)
    {
        sum += amplitude * valueNoise(p);
        p = p * 2.03 + vec2(17.1, 9.7);
        amplitude *= 0.5;
    }
    return sum / 0.9375;
}

float fbm3(vec3 p)
{
    float sum = 0.0;
    float amplitude = 0.5;
    for (int i = 0; i < 4; ++i)
    {
        sum += amplitude * valueNoise3(p);
        p = p * 2.03 + vec3(17.1, 9.7, 3.3);
        amplitude *= 0.5;
    }
    return sum / 0.9375;
}

// Anti-aliased coverage of grid lines ("pristine grid", Ben Golus); width is in cells.
float gridLines(vec2 uv, float lineWidth)
{
    vec4 uvDDXY      = vec4(dFdx(uv), dFdy(uv));
    vec2 uvDeriv     = vec2(length(uvDDXY.xz), length(uvDDXY.yw));
    vec2 targetWidth = vec2(lineWidth);
    vec2 drawWidth   = clamp(targetWidth, uvDeriv, vec2(0.5));
    vec2 lineAA      = max(uvDeriv, vec2(1e-6)) * 1.5;
    vec2 gridUV      = 1.0 - abs(fract(uv) * 2.0 - 1.0);
    vec2 grid2       = smoothstep(drawWidth + lineAA, drawWidth - lineAA, gridUV);
    grid2 *= clamp(targetWidth / drawWidth, 0.0, 1.0);
    grid2 = mix(grid2, targetWidth, clamp(uvDeriv * 2.0 - 1.0, 0.0, 1.0));
    return mix(grid2.x, 1.0, grid2.y);
}
