// Colour helpers. All lighting is in linear RGB; encoding happens once, in tonemap.frag.

// Khronos PBR Neutral tone mapper: keeps hues and base colours faithful, compresses highlights.
// https://github.com/KhronosGroup/ToneMapping/tree/main/PBR_Neutral
vec3 toneMapPbrNeutral(vec3 color)
{
    const float startCompression = 0.8 - 0.04;
    const float desaturation     = 0.15;

    float x      = min(color.r, min(color.g, color.b));
    float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
    color -= offset;

    float peak = max(color.r, max(color.g, color.b));
    if (peak < startCompression)
    {
        return color;
    }
    const float d       = 1.0 - startCompression;
    float       newPeak = 1.0 - d * d / (peak + d - startCompression);
    color *= newPeak / peak;
    float g = 1.0 - 1.0 / (desaturation * (peak - newPeak) + 1.0);
    return mix(color, vec3(newPeak), g);
}

vec3 linearToSrgb(vec3 linear)
{
    vec3 low  = linear * 12.92;
    vec3 high = 1.055 * pow(linear, vec3(1.0 / 2.4)) - 0.055;
    return mix(low, high, step(vec3(0.0031308), linear));
}

vec3 srgbToLinear(vec3 srgb)
{
    vec3 low  = srgb / 12.92;
    vec3 high = pow((srgb + 0.055) / 1.055, vec3(2.4));
    return mix(low, high, step(vec3(0.04045), srgb));
}
