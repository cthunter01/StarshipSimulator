#version 450

// A stand-in for the cloud deck: a closed cylinder just outside the deck's base, reaching from end
// to end of the habitat. Every view ray that meets cloud crosses it, and it is nearer than any land
// behind the deck, so the depth test throws away every pixel where the land stands in front before
// the expensive march in clouds.frag runs. From below the deck the renderer draws the faces where
// rays enter it, from inside the deck the faces where they leave, so each pixel is shaded once.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "habitat.glsl"
#include "clouds_shell.glsl"

layout(location = 0) out vec3 outCameraRelative;

void main()
{
    // Vertices 0 .. 6 SHELL_SEGMENTS - 1 make the side, a quad per segment; then a fan for each end.
    int   v      = gl_VertexIndex;
    float radius = shellRadius() / cos(PI / float(SHELL_SEGMENTS));  // the polygon clears the circle
    float zLow   = habitat.strips.z;
    float zHigh  = habitat.strips.w;
    vec3  p;
    if (v < 6 * SHELL_SEGMENTS)
    {
        // Two triangles, (0, 0) (1, 0) (1, 1) and (0, 0) (1, 1) (0, 1) in (around, along): outward
        // facing, counter-clockwise seen from outside.
        int   segment = v / 6;
        int   corner  = v % 6;
        int   around  = segment + ((corner == 1 || corner == 2 || corner == 4) ? 1 : 0);
        bool  high    = corner == 2 || corner == 4 || corner == 5;
        float angle   = 2.0 * PI * float(around) / float(SHELL_SEGMENTS);
        p             = vec3(radius * cos(angle), radius * sin(angle), high ? zHigh : zLow);
    }
    else
    {
        int  w       = v - (6 * SHELL_SEGMENTS);
        bool top     = w >= 3 * SHELL_SEGMENTS;
        int  i       = w % (3 * SHELL_SEGMENTS);
        int  segment = i / 3;
        int  corner  = i % 3;
        float z      = top ? zHigh : zLow;
        if (corner == 0)
        {
            p = vec3(0.0, 0.0, z);
        }
        else
        {
            int   around = segment + (((corner == 2) == top) ? 1 : 0);
            float angle  = 2.0 * PI * float(around) / float(SHELL_SEGMENTS);
            p            = vec3(radius * cos(angle), radius * sin(angle), z);
        }
    }
    outCameraRelative = p - frame.cameraPosition.xyz;
    gl_Position       = frame.viewProjection * vec4(outCameraRelative, 1.0);
}
