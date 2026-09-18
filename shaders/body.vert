#version 450

// A quad at infinity around Earth or the Moon, a little larger than its disk to fit the glow of
// the atmosphere. The fragment shader intersects each view ray with the sphere.
#define UNIFORM_SET 1
#include "frame.glsl"
#include "body.glsl"

layout(location = 0) out vec3 outView;  // view direction (habitat frame), not normalized

void main()
{
    int  corner = gl_VertexIndex % 6;
    vec2 offset = vec2(corner == 1 || corner == 2 || corner == 4 ? 1.0 : -1.0,
                       corner == 2 || corner == 4 || corner == 5 ? 1.0 : -1.0);

    vec3  d      = normalize(body.direction.xyz);
    vec3  helper = abs(d.z) < 0.9 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3  t      = normalize(cross(helper, d));
    vec3  b      = cross(d, t);
    // Half-size of the quad on the tangent plane at unit distance, with room for a few pixels
    // of edge antialiasing.
    float reach  = sin(body.direction.w) * (1.0 + ATMOSPHERE_REACH) * 1.05 + 4.0 * frame.viewport.w;
    vec3  corner3 = d + (offset.x * t + offset.y * b) * reach;

    outView     = corner3;
    gl_Position = frame.viewProjection * vec4(corner3, 0.0);  // at infinity
}
