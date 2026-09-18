#version 450

// A single triangle that covers the whole screen; draw it with 3 vertices and no vertex buffer.
layout(location = 0) out vec2 outNdc;  // normalized device coordinates, y up
layout(location = 1) out vec2 outUv;   // texture coordinates, origin top-left

void main()
{
    vec2 ndc = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2)) * 2.0 - 1.0;
    outNdc      = ndc;
    outUv       = vec2(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    gl_Position = vec4(ndc, 0.0, 1.0);
}
