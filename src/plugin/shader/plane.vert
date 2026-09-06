#version 330 core

const vec2 kQuadPos[4] = vec2[](
    vec2(-1.0, -1.0),
    vec2( 1.0, -1.0),
    vec2( 1.0,  1.0),
    vec2(-1.0,  1.0)
);

out vec2 vNdc;

void main() {
    vNdc = kQuadPos[gl_VertexID];
    gl_Position = vec4(kQuadPos[gl_VertexID], 0.0, 1.0);
}