#version 330 core

// ============================================================================
// 平面实例化顶点着色器
// ----------------------------------------------------------------------------
// 每个平面对应一个全屏四边形实例（顶点位置由 gl_VertexID 生成），
// 逐像素在片段着色器中做光线-平面解析求交（避免大平面光栅化的球面畸变）。
// 本阶段只做一件事：把 per-instance 的局部平面参数变换到世界空间，
// 通过 varying 传给片段着色器（替代原 CPU 端 PlaneCache 预变换）。
//
// aModel 为「模型空间 → 世界空间」的实例化矩阵（旋转 + 平移，无缩放）。
// 局部平面中心恒为原点，故世界中心 = aModel 的平移分量。
// ============================================================================

layout(location = 0) in mat4 aModel;         // 模型空间 → 世界空间（实例化矩阵）
layout(location = 4) in vec3 aPlaneNormal;   // 模型空间：平面法线
layout(location = 5) in vec3 aPlaneAxisU;    // 模型空间：平面 U 轴
layout(location = 6) in vec3 aPlaneAxisV;    // 模型空间：平面 V 轴
layout(location = 7) in vec2 aPlaneHalfSize; // 半尺寸（旋转不变）
layout(location = 8) in float aReflectance;  // 反射率

out vec2 vNdc;          // 全屏四边形 NDC 坐标（片段着色器据此反推射线方向）
out vec3 vWorldNormal;  // 世界空间：平面法线
out vec3 vWorldCenter;  // 世界空间：平面中心
out vec3 vWorldU;       // 世界空间：平面 U 轴
out vec3 vWorldV;       // 世界空间：平面 V 轴
out vec2 vHalfSize;     // 半尺寸
out float vReflectance; // 反射率

const vec2 kQuadPos[4] = vec2[](
    vec2(-1.0, -1.0),
    vec2( 1.0, -1.0),
    vec2( 1.0,  1.0),
    vec2(-1.0,  1.0)
);

void main() {
    vNdc = kQuadPos[gl_VertexID];
    gl_Position = vec4(kQuadPos[gl_VertexID], 0.0, 1.0);

    // —— 局部平面参数 → 世界空间 ——
    vWorldNormal = normalize(mat3(aModel) * aPlaneNormal);
    vWorldCenter = (aModel * vec4(0.0, 0.0, 0.0, 1.0)).xyz; // 局部中心恒为原点
    vWorldU      = mat3(aModel) * aPlaneAxisU;
    vWorldV      = mat3(aModel) * aPlaneAxisV;
    vHalfSize    = aPlaneHalfSize;
    vReflectance = aReflectance;
}