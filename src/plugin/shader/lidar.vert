#version 330 core

// ============================================================================
// 坐标系统与方向约定
// ----------------------------------------------------------------------------
// 本着色器按以下顺序变换坐标：
//   模型空间 → 世界空间 → 雷达局部空间 → 球面角 → NDC → 裁剪空间。
// 其中模型空间 / 世界空间 / 雷达局部空间均为右手系、Z 朝上。
//
// [模型空间]  aPos / aNorm
//     网格生成时的局部坐标，右手系、Z 朝上（轴与 Gazebo 世界一致），
//     由 aModel 变换到世界空间。
//
// [世界空间]  worldPosition / vWorldPos / vWorldNorm / uLidarPos
//     Gazebo / ROS 约定：右手系、Z 朝上。
//     +X 前 / +Y 左 / +Z 上。
//
// [雷达局部空间]  localRay / localDir / vLocalDir
//     雷达自身坐标系：右手系、Z 朝上。
//     +X 激光朝向 / +Y 左 / +Z 上。
//     用 uLidarRotInv（雷达旋转的逆/转置）把世界向量转到该空间。
//
// [球面角]  azimuth / elevation
//     azimuth   = 水平方位角（绕 Z 轴，即 yaw 轴），atan2(localDir.y, localDir.x)，
//                 正方向朝 +Y（左），范围 (-π, π]。
//     elevation = 俯仰角（绕 Y 轴，即 pitch 轴），asin(localDir.z)，
//                 正方向朝 +Z（上），范围 [-π/2, π/2]。
//
// [NDC 空间]  ndcX / ndcY / depthNdc
//     x 右 / y 上 / z 入屏（近 -1、远 +1；z 轴为左手系）。
//     ndcX     = -azimuth * uHzScale    - uHzOffset  （水平 FOV → [-1, +1]；取负消除水平镜像）
//     ndcY     = elevation * uVtScale                 （垂直 FOV → [-1, +1]）
//     depthNdc = dist      * uDepthScale + uDepthBias （距离 → 深度 [-1, +1]）
//
// [裁剪空间]  gl_Position = (ndcX*dist, ndcY*dist, depthNdc*dist, dist)
//     令 w = dist：透视除法后得到上述 NDC；同时 w=dist 使 varying
//     按 1/dist 加权做透视校正插值。
// ============================================================================

layout(location = 0) in vec3 aPos;          // 模型空间：顶点位置
layout(location = 1) in vec3 aNorm;         // 模型空间：法线
layout(location = 2) in float aReflectance; // 反射率（与坐标系无关）

layout(location = 3) in mat4 aModel;       // 模型空间 → 世界空间（实例化矩阵：旋转 + 平移，无缩放）

uniform vec3 uLidarPos;                    // 世界空间：雷达位置
uniform mat3 uLidarRotInv;                 // 世界空间 → 雷达局部空间（旋转的逆）
uniform float uHzScale, uHzOffset;         // azimuth(弧度) → ndcX 的缩放/偏移
uniform float uVtScale;                    // elevation(弧度) → ndcY 的缩放
uniform float uDepthScale, uDepthBias;     // dist(米) → depthNdc 的缩放/偏移

out vec3 vWorldPos;     // 世界空间：命中点位置（点云 xyz 的来源）
out vec3 vWorldNorm;    // 世界空间：表面法线
out vec3 vLocalDir;     // 雷达局部空间：单位射线方向
out float vReflectance; // 反射率

void main() {
    // —— 模型空间 → 世界空间 ——
    vec4 worldPosition = aModel * vec4(aPos, 1.0);
    vWorldPos = worldPosition.xyz;                          // 世界空间
    vWorldNorm = normalize(mat3(aModel) * aNorm);  // 世界空间（mat3(aModel) 取左上 3×3 = 旋转）
    vReflectance = aReflectance;

    // —— 世界空间 → 雷达局部空间 ——
    vec3 localRay = uLidarRotInv * (worldPosition.xyz - uLidarPos); // 雷达指向顶点的向量
    float dist = length(localRay);               // 径向距离（测距值）

    // 退化保护：顶点与雷达重合时方向无定义，丢到裁剪体之外
    if (dist < 1e-6) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        vLocalDir = vec3(0.0);
        return;
    }

    // —— 方向（雷达局部空间，单位向量）——
    vec3 localDir = localRay / dist;
    vLocalDir = localDir;

    // —— 球面角：方向 → (方位角, 俯仰角) ——
    float azimuth   = atan(localDir.y, localDir.x);       // 绕 Z（yaw），水平面内角度
    float elevation = asin(clamp(localDir.z, -1.0, 1.0)); // 绕 Y（pitch），离水平面的仰角

    // —— 球面角/距离 → NDC ——
    // 世界方位角以 +Y（左）为正，屏幕 ndcX 以 +X（右）为正，二者方向相反，故取负。
    float ndcX     = -azimuth * uHzScale - uHzOffset;  // 水平 → 屏幕 x
    float ndcY     = elevation * uVtScale;                // 垂直 → 屏幕 y
    float depthNdc = dist      * uDepthScale + uDepthBias; // 距离 → 深度 z

    // —— 裁剪空间：w = dist，透视除法后 = (ndcX, ndcY, depthNdc) ——
    gl_Position = vec4(ndcX, ndcY, depthNdc, 1.0) * dist;
}