#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNorm;
layout(location = 2) in float aReflectance;

layout(location = 3) in mat4 aModel;
layout(location = 7) in mat3 aNormalMat;

uniform vec3 uLidarPos;
uniform mat3 uLidarRotInv;
uniform float uHzScale, uHzOffset;
uniform float uVtScale;
uniform float uDepthScale, uDepthBias;

out vec3 vWorldPos;
out vec3 vWorldNorm;
out vec3 vLocalDir;
out float vReflectance;

void main() {
    vec4 wp = aModel * vec4(aPos, 1.0);
    vWorldPos = wp.xyz;
    vWorldNorm = normalize(aNormalMat * aNorm);
    vReflectance = aReflectance;

    vec3 localRay = uLidarRotInv * (wp.xyz - uLidarPos);
    float dist = length(localRay);

    if (dist < 1e-6) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        vLocalDir = vec3(0.0);
        return;
    }

    vec3 localDir = localRay / dist;
    vLocalDir = localDir;

    float azimuth   = atan(localDir.y, localDir.x);
    float elevation = asin(clamp(localDir.z, -1.0, 1.0));

    float ndcX     = azimuth   * uHzScale   + uHzOffset;
    float ndcY     = elevation * uVtScale;
    float depthNdc = dist      * uDepthScale + uDepthBias;

    gl_Position = vec4(ndcX * dist, ndcY * dist, depthNdc * dist, dist);
}