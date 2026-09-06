#version 330 core

in vec2 vNdc;
in vec3 vWorldNormal;
in vec3 vWorldCenter;
in vec3 vWorldU;
in vec3 vWorldV;
in vec2 vHalfSize;
in float vReflectance;

uniform vec3  uLidarPos;
uniform mat3  uLidarRot;
uniform float uHzScale;
uniform float uVtScale;
uniform float uDepthScale, uDepthBias;
uniform float uNearP, uFarP;

uniform float uMaxIntensity;
uniform uint  uFrameCounter;
uniform float uRangeNoiseStd;
uniform float uIntensityNoiseStd;
uniform sampler2D uChannelTex;
uniform int uGroupIndex;

layout(location = 0) out vec4 FragResult;

uint pcgHash(uint v) {
    uint state = v * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803537u;
    return (word >> 22u) ^ word;
}

float randFloat01(uint v) {
    return float(v) / 4294967295.0;
}

vec2 randNormal2(uvec2 seed) {
    float u1 = randFloat01(pcgHash(seed.x));
    float u2 = randFloat01(pcgHash(seed.y));
    float r = sqrt(-2.0 * log(max(u1, 1e-10)));
    return r * vec2(cos(6.28318530718 * u2), sin(6.28318530718 * u2));
}

void main() {
    int row = int(gl_FragCoord.y);
    vec4 ch = texelFetch(uChannelTex, ivec2(0, row), 0);
    if (int(ch.a) != uGroupIndex) {
        discard;
    }

    // 反推水平方位角。世界方位角以 +Y（左）为正，屏幕 ndcX 以 +X（右）为正，
    // 二者方向相反，故取负（ch.b 为该通道的方位角中心偏移）。
    float azimuth   = ch.b - vNdc.x / uHzScale;
    float elevation = vNdc.y / uVtScale;

    vec3 localDir = vec3(
        cos(elevation) * cos(azimuth),
        cos(elevation) * sin(azimuth),
        sin(elevation)
    );

    vec3 worldDir = uLidarRot * localDir;

    float denom = dot(vWorldNormal, worldDir);
    if (abs(denom) < 1e-6) discard;

    float t = dot(vWorldNormal, vWorldCenter - uLidarPos) / denom;
    if (t < ch.r || t > ch.g) discard;

    vec3 hitPos = uLidarPos + t * worldDir;

    vec3 offset = hitPos - vWorldCenter;
    float u = dot(offset, vWorldU);
    float v = dot(offset, vWorldV);
    if (abs(u) > vHalfSize.x || abs(v) > vHalfSize.y) discard;

    gl_FragDepth = t * uDepthScale + uDepthBias;

    float cosTheta = abs(denom);
    float intensity = min(uMaxIntensity * vReflectance
                    * cosTheta, uMaxIntensity);

    vec3 outPos = hitPos;
    float outIntensity = intensity;

    if (uRangeNoiseStd > 0.0 || uIntensityNoiseStd > 0.0) {
        uvec2 seed = uvec2(
            uint(gl_FragCoord.x) + uFrameCounter * 127u,
            uint(gl_FragCoord.y) + uFrameCounter * 311u
        );
        vec2 n = randNormal2(seed);

        if (uRangeNoiseStd > 0.0) {
            float noisyDist = t + n.x * uRangeNoiseStd;
            outPos = uLidarPos + worldDir * noisyDist;
        }
        if (uIntensityNoiseStd > 0.0) {
            outIntensity = intensity + n.y * uIntensityNoiseStd;
        }
    }

    FragResult = vec4(outPos, outIntensity);
}