#version 330 core

in vec2 vNdc;

uniform vec3  uLidarPos;
uniform mat3  uLidarRot;
uniform float uHzScale, uHzBias;
uniform float uHzOffset;
uniform float uVtScale, uVtBias;
uniform float uDepthScale, uDepthBias;
uniform float uNearP, uFarP;

uniform vec3  uPlaneNormal;
uniform vec3  uPlaneCenter;
uniform vec3  uPlaneTangent;
uniform vec3  uPlaneBitangent;
uniform vec2  uPlaneHalfSize;

uniform float uMaxIntensity;
uniform float uReflectance;
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
    if (int(ch.b) != uGroupIndex) {
        discard;
    }

    float azimuth   = (vNdc.x - uHzBias) / uHzScale + uHzOffset;
    float elevation = (vNdc.y - uVtBias) / uVtScale;

    vec3 localDir = vec3(
        cos(elevation) * cos(azimuth),
        cos(elevation) * sin(azimuth),
        sin(elevation)
    );

    vec3 worldDir = uLidarRot * localDir;

    float denom = dot(uPlaneNormal, worldDir);
    if (abs(denom) < 1e-6) discard;

    float t = dot(uPlaneNormal, uPlaneCenter - uLidarPos) / denom;
    if (t < ch.r || t > ch.g) discard;

    vec3 hitPos = uLidarPos + t * worldDir;

    vec3 offset = hitPos - uPlaneCenter;
    float u = dot(offset, uPlaneTangent);
    float v = dot(offset, uPlaneBitangent);
    if (abs(u) > uPlaneHalfSize.x || abs(v) > uPlaneHalfSize.y) discard;

    gl_FragDepth = t * uDepthScale + uDepthBias;

    float cosTheta = abs(denom);
    float intensity = min(uMaxIntensity * uReflectance
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