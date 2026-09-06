#version 330 core
in vec3 vWorldPos;
in vec3 vWorldNorm;
in vec3 vLocalDir;
uniform vec3 uLidarPos;
uniform float uMaxIntensity;
uniform float uAtmosAtten;
uniform float uSysEfficiency;
uniform float uNearP;
uniform float uFarP;
uniform uint uFrameCounter;
uniform float uRangeNoiseStd;
uniform float uIntensityNoiseStd;

uniform float uClambda;
uniform float uRoughness;
uniform float uF0Dielectric;
uniform int   uEnableRetro;
uniform float uKRetro;
uniform float uF0Retro;

layout(location = 0) out vec4 FragResult;

float BRDF_Monostatic(float cosTheta, float roughness, float F0) {
    float alpha = roughness * roughness;
    float a2 = alpha * alpha;
    float denom = cosTheta * cosTheta * (a2 - 1.0) + 1.0;
    float D = a2 / (3.14159265358979323846 * denom * denom);
    float k = alpha * 0.5;
    float G = cosTheta / (cosTheta * (1.0 - k) + k);
    float t = 1.0 - cosTheta;
    float t2 = t * t;
    float F = F0 + (1.0 - F0) * t2 * t2 * t;
    return D * F * G * G / (4.0 * cosTheta * cosTheta + 0.001);
}

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
    if (vLocalDir.x <= 0.0) {
        discard;
    }

    vec3 hitNorm = normalize(vWorldNorm);
    vec3 ray = vWorldPos - uLidarPos;
    float dist = length(ray);
    if (dist < 0.01) {
        FragResult = vec4(vWorldPos, 0.0);
        return;
    }
    if (dist < uNearP || dist > uFarP) {
        FragResult = vec4(vWorldPos, 0.0);
        return;
    }
    vec3 rayDir = ray / dist;
    float cosTheta = max(0.0, dot(-hitNorm, rayDir));
    float invSqAtten = (uNearP * uNearP) / (dist * dist);
    float atmosAtten = exp(-uAtmosAtten * dist);

    float mainBRDF = BRDF_Monostatic(cosTheta, uRoughness, uF0Dielectric);
    float brdf = uClambda * mainBRDF;

    if (uEnableRetro != 0) {
        float retroBRDF = uF0Retro * cosTheta;
        brdf += uKRetro * retroBRDF;
    }

    float intensity = uMaxIntensity * invSqAtten * brdf * cosTheta * atmosAtten * uSysEfficiency;

    vec3 outPos = vWorldPos;
    float outIntensity = intensity;

    if (uRangeNoiseStd > 0.0 || uIntensityNoiseStd > 0.0) {
        uvec2 seed = uvec2(
            uint(gl_FragCoord.x) + uFrameCounter * 127u,
            uint(gl_FragCoord.y) + uFrameCounter * 311u
        );
        vec2 n = randNormal2(seed);

        if (uRangeNoiseStd > 0.0) {
            float noisyDist = dist + n.x * uRangeNoiseStd;
            outPos = uLidarPos + rayDir * noisyDist;
        }
        if (uIntensityNoiseStd > 0.0) {
            outIntensity = intensity + n.y * uIntensityNoiseStd;
        }
    }

    FragResult = vec4(outPos, outIntensity);
}