#pragma once

#include <glad/egl.h>
#include <glad/gl.h>

#include <gz/math/Matrix3.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>

#include "mesh_generator.hpp"

#include <vector>
#include <string>
#include <cstdint>

namespace blgz {

using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::kNullEntity;

class GlRenderer {
public:
    GlRenderer();
    ~GlRenderer();

    bool Initialize();
    void Cleanup();

    bool CreateFramebuffer(int width, int height);

    bool LoadAndCompileShaders(const std::string &shaderDir);

    void SetParams(double hzMin, double hzMax,
                   double vtMin, double vtMax,
                   double nearP, double farP);
    
    void CollectGeometries(EntityComponentManager &_ecm, Entity lidarModelEntity = kNullEntity);
    void RenderScene(const gz::math::Pose3d &lidarPose,
                    const EntityComponentManager &_ecm,
                    double maxIntensity,
                    double reflectance,
                    double atmosAtten,
                    double sysEfficiency,
                    uint32_t frameCounter) const;

    void SetNoiseParams(double rangeNoiseStd, double intensityNoiseStd);
    
    std::vector<float> ReadRenderResult(int width, int renderHeight) const;
    void ReadRenderResultInto(std::vector<float> &buffer, int width, int renderHeight) const;

    bool CreatePBOs(int width, int renderHeight);
    void StartAsyncReadback(int width, int renderHeight);
    bool FinishAsyncReadbackInto(std::vector<float> &buffer, int width, int renderHeight);

    int GetFrameBufferWidth() const { return fbWidth_; }
    int GetRenderHeight() const { return renderHeight_; }
    bool IsInitialized() const { return glInitialized_; }

private:
    static GLuint CompileShader(GLenum type, const char *src);
    static GLuint BuildProgram(const char *vertSrc, const char *fragSrc,
                              const char *geomSrc = nullptr);

    static void SetMat4Uniform(GLint loc, const gz::math::Matrix4d &m);
    static void SetMat3Uniform(GLint loc, const gz::math::Matrix3d &m);

    EGLDisplay eglDisplay_;
    EGLContext eglContext_;
    EGLSurface eglSurface_;
    bool glInitialized_;

    GLuint program_;
    GLuint fbo_;
    GLuint resultTex_;
    GLuint depthRb_;

    int fbWidth_;
    int fbHeight_;
    int renderHeight_;

    struct Params {
        float hzScale, hzBias;
        float vtScale, vtBias;
        float depthScale, depthBias;
        float nearP, farP;
    } params_;

    float rangeNoiseStd_ = 0.0f;
    float intensityNoiseStd_ = 0.0f;

    struct UniformLocs {
        GLint lidarPos = -1;
        GLint lidarRotInv = -1;
        GLint hzScale = -1, hzBias = -1;
        GLint vtScale = -1, vtBias = -1;
        GLint depthScale = -1, depthBias = -1;
        GLint nearP = -1, farP = -1;
        GLint maxIntensity = -1;
        GLint reflectance = -1;
        GLint atmosAtten = -1;
        GLint sysEfficiency = -1;
        GLint model = -1;
        GLint normalMat = -1;
        GLint frameCounter = -1;
        GLint rangeNoiseStd = -1;
        GLint intensityNoiseStd = -1;
    } locs_;

    struct Renderable {
        GLuint vao;
        GLuint vbo;
        int vertexCount;
        Entity entity;
    };
    std::vector<Renderable> renderables_;

    GLuint pbo_[2] = {0, 0};
    int pboWriteIdx_ = 0;
    bool pboFirstFrame_ = true;
    int pboSize_ = 0;
};

} // namespace blgz