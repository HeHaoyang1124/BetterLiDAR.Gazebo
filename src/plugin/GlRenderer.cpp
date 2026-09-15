#include "GlRenderer.hpp"

#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>

#include <dlfcn.h>
#include <cmath>
#include <cstring>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>

namespace blgz {
    using gz::sim::worldPose;

    namespace components = gz::sim::components;

    GlRenderer::GlRenderer()
        : eglDisplay_(EGL_NO_DISPLAY),
          eglContext_(EGL_NO_CONTEXT),
          eglSurface_(EGL_NO_SURFACE),
          glInitialized_(false),
          program_(0),
          fbo_(0),
          resultTex_(0),
          depthRb_(0),
          fbWidth_(0), fbHeight_(0),
          renderHeight_(0), params_() {
    }

    GlRenderer::~GlRenderer() { Cleanup(); }

    bool GlRenderer::Initialize() {
        using SysEglGetProcAddress = void *(*)(const char *);
        const auto sysEglGetProcAddress = reinterpret_cast<SysEglGetProcAddress>(
            dlsym(RTLD_DEFAULT, "eglGetProcAddress"));
        if (!sysEglGetProcAddress) {
            gzerr << "GlRenderer: dlsym(eglGetProcAddress) failed" << std::endl;
            return false;
        }

        using EglInitializeT = EGLBoolean (*)(EGLDisplay, EGLint *, EGLint *);
        using EglTerminateT = EGLBoolean (*)(EGLDisplay);
        using EglGetDisplayT = EGLDisplay (*)(EGLNativeDisplayType);
        const auto sysEglInitialize = reinterpret_cast<EglInitializeT>(
            dlsym(RTLD_DEFAULT, "eglInitialize"));
        const auto sysEglTerminate = reinterpret_cast<EglTerminateT>(
            dlsym(RTLD_DEFAULT, "eglTerminate"));
        const auto sysEglGetDisplay = reinterpret_cast<EglGetDisplayT>(
            dlsym(RTLD_DEFAULT, "eglGetDisplay"));

        using QueryDevicesExtT = EGLint (*)(EGLint, EGLDeviceEXT *, EGLint *);
        using GetPlatformDisplayExtT = EGLDisplay (*)(EGLenum, void *, const EGLint *);
        const auto eglQueryDevicesEXT = reinterpret_cast<QueryDevicesExtT>(
            sysEglGetProcAddress("eglQueryDevicesEXT"));
        const auto eglGetPlatformDisplayEXT = reinterpret_cast<GetPlatformDisplayExtT>(
            sysEglGetProcAddress("eglGetPlatformDisplayEXT"));

        if (eglQueryDevicesEXT && eglGetPlatformDisplayEXT) {
            EGLDeviceEXT devices[16];
            EGLint numDevices = 0;
            if (eglQueryDevicesEXT(16, devices, &numDevices) && numDevices > 0) {
                for (int i = 0; i < numDevices; ++i) {
                    EGLDisplay d = eglGetPlatformDisplayEXT(
                        EGL_PLATFORM_DEVICE_EXT, devices[i], nullptr);
                    if (d == EGL_NO_DISPLAY) continue;

                    EGLint major = 0, minor = 0;
                    if (sysEglInitialize && sysEglInitialize(d, &major, &minor)) {
                        eglDisplay_ = d;
                        break;
                    }
                    if (sysEglTerminate) sysEglTerminate(d);
                }
            }
        }

        if (eglDisplay_ == EGL_NO_DISPLAY && sysEglGetDisplay) {
            eglDisplay_ = sysEglGetDisplay(EGL_DEFAULT_DISPLAY);
        }

        if (eglDisplay_ == EGL_NO_DISPLAY) {
            gzerr << "GlRenderer: Failed to get EGL display" << std::endl;
            return false;
        }

        const int eglVersion = gladLoadEGL(eglDisplay_, reinterpret_cast<GLADloadfunc>(sysEglGetProcAddress));
        if (!eglVersion) {
            gzerr << "GlRenderer: gladLoadEGL failed" << std::endl;
            return false;
        }

        if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) {
            gzerr << "GlRenderer: eglBindAPI(EGL_OPENGL_API) failed" << std::endl;
            return false;
        }

        const EGLint configAttribs[] = {
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 8,
            EGL_DEPTH_SIZE, 24,
            EGL_STENCIL_SIZE, 8,
            EGL_NONE
        };

        EGLConfig config = nullptr;
        EGLint numConfigs = 0;
        if (!eglChooseConfig(eglDisplay_, configAttribs, &config, 1, &numConfigs) || numConfigs < 1) {
            gzerr << "GlRenderer: eglChooseConfig failed" << std::endl;
            return false;
        }

        const EGLint ctx33Core[] = {
            EGL_CONTEXT_MAJOR_VERSION, 3,
            EGL_CONTEXT_MINOR_VERSION, 3,
            EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE
        };
        eglContext_ = eglCreateContext(eglDisplay_, config, EGL_NO_CONTEXT, ctx33Core);

        if (eglContext_ == EGL_NO_CONTEXT) {
            const EGLint ctx33Compat[] = {
                EGL_CONTEXT_MAJOR_VERSION, 3,
                EGL_CONTEXT_MINOR_VERSION, 3,
                EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT,
                EGL_NONE
            };
            eglContext_ = eglCreateContext(eglDisplay_, config, EGL_NO_CONTEXT, ctx33Compat);
        }

        if (eglContext_ == EGL_NO_CONTEXT) {
            const EGLint ctxAny[] = {
                EGL_CONTEXT_MAJOR_VERSION, 3,
                EGL_CONTEXT_MINOR_VERSION, 0,
                EGL_NONE
            };
            eglContext_ = eglCreateContext(eglDisplay_, config, EGL_NO_CONTEXT, ctxAny);
        }

        if (eglContext_ == EGL_NO_CONTEXT) {
            gzerr << "GlRenderer: eglCreateContext failed" << std::endl;
            return false;
        }

        const EGLint pbufferAttribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        eglSurface_ = eglCreatePbufferSurface(eglDisplay_, config, pbufferAttribs);
        if (eglSurface_ == EGL_NO_SURFACE) {
            gzerr << "GlRenderer: eglCreatePbufferSurface failed" << std::endl;
            return false;
        }

        if (eglMakeCurrent(eglDisplay_, eglSurface_, eglSurface_, eglContext_) != EGL_TRUE) {
            gzerr << "GlRenderer: eglMakeCurrent failed" << std::endl;
            return false;
        }

        const int glVersion = gladLoadGL(reinterpret_cast<GLADloadfunc>(sysEglGetProcAddress));
        if (!glVersion) {
            gzerr << "GlRenderer: gladLoadGL failed" << std::endl;
            return false;
        }

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CW);

        gzmsg << "GlRenderer: EGL " << GLAD_VERSION_MAJOR(eglVersion) << "."
                << GLAD_VERSION_MINOR(eglVersion)
                << " GL " << GLAD_VERSION_MAJOR(glVersion) << "."
                << GLAD_VERSION_MINOR(glVersion) << std::endl;

        glInitialized_ = true;
        return true;
    }

    void GlRenderer::Cleanup() {
        if (!glInitialized_) return;

        for (auto &r: renderables_) {
            glDeleteVertexArrays(1, &r.vao);
            glDeleteBuffers(1, &r.vbo);
        }
        renderables_.clear();

        // clang-format off
        if (program_)   glDeleteProgram(program_);
        if (resultTex_) glDeleteTextures(1, &resultTex_);
        if (depthRb_)   glDeleteRenderbuffers(1, &depthRb_);
        if (fbo_)       glDeleteFramebuffers(1, &fbo_);
        if (pbo_[0] || pbo_[1]) glDeleteBuffers(2, pbo_);
        // clang-format on

        pbo_[0] = pbo_[1] = 0;

        eglMakeCurrent(eglDisplay_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (eglContext_ != EGL_NO_CONTEXT) eglDestroyContext(eglDisplay_, eglContext_);
        if (eglSurface_ != EGL_NO_SURFACE) eglDestroySurface(eglDisplay_, eglSurface_);
        if (eglDisplay_ != EGL_NO_DISPLAY) eglTerminate(eglDisplay_);

        eglDisplay_ = EGL_NO_DISPLAY;
        eglContext_ = EGL_NO_CONTEXT;
        eglSurface_ = EGL_NO_SURFACE;
        glInitialized_ = false;
        program_ = 0;
        fbo_ = 0;
        resultTex_ = 0;
        depthRb_ = 0;
    }

    bool GlRenderer::CreateFramebuffer(const int width, const int height) {
        fbWidth_ = width;
        fbHeight_ = height;
        renderHeight_ = height;

        glGenFramebuffers(1, &fbo_);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

        glGenTextures(1, &resultTex_);
        glBindTexture(GL_TEXTURE_2D, resultTex_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resultTex_, 0);

        constexpr GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1, drawBuffers);

        glGenRenderbuffers(1, &depthRb_);
        glBindRenderbuffer(GL_RENDERBUFFER, depthRb_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRb_);

        const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (status != GL_FRAMEBUFFER_COMPLETE) {
            gzerr << "GlRenderer: FBO incomplete (0x"
                    << std::hex << status << std::dec << ")" << std::endl;
            return false;
        }
        return true;
    }

    bool GlRenderer::LoadAndCompileShaders(const std::string &shaderDir) {
        auto LoadShaderFile = [](const std::string &path) -> std::string {
            std::ifstream ifs(path);
            if (!ifs.is_open()) {
                std::cerr << "[GlRenderer] Failed to open shader file: " << path << std::endl;
                return {};
            }
            std::ostringstream oss;
            oss << ifs.rdbuf();
            return oss.str();
        };

        const std::string lidarFrag = LoadShaderFile(shaderDir + "/lidar.frag");
        const std::string lidarVert = LoadShaderFile(shaderDir + "/lidar.vert");

        if (lidarVert.empty() || lidarFrag.empty()) {
            gzerr << "GlRenderer: Failed to load shader files" << std::endl;
            return false;
        }

        program_ = BuildProgram(lidarVert.c_str(), lidarFrag.c_str());
        if (!program_) {
            gzerr << "GlRenderer: Failed to compile shaders" << std::endl;
            return false;
        }

        // clang-format off
        locs_.lidarPos      = glGetUniformLocation(program_, "uLidarPos");
        locs_.lidarRotInv   = glGetUniformLocation(program_, "uLidarRotInv");
        locs_.hzScale       = glGetUniformLocation(program_, "uHzScale");
        locs_.hzBias        = glGetUniformLocation(program_, "uHzBias");
        locs_.vtScale       = glGetUniformLocation(program_, "uVtScale");
        locs_.vtBias        = glGetUniformLocation(program_, "uVtBias");
        locs_.depthScale    = glGetUniformLocation(program_, "uDepthScale");
        locs_.depthBias     = glGetUniformLocation(program_, "uDepthBias");
        locs_.nearP         = glGetUniformLocation(program_, "uNearP");
        locs_.farP          = glGetUniformLocation(program_, "uFarP");
        locs_.maxIntensity  = glGetUniformLocation(program_, "uMaxIntensity");
        locs_.reflectance   = glGetUniformLocation(program_, "uReflectance");
        locs_.atmosAtten    = glGetUniformLocation(program_, "uAtmosAtten");
        locs_.sysEfficiency = glGetUniformLocation(program_, "uSysEfficiency");
        locs_.model         = glGetUniformLocation(program_, "uModel");
        locs_.normalMat     = glGetUniformLocation(program_, "uNormalMat");
        locs_.frameCounter  = glGetUniformLocation(program_, "uFrameCounter");
        locs_.rangeNoiseStd = glGetUniformLocation(program_, "uRangeNoiseStd");
        locs_.intensityNoiseStd = glGetUniformLocation(program_, "uIntensityNoiseStd");
        // clang-format off

        return true;
    }

    void GlRenderer::SetParams(const double hzMin, const double hzMax,
                               const double vtMin, const double vtMax,
                               const double nearP, const double farP) {
        // clang-format off
        const auto hzSpan = static_cast<float>(hzMax - hzMin);
        const auto vtSpan = static_cast<float>(vtMax - vtMin);
        const auto dpSpan = static_cast<float>(farP  - nearP);

        params_.hzScale    =  2.0f / hzSpan;
        params_.hzBias     = -2.0f * static_cast<float>(hzMin) / hzSpan - 1.0f;
        params_.vtScale    =  2.0f / vtSpan;
        params_.vtBias     = -2.0f * static_cast<float>(vtMin) / vtSpan - 1.0f;
        params_.depthScale =  2.0f / dpSpan;
        params_.depthBias  = -(static_cast<float>(farP + nearP)) / dpSpan;
        params_.nearP      = static_cast<float>(nearP);
        params_.farP       = static_cast<float>(farP);
        // clang-format on
    }

    void GlRenderer::SetNoiseParams(const double rangeNoiseStd,
                                    const double intensityNoiseStd) {
        rangeNoiseStd_ = static_cast<float>(rangeNoiseStd);
        intensityNoiseStd_ = static_cast<float>(intensityNoiseStd);
    }

    void GlRenderer::CollectGeometries(EntityComponentManager &_ecm,
                                       const Entity lidarModelEntity) {
        auto belongsToModel = [&](const Entity ent) -> bool {
            if (lidarModelEntity == kNullEntity) return false;
            for (auto cur = ent; cur != kNullEntity;) {
                if (cur == lidarModelEntity) return true;
                auto *parent = _ecm.Component<components::ParentEntity>(cur);
                if (!parent) break;
                cur = parent->Data();
            }
            return false;
        };

        auto collectFrom = [&]<typename T>(const T &) {
            _ecm.Each<T, components::Geometry, components::Name>(
                [&](const Entity &_ent, const auto *,
                    const components::Geometry *_geom, const components::Name *) -> bool {
                    if (belongsToModel(_ent)) return true;

                    const auto &g = _geom->Data();
                    const std::vector<Vertex> mesh = MakeGeometry(g);

                    if (mesh.empty()) return true;

                    GLuint vao = 0, vbo = 0;
                    glGenVertexArrays(1, &vao);
                    glGenBuffers(1, &vbo);
                    glBindVertexArray(vao);
                    glBindBuffer(GL_ARRAY_BUFFER, vbo);
                    glBufferData(GL_ARRAY_BUFFER,
                                 static_cast<GLsizeiptr>(mesh.size() * sizeof(Vertex)),
                                 mesh.data(), GL_STATIC_DRAW);
                    glEnableVertexAttribArray(0);
                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                          reinterpret_cast<void *>(0));
                    glEnableVertexAttribArray(1);
                    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                          reinterpret_cast<void *>(sizeof(float) * 3));
                    glBindVertexArray(0);

                    renderables_.push_back({
                        .vao = vao, .vbo = vbo, .vertexCount = static_cast<int>(mesh.size()), .entity = _ent
                    });
                    return true;
                });
        };

        collectFrom(components::Visual());
        if (renderables_.empty())
            collectFrom(components::Collision());
    }

    void GlRenderer::RenderScene(const gz::math::Pose3d &lidarPose,
                                 const EntityComponentManager &_ecm,
                                 const double maxIntensity,
                                 const double reflectance,
                                 const double atmosAtten,
                                 const double sysEfficiency,
                                 const uint32_t frameCounter) const {
        glViewport(0, 0, fbWidth_, renderHeight_);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(program_);

        glUniform3f(locs_.lidarPos,
                    static_cast<float>(lidarPose.Pos().X()),
                    static_cast<float>(lidarPose.Pos().Y()),
                    static_cast<float>(lidarPose.Pos().Z()));

        const gz::math::Matrix3d rotInv = gz::math::Matrix3d(lidarPose.Rot()).Transposed();
        SetMat3Uniform(locs_.lidarRotInv, rotInv);

        glUniform1f(locs_.hzScale, params_.hzScale);
        glUniform1f(locs_.hzBias, params_.hzBias);
        glUniform1f(locs_.vtScale, params_.vtScale);
        glUniform1f(locs_.vtBias, params_.vtBias);
        glUniform1f(locs_.depthScale, params_.depthScale);
        glUniform1f(locs_.depthBias, params_.depthBias);
        glUniform1f(locs_.nearP, params_.nearP);
        glUniform1f(locs_.farP, params_.farP);

        glUniform1f(locs_.maxIntensity,
                    static_cast<float>(maxIntensity));
        glUniform1f(locs_.reflectance,
                    static_cast<float>(reflectance));
        glUniform1f(locs_.atmosAtten,
                    static_cast<float>(atmosAtten));
        glUniform1f(locs_.sysEfficiency,
                    static_cast<float>(sysEfficiency));

        glUniform1ui(locs_.frameCounter, frameCounter);
        glUniform1f(locs_.rangeNoiseStd, rangeNoiseStd_);
        glUniform1f(locs_.intensityNoiseStd, intensityNoiseStd_);

        for (auto &r: renderables_) {
            auto pose = worldPose(r.entity, _ecm);

            const auto modelMat = gz::math::Matrix4d(pose);
            const auto normalMat = gz::math::Matrix3d(pose.Rot());

            SetMat4Uniform(locs_.model, modelMat);
            SetMat3Uniform(locs_.normalMat, normalMat);

            glBindVertexArray(r.vao);
            glDrawArrays(GL_TRIANGLES, 0, r.vertexCount);
        }

        glBindVertexArray(0);
        glUseProgram(0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    std::vector<float> GlRenderer::ReadRenderResult(const int width, const int renderHeight) const {
        const int totalPixels = width * renderHeight;
        std::vector<float> resultData(totalPixels * 4);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, renderHeight, GL_RGBA, GL_FLOAT, resultData.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        return resultData;
    }

    void GlRenderer::ReadRenderResultInto(std::vector<float> &buffer,
                                          const int width,
                                          const int renderHeight) const {
        const size_t needed = static_cast<size_t>(width * renderHeight) * 4;
        if (buffer.size() != needed) buffer.resize(needed);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, renderHeight, GL_RGBA, GL_FLOAT, buffer.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    bool GlRenderer::CreatePBOs(const int width, const int renderHeight) {
        pboSize_ = width * renderHeight * 4 * static_cast<int>(sizeof(float));
        glGenBuffers(2, pbo_);
        for (int i = 0; i < 2; ++i) {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[i]);
            glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(pboSize_),
                         nullptr, GL_STREAM_READ);
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        pboFirstFrame_ = true;
        pboWriteIdx_ = 0;
        return true;
    }

    void GlRenderer::StartAsyncReadback(const int width, const int renderHeight) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[pboWriteIdx_]);
        glReadPixels(0, 0, width, renderHeight, GL_RGBA, GL_FLOAT, 0);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        pboWriteIdx_ = 1 - pboWriteIdx_;
    }

    bool GlRenderer::FinishAsyncReadbackInto(std::vector<float> &buffer,
                                             const int width,
                                             const int renderHeight) {
        if (pboFirstFrame_) {
            pboFirstFrame_ = false;
            return false;
        }

        const int readIdx = 1 - pboWriteIdx_;
        const size_t needed = static_cast<size_t>(width * renderHeight) * 4;
        if (buffer.size() != needed) buffer.resize(needed);

        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[readIdx]);
        const void *mapped = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
                                              static_cast<GLsizeiptr>(pboSize_),
                                              GL_MAP_READ_BIT);
        if (mapped) {
            std::memcpy(buffer.data(), mapped, static_cast<size_t>(pboSize_));
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        return mapped != nullptr;
    }

    GLuint GlRenderer::CompileShader(const GLenum type, const char *src) {
        const GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok = 0;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(s, 512, nullptr, log);
            std::cerr << "[GlRenderer] Shader compile error: " << log << std::endl;
            glDeleteShader(s);
            return 0;
        }
        return s;
    }

    GLuint GlRenderer::BuildProgram(const char *vertSrc, const char *fragSrc,
                                    const char * /*geomSrc*/) {
        const GLuint vs = CompileShader(GL_VERTEX_SHADER, vertSrc);
        const GLuint fs = CompileShader(GL_FRAGMENT_SHADER, fragSrc);
        if (!vs || !fs) return 0;

        const GLuint prog = glCreateProgram();
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);
        glDeleteShader(vs);
        glDeleteShader(fs);

        GLint ok = 0;
        glGetProgramiv(prog, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetProgramInfoLog(prog, 512, nullptr, log);
            std::cerr << "[GlRenderer] Program link error: " << log << std::endl;
            glDeleteProgram(prog);
            return 0;
        }
        return prog;
    }

    void GlRenderer::SetMat4Uniform(const GLint loc, const gz::math::Matrix4d &m) {
        if (loc < 0) return;
        float f[16];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                f[i * 4 + j] = static_cast<float>(m(i, j));
        glUniformMatrix4fv(loc, 1, GL_TRUE, f);
    }

    void GlRenderer::SetMat3Uniform(const GLint loc, const gz::math::Matrix3d &m) {
        if (loc < 0) return;
        float f[9];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                f[i * 3 + j] = static_cast<float>(m(i, j));
        glUniformMatrix3fv(loc, 1, GL_TRUE, f);
    }
} // namespace blgz