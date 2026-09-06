#pragma once

#include <glad/egl.h>

namespace blgz {

class EglResource {
public:
    EglResource() noexcept = default;
    ~EglResource();

    EglResource(const EglResource&) = delete;
    EglResource& operator=(const EglResource&) = delete;

    bool initialize();

    int eglVersion() const noexcept { return eglVersion_; }

    using GetProcAddr = void *(*)(const char *);
    GetProcAddr getProcAddress() const noexcept { return getProcAddr_; }

private:
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLContext ctx_ = EGL_NO_CONTEXT;
    EGLSurface surf_ = EGL_NO_SURFACE;
    int eglVersion_ = 0;
    GetProcAddr getProcAddr_ = nullptr;
};

} // namespace blgz