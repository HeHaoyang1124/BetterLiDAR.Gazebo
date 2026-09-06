#pragma once

#include <glad/egl.h>

namespace blgz {
    class EglResource {
    public:
        EglResource() noexcept;

        ~EglResource();

        EglResource(const EglResource &) = delete;

        EglResource &operator=(const EglResource &) = delete;

        bool init();

        [[nodiscard]] int eglVersion() const noexcept { return egl_version_; }

        using GetProcAddr = void *(*)(const char *);
        [[nodiscard]] GetProcAddr getProcAddress() const noexcept { return sysEglGetProcAddress_; }

    private:
        // clang-format off
        template<typename F> static F loadCore(const char *name);
        template<typename F>        F loadExt (const char *name) const;

        using SysEglGetProcAddress   = void *(*)(const char *);
        using SysEglInitialize       = EGLBoolean (*)(EGLDisplay, EGLint *, EGLint *);
        using SysEglTerminate        = EGLBoolean (*)(EGLDisplay);
        using SysEglGetDisplay       = EGLDisplay (*)(EGLNativeDisplayType);
        using QueryDevicesExtT       = EGLint     (*)(EGLint, EGLDeviceEXT *, EGLint *);
        using GetPlatformDisplayExtT = EGLDisplay (*)(EGLenum, void *, const EGLint *);

        [[nodiscard]] EGLDisplay createDeviceDisplay() const;

        SysEglGetProcAddress   sysEglGetProcAddress_     = nullptr;
        SysEglInitialize       sysEglInitialize_         = nullptr;
        SysEglTerminate        sysEglTerminate_          = nullptr;
        SysEglGetDisplay       sysEglGetDisplay_         = nullptr;
        QueryDevicesExtT       eglQueryDevicesEXT_       = nullptr;
        GetPlatformDisplayExtT eglGetPlatformDisplayEXT_ = nullptr;


        EGLDisplay display_ = EGL_NO_DISPLAY;
        EGLContext ctx_     = EGL_NO_CONTEXT;
        EGLSurface surf_    = EGL_NO_SURFACE;
        int egl_version_     = 0;
            // clang-format on
    };
} // namespace blgz