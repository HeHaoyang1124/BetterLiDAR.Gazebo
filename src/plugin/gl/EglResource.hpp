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

    [[nodiscard]] int egl_version() const noexcept { return egl_version_; }

    using GetProcAddr = void *(*)(const char *);
    [[nodiscard]] GetProcAddr get_proc_address() const noexcept { return sys_egl_get_proc_address_; }

private:
    // clang-format off
    template<typename F> static F load_core(const char *name);
    template<typename F>        F load_ext (const char *name) const;

    [[nodiscard]] EGLDisplay create_device_display() const;

    using SysEglGetProcAddress   = void *(*)(const char *);
    using SysEglInitialize       = EGLBoolean (*)(EGLDisplay, EGLint *, EGLint *);
    using SysEglTerminate        = EGLBoolean (*)(EGLDisplay);
    using SysEglGetDisplay       = EGLDisplay (*)(EGLNativeDisplayType);
    using QueryDevicesExtT       = EGLint     (*)(EGLint, EGLDeviceEXT *, EGLint *);
    using GetPlatformDisplayExtT = EGLDisplay (*)(EGLenum, void *, const EGLint *);


    SysEglGetProcAddress   sys_egl_get_proc_address_     = nullptr;
    SysEglInitialize       sys_egl_initialize_           = nullptr;
    SysEglTerminate        sys_egl_terminate_            = nullptr;
    SysEglGetDisplay       sys_egl_get_display_          = nullptr;
    QueryDevicesExtT       egl_query_devices_ext_        = nullptr;
    GetPlatformDisplayExtT egl_get_platform_display_ext_ = nullptr;

    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLContext ctx_     = EGL_NO_CONTEXT;
    EGLSurface surf_    = EGL_NO_SURFACE;
    int egl_version_    = 0;
    // clang-format on
};
} // namespace blgz