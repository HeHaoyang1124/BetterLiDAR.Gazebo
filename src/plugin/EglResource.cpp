#include "EglResource.hpp"

#include <glad/egl.h>
#include <dlfcn.h>
#include <gz/common/Console.hh>

namespace blgz {
    namespace {
        template<typename F>
        F load(const char *name) {
            return reinterpret_cast<F>(dlsym(RTLD_DEFAULT, name));
        }

        template<typename F>
        F load(void *(*getProc)(const char *), const char *name) {
            return reinterpret_cast<F>(getProc(name));
        }
    }

    EglResource::~EglResource() {
        // unbind context
        if (ctx_ != EGL_NO_CONTEXT) {
            eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        }

        // destroy surface
        if (surf_ != EGL_NO_SURFACE) {
            eglDestroySurface(display_, surf_);
            surf_ = EGL_NO_SURFACE;
        }
        // destroy context
        if (ctx_ != EGL_NO_CONTEXT) {
            eglDestroyContext(display_, ctx_);
            ctx_ = EGL_NO_CONTEXT;
        }
        // terminate display
        if (display_ != EGL_NO_DISPLAY) {
            eglTerminate(display_);
            display_ = EGL_NO_DISPLAY;
        }
    }

    bool EglResource::initialize() {
        // 1. Load EGL core functions
        using SysEglGetProcAddress = void *(*)(const char *);
        const auto sysEglGetProcAddress = load<SysEglGetProcAddress>("eglGetProcAddress");
        const auto sysEglInitialize = load<EGLBoolean (*)(EGLDisplay, EGLint *, EGLint *)>("eglInitialize");
        const auto sysEglTerminate = load<EGLBoolean (*)(EGLDisplay)>("eglTerminate");
        const auto sysEglGetDisplay = load<EGLDisplay (*)(EGLNativeDisplayType)>("eglGetDisplay");
        if (!sysEglGetProcAddress) {
            gzerr << "EglResource: dlsym(eglGetProcAddress) failed" << std::endl;
            return false;
        }
        getProcAddr_ = sysEglGetProcAddress;

        // 2. Load EGL extensions functions
        using QueryDevicesExtT = EGLint (*)(EGLint, EGLDeviceEXT *, EGLint *);
        using GetPlatformDisplayExtT = EGLDisplay (*)(EGLenum, void *, const EGLint *);
        const auto eglQueryDevicesEXT = load<QueryDevicesExtT>(
            sysEglGetProcAddress, "eglQueryDevicesEXT");
        const auto eglGetPlatformDisplayEXT = load<GetPlatformDisplayExtT>(
            sysEglGetProcAddress, "eglGetPlatformDisplayEXT");

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
                        display_ = d;
                        break;
                    }
                    if (sysEglTerminate) sysEglTerminate(d);
                }
            }
        }

        if (display_ == EGL_NO_DISPLAY && sysEglGetDisplay) {
            display_ = sysEglGetDisplay(EGL_DEFAULT_DISPLAY);
        }

        if (display_ == EGL_NO_DISPLAY) {
            gzerr << "EglResource: Failed to get EGL display" << std::endl;
            return false;
        }

        eglVersion_ = gladLoadEGL(display_, reinterpret_cast<GLADloadfunc>(sysEglGetProcAddress));
        if (!eglVersion_) {
            gzerr << "EglResource: gladLoadEGL failed" << std::endl;
            return false;
        }

        if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) {
            gzerr << "EglResource: eglBindAPI failed" << std::endl;
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
        if (!eglChooseConfig(display_, configAttribs, &config, 1, &numConfigs) || numConfigs < 1) {
            gzerr << "EglResource: eglChooseConfig failed" << std::endl;
            return false;
        }

        const EGLint ctx33Core[] = {
            EGL_CONTEXT_MAJOR_VERSION, 3,
            EGL_CONTEXT_MINOR_VERSION, 3,
            EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE
        };
        ctx_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, ctx33Core);

        if (ctx_ == EGL_NO_CONTEXT) {
            const EGLint ctx33Compat[] = {
                EGL_CONTEXT_MAJOR_VERSION, 3,
                EGL_CONTEXT_MINOR_VERSION, 3,
                EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT,
                EGL_NONE
            };
            ctx_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, ctx33Compat);
        }

        if (ctx_ == EGL_NO_CONTEXT) {
            const EGLint ctxAny[] = {
                EGL_CONTEXT_MAJOR_VERSION, 3,
                EGL_CONTEXT_MINOR_VERSION, 0,
                EGL_NONE
            };
            ctx_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, ctxAny);
        }

        if (ctx_ == EGL_NO_CONTEXT) {
            gzerr << "EglResource: eglCreateContext failed" << std::endl;
            return false;
        }

        const EGLint pbufferAttribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        surf_ = eglCreatePbufferSurface(display_, config, pbufferAttribs);
        if (surf_ == EGL_NO_SURFACE) {
            gzerr << "EglResource: eglCreatePbufferSurface failed" << std::endl;
            return false;
        }

        if (eglMakeCurrent(display_, surf_, surf_, ctx_) != EGL_TRUE) {
            gzerr << "EglResource: eglMakeCurrent failed" << std::endl;
            return false;
        }

        return true;
    }
} // namespace blgz
