#include "EglResource.hpp"

#include <glad/egl.h>
#include <dlfcn.h>
#include <gz/common/Console.hh>
#include <span>

#define CHECK_EGL_FUNCTIONS(funcName)                               \
    if (!(funcName)) {                                              \
        gzerr << "EglResource: " #funcName " is null" << std::endl; \
        return false;                                               \
    }

namespace blgz {
    template<typename F>
    F EglResource::loadCore(const char *name) {
        return reinterpret_cast<F>(dlsym(RTLD_DEFAULT, name));
    }

    template<typename F>
    F EglResource::loadExt(const char *name) const {
        return sysEglGetProcAddress_
                   ? reinterpret_cast<F>(sysEglGetProcAddress_(name))
                   : nullptr;
    }

    EglResource::EglResource() noexcept :
        // clang-format off
        sysEglGetProcAddress_ (loadCore<SysEglGetProcAddress>("eglGetProcAddress")),
        sysEglInitialize_     (loadCore<SysEglInitialize>    ("eglInitialize")),
        sysEglTerminate_      (loadCore<SysEglTerminate>     ("eglTerminate")),
        sysEglGetDisplay_     (loadCore<SysEglGetDisplay>    ("eglGetDisplay")),
        eglQueryDevicesEXT_       (loadExt<QueryDevicesExtT>      ("eglQueryDevicesEXT")),
        eglGetPlatformDisplayEXT_ (loadExt<GetPlatformDisplayExtT>("eglGetPlatformDisplayEXT")) {
        // clang-format on
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

    EGLDisplay EglResource::createDeviceDisplay() const {
        EGLDeviceEXT devices[16];
        EGLint numDevices = 0;
        // return NO_DISPLAY if any function failed
        // or if no device is found
        if (!eglQueryDevicesEXT_ ||
            !eglGetPlatformDisplayEXT_ ||
            !eglQueryDevicesEXT_(16, devices, &numDevices) ||
            numDevices <= 0)
            return EGL_NO_DISPLAY;


        for (const auto device: std::span{devices, static_cast<size_t>(numDevices)}) {
            const auto dp = eglGetPlatformDisplayEXT_(EGL_PLATFORM_DEVICE_EXT, device, nullptr);
            // skip if display is not valid
            if (dp == EGL_NO_DISPLAY) continue;

            // try display initialization
            EGLint major = 0, minor = 0;
            if (sysEglInitialize_ && sysEglInitialize_(dp, &major, &minor))
                return dp;

            // terminate display if initialization failed
            if (sysEglTerminate_) sysEglTerminate_(dp);
        }

        return EGL_NO_DISPLAY;
    }

    bool EglResource::init() {
        // === 0. check if any function is valid ===
        CHECK_EGL_FUNCTIONS(sysEglGetProcAddress_);
        CHECK_EGL_FUNCTIONS(sysEglInitialize_);
        CHECK_EGL_FUNCTIONS(sysEglTerminate_);
        CHECK_EGL_FUNCTIONS(sysEglGetDisplay_);
        CHECK_EGL_FUNCTIONS(eglQueryDevicesEXT_);
        CHECK_EGL_FUNCTIONS(eglGetPlatformDisplayEXT_);

        // === 1. create display ===
        display_ = createDeviceDisplay();
        if (display_ == EGL_NO_DISPLAY) {
            gzerr << "EglResource: Failed to get EGL display" << std::endl;
            return false;
        }

        // === 2. load glad functions ===
        egl_version_ = gladLoadEGL(display_, reinterpret_cast<GLADloadfunc>(sysEglGetProcAddress_));
        if (!egl_version_) {
            gzerr << "EglResource: gladLoadEGL failed" << std::endl;
            return false;
        }

        // === 3. bind OpenGL API ===
        if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) {
            gzerr << "EglResource: eglBindAPI failed" << std::endl;
            return false;
        }

        // === 4. choose config ===
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

        // === 5. create context ===
        const EGLint ctx33Core[] = {
            EGL_CONTEXT_MAJOR_VERSION, 3,
            EGL_CONTEXT_MINOR_VERSION, 3,
            EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE
        };
        ctx_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, ctx33Core);
        if (ctx_ == EGL_NO_CONTEXT) {
            gzerr << "EglResource: eglCreateContext failed" << std::endl;
            return false;
        }

        // === 6. create surface ===
        const EGLint pbufferAttribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        surf_ = eglCreatePbufferSurface(display_, config, pbufferAttribs);
        if (surf_ == EGL_NO_SURFACE) {
            gzerr << "EglResource: eglCreatePbufferSurface failed" << std::endl;
            return false;
        }

        // === 7. make current ===
        if (eglMakeCurrent(display_, surf_, surf_, ctx_) != EGL_TRUE) {
            gzerr << "EglResource: eglMakeCurrent failed" << std::endl;
            return false;
        }

        return true;
    }
} // namespace blgz
