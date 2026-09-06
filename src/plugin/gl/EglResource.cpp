#include "EglResource.hpp"

#include <glad/egl.h>
#include <dlfcn.h>
#include <gz/common/Console.hh>
#include <span>

#define CHECK_EGL_FUNCTIONS(func_name)                             \
    if (!(func_name)) {                                             \
        gzerr << "EglResource: " #func_name " is null" << std::endl; \
        return false;                                               \
    }

namespace blgz {
    template<typename F>
    F EglResource::load_core(const char *name) {
        return reinterpret_cast<F>(dlsym(RTLD_DEFAULT, name));
    }

    template<typename F>
    F EglResource::load_ext(const char *name) const {
        return sys_egl_get_proc_address_
                   ? reinterpret_cast<F>(sys_egl_get_proc_address_(name))
                   : nullptr;
    }

    EglResource::EglResource() noexcept :
        // clang-format off
        sys_egl_get_proc_address_     (load_core<SysEglGetProcAddress> ("eglGetProcAddress")),
        sys_egl_initialize_           (load_core<SysEglInitialize>     ("eglInitialize")),
        sys_egl_terminate_            (load_core<SysEglTerminate>      ("eglTerminate")),
        sys_egl_get_display_          (load_core<SysEglGetDisplay>     ("eglGetDisplay")),
        egl_query_devices_ext_        (load_ext<QueryDevicesExtT>      ("eglQueryDevicesEXT")),
        egl_get_platform_display_ext_ (load_ext<GetPlatformDisplayExtT>("eglGetPlatformDisplayEXT")) {
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

    EGLDisplay EglResource::create_device_display() const {
        EGLDeviceEXT devices[16];
        EGLint num_devices = 0;
        // return NO_DISPLAY if any function failed
        // or if no device is found
        if (!egl_query_devices_ext_ ||
            !egl_get_platform_display_ext_ ||
            !egl_query_devices_ext_(16, devices, &num_devices) ||
            num_devices <= 0)
            return EGL_NO_DISPLAY;


        for (const auto device: std::span{devices, static_cast<size_t>(num_devices)}) {
            const auto dp = egl_get_platform_display_ext_(EGL_PLATFORM_DEVICE_EXT, device, nullptr);
            // skip if display is not valid
            if (dp == EGL_NO_DISPLAY) continue;

            // try display initialization
            EGLint major = 0, minor = 0;
            if (sys_egl_initialize_ && sys_egl_initialize_(dp, &major, &minor))
                return dp;

            // terminate display if initialization failed
            if (sys_egl_terminate_) sys_egl_terminate_(dp);
        }

        return EGL_NO_DISPLAY;
    }

    bool EglResource::init() {
        // === 0. check if any function is valid ===
        CHECK_EGL_FUNCTIONS(sys_egl_get_proc_address_);
        CHECK_EGL_FUNCTIONS(sys_egl_initialize_);
        CHECK_EGL_FUNCTIONS(sys_egl_terminate_);
        CHECK_EGL_FUNCTIONS(sys_egl_get_display_);
        CHECK_EGL_FUNCTIONS(egl_query_devices_ext_);
        CHECK_EGL_FUNCTIONS(egl_get_platform_display_ext_);

        // === 1. create display ===
        display_ = create_device_display();
        if (display_ == EGL_NO_DISPLAY) {
            gzerr << "EglResource: Failed to get EGL display" << std::endl;
            return false;
        }

        // === 2. load glad functions ===
        egl_version_ = gladLoadEGL(display_, reinterpret_cast<GLADloadfunc>(sys_egl_get_proc_address_));
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
        const EGLint config_attribs[] = {
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
        EGLint num_configs = 0;
        if (!eglChooseConfig(display_, config_attribs, &config, 1, &num_configs) || num_configs < 1) {
            gzerr << "EglResource: eglChooseConfig failed" << std::endl;
            return false;
        }

        // === 5. create context ===
        const EGLint ctx33_core[] = {
            EGL_CONTEXT_MAJOR_VERSION, 3,
            EGL_CONTEXT_MINOR_VERSION, 3,
            EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL_NONE
        };
        ctx_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, ctx33_core);
        if (ctx_ == EGL_NO_CONTEXT) {
            gzerr << "EglResource: eglCreateContext failed" << std::endl;
            return false;
        }

        // === 6. create surface ===
        const EGLint pbuffer_attribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        surf_ = eglCreatePbufferSurface(display_, config, pbuffer_attribs);
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