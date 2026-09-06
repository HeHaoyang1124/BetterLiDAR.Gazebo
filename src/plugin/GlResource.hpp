#pragma once

#include <glad/gl.h>
#include <utility>

namespace blgz {

    // clang-format off
    // --- Deleters (unified to void(GLuint)) ---
    inline void DeleteProgram     (const GLuint id) { if (id) glDeleteProgram(id); }
    inline void DeleteShader      (const GLuint id) { if (id) glDeleteShader(id); }
    inline void DeleteVertexArray (const GLuint id) { if (id) glDeleteVertexArrays(1, &id); }
    inline void DeleteBuffer      (const GLuint id) { if (id) glDeleteBuffers(1, &id); }
    inline void DeleteTexture     (const GLuint id) { if (id) glDeleteTextures(1, &id); }
    inline void DeleteFramebuffer (const GLuint id) { if (id) glDeleteFramebuffers(1, &id); }
    inline void DeleteRenderbuffer(const GLuint id) { if (id) glDeleteRenderbuffers(1, &id); }
    // clang-format on

    // clang-format off
    // --- Generators (unified to void(GLsizei, GLuint*)) ---
    inline void GenVertexArrays (const GLsizei n, GLuint *ids) { glGenVertexArrays(n, ids); }
    inline void GenBuffers      (const GLsizei n, GLuint *ids) { glGenBuffers(n, ids); }
    inline void GenTextures     (const GLsizei n, GLuint *ids) { glGenTextures(n, ids); }
    inline void GenFramebuffers (const GLsizei n, GLuint *ids) { glGenFramebuffers(n, ids); }
    inline void GenRenderbuffers(const GLsizei n, GLuint *ids) { glGenRenderbuffers(n, ids); }
    // clang-format on

    // --- RAII OpenGL resource ---

    template<void (*Deleter)(GLuint), void (*Generator)(GLsizei, GLuint *) = nullptr>
    class GlResource {
    public:
        // clang-format off

        GlResource() noexcept : id_(0) { }

        explicit GlResource(const GLuint id) noexcept : id_(id) { }

        GlResource(GlResource &&other) noexcept : id_(other.id_) { other.id_ = 0; }

        GlResource(const GlResource &) = delete;

        ~GlResource() { reset(); }

        GlResource &operator=(const GlResource &) = delete;

        GlResource &operator=(GlResource &&other) noexcept {
            std::swap(id_, other.id_);
            return *this;
            // 'other' will be reset in destructor after return
        }

        static GlResource create() noexcept {
            GLuint id = 0;
            if constexpr (Generator != nullptr) {
                Generator(1, &id);
            }
            return GlResource(id);
        }
        // clang-format on


        void reset(const GLuint newId = 0) noexcept {
            if (id_) Deleter(id_);
            id_ = newId;
        }


        [[nodiscard]] GLuint get() const noexcept { return id_; }

        operator GLuint() const noexcept { return id_; }

        // return true if it has valid id
        explicit operator bool() const noexcept { return id_ != 0; }

    private:
        GLuint id_;
    };

    // --- Type aliases ---

    using GlProgram      = GlResource<DeleteProgram>;
    using GlShader       = GlResource<DeleteShader>;
    using GlVertexArray  = GlResource<DeleteVertexArray,  GenVertexArrays>;
    using GlBuffer       = GlResource<DeleteBuffer,       GenBuffers>;
    using GlTexture      = GlResource<DeleteTexture,      GenTextures>;
    using GlFramebuffer  = GlResource<DeleteFramebuffer,  GenFramebuffers>;
    using GlRenderbuffer = GlResource<DeleteRenderbuffer, GenRenderbuffers>;
} // namespace blgz