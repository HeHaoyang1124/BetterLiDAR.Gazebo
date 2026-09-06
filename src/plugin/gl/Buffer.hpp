#pragma once

#include "GlResource.hpp"
#include <cstddef>

namespace blgz {
    class Buffer {
    public:
        Buffer() = default;

        static Buffer create();

        void bind(GLenum target) const;

        static void unbind(GLenum target);

        void upload(GLenum target,
                    const void *data,
                    size_t size,
                    GLenum usage = GL_STATIC_DRAW) const;

        explicit operator GLuint() const;

        explicit operator bool() const;

    private:
        explicit Buffer(GlBuffer h);

        GlBuffer handle_;
    };
} // namespace blgz
