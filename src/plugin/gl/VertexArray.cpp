#include "VertexArray.hpp"

namespace blgz {
    VertexArray VertexArray::create() { return VertexArray(GlVertexArray::create()); }

    void VertexArray::bind() const {
        glBindVertexArray(handle_);
    }

    void VertexArray::unbind() {
        glBindVertexArray(0);
    }

    void VertexArray::set_layout(const AttributeLayout &layout) const {
        bind();
        const GLsizei stride = layout.stride();
        const GLuint divisor = layout.divisor();
        for (const auto &[location, size, type, offset] : layout.attributes()) {
            glEnableVertexAttribArray(location);
            glVertexAttribPointer(location, size, type, GL_FALSE,
                                  stride, reinterpret_cast<const void *>(offset));
            glVertexAttribDivisor(location, divisor);
        }
    }

    void VertexArray::draw(const GLenum mode,
                           const GLint first,
                           const GLsizei count) const {
        bind();
        glDrawArrays(mode, first, count);
    }

    void VertexArray::draw_instanced(const GLenum mode,
                                     const GLint first,
                                     const GLsizei count_vertices,
                                     const GLsizei instance_count) const {
        bind();
        glDrawArraysInstanced(mode, first, count_vertices, instance_count);
    }

    VertexArray::operator GLuint() const { return handle_; }
    VertexArray::operator bool() const { return handle_ != 0; }

    VertexArray::VertexArray(GlVertexArray h) : handle_(std::move(h)) {
    }
} // namespace blgz