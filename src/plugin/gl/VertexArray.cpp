#include "VertexArray.hpp"

namespace blgz {
    VertexArray VertexArray::create() { return VertexArray(GlVertexArray::create()); }

    void VertexArray::bind() const {
        glBindVertexArray(handle_);
    }

    void VertexArray::unbind() {
        glBindVertexArray(0);
    }

    void VertexArray::set_layout(const VertexLayout &layout) const {
        bind();
        const GLsizei stride = layout.stride();
        for (const auto &elem: layout.elements()) {
            glEnableVertexAttribArray(elem.location);
            glVertexAttribPointer(elem.location, elem.size, elem.type, GL_FALSE,
                                  stride, reinterpret_cast<const void *>(elem.offset));
            if (elem.divisor > 0) {
                glVertexAttribDivisor(elem.location, elem.divisor);
            }
        }
    }

    void VertexArray::draw(GLenum mode, GLint first, GLsizei count) const {
        bind();
        glDrawArrays(mode, first, count);
    }

    void VertexArray::draw_instanced(GLenum mode, GLint first, GLsizei count_vertices, GLsizei instance_count) const {
        bind();
        glDrawArraysInstanced(mode, first, count_vertices, instance_count);
    }

    VertexArray::operator GLuint() const { return handle_; }
    VertexArray::operator bool() const { return handle_ != 0; }

    VertexArray::VertexArray(GlVertexArray h) : handle_(std::move(h)) {
    }
} // namespace blgz