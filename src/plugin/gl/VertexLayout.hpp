#pragma once

#include <glad/gl.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace blgz {

class VertexLayout {
public:
    struct Element {
        GLuint location;
        GLint size;
        GLenum type;
        size_t offset;
        GLuint divisor;
    };

    VertexLayout() = default;

    void add(GLuint location, GLint size, GLenum type);
    void add_instanced(GLuint location, GLint size, GLenum type);

    [[nodiscard]] GLsizei stride() const { return stride_; }
    [[nodiscard]] const std::vector<Element> &elements() const { return elements_; }

private:
    std::vector<Element> elements_;
    GLsizei stride_ = 0;
    size_t current_offset_ = 0;
};

} // namespace blgz