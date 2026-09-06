#pragma once

#include "GlResource.hpp"
#include "VertexLayout.hpp"

namespace blgz {

class VertexArray {
public:
    VertexArray() = default;
    static VertexArray create();

    void bind() const;
    static void unbind();

    void set_layout(const VertexLayout &layout) const;

    void draw(GLenum mode, GLint first, GLsizei count) const;
    void draw_instanced(GLenum mode, GLint first, GLsizei count_vertices, GLsizei instance_count) const;

    operator GLuint() const;
    explicit operator bool() const;

private:
    explicit VertexArray(GlVertexArray h);
    GlVertexArray handle_;
};

} // namespace blgz