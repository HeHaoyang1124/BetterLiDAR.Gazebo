#include "VertexLayout.hpp"

namespace blgz {

void VertexLayout::add(const GLuint location, const GLint size, const GLenum type) {
    elements_.push_back({location, size, type, current_offset_, 0});
    current_offset_ += static_cast<size_t>(size) * sizeof(float);
    stride_ = static_cast<GLsizei>(current_offset_);
}

void VertexLayout::add_instanced(const GLuint location, const GLint size, const GLenum type) {
    elements_.push_back({location, size, type, current_offset_, 1});
    current_offset_ += static_cast<size_t>(size) * sizeof(float);
    stride_ = static_cast<GLsizei>(current_offset_);
}

} // namespace blgz