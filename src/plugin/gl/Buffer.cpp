#include "Buffer.hpp"

namespace blgz {

Buffer Buffer::create() { return Buffer(GlBuffer::create()); }

void Buffer::bind(GLenum target) const { glBindBuffer(target, handle_); }
void Buffer::unbind(GLenum target) { glBindBuffer(target, 0); }

void Buffer::upload(GLenum target, const void *data, size_t size, GLenum usage) const {
    glBindBuffer(target, handle_);
    glBufferData(target, static_cast<GLsizeiptr>(size), data, usage);
}

Buffer::operator GLuint() const { return handle_; }
Buffer::operator bool() const { return handle_ != 0; }

Buffer::Buffer(GlBuffer h) : handle_(std::move(h)) {}

} // namespace blgz