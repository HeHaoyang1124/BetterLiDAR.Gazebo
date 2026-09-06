#include "Program.hpp"

namespace blgz {

Program Program::create() { return Program(GlProgram::create()); }
Program Program::adopt(GlProgram h) { return Program(std::move(h)); }

void Program::use() const { glUseProgram(handle_); }

GLint Program::uniform_location(const char *name) const {
    return glGetUniformLocation(handle_, name);
}

GLuint Program::uniform_block_index(const char *name) const {
    return glGetUniformBlockIndex(handle_, name);
}

void Program::set_uniform_block_binding(const char *name, GLuint binding) const {
    use();
    glUniformBlockBinding(handle_, glGetUniformBlockIndex(handle_, name), binding);
}

// clang-format off
void Program::set_uniform(const GLint loc, const int v)      const { use(); glUniform1i(loc, v); }
void Program::set_uniform(const GLint loc, const uint32_t v) const { use(); glUniform1ui(loc, v); }
void Program::set_uniform(const GLint loc, const float v)    const { use(); glUniform1f(loc, v); }
// clang-format on

void Program::set_uniform(GLint loc, const gz::math::Matrix3d &m) const {
    use();
    float mat[9] = {
        static_cast<float>(m(0, 0)), static_cast<float>(m(1, 0)), static_cast<float>(m(2, 0)),
        static_cast<float>(m(0, 1)), static_cast<float>(m(1, 1)), static_cast<float>(m(2, 1)),
        static_cast<float>(m(0, 2)), static_cast<float>(m(1, 2)), static_cast<float>(m(2, 2)),
    };
    glUniformMatrix3fv(loc, 1, GL_FALSE, mat);
}

void Program::set_uniform(GLint loc, const gz::math::Matrix4d &m) const {
    use();
    float mat[16] = {
        static_cast<float>(m(0, 0)), static_cast<float>(m(1, 0)), static_cast<float>(m(2, 0)), static_cast<float>(m(3, 0)),
        static_cast<float>(m(0, 1)), static_cast<float>(m(1, 1)), static_cast<float>(m(2, 1)), static_cast<float>(m(3, 1)),
        static_cast<float>(m(0, 2)), static_cast<float>(m(1, 2)), static_cast<float>(m(2, 2)), static_cast<float>(m(3, 2)),
        static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3)), static_cast<float>(m(2, 3)), static_cast<float>(m(3, 3)),
    };
    glUniformMatrix4fv(loc, 1, GL_FALSE, mat);
}

Program::operator GLuint() const { return handle_; }
Program::operator bool() const { return handle_ != 0; }

Program::Program(GlProgram h) : handle_(std::move(h)) {}

} // namespace blgz