#pragma once

#include "GlResource.hpp"

#include <gz/math/Matrix3.hh>
#include <gz/math/Matrix4.hh>
#include <array>
#include <cstdint>

namespace blgz {

class Program {
public:
    Program() = default;
    static Program create();
    static Program adopt(GlProgram h);

    void use() const;

    [[nodiscard]] GLint uniform_location(const char *name) const;
    [[nodiscard]] GLuint uniform_block_index(const char *name) const;

    void set_uniform_block_binding(const char *name, GLuint binding) const;

    void set_uniform(GLint loc, float v) const;
    void set_uniform(GLint loc, int v) const;
    void set_uniform(GLint loc, uint32_t v) const;
    template<size_t N>
    void set_uniform(GLint loc, const std::array<float, N> &v) const;

    void set_uniform(GLint loc, const gz::math::Matrix3d &m) const;
    void set_uniform(GLint loc, const gz::math::Matrix4d &m) const;

    operator GLuint() const;
    explicit operator bool() const;

private:
    explicit Program(GlProgram h);
    GlProgram handle_;
};

} // namespace blgz

// --- Template implementations ---

namespace blgz {

template<size_t N>
void Program::set_uniform(GLint loc, const std::array<float, N> &v) const {
    use();
    // clang-format off
    if      constexpr (N == 1) glUniform1fv(loc, 1, v.data());
    else if constexpr (N == 2) glUniform2fv(loc, 1, v.data());
    else if constexpr (N == 3) glUniform3fv(loc, 1, v.data());
    else if constexpr (N == 4) glUniform4fv(loc, 1, v.data());
    else    static_assert(N <= 4 && N >= 1, "set_uniform array size must be 1..4");
    // clang-format on
}

} // namespace blgz