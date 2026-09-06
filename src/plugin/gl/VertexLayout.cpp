#include "VertexLayout.hpp"

namespace blgz {
    template<GLenum Type>
    constexpr size_t AttributeLayout::gl_type_size() {
        // clang-format off
        if      constexpr (Type == GL_BYTE)           return sizeof(GLbyte);
        else if constexpr (Type == GL_SHORT)          return sizeof(GLshort);
        else if constexpr (Type == GL_INT)            return sizeof(GLint);
        else if constexpr (Type == GL_FLOAT)          return sizeof(GLfloat);
        else if constexpr (Type == GL_DOUBLE)         return sizeof(GLdouble);
        else if constexpr (Type == GL_UNSIGNED_BYTE)  return sizeof(GLubyte);
        else if constexpr (Type == GL_UNSIGNED_SHORT) return sizeof(GLushort);
        else if constexpr (Type == GL_UNSIGNED_INT)   return sizeof(GLuint);
        else if constexpr (Type == GL_HALF_FLOAT)     return sizeof(GLhalf);
        else static_assert(false, "unsupported vertex attribute type");
        // clang-format on
    }

    template<GLenum Type>
    void AttributeLayout::add(const GLuint location, const GLint size) {
        attributes_.push_back({
            .location = location,
            .size = size,
            .type = Type,
            .offset = current_offset_,
        });
        current_offset_ += static_cast<size_t>(size) * gl_type_size<Type>();
    }
    template void AttributeLayout::add<GL_FLOAT>(GLuint, GLint);
    // template void AttributeLayout::add<GL_BYTE>          (GLuint, GLint);
    // template void AttributeLayout::add<GL_SHORT>         (GLuint, GLint);
    // template void AttributeLayout::add<GL_INT>           (GLuint, GLint);
    // template void AttributeLayout::add<GL_DOUBLE>        (GLuint, GLint);
    // template void AttributeLayout::add<GL_UNSIGNED_BYTE> (GLuint, GLint);
    // template void AttributeLayout::add<GL_UNSIGNED_SHORT>(GLuint, GLint);
    // template void AttributeLayout::add<GL_UNSIGNED_INT>  (GLuint, GLint);
    // template void AttributeLayout::add<GL_HALF_FLOAT>    (GLuint, GLint);

    VertexLayout::VertexLayout()
        : AttributeLayout(0) {
    }

    InstanceLayout::InstanceLayout()
        : AttributeLayout(1) {
    }

} // namespace blgz