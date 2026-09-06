#pragma once

#include <cstddef>
#include <vector>

#include <glad/gl.h>

namespace blgz {
    class AttributeLayout {
    public:
        struct Attribute {
            GLuint location;
            GLint size;
            GLenum type;
            size_t offset;
        };

    private:
        std::vector<Attribute> attributes_;
        size_t current_offset_ = 0;
        GLuint divisor_;

    public:
        template<GLenum Type>
        static constexpr size_t gl_type_size();

        template<GLenum Type>
        void add(GLuint location, GLint size);

        [[nodiscard]] GLsizei stride() const { return static_cast<GLsizei>(current_offset_); }
        [[nodiscard]] GLuint divisor() const { return divisor_; }
        [[nodiscard]] const std::vector<Attribute> &attributes() const { return attributes_; }

    protected:
        explicit AttributeLayout(const GLuint divisor)
            : divisor_(divisor) {
        }
    };

    class VertexLayout final : public AttributeLayout {
    public:
        VertexLayout();
    };

    class InstanceLayout final : public AttributeLayout {
    public:
        InstanceLayout();
    };
} // namespace blgz