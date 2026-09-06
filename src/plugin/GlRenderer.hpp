#pragma once

#include <glad/egl.h>
#include <glad/gl.h>

#include <gz/math/Matrix3.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>
#include <sdf/Geometry.hh>

#include "mesh_generator.hpp"
#include "glb_parser.hpp"
#include "GlResource.hpp"
#include "EglResource.hpp"

#include <array>
#include <unordered_map>
#include <vector>
#include <string>
#include <cstdint>

struct ChannelEntry {
    float r_min;
    float r_max;
    float hz_offset;
    float group_index;
};

namespace blgz {
    using gz::sim::Entity;
    using gz::sim::EntityComponentManager;
    using gz::sim::kNullEntity;

    class GlRenderer {
    public:
        GlRenderer();

        ~GlRenderer() = default;

        bool init(int width, int height);

        bool create_framebuffer(int width, int height);

        bool load_channel_table(const std::string &table_path);

        std::vector<ChannelEntry> load_channel_data(const std::string &table_path) const;

        bool load_and_compile_shaders(const std::string &shader_dir);

        void set_params(double hz_min, double hz_max,
                       double vt_min, double vt_max,
                       double near_p, double far_p);

        void collect_and_upload_geometries(EntityComponentManager &_ecm, Entity lidar_model_entity = kNullEntity);

        void render_scene(const gz::math::Pose3d &lidar_pose,
                         const EntityComponentManager &_ecm,
                         double max_intensity,
                         double reflectance,
                         uint32_t frame_counter) const;

        void set_noise_params(double range_noise_std, double intensity_noise_std);

        void set_hz_offsets(const std::vector<ChannelEntry> &channel_data);

        bool create_channel_texture(const std::vector<ChannelEntry> &ch_data);

        std::vector<float> read_render_result(int width, int render_height) const;

        void read_render_result_into(std::vector<float> &buffer, int width, int render_height) const;

        bool create_readback_buffers(int width, int render_height);

        void start_async_readback(int width, int render_height);

        bool finish_async_readback_into(std::vector<float> &buffer, int width, int render_height);

        int get_frame_buffer_width() const { return fb_width_; }
        int get_render_height() const { return fb_height_; }
        bool is_initialized() const { return gl_initialized_; }

    private:
        static GlShader compile_shader(GLenum type, const char *src);

        static GlProgram build_program(const char *vert_src, const char *frag_src,
                                      const char *geom_src = nullptr);

        static GlProgram build_program_from_files(const std::string &vert_path,
                                               const std::string &frag_path);

        static std::string load_shader_file(const std::string &path);

        static void set_mat4_uniform(GLint loc, const gz::math::Matrix4d &m);

        static void set_mat3_uniform(GLint loc, const gz::math::Matrix3d &m);

        static bool belongs_to_model(EntityComponentManager &_ecm,
                                    Entity ent,
                                    Entity lidar_model_entity);

        struct MeshAsset {
            GlVertexArray vao;
            GlBuffer vbo;
            GlBuffer instance_vbo;
            std::vector<Vertex> vertices;
            std::vector<Entity> entities;
        };

        void collect_plane(Entity ent, const sdf::Geometry &g);
        void collect_mesh(Entity ent, const sdf::Geometry &geom);
        void collect_primitive(Entity ent, const sdf::Geometry &geom);
        void upload_asset(MeshAsset &asset, EntityComponentManager &_ecm);

        std::unordered_map<std::string, MeshAsset> url_asset_map_;
        std::vector<MeshAsset> primitive_assets_;

        EglResource egl_;
        bool gl_initialized_;

        GlProgram mesh_program_;
        GlProgram plane_program_;
        GlVertexArray quad_vao_;
        GlFramebuffer fbo_;
        GlTexture result_tex_;
        GlRenderbuffer depth_rb_;

        int fb_width_;
        int fb_height_;

        float hz_scale_ = 0.0f;
        float vt_scale_ = 0.0f;
        float depth_scale_ = 0.0f, depth_bias_ = 0.0f;
        float near_p_ = 0.0f, far_p_ = 0.0f;
        float hz_min_ = 0.0f, hz_max_ = 0.0f;
        float range_noise_std_ = 0.0f;
        float intensity_noise_std_ = 0.0f;

        struct MeshUniformLocs {
            GLint lidar_pos = -1;
            GLint lidar_rot_inv = -1;
            GLint hz_scale = -1, hz_offset = -1;
            GLint vt_scale = -1;
            GLint depth_scale = -1, depth_bias = -1;
            GLint near_p = -1, far_p = -1;
            GLint max_intensity = -1;
            GLint frame_counter = -1;
            GLint range_noise_std = -1;
            GLint intensity_noise_std = -1;
            GLint channel_tex = -1;
            GLint group_index = -1;
        } mesh_locs_;

        struct PlaneUniformLocs {
            GLint lidar_pos = -1;
            GLint lidar_rot = -1;
            GLint hz_scale = -1;
            GLint vt_scale = -1;
            GLint depth_scale = -1, depth_bias = -1;
            GLint near_p = -1, far_p = -1;
            GLint plane_normal = -1;
            GLint plane_center = -1;
            GLint plane_tangent = -1;
            GLint plane_bitangent = -1;
            GLint plane_half_size = -1;
            GLint max_intensity = -1;
            GLint reflectance = -1;
            GLint frame_counter = -1;
            GLint range_noise_std = -1;
            GLint intensity_noise_std = -1;
            GLint channel_tex = -1;
            GLint group_index = -1;
        } plane_locs_;

        struct PlaneRenderable {
            Entity entity{};
            std::array<float, 3> normal{};
            std::array<float, 3> center{};
            std::array<float, 3> tangent{};
            std::array<float, 3> bitangent{};
            std::array<float, 2> half_size{};
        };

        std::vector<PlaneRenderable> planes_;

        GlBuffer pbo_[2];
        int pbo_write_idx_ = 0;
        bool pbo_first_frame_ = true;
        int pbo_size_ = 0;

        GlTexture channel_tex_;
        std::vector<float> hz_offsets_{0.0f};
    };
} // namespace blgz