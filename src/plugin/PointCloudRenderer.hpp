#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <glad/gl.h>
#include <gz/math/Pose3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <sdf/Geometry.hh>

#include "gl/Buffer.hpp"
#include "gl/EglResource.hpp"
#include "gl/GlResource.hpp"
#include "gl/Program.hpp"
#include "gl/VertexArray.hpp"
#include "mesh_generator.hpp"

struct ChannelEntry {
    float r_min;
    float r_max;
    float hz_offset;
    float group_index;
};

struct RenderParams {
    int width = 0;
    int height = 0;

    double hz_min = 0.0;
    double hz_max = 0.0;
    double vt_min = 0.0;
    double vt_max = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
    double range_noise_std = 0.0;
    double intensity_noise_std = 0.0;

    std::string shader_dir;
    std::string channel_table_path;
};

namespace blgz {
    using gz::sim::Entity;
    using gz::sim::EntityComponentManager;
    using gz::sim::kNullEntity;

    class PointCloudRenderer {
    private:
        // === Mesh Assets === //
        struct MeshAsset {
            VertexArray vao;
            Buffer vbo;
            Buffer instance_vbo;
            std::vector<Vertex> vertices;
            std::vector<Entity> entities;
        };

        std::unordered_map<std::string, MeshAsset> url_asset_map_;
        std::vector<MeshAsset> primitive_assets_;

        // === OpenGL Resources === //
        EglResource egl_;
        bool gl_initialized_;

        Program mesh_program_;
        Program plane_program_;
        VertexArray plane_vao_;
        Buffer plane_instance_vbo_;
        GlFramebuffer fbo_;
        GlTexture result_tex_;
        GlRenderbuffer depth_rb_;

        int fb_width_;
        int fb_height_;

        float hz_scale_ = 0.0f;
        float vt_scale_ = 0.0f;
        float depth_scale_ = 0.0f;
        float depth_bias_ = 0.0f;
        float near_p_ = 0.0f;
        float far_p_ = 0.0f;
        double hz_min_ = 0.0;
        double hz_max_ = 0.0;
        float range_noise_std_ = 0.0f;
        float intensity_noise_std_ = 0.0f;

        // clang-format off
        struct MeshUniformLocs {
            GLint lidar_pos           = -1;
            GLint lidar_rot_inv       = -1;
            GLint hz_scale            = -1;
            GLint hz_offset           = -1;
            GLint vt_scale            = -1;
            GLint depth_scale         = -1;
            GLint depth_bias          = -1;
            GLint near_p              = -1;
            GLint far_p               = -1;
            GLint max_intensity       = -1;
            GLint frame_counter       = -1;
            GLint range_noise_std     = -1;
            GLint intensity_noise_std = -1;
            GLint channel_tex         = -1;
            GLint group_index         = -1;
        } mesh_locs_;

        struct PlaneUniformLocs {
            GLint lidar_pos           = -1;
            GLint lidar_rot           = -1;
            GLint hz_scale            = -1;
            GLint vt_scale            = -1;
            GLint depth_scale         = -1;
            GLint depth_bias          = -1;
            GLint near_p              = -1;
            GLint far_p               = -1;
            GLint max_intensity       = -1;
            GLint frame_counter       = -1;
            GLint range_noise_std     = -1;
            GLint intensity_noise_std = -1;
            GLint channel_tex         = -1;
            GLint group_index         = -1;
        } plane_locs_;

        struct PlaneRenderable {
            Entity entity{};
            std::array<float, 3> normal{};
            std::array<float, 3> center{};
            std::array<float, 3> axis_u{};
            std::array<float, 3> axis_v{};
            std::array<float, 2> half_size{};
            float reflectance = 0.0f;
        };
        // clang-format on

        std::vector<PlaneRenderable> planes_;

        Buffer pbo_[2];
        int pbo_write_idx_ = 0;
        bool pbo_first_frame_ = true;
        int pbo_size_ = 0;

        GlTexture channel_tex_;
        std::vector<float> hz_offsets_{0.0f};

    public:
        PointCloudRenderer();

        ~PointCloudRenderer() = default;

        bool Init(const RenderParams &params);

        void CollectAndUploadGeometries(EntityComponentManager &_ecm, Entity lidar_model_entity = kNullEntity);

        void RenderScene(const gz::math::Pose3d &lidar_pose,
                         const EntityComponentManager &_ecm,
                         double max_intensity,
                         uint32_t frame_counter) const;

        std::vector<float> ReadRenderResult(int width, int height) const;

        void ReadRenderResultInto(std::vector<float> &buffer, int width, int height) const;

        void StartAsyncReadback();

        void FinishAsyncReadbackInto(std::vector<float> &buffer);

        int get_frame_buffer_width() const { return fb_width_; }
        int get_render_height() const { return fb_height_; }
        bool is_initialized() const { return gl_initialized_; }

    private:
        bool CreateFramebuffer(int width, int height);

        bool CreateReadbackBuffers(int width, int height);

        bool LoadAndCompileShaders(const std::string &shader_dir);

        void SetParams(double hz_min, double hz_max,
                       double vt_min, double vt_max,
                       double near_p, double far_p);

        void SetNoiseParams(double range_noise_std, double intensity_noise_std);

        bool LoadChannelTable(const std::string &table_path);

        std::vector<ChannelEntry> LoadChannelData(const std::string &table_path) const;

        bool CreateChannelTexture(const std::vector<ChannelEntry> &ch_data);

        static GlShader CompileShader(GLenum type, const char *src);

        static Program BuildProgram(const char *vert_src,
                                    const char *frag_src);

        static Program BuildProgramFromFiles(const std::string &vert_path,
                                             const std::string &frag_path);

        static std::string LoadShaderFile(const std::string &path);

        static bool BelongsToModel(EntityComponentManager &_ecm,
                                   Entity ent,
                                   Entity lidar_model_entity);

        void CollectPlane(Entity ent, const sdf::Geometry &geom, float reflectance);

        void CollectMesh(Entity ent, const sdf::Geometry &geom, float reflectance);

        void CollectPrimitive(Entity ent, const sdf::Geometry &geom, float reflectance);

        static void UploadAsset(MeshAsset &asset, const EntityComponentManager &_ecm);

        void UploadPlanes(const EntityComponentManager &_ecm);
    };
} // namespace blgz