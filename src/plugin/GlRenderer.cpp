#include "GlRenderer.hpp"

#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>
#include <gz/common/Console.hh>

#include <dlfcn.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>

#define TINYEXR_USE_MINIZ 0
#include <zlib.h>
#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"

namespace blgz {
    using gz::sim::worldPose;

    namespace components = gz::sim::components;

    GlRenderer::GlRenderer()
        : gl_initialized_(false),
          fb_width_(0),
          fb_height_(0) {
    }

    bool GlRenderer::init(const int width, const int height) {
        // === 1. Initialize EGL context ===
        // ===    & Load OpenGL via glad ===
        if (!egl_.init()) return false;
        const int egl_version = egl_.eglVersion();
        const int gl_version = gladLoadGL(reinterpret_cast<GLADloadfunc>(egl_.getProcAddress()));
        if (!gl_version) {
            gzerr << "GlRenderer: gladLoadGL failed" << std::endl;
            return false;
        }

        // === 2. Configure global GL state ===
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CW);

        // === 3. Create offscreen framebuffer ===
        if (!create_framebuffer(width, height)) {
            gzerr << "GlRenderer: offscreen framebuffer init failed ("
                    << width << "x" << height << ")" << std::endl;
            return false;
        }

        // === 4. Create readback buffers ===
        if (!create_readback_buffers(width, height)) {
            gzerr << "GlRenderer: readback buffer allocation failed ("
                    << width << "x" << height << ")" << std::endl;
            return false;
        }

        // === 5. Create fullscreen quad VAO for plane rendering ===
        // plane.vert uses gl_VertexID, no VBO/attributes needed
        quad_vao_ = GlVertexArray::create();

        gzmsg << "GlRenderer:"
                << " EGL " << GLAD_VERSION_MAJOR(egl_version) << "." << GLAD_VERSION_MINOR(egl_version) << std::endl
                << " GL " << GLAD_VERSION_MAJOR(gl_version) << "." << GLAD_VERSION_MINOR(gl_version) << std::endl;

        return gl_initialized_ = true;
    }

    bool GlRenderer::create_framebuffer(const int width, const int height) {
        // === 1. Set framebuffer size ===
        fb_width_ = width;
        fb_height_ = height;

        // === 2. Create framebuffer object ===
        fbo_ = GlFramebuffer::create();
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

        // === 3. Create color attachment texture ===
        result_tex_ = GlTexture::create();
        glBindTexture(GL_TEXTURE_2D, result_tex_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, result_tex_, 0);

        // === 4. Set draw buffers ===
        constexpr GLenum draw_buffers[] = {GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1, draw_buffers);

        // === 5. Create depth renderbuffer ===
        depth_rb_ = GlRenderbuffer::create();
        glBindRenderbuffer(GL_RENDERBUFFER, depth_rb_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rb_);

        // === 6. Validate framebuffer completeness ===
        const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (status != GL_FRAMEBUFFER_COMPLETE) {
            gzerr << "GlRenderer: FBO incomplete (0x"
                    << std::hex << status << std::dec << ")" << std::endl;
            return false;
        }
        return true;
    }

    bool GlRenderer::load_and_compile_shaders(const std::string &shader_dir) {
        // === mesh program ===
        mesh_program_ = build_program_from_files(shader_dir + "/lidar.vert",
                                                 shader_dir + "/lidar.frag");
        if (!mesh_program_) {
            gzerr << "GlRenderer: Failed to compile mesh program" << std::endl;
            return false;
        }
        // clang-format off
        mesh_locs_ = {
            .lidar_pos           = glGetUniformLocation(mesh_program_, "uLidarPos"),
            .lidar_rot_inv       = glGetUniformLocation(mesh_program_, "uLidarRotInv"),
            .hz_scale            = glGetUniformLocation(mesh_program_, "uHzScale"),
            .hz_offset           = glGetUniformLocation(mesh_program_, "uHzOffset"),
            .vt_scale            = glGetUniformLocation(mesh_program_, "uVtScale"),
            .depth_scale         = glGetUniformLocation(mesh_program_, "uDepthScale"),
            .depth_bias          = glGetUniformLocation(mesh_program_, "uDepthBias"),
            .near_p              = glGetUniformLocation(mesh_program_, "uNearP"),
            .far_p               = glGetUniformLocation(mesh_program_, "uFarP"),
            .max_intensity       = glGetUniformLocation(mesh_program_, "uMaxIntensity"),
            .frame_counter       = glGetUniformLocation(mesh_program_, "uFrameCounter"),
            .range_noise_std     = glGetUniformLocation(mesh_program_, "uRangeNoiseStd"),
            .intensity_noise_std = glGetUniformLocation(mesh_program_, "uIntensityNoiseStd"),
            .channel_tex         = glGetUniformLocation(mesh_program_, "uChannelTex"),
            .group_index         = glGetUniformLocation(mesh_program_, "uGroupIndex"),
        };
        // clang-format on

        // === plane program ===
        plane_program_ = build_program_from_files(shader_dir + "/plane.vert",
                                                  shader_dir + "/plane.frag");
        if (!plane_program_) {
            gzerr << "GlRenderer: Failed to compile plane program" << std::endl;
            return false;
        }
        // clang-format off
        plane_locs_ = {
            .lidar_pos           = glGetUniformLocation(plane_program_, "uLidarPos"),
            .lidar_rot           = glGetUniformLocation(plane_program_, "uLidarRot"),
            .hz_scale            = glGetUniformLocation(plane_program_, "uHzScale"),
            .vt_scale            = glGetUniformLocation(plane_program_, "uVtScale"),
            .depth_scale         = glGetUniformLocation(plane_program_, "uDepthScale"),
            .depth_bias          = glGetUniformLocation(plane_program_, "uDepthBias"),
            .near_p              = glGetUniformLocation(plane_program_, "uNearP"),
            .far_p               = glGetUniformLocation(plane_program_, "uFarP"),
            .plane_normal        = glGetUniformLocation(plane_program_, "uPlaneNormal"),
            .plane_center        = glGetUniformLocation(plane_program_, "uPlaneCenter"),
            .plane_tangent       = glGetUniformLocation(plane_program_, "uPlaneTangent"),
            .plane_bitangent     = glGetUniformLocation(plane_program_, "uPlaneBitangent"),
            .plane_half_size     = glGetUniformLocation(plane_program_, "uPlaneHalfSize"),
            .max_intensity       = glGetUniformLocation(plane_program_, "uMaxIntensity"),
            .reflectance         = glGetUniformLocation(plane_program_, "uReflectance"),
            .frame_counter       = glGetUniformLocation(plane_program_, "uFrameCounter"),
            .range_noise_std     = glGetUniformLocation(plane_program_, "uRangeNoiseStd"),
            .intensity_noise_std = glGetUniformLocation(plane_program_, "uIntensityNoiseStd"),
            .channel_tex         = glGetUniformLocation(plane_program_, "uChannelTex"),
            .group_index         = glGetUniformLocation(plane_program_, "uGroupIndex"),
        };
        // clang-format on

        return true;
    }

    void GlRenderer::set_params(const double hz_min, const double hz_max,
                                const double vt_min, const double vt_max,
                                const double near_p, const double far_p) {
        // clang-format off
        const auto hz_span = static_cast<float>(hz_max - hz_min);
        const auto vt_span = static_cast<float>(vt_max - vt_min);
        const auto dp_span = static_cast<float>(far_p  - near_p);

        hz_scale_    =  2.0f / hz_span;
        hz_min_      =  static_cast<float>(hz_min);
        hz_max_      =  static_cast<float>(hz_max);

        vt_scale_    =  2.0f / vt_span;

        depth_scale_ =  2.0f / dp_span;
        depth_bias_  = -static_cast<float>(far_p + near_p) / dp_span;

        near_p_      =  static_cast<float>(near_p);
        far_p_       =  static_cast<float>(far_p);
        // clang-format on
    }

    void GlRenderer::set_noise_params(const double range_noise_std,
                                      const double intensity_noise_std) {
        range_noise_std_ = static_cast<float>(range_noise_std);
        intensity_noise_std_ = static_cast<float>(intensity_noise_std);
    }

    std::vector<ChannelEntry> GlRenderer::load_channel_data(const std::string &table_path) const {
        const int vt_samples = fb_height_;

        // === 0. Fallback: uniform channel table ===
        if (table_path.empty()) {
            std::vector<ChannelEntry> dummy(vt_samples);
            for (int row = 0; row < vt_samples; ++row) {
                dummy[row].r_min = 0.0f;
                dummy[row].r_max = std::numeric_limits<float>::infinity();
                dummy[row].hz_offset = 0.0f;
                dummy[row].group_index = 0.0f;
            }
            gzmsg << "GlRenderer: no channel table, uniform mode" << std::endl;
            return dummy;
        }

        // === 1. Load EXR from file ===
        int width = 0;
        int height = 0;
        float *pixels = nullptr;
        const char *err = nullptr;
        if (LoadEXR(&pixels, &width, &height, table_path.c_str(), &err) != TINYEXR_SUCCESS) {
            gzerr << "GlRenderer: failed to load channel table"
                    << " [" << table_path << "]: "
                    << (err ? err : "unknown error") << std::endl;
            FreeEXRErrorMessage(err);
            return {};
        }

        // === 2. Validate dimensions ===
        if (height != vt_samples || width != 1) {
            gzerr << "GlRenderer: channel table dimensions mismatch:\n "
                    << "   [" << table_path << "]\n"
                    << "   expected: " << vt_samples << "x1,\n"
                    << "   got:      " << height << "x" << width << std::endl;
            free(pixels);
            return {};
        }

        // === 3. Reinterpret flat pixels as ChannelEntry array ===
        const auto *src = reinterpret_cast<const ChannelEntry *>(pixels);
        std::vector<ChannelEntry> ch_data(src, src + vt_samples);
        free(pixels);

        gzmsg << "GlRenderer: loaded channel table from"
                << " [" << table_path << "]"
                << " (" << width << "x" << height << ")" << std::endl;

        return ch_data;
    }

    bool GlRenderer::load_channel_table(const std::string &table_path) {
        const auto channel_data = load_channel_data(table_path);
        if (channel_data.empty()) return false;

        set_hz_offsets(channel_data);

        return create_channel_texture(channel_data);
    }

    void GlRenderer::set_hz_offsets(const std::vector<ChannelEntry> &channel_data) {
        // === 1. Find max group index and pre-allocate hz_offsets_ ===
        const auto max_it = std::ranges::max_element(
            channel_data, [](const auto &x, const auto &y) { return x.group_index < y.group_index; });
        const int num_groups = static_cast<int>(max_it->group_index + 1);
        hz_offsets_.assign(num_groups, 0.0f);

        // === 2. Fill offsets and compute max absolute value ===
        float max_abs = 0.0f;
        for (const auto &entry: channel_data) {
            const auto group = static_cast<int>(entry.group_index);
            const float offset = entry.hz_offset;

            hz_offsets_[group] = offset;
            max_abs = std::max(max_abs, std::abs(offset));
        }

        // === 3. Recompute horizontal scale with margin for offsets ===
        const float margin_span = (hz_max_ - hz_min_) + 2.0f * max_abs;
        hz_scale_ = 2.0f / margin_span;
    }

    bool GlRenderer::create_channel_texture(const std::vector<ChannelEntry> &ch_data) {
        // Create and bind texture
        channel_tex_ = GlTexture::create();
        glBindTexture(GL_TEXTURE_2D, channel_tex_);

        // Upload channel data as RGBA32F 1D texture
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 1, static_cast<int>(ch_data.size()),
                     0, GL_RGBA, GL_FLOAT, reinterpret_cast<const float *>(ch_data.data()));

        // Set sampling parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);

        return channel_tex_ != 0;
    }

    bool GlRenderer::belongs_to_model(EntityComponentManager &_ecm,
                                      const Entity ent,
                                      const Entity lidar_model_entity) {
        if (lidar_model_entity == kNullEntity) return false;
        for (auto cur = ent; cur != kNullEntity;) {
            if (cur == lidar_model_entity) return true;
            auto *parent = _ecm.Component<components::ParentEntity>(cur);
            if (!parent) break;
            cur = parent->Data();
        }
        return false;
    }

    void GlRenderer::collect_plane(const Entity ent, const sdf::Geometry &g) {
        const auto *plane = g.PlaneShape();
        if (!plane) return;

        // === 1. extract plane normal (fallback to Z-up if degenerate) ===
        const auto &size = plane->Size();
        auto normal = plane->Normal();
        if (normal.Length() > 1e-6) {
            normal.Normalize();
        } else {
            normal.Set(0, 0, 1);
        }

        // === 2. build orthonormal tangent frame via Gram-Schmidt ===
        auto tangent = (std::abs(normal.Z()) < 0.9)
                           ? gz::math::Vector3d(0, 0, 1)
                           : gz::math::Vector3d(1, 0, 0);
        const auto bitangent = normal.Cross(tangent).Normalize();
        tangent = bitangent.Cross(normal);


        // === 3. push plane instance (center always at origin in local space) ===
        // clang-format off
        auto cast_vec = []<typename T>(const T &v) {
            if constexpr (requires { v.Z(); })
                return std::array{
                    static_cast<float>(v.X()),
                    static_cast<float>(v.Y()),
                    static_cast<float>(v.Z())
                };
            else
                return std::array{
                    static_cast<float>(v.X()),
                    static_cast<float>(v.Y())
                };
        };
        planes_.push_back({
            .entity    = ent,
            .normal    = cast_vec(normal),
            .center    = {0.f, 0.f, 0.f},
            .tangent   = cast_vec(tangent),
            .bitangent = cast_vec(bitangent),
            .half_size = cast_vec(size * 0.5)
        });
        // clang-format on
    }

    void GlRenderer::collect_mesh(const Entity ent, const sdf::Geometry &geom) {
        const auto *meshShape = geom.MeshShape();
        if (!meshShape) return;

        const auto it = url_asset_map_.find(meshShape->Uri());
        if (it != url_asset_map_.end()) {
            it->second.entities.push_back(ent);
            return;
        }

        auto &asset = url_asset_map_[meshShape->Uri()];
        for (auto &sub_vertices: MakeSubMeshes(geom))
            asset.vertices.insert(asset.vertices.end(),
                                  sub_vertices.begin(), sub_vertices.end());
        asset.entities.push_back(ent);
    }

    void GlRenderer::collect_primitive(Entity ent, const sdf::Geometry &geom) {
        MeshAsset asset;
        for (auto &sub_vertices: MakeSubMeshes(geom))
            asset.vertices.insert(asset.vertices.end(),
                                  sub_vertices.begin(), sub_vertices.end());
        asset.entities.push_back(ent);
        primitive_assets_.push_back(std::move(asset));
    }

    void GlRenderer::upload_asset(MeshAsset &asset, EntityComponentManager &_ecm) {
        if (asset.vertices.empty() || asset.entities.empty()) return;

        asset.vao = GlVertexArray::create();
        asset.vbo = GlBuffer::create();
        asset.instance_vbo = GlBuffer::create();

        glBindVertexArray(asset.vao);

        glBindBuffer(GL_ARRAY_BUFFER, asset.vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(asset.vertices.size() * sizeof(Vertex)),
                     asset.vertices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(sizeof(float) * 3));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(sizeof(float) * 6));

        std::vector<float> inst_data;
        for (auto ent: asset.entities) {
            auto pose = worldPose(ent, _ecm);
            const gz::math::Matrix4d model_mat(pose);
            const gz::math::Matrix3d normal_mat(pose.Rot());

            for (int col = 0; col < 4; ++col)
                for (int row = 0; row < 4; ++row)
                    inst_data.push_back(static_cast<float>(model_mat(row, col)));
            for (int col = 0; col < 3; ++col)
                for (int row = 0; row < 3; ++row)
                    inst_data.push_back(static_cast<float>(normal_mat(row, col)));
        }

        glBindBuffer(GL_ARRAY_BUFFER, asset.instance_vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(inst_data.size() * sizeof(float)),
                     inst_data.data(), GL_STATIC_DRAW);

        constexpr int inst_stride = static_cast<int>(sizeof(float) * 25);
        for (int i = 0; i < 4; ++i) {
            glEnableVertexAttribArray(3 + i);
            glVertexAttribPointer(3 + i, 4, GL_FLOAT, GL_FALSE, inst_stride,
                                  reinterpret_cast<void *>(sizeof(float) * (i * 4)));
            glVertexAttribDivisor(3 + i, 1);
        }
        for (int i = 0; i < 3; ++i) {
            glEnableVertexAttribArray(7 + i);
            glVertexAttribPointer(7 + i, 3, GL_FLOAT, GL_FALSE, inst_stride,
                                  reinterpret_cast<void *>(sizeof(float) * (16 + i * 3)));
            glVertexAttribDivisor(7 + i, 1);
        }

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void GlRenderer::collect_and_upload_geometries(EntityComponentManager &_ecm,
                                                   const Entity lidar_model_entity) {
        url_asset_map_.clear();
        primitive_assets_.clear();

        _ecm.Each<components::Visual, components::Geometry>([&](const Entity &_ent,
                                                                const components::Visual *,
                                                                const components::Geometry *_geom) -> bool {
            if (belongs_to_model(_ecm, _ent, lidar_model_entity)) return true;

            // clang-format off
            switch (const auto &geometry = _geom->Data(); geometry.Type()) {
                using sdf::GeometryType;
                case GeometryType::PLANE: collect_plane(_ent, geometry); break;
                case GeometryType::MESH:  collect_mesh(_ent, geometry);  break;
                default:                  collect_primitive(_ent, geometry); break;
            }
            // clang-format on
            return true;
        });

        for (auto &[url, asset]: url_asset_map_)
            upload_asset(asset, _ecm);
        for (auto &asset: primitive_assets_)
            upload_asset(asset, _ecm);
    }

    void GlRenderer::render_scene(const gz::math::Pose3d &lidar_pose,
                                  const EntityComponentManager &_ecm,
                                  const double max_intensity,
                                  const double reflectance,
                                  const uint32_t frame_counter) const {
        // === 1. Setup framebuffer and clear ===
        glViewport(0, 0, fb_width_, fb_height_);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto lidar_rot = gz::math::Matrix3d(lidar_pose.Rot());
        const gz::math::Matrix3d rot_inv = lidar_rot.Transposed();

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, channel_tex_);

        // === 2. Pre-compute world-space transforms ===
        struct PlaneCache {
            size_t idx;
            float wn[3], wc[3], wt[3], wb[3];
        };

        std::vector<PlaneCache> plane_cache;

        for (size_t pi = 0; pi < planes_.size(); ++pi) {
            const auto &r = planes_[pi];
            auto pose = worldPose(r.entity, _ecm);
            const auto rot_mat = gz::math::Matrix3d(pose.Rot());
            const auto &p = pose.Pos();
            PlaneCache pc{};
            pc.idx = pi;
            for (int i = 0; i < 3; ++i) {
                pc.wn[i] = static_cast<float>(
                    rot_mat(i, 0) * r.normal[0] +
                    rot_mat(i, 1) * r.normal[1] +
                    rot_mat(i, 2) * r.normal[2]);
                pc.wt[i] = static_cast<float>(
                    rot_mat(i, 0) * r.tangent[0] +
                    rot_mat(i, 1) * r.tangent[1] +
                    rot_mat(i, 2) * r.tangent[2]);
                pc.wb[i] = static_cast<float>(
                    rot_mat(i, 0) * r.bitangent[0] +
                    rot_mat(i, 1) * r.bitangent[1] +
                    rot_mat(i, 2) * r.bitangent[2]);
                pc.wc[i] = static_cast<float>(p[i]) +
                           rot_mat(i, 0) * r.center[0] +
                           rot_mat(i, 1) * r.center[1] +
                           rot_mat(i, 2) * r.center[2];
            }
            plane_cache.push_back(pc);
        }

        const bool has_planes = !plane_cache.empty();
        const bool has_meshes = !url_asset_map_.empty() || !primitive_assets_.empty();

        // === 3. Set frame-invariant uniforms ===
        if (has_planes) {
            glUseProgram(plane_program_);

            glUniform3f(plane_locs_.lidar_pos,
                        static_cast<float>(lidar_pose.Pos().X()),
                        static_cast<float>(lidar_pose.Pos().Y()),
                        static_cast<float>(lidar_pose.Pos().Z()));
            set_mat3_uniform(plane_locs_.lidar_rot, lidar_rot);

            glUniform1f(plane_locs_.hz_scale, hz_scale_);
            glUniform1f(plane_locs_.vt_scale, vt_scale_);
            glUniform1f(plane_locs_.depth_scale, depth_scale_);
            glUniform1f(plane_locs_.depth_bias, depth_bias_);
            glUniform1f(plane_locs_.near_p, near_p_);
            glUniform1f(plane_locs_.far_p, far_p_);

            glUniform1f(plane_locs_.max_intensity, static_cast<float>(max_intensity));
            glUniform1f(plane_locs_.reflectance, static_cast<float>(reflectance));
            glUniform1ui(plane_locs_.frame_counter, frame_counter);
            glUniform1f(plane_locs_.range_noise_std, range_noise_std_);
            glUniform1f(plane_locs_.intensity_noise_std, intensity_noise_std_);
            glUniform1i(plane_locs_.channel_tex, 1);
        }

        if (has_meshes) {
            glUseProgram(mesh_program_);

            glUniform3f(mesh_locs_.lidar_pos,
                        static_cast<float>(lidar_pose.Pos().X()),
                        static_cast<float>(lidar_pose.Pos().Y()),
                        static_cast<float>(lidar_pose.Pos().Z()));
            set_mat3_uniform(mesh_locs_.lidar_rot_inv, rot_inv);

            glUniform1f(mesh_locs_.hz_scale, hz_scale_);
            glUniform1f(mesh_locs_.vt_scale, vt_scale_);
            glUniform1f(mesh_locs_.depth_scale, depth_scale_);
            glUniform1f(mesh_locs_.depth_bias, depth_bias_);
            glUniform1f(mesh_locs_.near_p, near_p_);
            glUniform1f(mesh_locs_.far_p, far_p_);

            glUniform1f(mesh_locs_.max_intensity, static_cast<float>(max_intensity));
            glUniform1ui(mesh_locs_.frame_counter, frame_counter);
            glUniform1f(mesh_locs_.range_noise_std, range_noise_std_);
            glUniform1f(mesh_locs_.intensity_noise_std, intensity_noise_std_);
            glUniform1i(mesh_locs_.channel_tex, 1);
        }

        // === 4. Multi-pass render: one pass per group with hz channel offset ===
        const int num_groups = static_cast<int>(hz_offsets_.size());
        for (int pass = 0; pass < num_groups; ++pass) {
            const float hz_offset_pass = -hz_offsets_[static_cast<size_t>(pass)] * hz_scale_;

            if (has_planes) {
                glUseProgram(plane_program_);
                glDisable(GL_DEPTH_TEST);
                glDisable(GL_CULL_FACE);

                glUniform1i(plane_locs_.group_index, pass);

                for (const auto &pc: plane_cache) {
                    const auto &r = planes_[pc.idx];
                    glUniform3fv(plane_locs_.plane_normal, 1, pc.wn);
                    glUniform3fv(plane_locs_.plane_center, 1, pc.wc);
                    glUniform3fv(plane_locs_.plane_tangent, 1, pc.wt);
                    glUniform3fv(plane_locs_.plane_bitangent, 1, pc.wb);
                    glUniform2f(plane_locs_.plane_half_size,
                                r.half_size[0], r.half_size[1]);

                    glBindVertexArray(quad_vao_);
                    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
                }
            }

            if (has_meshes) {
                glUseProgram(mesh_program_);
                glEnable(GL_DEPTH_TEST);
                glEnable(GL_CULL_FACE);

                glUniform1f(mesh_locs_.hz_offset, hz_offset_pass);
                glUniform1i(mesh_locs_.group_index, pass);

                auto draw_asset = [&](const MeshAsset &asset) {
                    glBindVertexArray(asset.vao);
                    glDrawArraysInstanced(GL_TRIANGLES, 0,
                                          static_cast<GLsizei>(asset.vertices.size()),
                                          static_cast<GLsizei>(asset.entities.size()));
                };

                for (const auto &[url, asset]: url_asset_map_)
                    draw_asset(asset);
                for (const auto &asset: primitive_assets_)
                    draw_asset(asset);
            }
        }

        // === 5. Cleanup ===
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);

        glBindVertexArray(0);
        glUseProgram(0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    std::vector<float> GlRenderer::read_render_result(const int width, const int render_height) const {
        // === 1. Allocate buffer ===
        const int total_pixels = width * render_height;
        std::vector<float> result_data(total_pixels * 4);

        // === 2. Read pixels from framebuffer ===
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, render_height, GL_RGBA, GL_FLOAT, result_data.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        return result_data;
    }

    void GlRenderer::read_render_result_into(std::vector<float> &buffer,
                                             const int width,
                                             const int render_height) const {
        // === 1. Ensure buffer size ===
        const size_t needed = static_cast<size_t>(width * render_height) * 4;
        if (buffer.size() != needed) buffer.resize(needed);

        // === 2. Read pixels from framebuffer ===
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, render_height, GL_RGBA, GL_FLOAT, buffer.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    bool GlRenderer::create_readback_buffers(const int width, const int render_height) {
        // === 1. Allocate double-buffered PBOs ===
        pbo_size_ = width * render_height * 4 * static_cast<int>(sizeof(float));
        pbo_[0] = GlBuffer::create();
        pbo_[1] = GlBuffer::create();
        for (int i = 0; i < 2; ++i) {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[i]);
            glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(pbo_size_),
                         nullptr, GL_STREAM_READ);
            if (glGetError() != GL_NO_ERROR) {
                gzerr << "GlRenderer: glBufferData failed ("
                        << width << "x" << render_height << ")" << std::endl;
                glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
                return false;
            }
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

        // === 2. Init state ===
        pbo_first_frame_ = true;
        pbo_write_idx_ = 0;
        return true;
    }

    void GlRenderer::start_async_readback(const int width, const int render_height) {
        // === 1. Initiate async read into current write PBO ===
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[pbo_write_idx_]);
        glReadPixels(0, 0, width, render_height, GL_RGBA, GL_FLOAT, 0);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // === 2. Swap write index ===
        pbo_write_idx_ = 1 - pbo_write_idx_;
    }

    bool GlRenderer::finish_async_readback_into(std::vector<float> &buffer,
                                                const int width,
                                                const int render_height) {
        // === 1. First frame has no data yet ===
        if (pbo_first_frame_) {
            pbo_first_frame_ = false;
            return false;
        }

        // === 2. Map and copy from previous-frame PBO ===
        const int read_idx = 1 - pbo_write_idx_;
        const size_t needed = static_cast<size_t>(width * render_height) * 4;
        if (buffer.size() != needed) buffer.resize(needed);

        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[read_idx]);
        const void *mapped = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
                                              static_cast<GLsizeiptr>(pbo_size_),
                                              GL_MAP_READ_BIT);
        if (mapped) {
            std::memcpy(buffer.data(), mapped, static_cast<size_t>(pbo_size_));
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        return mapped != nullptr;
    }

    std::string GlRenderer::load_shader_file(const std::string &path) {
        // === 1. Read entire file into string ===
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            std::cerr << "[GlRenderer] Failed to open shader file: " << path << std::endl;
            return {};
        }
        std::ostringstream oss;
        oss << ifs.rdbuf();
        return oss.str();
    }

    GlProgram GlRenderer::build_program_from_files(const std::string &vert_path,
                                                   const std::string &frag_path) {
        // === 1. Load sources from disk ===
        const std::string vert_src = load_shader_file(vert_path);
        const std::string frag_src = load_shader_file(frag_path);
        if (vert_src.empty() || frag_src.empty()) return GlProgram();

        // === 2. Compile and link ===
        return build_program(vert_src.c_str(), frag_src.c_str());
    }

    GlShader GlRenderer::compile_shader(GLenum type, const char *src) {
        // === 1. Create and compile ===
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);

        // === 2. Check result ===
        GLint ok = 0;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(s, 512, nullptr, log);
            std::cerr << "[GlRenderer] Shader compile error: " << log << std::endl;
            glDeleteShader(s);
            return GlShader();
        }
        return GlShader(s);
    }

    GlProgram GlRenderer::build_program(const char *vert_src, const char *frag_src,
                                        const char * /*geom_src*/) {
        // === 1. Compile shaders ===
        GlShader vs = compile_shader(GL_VERTEX_SHADER, vert_src);
        GlShader fs = compile_shader(GL_FRAGMENT_SHADER, frag_src);
        if (!vs || !fs) return GlProgram();

        // === 2. Link program ===
        GlProgram prog(glCreateProgram());
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);

        // === 3. Check result ===
        GLint ok = 0;
        glGetProgramiv(prog, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetProgramInfoLog(prog, 512, nullptr, log);
            std::cerr << "[GlRenderer] Program link error: " << log << std::endl;
            return GlProgram();
        }
        return prog;
    }

    void GlRenderer::set_mat4_uniform(const GLint loc, const gz::math::Matrix4d &m) {
        if (loc < 0) return;
        float f[16];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                f[i * 4 + j] = static_cast<float>(m(i, j));
        glUniformMatrix4fv(loc, 1, GL_TRUE, f);
    }

    void GlRenderer::set_mat3_uniform(const GLint loc, const gz::math::Matrix3d &m) {
        if (loc < 0) return;
        float f[9];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                f[i * 3 + j] = static_cast<float>(m(i, j));
        glUniformMatrix3fv(loc, 1, GL_TRUE, f);
    }
} // namespace blgz