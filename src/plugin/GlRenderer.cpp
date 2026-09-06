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
#include <ranges>

#include "tinyexr.h"

namespace blgz {
    using gz::sim::worldPose;

    namespace components = gz::sim::components;

    GlRenderer::GlRenderer()
        : gl_initialized_(false),
          fb_width_(0),
          fb_height_(0) {
    }

    bool GlRenderer::Init(const int width, const int height) {
        // === 1. Initialize EGL context ===
        // ===    & Load OpenGL via glad ===
        if (!egl_.init()) return false;
        const int egl_version = egl_.egl_version();
        const int gl_version = gladLoadGL(reinterpret_cast<GLADloadfunc>(egl_.get_proc_address()));
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
        if (!CreateFramebuffer(width, height)) {
            gzerr << "GlRenderer: offscreen framebuffer init failed ("
                    << width << "x" << height << ")" << std::endl;
            return false;
        }

        // === 4. Create readback buffers ===
        if (!CreateReadbackBuffers(width, height)) {
            gzerr << "GlRenderer: readback buffer allocation failed ("
                    << width << "x" << height << ")" << std::endl;
            return false;
        }

        // === 5. Create fullscreen quad VAO for plane rendering ===
        // plane.vert uses gl_VertexID, no VBO/attributes needed
        quad_vao_ = VertexArray::create();

        gzmsg << "GlRenderer:"
                << " EGL " << GLAD_VERSION_MAJOR(egl_version) << "." << GLAD_VERSION_MINOR(egl_version) << std::endl
                << " GL " << GLAD_VERSION_MAJOR(gl_version) << "." << GLAD_VERSION_MINOR(gl_version) << std::endl;

        return gl_initialized_ = true;
    }

    bool GlRenderer::CreateFramebuffer(const int width, const int height) {
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

    bool GlRenderer::LoadAndCompileShaders(const std::string &shader_dir) {
        // === mesh program ===
        mesh_program_ = BuildProgramFromFiles(shader_dir + "/lidar.vert",
                                                 shader_dir + "/lidar.frag");
        if (!mesh_program_) {
            gzerr << "GlRenderer: Failed to compile mesh program" << std::endl;
            return false;
        }
        // clang-format off
        mesh_locs_ = {
            .lidar_pos           = mesh_program_.uniform_location("uLidarPos"),
            .lidar_rot_inv       = mesh_program_.uniform_location("uLidarRotInv"),
            .hz_scale            = mesh_program_.uniform_location("uHzScale"),
            .hz_offset           = mesh_program_.uniform_location("uHzOffset"),
            .vt_scale            = mesh_program_.uniform_location("uVtScale"),
            .depth_scale         = mesh_program_.uniform_location("uDepthScale"),
            .depth_bias          = mesh_program_.uniform_location("uDepthBias"),
            .near_p              = mesh_program_.uniform_location("uNearP"),
            .far_p               = mesh_program_.uniform_location("uFarP"),
            .max_intensity       = mesh_program_.uniform_location("uMaxIntensity"),
            .frame_counter       = mesh_program_.uniform_location("uFrameCounter"),
            .range_noise_std     = mesh_program_.uniform_location("uRangeNoiseStd"),
            .intensity_noise_std = mesh_program_.uniform_location("uIntensityNoiseStd"),
            .channel_tex         = mesh_program_.uniform_location("uChannelTex"),
            .group_index         = mesh_program_.uniform_location("uGroupIndex"),
        };
        // clang-format on

        // === plane program ===
        plane_program_ = BuildProgramFromFiles(shader_dir + "/plane.vert",
                                                  shader_dir + "/plane.frag");
        if (!plane_program_) {
            gzerr << "GlRenderer: Failed to compile plane program" << std::endl;
            return false;
        }
        // clang-format off
        plane_locs_ = {
            .lidar_pos           = plane_program_.uniform_location("uLidarPos"),
            .lidar_rot           = plane_program_.uniform_location("uLidarRot"),
            .hz_scale            = plane_program_.uniform_location("uHzScale"),
            .vt_scale            = plane_program_.uniform_location("uVtScale"),
            .depth_scale         = plane_program_.uniform_location("uDepthScale"),
            .depth_bias          = plane_program_.uniform_location("uDepthBias"),
            .near_p              = plane_program_.uniform_location("uNearP"),
            .far_p               = plane_program_.uniform_location("uFarP"),
            .plane_normal        = plane_program_.uniform_location("uPlaneNormal"),
            .plane_center        = plane_program_.uniform_location("uPlaneCenter"),
            .plane_u             = plane_program_.uniform_location("uPlaneU"),
            .plane_v             = plane_program_.uniform_location("uPlaneV"),
            .plane_half_size     = plane_program_.uniform_location("uPlaneHalfSize"),
            .max_intensity       = plane_program_.uniform_location("uMaxIntensity"),
            .reflectance         = plane_program_.uniform_location("uReflectance"),
            .frame_counter       = plane_program_.uniform_location("uFrameCounter"),
            .range_noise_std     = plane_program_.uniform_location("uRangeNoiseStd"),
            .intensity_noise_std = plane_program_.uniform_location("uIntensityNoiseStd"),
            .channel_tex         = plane_program_.uniform_location("uChannelTex"),
            .group_index         = plane_program_.uniform_location("uGroupIndex"),
        };
        // clang-format on

        return true;
    }

    void GlRenderer::SetParams(const double hz_min, const double hz_max,
                                const double vt_min, const double vt_max,
                                const double near_p, const double far_p) {
        // clang-format off
        const auto hz_span = static_cast<float>(hz_max - hz_min);
        const auto vt_span = static_cast<float>(vt_max - vt_min);
        const auto dp_span = static_cast<float>(far_p  - near_p);

        hz_scale_    =  2.0f / hz_span;
        hz_min_      =  hz_min;
        hz_max_      =  hz_max;

        vt_scale_    =  2.0f / vt_span;

        depth_scale_ =  2.0f / dp_span;
        depth_bias_  = -static_cast<float>(far_p + near_p) / dp_span;

        near_p_      =  static_cast<float>(near_p);
        far_p_       =  static_cast<float>(far_p);
        // clang-format on
    }

    void GlRenderer::SetNoiseParams(const double range_noise_std,
                                      const double intensity_noise_std) {
        range_noise_std_ = static_cast<float>(range_noise_std);
        intensity_noise_std_ = static_cast<float>(intensity_noise_std);
    }

    // Reads the per-entity reflectance tag from <material>, returns 0.0 when absent.
    static float EntityReflectance(const Entity ent,
                                   const EntityComponentManager &_ecm) {
        const auto *material = _ecm.Component<components::Material>(ent);
        if (!material || !material->Data().Element()) return 0.0f;

        const auto element = material->Data().Element();
        if (!element || !element->HasElement("reflectance")) return 0.0f;

        return static_cast<float>(element->Get<double>("reflectance"));
    }

    std::vector<ChannelEntry> GlRenderer::LoadChannelData(const std::string &table_path) const {
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

    bool GlRenderer::LoadChannelTable(const std::string &table_path) {
        const auto channel_data = LoadChannelData(table_path);
        if (channel_data.empty()) return false;

        SetHzOffsets(channel_data);

        return CreateChannelTexture(channel_data);
    }

    void GlRenderer::SetHzOffsets(const std::vector<ChannelEntry> &channel_data) {
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
        const double margin_span = (hz_max_ - hz_min_) + 2.0f * max_abs;
        hz_scale_ = static_cast<float>(2.0 / margin_span);
    }

    bool GlRenderer::CreateChannelTexture(const std::vector<ChannelEntry> &ch_data) {
        // Create and bind texture
        channel_tex_ = GlTexture::create();
        glBindTexture(GL_TEXTURE_2D, channel_tex_);

        // Upload channel data as RGBA32F 1D texture
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
                     1, static_cast<int>(ch_data.size()),
                     0, GL_RGBA, GL_FLOAT,
                     reinterpret_cast<const float *>(ch_data.data()));

        // Set sampling parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);

        return channel_tex_ != 0;
    }

    bool GlRenderer::BelongsToModel(EntityComponentManager &_ecm,
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

    void GlRenderer::CollectPlane(const Entity ent, const sdf::Geometry &g,
                                const float reflectance) {
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

        // === 2. build orthonormal basis (axis_u, axis_v, normal) via Gram-Schmidt ===
        auto axis_u = (std::abs(normal.Z()) < 0.9)
                           ? gz::math::Vector3d(0, 0, 1)
                           : gz::math::Vector3d(1, 0, 0);
        const auto axis_v = normal.Cross(axis_u).Normalize();
        axis_u = axis_v.Cross(normal);


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
            .axis_u    = cast_vec(axis_u),
            .axis_v    = cast_vec(axis_v),
            .half_size = cast_vec(size * 0.5),
            .reflectance = reflectance
        });
        // clang-format on
    }

    void GlRenderer::CollectMesh(const Entity ent, const sdf::Geometry &geom,
                              const float reflectance) {
        const auto *meshShape = geom.MeshShape();
        if (!meshShape) return;

        const auto it = url_asset_map_.find(meshShape->Uri());
        if (it != url_asset_map_.end()) {
            it->second.entities.push_back(ent);
            return;
        }

        auto &asset = url_asset_map_[meshShape->Uri()];
        asset.vertices = MakeSubMeshes(geom, reflectance);
        asset.entities.push_back(ent);
    }

    void GlRenderer::CollectPrimitive(Entity ent, const sdf::Geometry &geom,
                                      const float reflectance) {
        MeshAsset asset;
        asset.vertices = MakeSubMeshes(geom, reflectance);
        asset.entities.push_back(ent);
        primitive_assets_.push_back(std::move(asset));
    }

    void GlRenderer::UploadAsset(MeshAsset &asset, EntityComponentManager &_ecm) {
        if (asset.vertices.empty() || asset.entities.empty()) return;

        asset.vao = VertexArray::create();
        asset.vbo = Buffer::create();
        asset.instance_vbo = Buffer::create();

        asset.vao.bind();

        asset.vbo.upload(GL_ARRAY_BUFFER, asset.vertices.data(),
                         asset.vertices.size() * sizeof(Vertex), GL_STATIC_DRAW);

        VertexLayout vertex_layout;
        vertex_layout.add(0, 3, GL_FLOAT);
        vertex_layout.add(1, 3, GL_FLOAT);
        vertex_layout.add(2, 1, GL_FLOAT);
        asset.vao.set_layout(vertex_layout);

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

        asset.instance_vbo.upload(GL_ARRAY_BUFFER, inst_data.data(),
                             inst_data.size() * sizeof(float), GL_STATIC_DRAW);

        VertexLayout instance_layout;
        for (int i = 0; i < 4; ++i)
            instance_layout.add_instanced(3 + i, 4, GL_FLOAT);
        for (int i = 0; i < 3; ++i)
            instance_layout.add_instanced(7 + i, 3, GL_FLOAT);
        asset.vao.set_layout(instance_layout);

        VertexArray::unbind();
        Buffer::unbind(GL_ARRAY_BUFFER);
    }

    void GlRenderer::CollectAndUploadGeometries(EntityComponentManager &_ecm,
                                                   const Entity lidar_model_entity) {
        url_asset_map_.clear();
        primitive_assets_.clear();

        _ecm.Each<components::Visual, components::Geometry>([&](const Entity &_ent,
                                                                const components::Visual *,
                                                                const components::Geometry *_geom) -> bool {
            if (BelongsToModel(_ecm, _ent, lidar_model_entity)) return true;

            const float reflectance = EntityReflectance(_ent, _ecm);

            // clang-format off
            switch (const auto &geometry = _geom->Data(); geometry.Type()) {
                using sdf::GeometryType;
                case GeometryType::PLANE: CollectPlane(_ent, geometry, reflectance); break;
                case GeometryType::MESH:  CollectMesh(_ent, geometry, reflectance);  break;
                default:                  CollectPrimitive(_ent, geometry, reflectance); break;
            }
            // clang-format on
            return true;
        });

        for (auto &asset: url_asset_map_ | std::views::values)
            UploadAsset(asset, _ecm);
        for (auto &asset: primitive_assets_)
            UploadAsset(asset, _ecm);
    }

    void GlRenderer::RenderScene(const gz::math::Pose3d &lidar_pose,
                                  const EntityComponentManager &_ecm,
                                  const double max_intensity,
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
            std::array<float, 3> wn, wc, wu, wv;
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
                pc.wu[i] = static_cast<float>(
                    rot_mat(i, 0) * r.axis_u[0] +
                    rot_mat(i, 1) * r.axis_u[1] +
                    rot_mat(i, 2) * r.axis_u[2]);
                pc.wv[i] = static_cast<float>(
                    rot_mat(i, 0) * r.axis_v[0] +
                    rot_mat(i, 1) * r.axis_v[1] +
                    rot_mat(i, 2) * r.axis_v[2]);
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
            plane_program_.set_uniform(plane_locs_.lidar_pos,
                std::array<float, 3>{
                    static_cast<float>(lidar_pose.Pos().X()),
                    static_cast<float>(lidar_pose.Pos().Y()),
                    static_cast<float>(lidar_pose.Pos().Z())});
            plane_program_.set_uniform(plane_locs_.lidar_rot, lidar_rot);

            plane_program_.set_uniform(plane_locs_.hz_scale, hz_scale_);
            plane_program_.set_uniform(plane_locs_.vt_scale, vt_scale_);
            plane_program_.set_uniform(plane_locs_.depth_scale, depth_scale_);
            plane_program_.set_uniform(plane_locs_.depth_bias, depth_bias_);
            plane_program_.set_uniform(plane_locs_.near_p, near_p_);
            plane_program_.set_uniform(plane_locs_.far_p, far_p_);

            plane_program_.set_uniform(plane_locs_.max_intensity, static_cast<float>(max_intensity));
            plane_program_.set_uniform(plane_locs_.frame_counter, frame_counter);
            plane_program_.set_uniform(plane_locs_.range_noise_std, range_noise_std_);
            plane_program_.set_uniform(plane_locs_.intensity_noise_std, intensity_noise_std_);
            plane_program_.set_uniform(plane_locs_.channel_tex, 1);
        }

        if (has_meshes) {
            mesh_program_.set_uniform(mesh_locs_.lidar_pos,
                std::array<float, 3>{
                    static_cast<float>(lidar_pose.Pos().X()),
                    static_cast<float>(lidar_pose.Pos().Y()),
                    static_cast<float>(lidar_pose.Pos().Z())});
            mesh_program_.set_uniform(mesh_locs_.lidar_rot_inv, rot_inv);

            mesh_program_.set_uniform(mesh_locs_.hz_scale, hz_scale_);
            mesh_program_.set_uniform(mesh_locs_.vt_scale, vt_scale_);
            mesh_program_.set_uniform(mesh_locs_.depth_scale, depth_scale_);
            mesh_program_.set_uniform(mesh_locs_.depth_bias, depth_bias_);
            mesh_program_.set_uniform(mesh_locs_.near_p, near_p_);
            mesh_program_.set_uniform(mesh_locs_.far_p, far_p_);

            mesh_program_.set_uniform(mesh_locs_.max_intensity, static_cast<float>(max_intensity));
            mesh_program_.set_uniform(mesh_locs_.frame_counter, frame_counter);
            mesh_program_.set_uniform(mesh_locs_.range_noise_std, range_noise_std_);
            mesh_program_.set_uniform(mesh_locs_.intensity_noise_std, intensity_noise_std_);
            mesh_program_.set_uniform(mesh_locs_.channel_tex, 1);
        }

        // === 4. Multi-pass render: one pass per group with hz channel offset ===
        const int num_groups = static_cast<int>(hz_offsets_.size());
        for (int pass = 0; pass < num_groups; ++pass) {
            const float hz_offset_pass = -hz_offsets_[static_cast<size_t>(pass)] * hz_scale_;

            if (has_planes) {
                plane_program_.use();
                glDisable(GL_DEPTH_TEST);
                glDisable(GL_CULL_FACE);

                plane_program_.set_uniform(plane_locs_.group_index, pass);

                for (const auto &pc: plane_cache) {
                    const auto &r = planes_[pc.idx];
                    plane_program_.set_uniform(plane_locs_.plane_normal, pc.wn);
                    plane_program_.set_uniform(plane_locs_.plane_center, pc.wc);
                    plane_program_.set_uniform(plane_locs_.plane_u, pc.wu);
                    plane_program_.set_uniform(plane_locs_.plane_v, pc.wv);
                    plane_program_.set_uniform(plane_locs_.plane_half_size,
                        std::array<float, 2>{r.half_size[0], r.half_size[1]});
                    plane_program_.set_uniform(plane_locs_.reflectance, r.reflectance);

                    quad_vao_.draw(GL_TRIANGLE_FAN, 0, 4);
                }
            }

            if (has_meshes) {
                mesh_program_.use();
                glEnable(GL_DEPTH_TEST);
                glEnable(GL_CULL_FACE);

                mesh_program_.set_uniform(mesh_locs_.hz_offset, hz_offset_pass);
                mesh_program_.set_uniform(mesh_locs_.group_index, pass);

                auto draw_asset = [&](const MeshAsset &asset) {
                    asset.vao.draw_instanced(GL_TRIANGLES, 0,
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

        VertexArray::unbind();
        glUseProgram(0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    std::vector<float> GlRenderer::ReadRenderResult(const int width, const int render_height) const {
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

    void GlRenderer::ReadRenderResultInto(std::vector<float> &buffer,
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

    bool GlRenderer::CreateReadbackBuffers(const int width, const int render_height) {
        // === 1. Allocate double-buffered PBOs ===
        pbo_size_ = width * render_height * 4 * static_cast<int>(sizeof(float));
        pbo_[0] = Buffer::create();
        pbo_[1] = Buffer::create();
        for (int i = 0; i < 2; ++i) {
            pbo_[i].upload(GL_PIXEL_PACK_BUFFER, nullptr,
                           static_cast<size_t>(pbo_size_), GL_STREAM_READ);
            if (glGetError() != GL_NO_ERROR) {
                gzerr << "GlRenderer: glBufferData failed ("
                        << width << "x" << render_height << ")" << std::endl;
                Buffer::unbind(GL_PIXEL_PACK_BUFFER);
                return false;
            }
        }
        Buffer::unbind(GL_PIXEL_PACK_BUFFER);

        // === 2. Init state ===
        pbo_first_frame_ = true;
        pbo_write_idx_ = 0;
        return true;
    }

    void GlRenderer::StartAsyncReadback(const int width, const int render_height) {
        // === 1. Initiate async read into current write PBO ===
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        pbo_[pbo_write_idx_].bind(GL_PIXEL_PACK_BUFFER);
        glReadPixels(0, 0, width, render_height, GL_RGBA, GL_FLOAT, 0);
        Buffer::unbind(GL_PIXEL_PACK_BUFFER);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // === 2. Swap write index ===
        pbo_write_idx_ = 1 - pbo_write_idx_;
    }

    bool GlRenderer::FinishAsyncReadbackInto(std::vector<float> &buffer,
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

        pbo_[read_idx].bind(GL_PIXEL_PACK_BUFFER);
        const void *mapped = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
                                              static_cast<GLsizeiptr>(pbo_size_),
                                              GL_MAP_READ_BIT);
        if (mapped) {
            std::memcpy(buffer.data(), mapped, static_cast<size_t>(pbo_size_));
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        }
        Buffer::unbind(GL_PIXEL_PACK_BUFFER);
        return mapped != nullptr;
    }

    std::string GlRenderer::LoadShaderFile(const std::string &path) {
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

    Program GlRenderer::BuildProgramFromFiles(const std::string &vert_path,
                                                   const std::string &frag_path) {
        // === 1. Load sources from disk ===
        const std::string vert_src = LoadShaderFile(vert_path);
        const std::string frag_src = LoadShaderFile(frag_path);
        if (vert_src.empty() || frag_src.empty()) return Program();

        // === 2. Compile and link ===
        return BuildProgram(vert_src.c_str(), frag_src.c_str());
    }

    GlShader GlRenderer::CompileShader(GLenum type, const char *src) {
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
            return {};
        }
        return GlShader(s);
    }

    Program GlRenderer::BuildProgram(const char *vert_src, const char *frag_src,
                                        const char * /*geom_src*/) {
        // === 1. Compile shaders ===
        GlShader vs = CompileShader(GL_VERTEX_SHADER, vert_src);
        GlShader fs = CompileShader(GL_FRAGMENT_SHADER, frag_src);
        if (!vs || !fs) return Program();

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
            return Program();
        }
        return Program::adopt(std::move(prog));
    }
} // namespace blgz