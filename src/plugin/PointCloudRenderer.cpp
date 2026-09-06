#include "PointCloudRenderer.hpp"
#include "glb_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <ranges>
#include <sstream>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/components.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

#define TINYEXR_USE_MINIZ 0
#include <zlib.h>
#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"

namespace blgz {
    using gz::sim::worldPose;

    namespace components = gz::sim::components;

    PointCloudRenderer::PointCloudRenderer()
        : gl_initialized_(false),
          fb_width_(0),
          fb_height_(0) {
    }

    bool PointCloudRenderer::Init(const RenderParams &params) {
        const int width = params.width;
        const int height = params.height;

        // === 1. Initialize EGL context ===
        // ===    & Load OpenGL via glad ===
        if (!egl_.init()) return false;
        const int egl_version = egl_.egl_version();
        const int gl_version = gladLoadGL(reinterpret_cast<GLADloadfunc>(egl_.get_proc_address()));
        if (!gl_version) {
            gzerr << "PointCloudRenderer: gladLoadGL failed" << std::endl;
            return false;
        }

        // === 2. Configure global GL state ===
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);

        // === 3. Create offscreen framebuffer ===
        if (!CreateFramebuffer(width, height)) {
            gzerr << "PointCloudRenderer: offscreen framebuffer init failed ("
                    << width << "x" << height << ")" << std::endl;
            return false;
        }

        // === 4. Create readback buffers ===
        if (!CreateReadbackBuffers(width, height)) {
            gzerr << "PointCloudRenderer: readback buffer allocation failed ("
                    << width << "x" << height << ")" << std::endl;
            return false;
        }

        // === 5. Create fullscreen quad VAO + instance VBO for plane rendering ===
        // plane.vert 使用 gl_VertexID 生成全屏四边形，无需顶点属性；
        // 每个平面实例的 pose 与局部参数通过 per-instance 属性上传。
        plane_vao_ = VertexArray::create();
        plane_instance_vbo_ = Buffer::create();

        // === 6. Compile shaders ===
        if (!LoadAndCompileShaders(params.shader_dir)) {
            gzerr << "PointCloudRenderer: failed to compile shaders" << std::endl;
            return false;
        }

        // === 7. Set renderer parameters ===
        SetParams(params.hz_min, params.hz_max,
                  params.vt_min, params.vt_max,
                  params.range_min, params.range_max);

        SetNoiseParams(params.range_noise_std, params.intensity_noise_std);

        // === 8. Load LiDAR channel table (optional) ===
        if (!LoadChannelTable(params.channel_table_path)) {
            gzerr << "PointCloudRenderer: failed to load channel table" << std::endl;
            return false;
        }

        gzmsg << "PointCloudRenderer:"
                << " EGL " << GLAD_VERSION_MAJOR(egl_version) << "." << GLAD_VERSION_MINOR(egl_version) << std::endl
                << " GL " << GLAD_VERSION_MAJOR(gl_version) << "." << GLAD_VERSION_MINOR(gl_version) << std::endl;

        return gl_initialized_ = true;
    }

    bool PointCloudRenderer::CreateFramebuffer(const int width, const int height) {
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
            gzerr << "PointCloudRenderer: FBO incomplete (0x"
                    << std::hex << status << std::dec << ")" << std::endl;
            return false;
        }
        return true;
    }

    bool PointCloudRenderer::LoadAndCompileShaders(const std::string &shader_dir) {
        // === mesh program ===
        mesh_program_ = BuildProgramFromFiles(shader_dir + "/lidar.vert",
                                              shader_dir + "/lidar.frag");
        if (!mesh_program_) {
            gzerr << "PointCloudRenderer: Failed to compile mesh program" << std::endl;
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
            gzerr << "PointCloudRenderer: Failed to compile plane program" << std::endl;
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
            .max_intensity       = plane_program_.uniform_location("uMaxIntensity"),
            .frame_counter       = plane_program_.uniform_location("uFrameCounter"),
            .range_noise_std     = plane_program_.uniform_location("uRangeNoiseStd"),
            .intensity_noise_std = plane_program_.uniform_location("uIntensityNoiseStd"),
            .channel_tex         = plane_program_.uniform_location("uChannelTex"),
            .group_index         = plane_program_.uniform_location("uGroupIndex"),
        };
        // clang-format on

        return true;
    }

    void PointCloudRenderer::SetParams(const double hz_min, const double hz_max,
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

    void PointCloudRenderer::SetNoiseParams(const double range_noise_std,
                                            const double intensity_noise_std) {
        range_noise_std_ = static_cast<float>(range_noise_std);
        intensity_noise_std_ = static_cast<float>(intensity_noise_std);
    }

    // Reads the per-entity reflectance from the custom
    // <visual><plugin><lidar_lambert><reflectance> tag. Falls back to
    // kUnsetReflectance (a tiny value) when the tag is absent, so that
    // unconfigured entities show up as near-zero intensity in the cloud.
    static float EntityReflectance(const Entity ent,
                                   const EntityComponentManager &_ecm) {
        const auto *plugin = _ecm.Component<components::VisualPlugin>(ent);
        if (!plugin) return kDefaultReflectance;

        const sdf::ElementPtr plugin_elem = plugin->Data();
        const sdf::ElementPtr lambert =
                plugin_elem ? plugin_elem->FindElement("lidar_lambert") : nullptr;
        const sdf::ElementPtr reflectance =
                lambert ? lambert->FindElement("reflectance") : nullptr;
        return reflectance ? reflectance->Get<float>() : kDefaultReflectance;
    }

    std::vector<ChannelEntry> PointCloudRenderer::LoadChannelData(const std::string &table_path) const {
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
            gzmsg << "PointCloudRenderer: no channel table, uniform mode" << std::endl;
            return dummy;
        }

        // === 1. Load EXR from file ===
        int width = 0;
        int height = 0;
        float *pixels = nullptr;
        const char *err = nullptr;
        if (LoadEXR(&pixels, &width, &height, table_path.c_str(), &err) != TINYEXR_SUCCESS) {
            gzerr << "PointCloudRenderer: failed to load channel table"
                    << " [" << table_path << "]: "
                    << (err ? err : "unknown error") << std::endl;
            FreeEXRErrorMessage(err);
            return {};
        }

        // === 2. Validate dimensions ===
        if (height != vt_samples || width != 1) {
            gzerr << "PointCloudRenderer: channel table dimensions mismatch:\n "
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

        gzmsg << "PointCloudRenderer: loaded channel table from"
                << " [" << table_path << "]"
                << " (" << width << "x" << height << ")" << std::endl;

        return ch_data;
    }

    bool PointCloudRenderer::LoadChannelTable(const std::string &table_path) {
        const auto channel_data = LoadChannelData(table_path);
        if (channel_data.empty()) return false;

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

        return CreateChannelTexture(channel_data);
    }

    bool PointCloudRenderer::CreateChannelTexture(const std::vector<ChannelEntry> &ch_data) {
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

    bool PointCloudRenderer::BelongsToModel(EntityComponentManager &_ecm,
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

    void PointCloudRenderer::CollectPlane(const Entity ent,
                                          const sdf::Geometry &geom,
                                          const float reflectance) {
        const auto *plane = geom.PlaneShape();
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

    void PointCloudRenderer::CollectMesh(const Entity ent,
                                         const sdf::Geometry &geom,
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

    void PointCloudRenderer::CollectPrimitive(const Entity ent,
                                              const sdf::Geometry &geom,
                                              const float reflectance) {
        MeshAsset asset;
        asset.vertices = MakeSubMeshes(geom, reflectance);
        asset.entities.push_back(ent);
        primitive_assets_.push_back(std::move(asset));
    }

    void PointCloudRenderer::UploadAsset(MeshAsset &asset,
                                         const EntityComponentManager &_ecm) {
        if (asset.vertices.empty() || asset.entities.empty()) return;

        asset.vao = VertexArray::create();
        asset.vbo = Buffer::create();
        asset.instance_vbo = Buffer::create();

        asset.vao.bind();

        asset.vbo.upload(GL_ARRAY_BUFFER, asset.vertices.data(),
                         asset.vertices.size() * sizeof(Vertex), GL_STATIC_DRAW);

        VertexLayout vertex_layout;
        vertex_layout.add<GL_FLOAT>(0, 3);
        vertex_layout.add<GL_FLOAT>(1, 3);
        vertex_layout.add<GL_FLOAT>(2, 1);
        asset.vao.set_layout(vertex_layout);

        std::vector<float> inst_data;
        for (auto ent: asset.entities) {
            auto pose = worldPose(ent, _ecm);
            const gz::math::Matrix4d model_mat(pose);

            for (int col = 0; col < 4; ++col)
                for (int row = 0; row < 4; ++row)
                    inst_data.push_back(static_cast<float>(model_mat(row, col)));
        }

        asset.instance_vbo.upload(GL_ARRAY_BUFFER, inst_data.data(),
                                  inst_data.size() * sizeof(float), GL_STATIC_DRAW);

        InstanceLayout instance_layout;
        for (int i = 0; i < 4; ++i)
            instance_layout.add<GL_FLOAT>(3 + i, 4);
        asset.vao.set_layout(instance_layout);

        VertexArray::unbind();
        Buffer::unbind(GL_ARRAY_BUFFER);
    }

    void PointCloudRenderer::UploadPlanes(const EntityComponentManager &_ecm) {
        if (planes_.empty()) return;

        // 每个平面实例打包为 per-instance 数据：
        //   aModel(mat4, 16) + normal(3) + axis_u(3) + axis_v(3)
        //   + half_size(2) + reflectance(1) = 28 个 float。
        // 地图（地面/锥桶）默认静态，故以 GL_STATIC_DRAW 一次性上传；
        // 未来若雷达随 FSAE 无人车移动、需要动态位姿，仅需把 usage 改为
        // GL_DYNAMIC_DRAW 并在每帧调用本方法重传实例 pose 即可，架构无需变动。
        std::vector<float> inst_data;
        inst_data.reserve(planes_.size() * 28);

        for (const auto &plane : planes_) {
            const auto pose = worldPose(plane.entity, _ecm);
            const gz::math::Matrix4d model_mat(pose);

            // —— aModel：列主序（外列内行），与 mesh 实例化一致 ——
            for (int col = 0; col < 4; ++col)
                for (int row = 0; row < 4; ++row)
                    inst_data.push_back(static_cast<float>(model_mat(row, col)));

            // —— 局部平面参数（法线/两轴/半尺寸/反射率）——
            for (const float v : plane.normal) inst_data.push_back(v);
            for (const float v : plane.axis_u) inst_data.push_back(v);
            for (const float v : plane.axis_v) inst_data.push_back(v);
            inst_data.push_back(plane.half_size[0]);
            inst_data.push_back(plane.half_size[1]);
            inst_data.push_back(plane.reflectance);
        }

        plane_vao_.bind();
        plane_instance_vbo_.bind(GL_ARRAY_BUFFER);
        plane_instance_vbo_.upload(GL_ARRAY_BUFFER, inst_data.data(),
                                   inst_data.size() * sizeof(float), GL_STATIC_DRAW);

        InstanceLayout instance_layout;
        for (int i = 0; i < 4; ++i)
            instance_layout.add<GL_FLOAT>(static_cast<GLuint>(i), 4); // aModel
        instance_layout.add<GL_FLOAT>(4, 3); // aPlaneNormal
        instance_layout.add<GL_FLOAT>(5, 3); // aPlaneAxisU
        instance_layout.add<GL_FLOAT>(6, 3); // aPlaneAxisV
        instance_layout.add<GL_FLOAT>(7, 2); // aPlaneHalfSize
        instance_layout.add<GL_FLOAT>(8, 1); // aReflectance
        plane_vao_.set_layout(instance_layout);

        VertexArray::unbind();
        Buffer::unbind(GL_ARRAY_BUFFER);
    }

    void PointCloudRenderer::CollectAndUploadGeometries(EntityComponentManager &_ecm,
                                                        const Entity lidar_model_entity) {
        url_asset_map_.clear();
        primitive_assets_.clear();
        planes_.clear();

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

        UploadPlanes(_ecm);
    }

    void PointCloudRenderer::RenderScene(const gz::math::Pose3d &lidar_pose,
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

        // === 2. 平面 pose 已在 UploadPlanes 阶段作为 per-instance 属性上传，
        //        此处无需 CPU 端再做局部→世界变换。 ===
        const bool has_planes = !planes_.empty();
        const bool has_meshes = !url_asset_map_.empty() || !primitive_assets_.empty();

        // === 3. Set frame-invariant uniforms ===
        if (has_planes) {
            plane_program_.set_uniform(plane_locs_.lidar_pos, std::array{
                                           static_cast<float>(lidar_pose.Pos().X()),
                                           static_cast<float>(lidar_pose.Pos().Y()),
                                           static_cast<float>(lidar_pose.Pos().Z())
                                       });
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
            mesh_program_.set_uniform(mesh_locs_.lidar_pos, std::array{
                                          static_cast<float>(lidar_pose.Pos().X()),
                                          static_cast<float>(lidar_pose.Pos().Y()),
                                          static_cast<float>(lidar_pose.Pos().Z())
                                      });
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

                plane_program_.set_uniform(plane_locs_.group_index, pass);

                // 每个平面对应一个全屏四边形实例，逐像素做光线-平面解析求交。
                plane_vao_.draw_instanced(GL_TRIANGLE_FAN, 0, 4,
                                          static_cast<GLsizei>(planes_.size()));
            }

            if (has_meshes) {
                mesh_program_.use();
                glEnable(GL_DEPTH_TEST);

                mesh_program_.set_uniform(mesh_locs_.hz_offset, hz_offset_pass);
                mesh_program_.set_uniform(mesh_locs_.group_index, pass);

                auto draw_asset = [&](const MeshAsset &asset) {
                    asset.vao.draw_instanced(GL_TRIANGLES,
                                             0,
                                             static_cast<GLsizei>(asset.vertices.size()),
                                             static_cast<GLsizei>(asset.entities.size()));
                };

                for (const auto &asset: url_asset_map_ | std::views::values)
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

    std::vector<float> PointCloudRenderer::ReadRenderResult(const int width, const int height) const {
        // === 1. Allocate buffer ===
        const int total_pixels = width * height;
        std::vector<float> result_data(total_pixels * 4);

        // === 2. Read pixels from framebuffer ===
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_FLOAT, result_data.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        return result_data;
    }

    void PointCloudRenderer::ReadRenderResultInto(std::vector<float> &buffer,
                                                  const int width,
                                                  const int height) const {
        // === 1. Ensure buffer size ===
        const size_t needed = static_cast<size_t>(width * height) * 4;
        if (buffer.size() != needed) buffer.resize(needed);

        // === 2. Read pixels from framebuffer ===
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_FLOAT, buffer.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    bool PointCloudRenderer::CreateReadbackBuffers(const int width, const int height) {
        // === 1. Allocate double-buffered PBOs ===
        pbo_size_ = width * height * 4 * static_cast<int>(sizeof(float));
        pbo_[0] = Buffer::create();
        pbo_[1] = Buffer::create();
        for (const auto &i: pbo_) {
            i.upload(GL_PIXEL_PACK_BUFFER, nullptr,
                     static_cast<size_t>(pbo_size_), GL_STREAM_READ);
            if (glGetError() != GL_NO_ERROR) {
                gzerr << "PointCloudRenderer: glBufferData failed ("
                        << width << "x" << height << ")" << std::endl;
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

    void PointCloudRenderer::StartAsyncReadback() {
        // === 1. Initiate async read into current write PBO ===
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        pbo_[pbo_write_idx_].bind(GL_PIXEL_PACK_BUFFER);
        glReadPixels(0, 0, fb_width_, fb_height_, GL_RGBA, GL_FLOAT, nullptr);
        Buffer::unbind(GL_PIXEL_PACK_BUFFER);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // === 2. Swap write index ===
        pbo_write_idx_ = 1 - pbo_write_idx_;
    }

    void PointCloudRenderer::FinishAsyncReadbackInto(std::vector<float> &buffer) {
        // On failure (first frame / map error) the buffer is left empty, so the
        // caller can use buffer.empty() as the "has data" signal.
        // === 1. First frame has no data yet ===
        if (pbo_first_frame_) {
            pbo_first_frame_ = false;
            buffer.clear();
            return;
        }

        // === 2. Map previous-frame PBO (block until DMA done) ===
        const int read_idx = 1 - pbo_write_idx_;
        buffer.resize(static_cast<size_t>(pbo_size_) / sizeof(float));

        pbo_[read_idx].bind(GL_PIXEL_PACK_BUFFER);
        const void *mapped = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
                                              static_cast<GLsizeiptr>(pbo_size_),
                                              GL_MAP_READ_BIT);
        if (!mapped) {
            gzerr << "PointCloudRenderer: async readback map failed" << std::endl;
            Buffer::unbind(GL_PIXEL_PACK_BUFFER);
            buffer.clear();
            return;
        }

        // === 3. Copy into caller buffer ===
        std::memcpy(buffer.data(), mapped, static_cast<size_t>(pbo_size_));
        glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        Buffer::unbind(GL_PIXEL_PACK_BUFFER);
    }

    std::string PointCloudRenderer::LoadShaderFile(const std::string &path) {
        // === 1. Read entire file into string ===
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            std::cerr << "[PointCloudRenderer] Failed to open shader file: " << path << std::endl;
            return {};
        }
        std::ostringstream oss;
        oss << ifs.rdbuf();
        return oss.str();
    }

    Program PointCloudRenderer::BuildProgramFromFiles(const std::string &vert_path,
                                                      const std::string &frag_path) {
        // === 1. Load sources from disk ===
        const std::string vert_src = LoadShaderFile(vert_path);
        const std::string frag_src = LoadShaderFile(frag_path);
        if (vert_src.empty() || frag_src.empty()) return {};

        // === 2. Compile and link ===
        return BuildProgram(vert_src.c_str(), frag_src.c_str());
    }

    GlShader PointCloudRenderer::CompileShader(const GLenum type,
                                               const char *src) {
        // === 1. Create and compile ===
        const GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);

        // === 2. Check result ===
        GLint ok = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(shader, 512, nullptr, log);
            std::cerr << "[PointCloudRenderer] Shader compile error: " << log << std::endl;
            glDeleteShader(shader);
            return {};
        }
        return GlShader(shader);
    }

    Program PointCloudRenderer::BuildProgram(const char *vert_src,
                                             const char *frag_src) {
        // === 1. Compile shaders ===
        const GlShader vs = CompileShader(GL_VERTEX_SHADER, vert_src);
        const GlShader fs = CompileShader(GL_FRAGMENT_SHADER, frag_src);
        if (!vs || !fs) return {};

        // === 2. Link program ===
        GlProgram program(glCreateProgram());
        glAttachShader(program, vs);
        glAttachShader(program, fs);
        glLinkProgram(program);

        // === 3. Check result ===
        GLint ok = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetProgramInfoLog(program, 512, nullptr, log);
            std::cerr << "[PointCloudRenderer] Program link error: " << log << std::endl;
            return {};
        }
        return Program::adopt(std::move(program));
    }
} // namespace blgz