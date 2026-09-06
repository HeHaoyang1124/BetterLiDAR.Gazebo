#include "GlRenderer.hpp"

#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>
#include <gz/common/Util.hh>
#include <gz/common/Console.hh>

#include <dlfcn.h>
#include <cmath>
#include <cstring>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

namespace blgz {
    using gz::sim::worldPose;

    namespace components = gz::sim::components;

    GlRenderer::GlRenderer()
        : glInitialized_(false),
          fbWidth_(0), fbHeight_(0),
          renderHeight_(0), params_() {
    }

    bool GlRenderer::Initialize(const int width, const int height) {
        if (!egl_.initialize()) return false;

        const int glVersion = gladLoadGL(reinterpret_cast<GLADloadfunc>(egl_.getProcAddress()));
        if (!glVersion) {
            gzerr << "GlRenderer: gladLoadGL failed" << std::endl;
            return false;
        }

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CW);

        gzmsg << "GlRenderer: EGL " << GLAD_VERSION_MAJOR(egl_.eglVersion()) << "."
                << GLAD_VERSION_MINOR(egl_.eglVersion())
                << " GL " << GLAD_VERSION_MAJOR(glVersion) << "."
                << GLAD_VERSION_MINOR(glVersion) << std::endl;

        glInitialized_ = true;

        if (!CreateFramebuffer(width, height)) {
            gzerr << "GlRenderer: Failed to create framebuffer" << std::endl;
            return false;
        }
        if (!CreatePBOs(width, height)) {
            gzerr << "GlRenderer: Failed to create PBOs" << std::endl;
            return false;
        }

        return true;
    }

    bool GlRenderer::CreateFramebuffer(const int width, const int height) {
        fbWidth_ = width;
        fbHeight_ = height;
        renderHeight_ = height;

        fbo_ = GlFramebuffer::create();
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

        resultTex_ = GlTexture::create();
        glBindTexture(GL_TEXTURE_2D, resultTex_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resultTex_, 0);

        constexpr GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1, drawBuffers);

        depthRb_ = GlRenderbuffer::create();
        glBindRenderbuffer(GL_RENDERBUFFER, depthRb_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRb_);

        const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (status != GL_FRAMEBUFFER_COMPLETE) {
            gzerr << "GlRenderer: FBO incomplete (0x"
                    << std::hex << status << std::dec << ")" << std::endl;
            return false;
        }
        return true;
    }

    bool GlRenderer::LoadAndCompileShaders(const std::string &shaderDir) {
        auto LoadShaderFile = [](const std::string &path) -> std::string {
            std::ifstream ifs(path);
            if (!ifs.is_open()) {
                std::cerr << "[GlRenderer] Failed to open shader file: " << path << std::endl;
                return {};
            }
            std::ostringstream oss;
            oss << ifs.rdbuf();
            return oss.str();
        };

        const std::string lidarFrag = LoadShaderFile(shaderDir + "/lidar.frag");
        const std::string lidarVert = LoadShaderFile(shaderDir + "/lidar.vert");

        if (lidarVert.empty() || lidarFrag.empty()) {
            gzerr << "GlRenderer: Failed to load shader files" << std::endl;
            return false;
        }

        program_ = BuildProgram(lidarVert.c_str(), lidarFrag.c_str());
        if (!program_) {
            gzerr << "GlRenderer: Failed to compile shaders" << std::endl;
            return false;
        }

        // clang-format off
        locs_.lidarPos          = glGetUniformLocation(program_, "uLidarPos");
        locs_.lidarRotInv       = glGetUniformLocation(program_, "uLidarRotInv");
        locs_.hzScale           = glGetUniformLocation(program_, "uHzScale");
        locs_.hzBias            = glGetUniformLocation(program_, "uHzBias");
        locs_.vtScale           = glGetUniformLocation(program_, "uVtScale");
        locs_.vtBias            = glGetUniformLocation(program_, "uVtBias");
        locs_.depthScale        = glGetUniformLocation(program_, "uDepthScale");
        locs_.depthBias         = glGetUniformLocation(program_, "uDepthBias");
        locs_.nearP             = glGetUniformLocation(program_, "uNearP");
        locs_.farP              = glGetUniformLocation(program_, "uFarP");
        locs_.maxIntensity      = glGetUniformLocation(program_, "uMaxIntensity");
        locs_.reflectance       = glGetUniformLocation(program_, "uReflectance");
        locs_.model             = glGetUniformLocation(program_, "uModel");
        locs_.normalMat         = glGetUniformLocation(program_, "uNormalMat");
        locs_.frameCounter      = glGetUniformLocation(program_, "uFrameCounter");
        locs_.rangeNoiseStd     = glGetUniformLocation(program_, "uRangeNoiseStd");
        locs_.intensityNoiseStd = glGetUniformLocation(program_, "uIntensityNoiseStd");
        locs_.channelTex        = glGetUniformLocation(program_, "uChannelTex");
        locs_.groupIndex        = glGetUniformLocation(program_, "uGroupIndex");

        locs_.uReflectance    = glGetUniformLocation(program_, "uReflectance");
        // clang-format on

        const std::string planeFrag = LoadShaderFile(shaderDir + "/plane.frag");
        const std::string planeVert = LoadShaderFile(shaderDir + "/plane.vert");

        if (planeVert.empty() || planeFrag.empty()) {
            gzerr << "GlRenderer: Failed to load plane shader files" << std::endl;
            return false;
        }

        planeProgram_ = BuildProgram(planeVert.c_str(), planeFrag.c_str());
        if (!planeProgram_) {
            gzerr << "GlRenderer: Failed to compile plane shaders" << std::endl;
            return false;
        }

        // clang-format off
        planeLocs_.lidarPos          = glGetUniformLocation(planeProgram_, "uLidarPos");
        planeLocs_.lidarRot          = glGetUniformLocation(planeProgram_, "uLidarRot");
        planeLocs_.hzScale           = glGetUniformLocation(planeProgram_, "uHzScale");
        planeLocs_.hzBias            = glGetUniformLocation(planeProgram_, "uHzBias");
        planeLocs_.vtScale           = glGetUniformLocation(planeProgram_, "uVtScale");
        planeLocs_.vtBias            = glGetUniformLocation(planeProgram_, "uVtBias");
        planeLocs_.depthScale        = glGetUniformLocation(planeProgram_, "uDepthScale");
        planeLocs_.depthBias         = glGetUniformLocation(planeProgram_, "uDepthBias");
        planeLocs_.nearP             = glGetUniformLocation(planeProgram_, "uNearP");
        planeLocs_.farP              = glGetUniformLocation(planeProgram_, "uFarP");
        planeLocs_.planeNormal       = glGetUniformLocation(planeProgram_, "uPlaneNormal");
        planeLocs_.planeCenter       = glGetUniformLocation(planeProgram_, "uPlaneCenter");
        planeLocs_.planeTangent      = glGetUniformLocation(planeProgram_, "uPlaneTangent");
        planeLocs_.planeBitangent    = glGetUniformLocation(planeProgram_, "uPlaneBitangent");
        planeLocs_.planeHalfSize     = glGetUniformLocation(planeProgram_, "uPlaneHalfSize");
        planeLocs_.maxIntensity      = glGetUniformLocation(planeProgram_, "uMaxIntensity");
        planeLocs_.reflectance       = glGetUniformLocation(planeProgram_, "uReflectance");
        planeLocs_.frameCounter      = glGetUniformLocation(planeProgram_, "uFrameCounter");
        planeLocs_.rangeNoiseStd     = glGetUniformLocation(planeProgram_, "uRangeNoiseStd");
        planeLocs_.intensityNoiseStd = glGetUniformLocation(planeProgram_, "uIntensityNoiseStd");
        planeLocs_.channelTex        = glGetUniformLocation(planeProgram_, "uChannelTex");
        planeLocs_.groupIndex        = glGetUniformLocation(planeProgram_, "uGroupIndex");
        // clang-format on

        quadVao_ = GlVertexArray::create();

        return true;
    }

    void GlRenderer::SetParams(const double hzMin, const double hzMax,
                               const double vtMin, const double vtMax,
                               const double nearP, const double farP) {
        // clang-format off
        const auto hzSpan = static_cast<float>(hzMax - hzMin);
        const auto vtSpan = static_cast<float>(vtMax - vtMin);
        const auto dpSpan = static_cast<float>(farP  - nearP);

        params_.hzScale    =  2.0f / hzSpan;
        params_.hzBias     = -2.0f * static_cast<float>(hzMin) / hzSpan - 1.0f;
        params_.hzMin      =  static_cast<float>(hzMin);
        params_.hzMax      =  static_cast<float>(hzMax);
        params_.vtScale    =  2.0f / vtSpan;
        params_.vtBias     = -2.0f * static_cast<float>(vtMin) / vtSpan - 1.0f;
        params_.depthScale =  2.0f / dpSpan;
        params_.depthBias  = -(static_cast<float>(farP + nearP)) / dpSpan;
        params_.nearP      = static_cast<float>(nearP);
        params_.farP       = static_cast<float>(farP);
        // clang-format on
    }

    void GlRenderer::SetNoiseParams(const double rangeNoiseStd,
                                    const double intensityNoiseStd) {
        rangeNoiseStd_ = static_cast<float>(rangeNoiseStd);
        intensityNoiseStd_ = static_cast<float>(intensityNoiseStd);
    }

    bool GlRenderer::LoadChannelTable(const std::string &channelTableFile,
                                      const std::string &sdfFilePath) {
        const int vtSamples = fbHeight_;
        std::vector<float> chData;
        bool loaded = false;

        if (!channelTableFile.empty()) {
            namespace fs = std::filesystem;
            fs::path tablePath(channelTableFile);
            std::string resolvedPath;

            if (tablePath.is_absolute()) {
                if (fs::exists(tablePath))
                    resolvedPath = tablePath.string();
            } else {
                if (!sdfFilePath.empty()) {
                    fs::path candidate = fs::path(sdfFilePath).parent_path() / tablePath;
                    if (fs::exists(candidate))
                        resolvedPath = candidate.string();
                }
                if (resolvedPath.empty()) {
                    std::string found = gz::common::findFile(channelTableFile);
                    if (!found.empty() && fs::exists(found))
                        resolvedPath = found;
                }
            }

            if (!resolvedPath.empty()) {
                std::ifstream f(resolvedPath, std::ios::binary);
                if (f) {
                    const size_t count = static_cast<size_t>(vtSamples) * 4;
                    chData.resize(count);
                    f.read(reinterpret_cast<char *>(chData.data()),
                           static_cast<std::streamsize>(count * sizeof(float)));
                    loaded = f.good();
                }
            }
            if (loaded) {
                gzmsg << "GlRenderer: loaded channel table from ["
                        << resolvedPath << "]" << std::endl;
            } else {
                gzerr << "GlRenderer: failed to load channel table ["
                        << channelTableFile << "], falling back to built-in"
                        << std::endl;
            }
        }
        if (!loaded) {
            chData = BuildChannelTextureData(vtSamples);
        }

        float hzOffsets[4] = {};
        bool found[4] = {};
        for (int i = 0; i < vtSamples; ++i) {
            const size_t base = static_cast<size_t>(i) * 4;
            int g = static_cast<int>(chData[base + 3]);
            if (g >= 0 && g < 4 && !found[g]) {
                hzOffsets[g] = chData[base + 2];
                found[g] = true;
            }
        }
        SetHzOffsets(hzOffsets[0], hzOffsets[1], hzOffsets[2], hzOffsets[3]);
        return CreateChannelTexture(chData.data(), vtSamples);
    }

    std::vector<float> GlRenderer::BuildChannelTextureData(int n) {
        constexpr float degToRad = static_cast<float>(M_PI) / 180.0f;
        const float hzOffsetsDeg[4] = {2.4f, -0.65f, -2.4f, 0.65f};

        std::vector<float> chData(static_cast<size_t>(n) * 4);
        for (int i = 0; i < n; ++i) {
            const int block = i / 16;
            const int rem = i % 4;

            int group;
            if (block % 2 == 0) {
                group = rem;
            } else {
                constexpr int oddMap[4] = {2, 0, 3, 1};
                group = oddMap[rem];
            }

            float rMin = (i % 4 == 2) ? 0.5f : 4.5f;
            float rMax = (i >= 32 && i <= 95) ? 260.0f : 90.0f;
            float hzOffset = hzOffsetsDeg[group] * degToRad;

            const size_t base = static_cast<size_t>(n - 1 - i) * 4;
            chData[base + 0] = rMin;
            chData[base + 1] = rMax;
            chData[base + 2] = hzOffset;
            chData[base + 3] = static_cast<float>(group);
        }
        return chData;
    }

    void GlRenderer::SetHzOffsets(const float o0, const float o1,
                                  const float o2, const float o3) {
        hzOffsets_[0] = o0;
        hzOffsets_[1] = o1;
        hzOffsets_[2] = o2;
        hzOffsets_[3] = o3;

        float maxAbs = std::abs(o0);
        maxAbs = std::max(maxAbs, std::abs(o1));
        maxAbs = std::max(maxAbs, std::abs(o2));
        maxAbs = std::max(maxAbs, std::abs(o3));

        const float marginSpan = (params_.hzMax - params_.hzMin) + 2.0f * maxAbs;
        params_.hzScale = 2.0f / marginSpan;
        params_.hzBias = -(params_.hzMax + params_.hzMin) / marginSpan;
    }

    bool GlRenderer::CreateChannelTexture(const float *data, const int vtSamples) {
        channelTex_ = GlTexture::create();
        glBindTexture(GL_TEXTURE_2D, channelTex_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 1, vtSamples, 0,
                     GL_RGBA, GL_FLOAT, data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);

        return channelTex_ != 0;
    }

    void GlRenderer::CollectGeometries(EntityComponentManager &_ecm,
                                       const Entity lidarModelEntity) {
        auto belongsToModel = [&](const Entity ent) -> bool {
            if (lidarModelEntity == kNullEntity) return false;
            for (auto cur = ent; cur != kNullEntity;) {
                if (cur == lidarModelEntity) return true;
                auto *parent = _ecm.Component<components::ParentEntity>(cur);
                if (!parent) break;
                cur = parent->Data();
            }
            return false;
        };

        auto collectFrom = [&]<typename T>(const T &) {
            _ecm.Each<T, components::Geometry, components::Name>(
                [&](const Entity &_ent, const auto *,
                    const components::Geometry *_geom, const components::Name *) -> bool {
                    if (belongsToModel(_ent)) return true;

                    const auto &g = _geom->Data();

                    if (g.Type() == sdf::GeometryType::PLANE) {
                        const auto *plane = g.PlaneShape();
                        if (!plane) return true;

                        const auto &n = plane->Normal();
                        const auto &size = plane->Size();

                        float nx = static_cast<float>(n.X());
                        float ny = static_cast<float>(n.Y());
                        float nz = static_cast<float>(n.Z());
                        const float nLen = std::sqrt(nx * nx + ny * ny + nz * nz);
                        if (nLen > 1e-6f) {
                            nx /= nLen;
                            ny /= nLen;
                            nz /= nLen;
                        } else {
                            nx = 0.f;
                            ny = 0.f;
                            nz = 1.f;
                        }

                        float tx, ty, tz;
                        if (std::abs(nz) < 0.9f) {
                            tx = 0;
                            ty = 0;
                            tz = 1;
                        } else {
                            tx = 1;
                            ty = 0;
                            tz = 0;
                        }

                        float bx = ny * tz - nz * ty;
                        float by = nz * tx - nx * tz;
                        float bz = nx * ty - ny * tx;
                        const float bLen = std::sqrt(bx * bx + by * by + bz * bz);
                        bx /= bLen;
                        by /= bLen;
                        bz /= bLen;

                        tx = by * nz - bz * ny;
                        ty = bz * nx - bx * nz;
                        tz = bx * ny - by * nx;

                        renderables_.push_back({
                            .vertexCount = 0,
                            .entity = _ent, .type = RenderableType::Plane,
                            .plane = {
                                .normal = {nx, ny, nz},
                                .center = {0.f, 0.f, 0.f},
                                .tangent = {tx, ty, tz},
                                .bitangent = {bx, by, bz},
                                .halfSize = {
                                    static_cast<float>(size.X() * 0.5),
                                    static_cast<float>(size.Y() * 0.5)
                                }
                            }
                        });
                        return true;
                    }

                    if (g.Type() == sdf::GeometryType::MESH) {
                        auto subMeshes = MakeSubMeshes(g);
                        if (subMeshes.empty()) return true;

                        for (auto &sm: subMeshes) {
                            auto vao = GlVertexArray::create();
                            auto vbo = GlBuffer::create();
                            glBindVertexArray(vao);
                            glBindBuffer(GL_ARRAY_BUFFER, vbo);
                            glBufferData(GL_ARRAY_BUFFER,
                                         static_cast<GLsizeiptr>(sm.vertices.size() * sizeof(Vertex)),
                                         sm.vertices.data(), GL_STATIC_DRAW);
                            glEnableVertexAttribArray(0);
                            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                                  reinterpret_cast<void *>(0));
                            glEnableVertexAttribArray(1);
                            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                                  reinterpret_cast<void *>(sizeof(float) * 3));
                            glBindVertexArray(0);

                            renderables_.push_back({
                                .vao = std::move(vao), .vbo = std::move(vbo),
                                .vertexCount = static_cast<int>(sm.vertices.size()),
                                .entity = _ent, .type = RenderableType::Mesh,
                                .material = sm.material
                            });
                        }
                        return true;
                    }

                    const std::vector<Vertex> mesh = MakeGeometry(g);

                    if (mesh.empty()) return true;

                    auto vao = GlVertexArray::create();
                    auto vbo = GlBuffer::create();
                    glBindVertexArray(vao);
                    glBindBuffer(GL_ARRAY_BUFFER, vbo);
                    glBufferData(GL_ARRAY_BUFFER,
                                 static_cast<GLsizeiptr>(mesh.size() * sizeof(Vertex)),
                                 mesh.data(), GL_STATIC_DRAW);
                    glEnableVertexAttribArray(0);
                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                          reinterpret_cast<void *>(0));
                    glEnableVertexAttribArray(1);
                    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                          reinterpret_cast<void *>(sizeof(float) * 3));
                    glBindVertexArray(0);

                    renderables_.push_back({
                        .vao = std::move(vao), .vbo = std::move(vbo),
                        .vertexCount = static_cast<int>(mesh.size()),
                        .entity = _ent, .type = RenderableType::Mesh
                    });
                    return true;
                });
        };

        collectFrom(components::Visual());
        if (renderables_.empty())
            collectFrom(components::Collision());
    }

    void GlRenderer::RenderScene(const gz::math::Pose3d &lidarPose,
                                 const EntityComponentManager &_ecm,
                                 const double maxIntensity,
                                 const double reflectance,
                                 const uint32_t frameCounter) const {
        glViewport(0, 0, fbWidth_, renderHeight_);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto lidarRot = gz::math::Matrix3d(lidarPose.Rot());
        const gz::math::Matrix3d rotInv = lidarRot.Transposed();

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, channelTex_);

        // --- Pre-compute world-space transforms once per frame ---
        struct PlaneCache {
            size_t idx;
            float wn[3], wc[3], wt[3], wb[3];
        };
        struct MeshCache {
            size_t idx{};
            gz::math::Matrix4d modelMat;
            gz::math::Matrix3d normalMat;
        };

        std::vector<PlaneCache> planeCache;
        std::vector<MeshCache> meshCache;

        for (size_t ri = 0; ri < renderables_.size(); ++ri) {
            const auto &r = renderables_[ri];
            auto pose = worldPose(r.entity, _ecm);

            if (r.type == RenderableType::Plane) {
                const auto rotMat = gz::math::Matrix3d(pose.Rot());
                const auto &p = pose.Pos();
                PlaneCache pc{};
                pc.idx = ri;
                for (int i = 0; i < 3; ++i) {
                    pc.wn[i] = static_cast<float>(
                        rotMat(i, 0) * r.plane.normal[0] +
                        rotMat(i, 1) * r.plane.normal[1] +
                        rotMat(i, 2) * r.plane.normal[2]);
                    pc.wt[i] = static_cast<float>(
                        rotMat(i, 0) * r.plane.tangent[0] +
                        rotMat(i, 1) * r.plane.tangent[1] +
                        rotMat(i, 2) * r.plane.tangent[2]);
                    pc.wb[i] = static_cast<float>(
                        rotMat(i, 0) * r.plane.bitangent[0] +
                        rotMat(i, 1) * r.plane.bitangent[1] +
                        rotMat(i, 2) * r.plane.bitangent[2]);
                    pc.wc[i] = static_cast<float>(p[i]) +
                               rotMat(i, 0) * r.plane.center[0] +
                               rotMat(i, 1) * r.plane.center[1] +
                               rotMat(i, 2) * r.plane.center[2];
                }
                planeCache.push_back(pc);
            } else if (r.type == RenderableType::Mesh) {
                MeshCache mc;
                mc.idx = ri;
                mc.modelMat = gz::math::Matrix4d(pose);
                mc.normalMat = gz::math::Matrix3d(pose.Rot());
                meshCache.push_back(mc);
            }
        }

        const bool hasPlanes = !planeCache.empty();
        const bool hasMeshes = !meshCache.empty();

        // --- Invariant uniforms for plane program ---
        if (hasPlanes) {
            glUseProgram(planeProgram_);

            glUniform3f(planeLocs_.lidarPos,
                        static_cast<float>(lidarPose.Pos().X()),
                        static_cast<float>(lidarPose.Pos().Y()),
                        static_cast<float>(lidarPose.Pos().Z()));
            SetMat3Uniform(planeLocs_.lidarRot, lidarRot);

            glUniform1f(planeLocs_.hzScale, params_.hzScale);
            glUniform1f(planeLocs_.vtScale, params_.vtScale);
            glUniform1f(planeLocs_.vtBias, params_.vtBias);
            glUniform1f(planeLocs_.depthScale, params_.depthScale);
            glUniform1f(planeLocs_.depthBias, params_.depthBias);
            glUniform1f(planeLocs_.nearP, params_.nearP);
            glUniform1f(planeLocs_.farP, params_.farP);

            glUniform1f(planeLocs_.maxIntensity, static_cast<float>(maxIntensity));
            glUniform1f(planeLocs_.reflectance, static_cast<float>(reflectance));
            glUniform1ui(planeLocs_.frameCounter, frameCounter);
            glUniform1f(planeLocs_.rangeNoiseStd, rangeNoiseStd_);
            glUniform1f(planeLocs_.intensityNoiseStd, intensityNoiseStd_);
            glUniform1i(planeLocs_.channelTex, 1);
        }

        // --- Invariant uniforms for mesh program ---
        if (hasMeshes) {
            glUseProgram(program_);

            glUniform3f(locs_.lidarPos,
                        static_cast<float>(lidarPose.Pos().X()),
                        static_cast<float>(lidarPose.Pos().Y()),
                        static_cast<float>(lidarPose.Pos().Z()));
            SetMat3Uniform(locs_.lidarRotInv, rotInv);

            glUniform1f(locs_.hzScale, params_.hzScale);
            glUniform1f(locs_.vtScale, params_.vtScale);
            glUniform1f(locs_.vtBias, params_.vtBias);
            glUniform1f(locs_.depthScale, params_.depthScale);
            glUniform1f(locs_.depthBias, params_.depthBias);
            glUniform1f(locs_.nearP, params_.nearP);
            glUniform1f(locs_.farP, params_.farP);

            glUniform1f(locs_.maxIntensity, static_cast<float>(maxIntensity));
            glUniform1f(locs_.reflectance, static_cast<float>(reflectance));
            glUniform1ui(locs_.frameCounter, frameCounter);
            glUniform1f(locs_.rangeNoiseStd, rangeNoiseStd_);
            glUniform1f(locs_.intensityNoiseStd, intensityNoiseStd_);
            glUniform1i(locs_.channelTex, 1);
        }

        for (int pass = 0; pass < 4; ++pass) {
            const float hzOffset = hzOffsets_[pass];
            const float hzBiasPass = params_.hzBias - hzOffset * params_.hzScale;

            if (hasPlanes) {
                glUseProgram(planeProgram_);
                glDisable(GL_DEPTH_TEST);
                glDisable(GL_CULL_FACE);

                glUniform1f(planeLocs_.hzBias, params_.hzBias);
                glUniform1i(planeLocs_.groupIndex, pass);

                for (const auto &pc: planeCache) {
                    const auto &r = renderables_[pc.idx];
                    glUniform3fv(planeLocs_.planeNormal, 1, pc.wn);
                    glUniform3fv(planeLocs_.planeCenter, 1, pc.wc);
                    glUniform3fv(planeLocs_.planeTangent, 1, pc.wt);
                    glUniform3fv(planeLocs_.planeBitangent, 1, pc.wb);
                    glUniform2f(planeLocs_.planeHalfSize,
                                r.plane.halfSize[0], r.plane.halfSize[1]);

                    glBindVertexArray(quadVao_);
                    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
                }
            }

            if (hasMeshes) {
                glUseProgram(program_);
                glEnable(GL_DEPTH_TEST);
                glEnable(GL_CULL_FACE);

                glUniform1f(locs_.hzBias, hzBiasPass);
                glUniform1i(locs_.groupIndex, pass);

                for (const auto &mc: meshCache) {
                    const auto &r = renderables_[mc.idx];
                    SetMat4Uniform(locs_.model, mc.modelMat);
                    SetMat3Uniform(locs_.normalMat, mc.normalMat);
                    glUniform1f(locs_.uReflectance, r.material.reflectance);

                    glBindVertexArray(r.vao);
                    glDrawArrays(GL_TRIANGLES, 0, r.vertexCount);
                }
            }
        }

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);

        glBindVertexArray(0);
        glUseProgram(0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    std::vector<float> GlRenderer::ReadRenderResult(const int width, const int renderHeight) const {
        const int totalPixels = width * renderHeight;
        std::vector<float> resultData(totalPixels * 4);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, renderHeight, GL_RGBA, GL_FLOAT, resultData.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        return resultData;
    }

    void GlRenderer::ReadRenderResultInto(std::vector<float> &buffer,
                                          const int width,
                                          const int renderHeight) const {
        const size_t needed = static_cast<size_t>(width * renderHeight) * 4;
        if (buffer.size() != needed) buffer.resize(needed);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, width, renderHeight, GL_RGBA, GL_FLOAT, buffer.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    bool GlRenderer::CreatePBOs(const int width, const int renderHeight) {
        pboSize_ = width * renderHeight * 4 * static_cast<int>(sizeof(float));
        pbo_[0] = GlBuffer::create();
        pbo_[1] = GlBuffer::create();
        for (int i = 0; i < 2; ++i) {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[i]);
            glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(pboSize_),
                         nullptr, GL_STREAM_READ);
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        pboFirstFrame_ = true;
        pboWriteIdx_ = 0;
        return true;
    }

    void GlRenderer::StartAsyncReadback(const int width, const int renderHeight) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[pboWriteIdx_]);
        glReadPixels(0, 0, width, renderHeight, GL_RGBA, GL_FLOAT, 0);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        pboWriteIdx_ = 1 - pboWriteIdx_;
    }

    bool GlRenderer::FinishAsyncReadbackInto(std::vector<float> &buffer,
                                             const int width,
                                             const int renderHeight) {
        if (pboFirstFrame_) {
            pboFirstFrame_ = false;
            return false;
        }

        const int readIdx = 1 - pboWriteIdx_;
        const size_t needed = static_cast<size_t>(width * renderHeight) * 4;
        if (buffer.size() != needed) buffer.resize(needed);

        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[readIdx]);
        const void *mapped = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
                                              static_cast<GLsizeiptr>(pboSize_),
                                              GL_MAP_READ_BIT);
        if (mapped) {
            std::memcpy(buffer.data(), mapped, static_cast<size_t>(pboSize_));
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        return mapped != nullptr;
    }

    GlShader GlRenderer::CompileShader(const GLenum type, const char *src) {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
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

    GlProgram GlRenderer::BuildProgram(const char *vertSrc, const char *fragSrc,
                                        const char * /*geomSrc*/) {
        GlShader vs = CompileShader(GL_VERTEX_SHADER, vertSrc);
        GlShader fs = CompileShader(GL_FRAGMENT_SHADER, fragSrc);
        if (!vs || !fs) return GlProgram();

        GlProgram prog(glCreateProgram());
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);

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

    void GlRenderer::SetMat4Uniform(const GLint loc, const gz::math::Matrix4d &m) {
        if (loc < 0) return;
        float f[16];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                f[i * 4 + j] = static_cast<float>(m(i, j));
        glUniformMatrix4fv(loc, 1, GL_TRUE, f);
    }

    void GlRenderer::SetMat3Uniform(const GLint loc, const gz::math::Matrix3d &m) {
        if (loc < 0) return;
        float f[9];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                f[i * 3 + j] = static_cast<float>(m(i, j));
        glUniformMatrix3fv(loc, 1, GL_TRUE, f);
    }
} // namespace blgz