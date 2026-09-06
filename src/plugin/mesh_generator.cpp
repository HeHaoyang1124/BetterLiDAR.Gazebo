#include "mesh_generator.hpp"

#include "glb_parser.hpp"

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cone.hh>
#include <sdf/Mesh.hh>

#include <gz/common/MeshManager.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include <cmath>
#include <iostream>

namespace blgz {
    constexpr int SPHERE_LATITUDE_SEGMENTS = 24;
    constexpr int SPHERE_LONGITUDE_SEGMENTS = 48;
    constexpr int CONE_SEGMENTS = 48;

    static std::vector<Vertex> MakeBox(const sdf::Geometry &g, const float reflectance) {
        const auto *box = g.BoxShape();
        if (!box) return {};

        const auto &size = box->Size();
        const auto hx = static_cast<float>(size.X() * 0.5);
        const auto hy = static_cast<float>(size.Y() * 0.5);
        const auto hz = static_cast<float>(size.Z() * 0.5);

        constexpr float targetCell = 0.1f;

        std::vector<Vertex> v;

        auto AddFace = [&](const float nx, const float ny, const float nz,
                           const float ax0, const float ay0, const float az0,
                           const float ax1, const float ay1, const float az1,
                           const float cx, const float cy, const float cz) {
            const float dimU = 2.0f * std::sqrt(ax0 * ax0 + ay0 * ay0 + az0 * az0);
            const float dimV = 2.0f * std::sqrt(ax1 * ax1 + ay1 * ay1 + az1 * az1);

            const int nu = std::max(1, static_cast<int>(std::ceil(dimU / targetCell)));
            const int nv = std::max(1, static_cast<int>(std::ceil(dimV / targetCell)));

            for (int i = 0; i < nu; ++i) {
                for (int j = 0; j < nv; ++j) {
                    const float u0 = static_cast<float>(i) / static_cast<float>(nu);
                    const float v0 = static_cast<float>(j) / static_cast<float>(nv);
                    const float u1 = static_cast<float>(i + 1) / static_cast<float>(nu);
                    const float v1 = static_cast<float>(j + 1) / static_cast<float>(nv);

                    auto P = [&](const float u, const float vv) -> Vertex {
                        return {
                            .pos = {
                                cx + (2.0f * u - 1.0f) * ax0 + (2.0f * vv - 1.0f) * ax1,
                                cy + (2.0f * u - 1.0f) * ay0 + (2.0f * vv - 1.0f) * ay1,
                                cz + (2.0f * u - 1.0f) * az0 + (2.0f * vv - 1.0f) * az1
                            },
                            .norm = {nx, ny, nz},
                            .reflectance = reflectance
                        };
                    };

                    Vertex p00 = P(u0, v0);
                    Vertex p10 = P(u1, v0);
                    Vertex p11 = P(u1, v1);
                    Vertex p01 = P(u0, v1);

                    v.push_back(p00);
                    v.push_back(p10);
                    v.push_back(p11);
                    v.push_back(p00);
                    v.push_back(p11);
                    v.push_back(p01);
                }
            }
        };

        AddFace(0, 0, 1, hx, 0, 0, 0, hy, 0, 0, 0, hz);
        AddFace(0, 0, -1, 0, hy, 0, hx, 0, 0, 0, 0, -hz);
        AddFace(1, 0, 0, 0, hy, 0, 0, 0, hz, hx, 0, 0);
        AddFace(-1, 0, 0, 0, 0, hz, 0, hy, 0, -hx, 0, 0);
        AddFace(0, 1, 0, 0, 0, hz, hx, 0, 0, 0, hy, 0);
        AddFace(0, -1, 0, hx, 0, 0, 0, 0, hz, 0, -hy, 0);

        return v;
    }

    static std::vector<Vertex> MakeCylinder(const sdf::Geometry &g, const float reflectance) {
        const auto *cyl = g.CylinderShape();
        if (!cyl) return {};

        constexpr int seg = 48;
        const auto r = static_cast<float>(cyl->Radius());
        const auto hl = static_cast<float>(cyl->Length() * 0.5);
        constexpr float pi2 = 2.0f * static_cast<float>(M_PI);

        std::vector<Vertex> v;

        for (int i = 0; i < seg; ++i) {
            const float a0 = pi2 * static_cast<float>(i) / static_cast<float>(seg);
            const float a1 = pi2 * static_cast<float>(i + 1) / static_cast<float>(seg);
            const float c0 = std::cos(a0), s0 = std::sin(a0);
            const float c1 = std::cos(a1), s1 = std::sin(a1);

            v.push_back({.pos = {r * c0, r * s0, -hl}, .norm = {c0, s0, 0}, .reflectance = reflectance});
            v.push_back({.pos = {r * c1, r * s1, -hl}, .norm = {c1, s1, 0}, .reflectance = reflectance});
            v.push_back({.pos = {r * c1, r * s1, hl}, .norm = {c1, s1, 0}, .reflectance = reflectance});
            v.push_back({.pos = {r * c0, r * s0, -hl}, .norm = {c0, s0, 0}, .reflectance = reflectance});
            v.push_back({.pos = {r * c1, r * s1, hl}, .norm = {c1, s1, 0}, .reflectance = reflectance});
            v.push_back({.pos = {r * c0, r * s0, hl}, .norm = {c0, s0, 0}, .reflectance = reflectance});
        }

        for (int i = 0; i < seg; ++i) {
            const float a0 = pi2 * static_cast<float>(i) / static_cast<float>(seg);
            const float a1 = pi2 * static_cast<float>(i + 1) / static_cast<float>(seg);
            v.push_back({.pos = {0, 0, hl}, .norm = {0, 0, 1}, .reflectance = reflectance});
            v.push_back({.pos = {r * std::cos(a0), r * std::sin(a0), hl}, .norm = {0, 0, 1}, .reflectance = reflectance});
            v.push_back({.pos = {r * std::cos(a1), r * std::sin(a1), hl}, .norm = {0, 0, 1}, .reflectance = reflectance});
        }

        for (int i = 0; i < seg; ++i) {
            const float a0 = pi2 * static_cast<float>(i) / static_cast<float>(seg);
            const float a1 = pi2 * static_cast<float>(i + 1) / static_cast<float>(seg);
            v.push_back({.pos = {0, 0, -hl}, .norm = {0, 0, -1}, .reflectance = reflectance});
            v.push_back({.pos = {r * std::cos(a1), r * std::sin(a1), -hl}, .norm = {0, 0, -1}, .reflectance = reflectance});
            v.push_back({.pos = {r * std::cos(a0), r * std::sin(a0), -hl}, .norm = {0, 0, -1}, .reflectance = reflectance});
        }

        return v;
    }

    static std::vector<Vertex> MakeSphere(const sdf::Geometry &g, const float reflectance) {
        const auto *sphere_shape = g.SphereShape();
        if (!sphere_shape) return {};

        const auto radius = static_cast<float>(sphere_shape->Radius());
        constexpr float delta_lat = static_cast<float>(M_PI) / static_cast<float>(SPHERE_LATITUDE_SEGMENTS);
        constexpr float delta_lon = static_cast<float>(M_PI) / static_cast<float>(SPHERE_LONGITUDE_SEGMENTS) * 2.0f;

        std::vector<Vertex> vertices;

        for (int lat_idx = 0; lat_idx < SPHERE_LATITUDE_SEGMENTS; ++lat_idx) {
            const float top_lat = delta_lat * static_cast<float>(lat_idx);
            const float bot_lat = delta_lat * static_cast<float>(lat_idx + 1);

            for (int lon_idx = 0; lon_idx < SPHERE_LONGITUDE_SEGMENTS; ++lon_idx) {
                const float left_lon = delta_lon * static_cast<float>(lon_idx);
                const float right_lon = delta_lon * static_cast<float>(lon_idx + 1);

                auto vertex = [&](const float lat, const float lon) -> Vertex {
                    const float sin_lat = std::sin(lat), cos_lat = std::cos(lat);
                    const float sin_lon = std::sin(lon), cos_lon = std::cos(lon);
                    return {
                        .pos = {
                            radius * sin_lat * cos_lon,
                            radius * sin_lat * sin_lon,
                            radius * cos_lat
                        },
                        .norm = {
                            sin_lat * cos_lon,
                            sin_lat * sin_lon,
                            cos_lat
                        },
                        .reflectance = reflectance
                    };
                };

                /*
                 * 0
                 * | \
                 * |  \
                 * 1---2
                 */
                vertices.push_back(vertex(top_lat, left_lon));
                vertices.push_back(vertex(bot_lat, left_lon));
                vertices.push_back(vertex(bot_lat, right_lon));
                /*
                 * 3---5
                 *  \  |
                 *   \ |
                 *    \4
                 */
                vertices.push_back(vertex(top_lat, left_lon));
                vertices.push_back(vertex(bot_lat, right_lon));
                vertices.push_back(vertex(top_lat, right_lon));
            }
        }
        return vertices;
    }

    static std::vector<Vertex> MakeCone(const sdf::Geometry &g, const float reflectance) {
        const auto *cone = g.ConeShape();
        if (!cone) return {};

        // === 0. cone parameters ===
        const auto radius = static_cast<float>(cone->Radius());
        const auto length = static_cast<float>(cone->Length());
        const auto half_length = length * 0.5f;

        // slant height (mother-line length) for unit normal of the lateral surface
        const float slant_height = std::hypot(radius, length);
        const float normal_r = length / slant_height;
        const float normal_z = radius / slant_height;
        // angular step per sector
        constexpr float delta_angle = 2.0f * static_cast<float>(M_PI) / static_cast<float>(CONE_SEGMENTS);

        std::vector<Vertex> vertices;

        // === 1. lateral surface ===
        // cone is centered at origin: apex at +z, circular base at -z
        auto side_vertex = [&](const float angle) -> Vertex {
            const float sin_angle = std::sin(angle), cos_angle = std::cos(angle);
            return {
                .pos = {
                    radius * cos_angle,
                    radius * sin_angle,
                    -half_length
                },
                .norm = {
                    normal_r * cos_angle,
                    normal_r * sin_angle,
                    normal_z
                },
                .reflectance = reflectance
            };
        };
        auto apex_vertex = [&](const float angle) -> Vertex {
            // apex: position is fixed;
            // the normal takes the mid-angle
            return {
                .pos = {0, 0, half_length},
                .norm = {
                    normal_r * std::cos(angle),
                    normal_r * std::sin(angle),
                    normal_z
                },
                .reflectance = reflectance
            };
        };

        // collect lateral surface vertices
        for (int ang_idx = 0; ang_idx < CONE_SEGMENTS; ++ang_idx) {
            const float angle0 = delta_angle * static_cast<float>(ang_idx);
            const float angle1 = delta_angle * static_cast<float>(ang_idx + 1);
            const float angle_mid = 0.5f * (angle0 + angle1);

            vertices.push_back(side_vertex(angle0));
            vertices.push_back(side_vertex(angle1));
            vertices.push_back(apex_vertex(angle_mid));
        }

        // === 2. base circle ===
        auto base_vertex = [&](const float angle) -> Vertex {
            return {
                .pos = {
                    radius * std::cos(angle),
                    radius * std::sin(angle),
                    -half_length
                },
                .norm = {0, 0, -1},
                .reflectance = reflectance
            };
        };
        auto base_center_vertex = [&]() -> Vertex {
            return {
                .pos = {0, 0, -half_length},
                .norm = {0, 0, -1},
                .reflectance = reflectance
            };
        };

        // collect base circle vertices
        for (int ang_idx = 0; ang_idx < CONE_SEGMENTS; ++ang_idx) {
            const float angle0 = delta_angle * static_cast<float>(ang_idx);
            const float angle1 = delta_angle * static_cast<float>(ang_idx + 1);

            vertices.push_back(base_center_vertex());
            vertices.push_back(base_vertex(angle1));
            vertices.push_back(base_vertex(angle0));
        }

        return vertices;
    }

    static std::vector<Vertex> MakeMeshes(const sdf::Geometry &g, const float reflectance) {
        const auto *meshShape = g.MeshShape();
        if (!meshShape) return {};

        const auto &uri = meshShape->Uri();
        const auto &scale = meshShape->Scale();

        const auto materials = ParseGlbMaterials(uri);

        std::vector<Vertex> result;

        const gz::common::Mesh *mesh = gz::common::MeshManager::Instance()->Load(uri);
        if (!mesh) {
            std::cerr << "[MakeSubMeshes] failed to load mesh URI [" << uri << "]" << std::endl;
            return result;
        }

        for (unsigned int si = 0; si < mesh->SubMeshCount(); ++si) {
            auto subWeak = mesh->SubMeshByIndex(si);
            const auto sub = subWeak.lock();
            if (!sub) continue;

            if (sub->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES) continue;

            std::vector<Vertex> vertices;
            const auto material_index = sub->GetMaterialIndex();
            const float reflectance = (material_index.has_value() &&
                                       material_index.value() < materials.size())
                                          ? materials[material_index.value()].reflectance
                                          : reflectance;

            const unsigned int idxCount = sub->IndexCount();
            for (unsigned int i = 0; i < idxCount; ++i) {
                const auto idx = static_cast<unsigned int>(sub->Index(i));
                gz::math::Vector3d pos = sub->Vertex(idx);
                pos.X() *= scale.X();
                pos.Y() *= scale.Y();
                pos.Z() *= scale.Z();

                gz::math::Vector3d norm = sub->Normal(idx);
                if (const double length = norm.Length(); length > 1e-8)
                    norm = norm / length;

                vertices.push_back({
                    .pos = {static_cast<float>(pos.X()), static_cast<float>(pos.Y()), static_cast<float>(pos.Z())},
                    .norm = {static_cast<float>(norm.X()), static_cast<float>(norm.Y()), static_cast<float>(norm.Z())},
                    .reflectance = reflectance
                });
            }

            if (!vertices.empty())
                result.insert(result.end(), vertices.begin(), vertices.end());
        }

        if (result.empty())
            std::cerr << "[MakeSubMeshes] no triangle data extracted from [" << uri << "]" << std::endl;

        return result;
    }

    std::vector<Vertex> MakeSubMeshes(const sdf::Geometry &geom, const float reflectance) {
        // clang-format off
        switch (geom.Type()) {
            case sdf::GeometryType::BOX:      return MakeBox(geom, reflectance);
            case sdf::GeometryType::CYLINDER: return MakeCylinder(geom, reflectance);
            case sdf::GeometryType::SPHERE:   return MakeSphere(geom, reflectance);
            case sdf::GeometryType::CONE:     return MakeCone(geom, reflectance);
            case sdf::GeometryType::PLANE:    return {};
            case sdf::GeometryType::MESH:     return MakeMeshes(geom, reflectance);
            default:                          return {};
        }
        // clang-format on
    }
} // namespace blgz