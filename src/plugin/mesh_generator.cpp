#include "mesh_generator.hpp"

#include <array>
#include <cmath>
#include <iostream>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Box.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Mesh.hh>
#include <sdf/Sphere.hh>

#include "glb_parser.hpp"

namespace blgz {
    constexpr int SPHERE_LATITUDE_SEGMENTS = 24;
    constexpr int SPHERE_LONGITUDE_SEGMENTS = 48;
    constexpr int CONE_SEGMENTS = 48;
    constexpr int CYLINDER_SEGMENTS = 48;
    constexpr float BOX_CELL_SIZE = 0.1f;

    /**
     * @brief 生成 sdf box 几何体的三角形顶点
     * @note 轴对齐立方体，中心在原点
     * @note 每个面细分为网格，每格拆成两个三角形
     * @param geom sdf 几何体，需为 box 形状
     * @param reflectance 应用到所有顶点的雷达反射率
     * @return 所有三角形的顶点（位置 + 法线 + 反射率）
     */
    static std::vector<Vertex> MakeBox(const sdf::Geometry &geom, const float reflectance) {
        const auto *box = geom.BoxShape();
        if (!box) return {};

        const auto &box_size = box->Size();

        const gz::math::Vector3f axis_x{1, 0, 0};
        const gz::math::Vector3f axis_y{0, 1, 0};
        const gz::math::Vector3f axis_z{0, 0, 1};

        const gz::math::Vector3f box_x = axis_x * static_cast<float>(box_size.X());
        const gz::math::Vector3f box_y = axis_y * static_cast<float>(box_size.Y());
        const gz::math::Vector3f box_z = axis_z * static_cast<float>(box_size.Z());

        std::vector<Vertex> vertices;

        auto AddFace = [&](const gz::math::Vector3f &n_vector,
                           const gz::math::Vector3f &u_vector,
                           const gz::math::Vector3f &v_vector) {
            const gz::math::Vector3f center = n_vector * 0.5f;
            const gz::math::Vector3f n_axis = n_vector.Normalized();

            const int nu = std::max(1, static_cast<int>(std::ceil(u_vector.Length() / BOX_CELL_SIZE)));
            const int nv = std::max(1, static_cast<int>(std::ceil(v_vector.Length() / BOX_CELL_SIZE)));

            for (int i = 0; i < nu; ++i) {
                for (int j = 0; j < nv; ++j) {
                    const float u0 = static_cast<float>(i) / static_cast<float>(nu);
                    const float v0 = static_cast<float>(j) / static_cast<float>(nv);
                    const float u1 = static_cast<float>(i + 1) / static_cast<float>(nu);
                    const float v1 = static_cast<float>(j + 1) / static_cast<float>(nv);

                    auto vertex = [&](const float u, const float v) -> Vertex {
                        // [0,1] -> [-0.5, 0.5]
                        const float tu = u - 0.5f;
                        const float tv = v - 0.5f;
                        const gz::math::Vector3f pos = center
                                                       + u_vector * tu
                                                       + v_vector * tv;
                        return {
                            .pos = {pos.X(), pos.Y(), pos.Z()},
                            .norm = {n_axis.X(), n_axis.Y(), n_axis.Z()},
                            .reflectance = reflectance
                        };
                    };

                    /*
                     *   v-axis
                     *   ↑
                     *   0-----2
                     *   |   / |
                     *   | /   |
                     *   1-----3 → u-axis
                     * ⊙ n-axis (pointing outward)
                     */
                    Vertex p0 = vertex(u0, v1);
                    Vertex p1 = vertex(u0, v0);
                    Vertex p2 = vertex(u1, v1);
                    Vertex p3 = vertex(u1, v0);
                    vertices.push_back(p0), vertices.push_back(p1), vertices.push_back(p2);
                    vertices.push_back(p1), vertices.push_back(p3), vertices.push_back(p2);
                }
            }
        };

        // clang-format off
        AddFace( box_z,  box_x,  box_y);  // +Z Top
        AddFace(-box_z,  box_y,  box_x);  // -Z Bottom
        AddFace( box_x,  box_y,  box_z);  // +X Front
        AddFace(-box_x,  box_z,  box_y);  // -X Back
        AddFace( box_y,  box_z,  box_x);  // +Y Right
        AddFace(-box_y,  box_x,  box_z);  // -Y Left
        // clang-format on

        return vertices;
    }

    /**
     * @brief 生成 sdf cylinder 几何体的三角形顶点
     * @note 圆柱中心在原点、轴沿 +z，侧面/顶盖/底盖分别细分
     * @param geom sdf 几何体，需为 cylinder 形状
     * @param reflectance 应用到所有顶点的雷达反射率
     * @return 所有三角形的顶点（位置 + 法线 + 反射率）
     */
    static std::vector<Vertex> MakeCylinder(const sdf::Geometry &geom, const float reflectance) {
        const auto *shape = geom.CylinderShape();
        if (!shape) return {};

        // === 0. Cylinder parameters ===
        const auto radius = static_cast<float>(shape->Radius());
        const auto half_length = static_cast<float>(shape->Length() * 0.5f);
        // angular step per sector
        constexpr float delta_angle = 2.0f * static_cast<float>(M_PI) / static_cast<float>(CYLINDER_SEGMENTS);

        std::vector<Vertex> vertices;
        vertices.reserve(static_cast<size_t>(CYLINDER_SEGMENTS) * 12);

        // === 1. Lateral surface ===
        // cylinder is centered at origin: axis along +z, from -hl to +hl
        // unit normal of the lateral surface points radially outward
        auto side_vertex = [&](const float angle, const float z) -> Vertex {
            const float c = std::cos(angle), s = std::sin(angle);
            return {
                .pos = {radius * c, radius * s, z},
                .norm = {c, s, 0},
                .reflectance = reflectance
            };
        };
        // cap rim vertex: position on the rim circle, normal along +z / -z
        auto cap_vertex = [&
                ](const float angle, const std::array<float, 3> &normal, const float half_length) -> Vertex {
            return {
                .pos = {
                    radius * std::cos(angle),
                    radius * std::sin(angle),
                    half_length
                },
                .norm = {normal[0], normal[1], normal[2]},
                .reflectance = reflectance
            };
        };

        for (int i = 0; i < CYLINDER_SEGMENTS; ++i) {
            const float a0 = delta_angle * static_cast<float>(i);
            const float a1 = delta_angle * static_cast<float>(i + 1);

            // lateral quad -> two triangles
            const Vertex b0 = side_vertex(a0, -half_length);
            const Vertex b1 = side_vertex(a1, -half_length);
            const Vertex t0 = side_vertex(a0, half_length);
            const Vertex t1 = side_vertex(a1, half_length);
            vertices.push_back(b0), vertices.push_back(b1), vertices.push_back(t1);
            vertices.push_back(b0), vertices.push_back(t1), vertices.push_back(t0);

            // top cap (+z)
            constexpr std::array<float, 3> top_norm = {0, 0, 1};
            vertices.push_back(cap_vertex(a0, top_norm, half_length));
            vertices.push_back(cap_vertex(a1, top_norm, half_length));
            vertices.push_back({
                .pos = {0, 0, half_length},
                .norm = top_norm,
                .reflectance = reflectance
            });

            // bottom cap (-z)
            constexpr std::array<float, 3> bot_norm = {0, 0, -1};
            vertices.push_back(cap_vertex(a1, bot_norm, -half_length));
            vertices.push_back(cap_vertex(a0, bot_norm, -half_length));
            vertices.push_back({
                .pos = {0, 0, -half_length},
                .norm = bot_norm,
                .reflectance = reflectance
            });
        }

        return vertices;
    }

    /**
     * @brief 生成 sdf sphere 几何体的三角形顶点
     * @note 球心在原点，按经纬度分
     * @param geom sdf 几何体，需为 sphere 形状
     * @param reflectance 应用到所有顶点的雷达反射率
     * @return 所有三角形的顶点（位置 + 法线 + 反射率）
     */
    static std::vector<Vertex> MakeSphere(const sdf::Geometry &geom, const float reflectance) {
        const auto *sphere_shape = geom.SphereShape();
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
                 * 0-----2
                 * |   / |
                 * | /   |
                 * 1-----3
                 */
                Vertex p0 = vertex(top_lat, left_lon);
                Vertex p1 = vertex(bot_lat, left_lon);
                Vertex p2 = vertex(top_lat, right_lon);
                Vertex p3 = vertex(bot_lat, right_lon);
                vertices.push_back(p0), vertices.push_back(p1), vertices.push_back(p2);
                vertices.push_back(p1), vertices.push_back(p3), vertices.push_back(p2);
            }
        }
        return vertices;
    }

    /**
     * @brief 生成 sdf cone 几何体的三角形顶点
     * @note 圆锥中心在原点、顶点在 +z、底面圆在 -z，侧面/底面分别细分
     * @param geom sdf 几何体，需为 cone 形状
     * @param reflectance 应用到所有顶点的雷达反射率
     * @return 所有三角形的顶点（位置 + 法线 + 反射率）
     */
    static std::vector<Vertex> MakeCone(const sdf::Geometry &geom, const float reflectance) {
        const auto *cone = geom.ConeShape();
        if (!cone) return {};

        // === 0. Cone parameters ===
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

        // === 1. Lateral surface ===
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

        // === 2. Base circle ===
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

    /**
     * @brief 从 sdf 中 mesh 类型的几何体中提取所有三角形顶点
     * @note 期望输入 mesh 正面顶点绕序为【逆时针方向】
     * @note mesh 默认为 glb 文件，材质反射率从 extras -> lidar_lambert 读取
     * @param geom sdf 几何体，需为网格形状
     * @param default_reflectance 默认雷达反射率，如果在材质中未指定反射率
     * @return 所有三角形的顶点（包含位置 + 法线 + 反射率）
     */
    static std::vector<Vertex> MakeMeshes(const sdf::Geometry &geom,
                                          const float default_reflectance = kDefaultReflectance) {
        // === 0. Mesh parameters ===
        const auto *meshShape = geom.MeshShape();
        if (!meshShape) return {};

        const auto &uri = meshShape->Uri();
        const auto &scale = meshShape->Scale();

        // === 1. Load material (reflectance) ===
        const auto materials = ParseGlbMaterials(uri);

        // === 2. Load mesh ===
        const gz::common::Mesh *mesh = gz::common::MeshManager::Instance()->Load(uri);
        if (!mesh) {
            gzerr << "MakeMeshes: failed to load mesh URI [" << uri << "]" << std::endl;
            return {};
        }

        // === 3. Iterate sub-meshes ===
        std::vector<Vertex> result;
        for (unsigned int sub_mesh_index = 0; sub_mesh_index < mesh->SubMeshCount(); ++sub_mesh_index) {
            const auto sub_mesh = mesh->SubMeshByIndex(sub_mesh_index).lock();

            // skip if sub-mesh is not valid
            if (!sub_mesh) continue;
            // skip if not triangle mesh
            if (sub_mesh->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES) continue;

            std::vector<Vertex> vertices;
            // get reflectance if present
            const auto material_index = sub_mesh->GetMaterialIndex();
            const float reflectance = material_index.has_value() &&
                                      material_index.value() < materials.size()
                                          ? materials[material_index.value()].reflectance
                                          : default_reflectance;

            // extract triangle vertices (position + unit normal) with scale applied
            const unsigned int index_count = sub_mesh->IndexCount();
            for (unsigned int i = 0; i < index_count; ++i) {
                const auto vertex_index = static_cast<unsigned int>(sub_mesh->Index(i));
                gz::math::Vector3d pos = sub_mesh->Vertex(vertex_index);
                gz::math::Vector3d norm = sub_mesh->Normal(vertex_index);
                if (const double length = norm.Length(); length > 1e-8)
                    norm = norm / length;

                vertices.push_back({
                    .pos = {
                        static_cast<float>(pos.X() * scale.X()),
                        static_cast<float>(pos.Y() * scale.Y()),
                        static_cast<float>(pos.Z() * scale.Z())
                    },
                    .norm = {
                        static_cast<float>(norm.X()),
                        static_cast<float>(norm.Y()),
                        static_cast<float>(norm.Z())
                    },
                    .reflectance = reflectance
                });
            }

            result.insert(result.end(), vertices.begin(), vertices.end());
        }

        if (result.empty()) {
            gzwarn << "MakeMeshes: no triangle data extracted from [" << uri << "]" << std::endl;
        }

        return result;
    }

    /**
     * @brief 按 sdf 几何体类型分发，生成对应的三角形顶点
     * @note plane 及未支持的几何体类型返回空
     * @param geom sdf 几何体
     * @param reflectance 应用到所有顶点的雷达反射率（mesh 作为材质未指定反射率时的默认值）
     * @return 所有三角形的顶点（位置 + 法线 + 反射率）
     */
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
