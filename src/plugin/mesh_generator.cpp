#include "mesh_generator.hpp"

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cone.hh>
#include <sdf/Plane.hh>
#include <sdf/Mesh.hh>

#include <gz/common/MeshManager.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace blgz {
    static std::vector<Vertex> MakeBox(const sdf::Geometry &g) {
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
                            .norm = {nx, ny, nz}
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

    static std::vector<Vertex> MakeCylinder(const sdf::Geometry &g) {
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

            v.push_back({.pos = {r * c0, r * s0, -hl}, .norm = {c0, s0, 0}});
            v.push_back({.pos = {r * c1, r * s1, -hl}, .norm = {c1, s1, 0}});
            v.push_back({.pos = {r * c1, r * s1, hl}, .norm = {c1, s1, 0}});
            v.push_back({.pos = {r * c0, r * s0, -hl}, .norm = {c0, s0, 0}});
            v.push_back({.pos = {r * c1, r * s1, hl}, .norm = {c1, s1, 0}});
            v.push_back({.pos = {r * c0, r * s0, hl}, .norm = {c0, s0, 0}});
        }

        for (int i = 0; i < seg; ++i) {
            const float a0 = pi2 * static_cast<float>(i) / static_cast<float>(seg);
            const float a1 = pi2 * static_cast<float>(i + 1) / static_cast<float>(seg);
            v.push_back({.pos = {0, 0, hl}, .norm = {0, 0, 1}});
            v.push_back({.pos = {r * std::cos(a0), r * std::sin(a0), hl}, .norm = {0, 0, 1}});
            v.push_back({.pos = {r * std::cos(a1), r * std::sin(a1), hl}, .norm = {0, 0, 1}});
        }

        for (int i = 0; i < seg; ++i) {
            const float a0 = pi2 * static_cast<float>(i) / static_cast<float>(seg);
            const float a1 = pi2 * static_cast<float>(i + 1) / static_cast<float>(seg);
            v.push_back({.pos = {0, 0, -hl}, .norm = {0, 0, -1}});
            v.push_back({.pos = {r * std::cos(a1), r * std::sin(a1), -hl}, .norm = {0, 0, -1}});
            v.push_back({.pos = {r * std::cos(a0), r * std::sin(a0), -hl}, .norm = {0, 0, -1}});
        }

        return v;
    }

    static std::vector<Vertex> MakeSphere(const sdf::Geometry &g) {
        const auto *sph = g.SphereShape();
        if (!sph) return {};

        constexpr int latSeg = 24;
        const auto r = static_cast<float>(sph->Radius());
        std::vector<Vertex> v;

        for (int lat = 0; lat < latSeg; ++lat) {
            constexpr int lonSeg = 48;
            const float th0 = static_cast<float>(M_PI) * static_cast<float>(lat) / static_cast<float>(latSeg);
            const float th1 = static_cast<float>(M_PI) * static_cast<float>(lat + 1) / static_cast<float>(latSeg);
            for (int lon = 0; lon < lonSeg; ++lon) {
                const float ph0 = 2.0f * static_cast<float>(M_PI) * static_cast<float>(lon) / static_cast<float>(
                                      lonSeg);
                const float ph1 = 2.0f * static_cast<float>(M_PI) * static_cast<float>(lon + 1) / static_cast<float>(
                                      lonSeg);

                auto p = [&](const float th, const float ph) -> Vertex {
                    const float sth = std::sin(th), cth = std::cos(th);
                    const float sph = std::sin(ph), cph = std::cos(ph);
                    return {.pos = {r * sth * cph, r * sth * sph, r * cth}, .norm = {sth * cph, sth * sph, cth}};
                };

                v.push_back(p(th0, ph0));
                v.push_back(p(th1, ph0));
                v.push_back(p(th1, ph1));
                v.push_back(p(th0, ph0));
                v.push_back(p(th1, ph1));
                v.push_back(p(th0, ph1));
            }
        }
        return v;
    }

    static std::vector<Vertex> MakeCone(const sdf::Geometry &g) {
        const auto *cone = g.ConeShape();
        if (!cone) return {};

        constexpr int seg = 48;
        const auto r = static_cast<float>(cone->Radius());
        const auto length = static_cast<float>(cone->Length());
        const auto hl = length * 0.5f;
        constexpr float pi2 = 2.0f * static_cast<float>(M_PI);

        const float slantLen = std::sqrt(r * r + length * length);
        const float nr = length / slantLen;
        const float nz = r / slantLen;

        std::vector<Vertex> v;

        for (int i = 0; i < seg; ++i) {
            const float a0 = pi2 * static_cast<float>(i) / static_cast<float>(seg);
            const float a1 = pi2 * static_cast<float>(i + 1) / static_cast<float>(seg);
            const float c0 = std::cos(a0), s0 = std::sin(a0);
            const float c1 = std::cos(a1), s1 = std::sin(a1);

            v.push_back({.pos = {r * c0, r * s0, -hl}, .norm = {nr * c0, nr * s0, nz}});
            v.push_back({.pos = {r * c1, r * s1, -hl}, .norm = {nr * c1, nr * s1, nz}});
            v.push_back({.pos = {0, 0, hl}, .norm = {nr * c0, nr * s0, nz}});

            v.push_back({.pos = {r * c1, r * s1, -hl}, .norm = {nr * c1, nr * s1, nz}});
            v.push_back({.pos = {r * c0, r * s0, -hl}, .norm = {nr * c0, nr * s0, nz}});
            v.push_back({.pos = {0, 0, hl}, .norm = {nr * c1, nr * s1, nz}});
        }

        for (int i = 0; i < seg; ++i) {
            const float a0 = pi2 * static_cast<float>(i) / static_cast<float>(seg);
            const float a1 = pi2 * static_cast<float>(i + 1) / static_cast<float>(seg);
            v.push_back({.pos = {0, 0, -hl}, .norm = {0, 0, -1}});
            v.push_back({.pos = {r * std::cos(a1), r * std::sin(a1), -hl}, .norm = {0, 0, -1}});
            v.push_back({.pos = {r * std::cos(a0), r * std::sin(a0), -hl}, .norm = {0, 0, -1}});
        }

        return v;
    }

    static std::vector<Vertex> MakePlane(const sdf::Geometry &g) {
        const auto *plane = g.PlaneShape();
        if (!plane) return {};

        const auto &size = plane->Size();
        const auto hw = static_cast<float>(size.X() * 0.5);
        const auto hh = static_cast<float>(size.Y() * 0.5);

        const auto &n = plane->Normal();
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

        constexpr float targetCell = 0.1f;
        const int nu = std::max(1, static_cast<int>(std::ceil(2.0f * hw / targetCell)));
        const int nv = std::max(1, static_cast<int>(std::ceil(2.0f * hh / targetCell)));

        std::vector<Vertex> v;

        for (int i = 0; i < nu; ++i) {
            for (int j = 0; j < nv; ++j) {
                const float u0 = -hw + 2.0f * hw * static_cast<float>(i) / static_cast<float>(nu);
                const float u1 = -hw + 2.0f * hw * static_cast<float>(i + 1) / static_cast<float>(nu);
                const float v0 = -hh + 2.0f * hh * static_cast<float>(j) / static_cast<float>(nv);
                const float v1 = -hh + 2.0f * hh * static_cast<float>(j + 1) / static_cast<float>(nv);

                auto P = [&](const float u, const float vv) -> Vertex {
                    return {
                        .pos = {u * tx + vv * bx, u * ty + vv * by, u * tz + vv * bz},
                        .norm = {nx, ny, nz}
                    };
                };

                v.push_back(P(u0, v0));
                v.push_back(P(u1, v0));
                v.push_back(P(u1, v1));
                v.push_back(P(u0, v0));
                v.push_back(P(u1, v1));
                v.push_back(P(u0, v1));
            }
        }

        return v;
    }

    static std::vector<Vertex> MakeMesh(const sdf::Geometry &g) {
        const auto *meshShape = g.MeshShape();
        if (!meshShape) return {};

        const auto &uri = meshShape->Uri();
        const auto &scale = meshShape->Scale();

        std::vector<Vertex> v;

        const gz::common::Mesh *mesh = gz::common::MeshManager::Instance()->Load(uri);
        if (!mesh) {
            std::cerr << "[MakeMesh] failed to load mesh URI [" << uri << "]" << std::endl;
            return v;
        }

        for (unsigned int si = 0; si < mesh->SubMeshCount(); ++si) {
            auto subWeak = mesh->SubMeshByIndex(si);
            const auto sub = subWeak.lock();
            if (!sub) continue;

            if (sub->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES) continue;

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

                v.push_back({
                    .pos = {static_cast<float>(pos.X()), static_cast<float>(pos.Y()), static_cast<float>(pos.Z())},
                    .norm = {static_cast<float>(norm.X()), static_cast<float>(norm.Y()), static_cast<float>(norm.Z())}
                });
            }
        }

        if (v.empty())
            std::cerr << "[MakeMesh] no triangle data extracted from [" << uri << "]" << std::endl;

        return v;
    }


    std::vector<Vertex> MakeGeometry(const sdf::Geometry &geom) {
        // clang-format off
        switch (geom.Type()) {
            case sdf::GeometryType::BOX:       return MakeBox(geom);
            case sdf::GeometryType::CYLINDER:  return MakeCylinder(geom);
            case sdf::GeometryType::PLANE:     return {};
            case sdf::GeometryType::SPHERE:    return MakeSphere(geom);
            case sdf::GeometryType::MESH:      return MakeMesh(geom);
            case sdf::GeometryType::CONE:      return MakeCone(geom);
            default:                           return {};
        }
        // clang-format on
    }
} // namespace blgz