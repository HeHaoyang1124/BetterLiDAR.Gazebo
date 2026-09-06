#pragma once

#include <array>
#include <vector>

#include <sdf/Geometry.hh>

namespace blgz {

struct Vertex {
    std::array<float, 3> pos;
    std::array<float, 3> norm;
    float reflectance;
};

std::vector<Vertex> MakeSubMeshes(const sdf::Geometry &geom, float reflectance);

} // namespace blgz