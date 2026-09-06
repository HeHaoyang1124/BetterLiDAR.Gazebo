#pragma once

#include <sdf/Geometry.hh>

#include <vector>

namespace blgz {

struct Vertex {
    float pos[3];
    float norm[3];
    float reflectance;
};

std::vector<std::vector<Vertex>> MakeSubMeshes(const sdf::Geometry &geom);

} // namespace blgz