#pragma once

#include <sdf/Geometry.hh>

#include <vector>

namespace blgz {

struct Vertex {
    float pos[3];
    float norm[3];
};

std::vector<Vertex> MakeGeometry(const sdf::Geometry &geom);

} // namespace blgz