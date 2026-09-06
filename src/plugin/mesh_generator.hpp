#pragma once

#include <sdf/Geometry.hh>

#include "glb_parser.hpp"

#include <vector>

namespace blgz {

struct Vertex {
    float pos[3];
    float norm[3];
};

struct SubMeshGeometry {
    std::vector<Vertex> vertices;
    LidarSchlickParams  material;
};

std::vector<Vertex> MakeGeometry(const sdf::Geometry &geom);

std::vector<SubMeshGeometry> MakeSubMeshes(const sdf::Geometry &geom);

} // namespace blgz