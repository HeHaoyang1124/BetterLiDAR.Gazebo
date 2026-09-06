#pragma once

#include <string>
#include <vector>

namespace blgz {

// Reflectance assigned to entities whose material does not carry explicit
// lidar_lambert data. Deliberately tiny so unconfigured entities stand out
// as near-zero intensity in the point cloud.
inline constexpr float kDefaultReflectance = 0.001f;

struct LidarLambertParams {
    float reflectance = kDefaultReflectance;
};

std::vector<LidarLambertParams> ParseGlbMaterials(const std::string &path);

} // namespace blgz