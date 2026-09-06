#pragma once

#include <string>
#include <vector>

namespace blgz {

struct LidarLambertParams {
    float reflectance = 0.3f;
};

std::vector<LidarLambertParams> ParseGlbMaterials(const std::string &path);

} // namespace blgz