#pragma once

#include <string>
#include <vector>

namespace blgz {

struct LidarSchlickParams {
    float Clambda       = 0.22f;
    float r             = 0.5f;
    float p             = 0.0f;
    bool  isTransparent = false;
    bool  enableRetro   = false;
    float kRetro        = 0.0f;
    float rRetro        = 0.02f;
    float pRetro        = 0.0f;
    float f0Retro       = 0.95f;
};

std::vector<LidarSchlickParams> ParseGlbMaterials(const std::string &path);

} // namespace blgz