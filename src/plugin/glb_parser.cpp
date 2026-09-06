#include "glb_parser.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <cstring>

namespace blgz {

std::vector<LidarSchlickParams> ParseGlbMaterials(const std::string &path) {
    std::vector<LidarSchlickParams> result;

    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[GLB] Cannot open file: " << path << std::endl;
        return result;
    }

    uint32_t magic;
    uint32_t version;
    file.read(reinterpret_cast<char *>(&magic), 4);
    file.read(reinterpret_cast<char *>(&version), 4);
    if (magic != 0x46546C67 || version != 2) {
        std::cerr << "[GLB] Not a valid GLB v2 file: " << path << std::endl;
        return result;
    }

    uint32_t jsonLength = 0;
    {
        uint32_t totalLength;
        file.read(reinterpret_cast<char *>(&totalLength), 4);
        file.read(reinterpret_cast<char *>(&jsonLength), 4);
        uint32_t jsonType;
        file.read(reinterpret_cast<char *>(&jsonType), 4);
        if (jsonType != 0x4E4F534A) {
            std::cerr << "[GLB] JSON chunk not found: " << path << std::endl;
            return result;
        }
    }

    std::string jsonData(jsonLength, '\0');
    file.read(&jsonData[0], static_cast<std::streamsize>(jsonLength));

    nlohmann::json root;
    try {
        root = nlohmann::json::parse(jsonData);
    } catch (const nlohmann::json::parse_error &e) {
        std::cerr << "[GLB] JSON parse error: " << e.what() << std::endl;
        return result;
    }

    if (!root.contains("materials") || !root["materials"].is_array()) {
        return result;
    }

    for (const auto &mat : root["materials"]) {
        LidarSchlickParams params;

        if (mat.contains("extras") && mat["extras"].contains("lidar_schlick")) {
            const auto &s = mat["extras"]["lidar_schlick"];
            params.Clambda       = s.value("Clambda",       0.22f);
            params.r             = s.value("r",             0.5f);
            params.p             = s.value("p",             0.0f);
            params.isTransparent = s.value("is_transparent", false);
            params.enableRetro   = s.value("enable_retro",   false);
            params.kRetro        = s.value("k_retro",        0.0f);
            params.rRetro        = s.value("r_retro",        0.02f);
            params.pRetro        = s.value("p_retro",        0.0f);
            params.f0Retro       = s.value("f0_retro",       0.95f);
        }

        result.push_back(params);
    }

    return result;
}

} // namespace blgz