#include "glb_parser.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <cstring>

namespace blgz {
    std::vector<LidarLambertParams> ParseGlbMaterials(const std::string &path) {
        std::vector<LidarLambertParams> result;

        // === 1. Open GLB file ===
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "[GLB] Cannot open file: " << path << std::endl;
            return result;
        }

        // === 2. Read JSON from GLB file ===
        // Check header
        std::string header(5, '\0');
        file.read(&header[0], 5);
        file.ignore(3);
        if (header != "glTF\x02") {
            std::cerr << "[GLB] Not a valid GLB v2 file: " << path << std::endl;
            return result;
        }

        // Read JSON chunk length
        uint32_t jsonLength, totalLength;
        file.read(reinterpret_cast<char *>(&totalLength), 4);
        file.read(reinterpret_cast<char *>(&jsonLength), 4);

        // Check JSON chunk type
        std::string jsonType(4, '\0');
        file.read(&jsonType[0], 4);
        if (jsonType != "JSON") {
            std::cerr << "[GLB] JSON chunk not found: " << path << std::endl;
            return result;
        }

        // read JSON data
        nlohmann::json root;
        std::string jsonData(jsonLength, '\0');
        file.read(&jsonData[0], jsonLength);
        try {
            root = nlohmann::json::parse(jsonData);
        } catch (const nlohmann::json::parse_error &e) {
            std::cerr << "[GLB] JSON parse error: " << e.what() << std::endl;
            return result;
        }

        // === 3. parse materials ===
        // return if 'materials' array is not found
        if (!root.contains("materials") ||
            !root["materials"].is_array())
            return result;

        // parse materials
        for (const auto &mat: root["materials"]) {
            LidarLambertParams params;
            if (mat.contains("extras") &&
                mat["extras"].contains("lidar_lambert")) {
                params.reflectance = mat["extras"]["lidar_lambert"].at("reflectance").get<float>();
            }
            result.push_back(params);
        }

        return result;
    }
} // namespace blgz
