/*
 * Copyright (c) 2026 何昊阳(He Haoyang) <hehaoyang1124@outlook.com>
 * Licensed under MIT License
 *
 * BetterLidar — Gazebo plugin registration.
 */

#include "BetterLidar.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

namespace blgz {
    namespace {
        // No-op system plugin serving purely as a data carrier for the custom
        // <visual><plugin><lidar_lambert> reflectance metadata. Registering it keeps
        // the server-side SystemLoader from reporting "library does not contain
        // requested plugin"; the reflectance value itself is read from the
        // VisualPlugin component.
        class BetterLidarPlugin : public gz::sim::System {
        public:
            BetterLidarPlugin() = default;
            ~BetterLidarPlugin() override = default;
        };
    }

} // namespace blgz

GZ_ADD_PLUGIN(
    blgz::BetterLidar,
    gz::sim::System,
    blgz::BetterLidar::ISystemConfigure,
    blgz::BetterLidar::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    blgz::BetterLidar,
    "blgz::BetterLidar")

GZ_ADD_PLUGIN(
    blgz::BetterLidarPlugin,
    gz::sim::System)

GZ_ADD_PLUGIN_ALIAS(
    blgz::BetterLidarPlugin,
    "blgz::BetterLidarPlugin")