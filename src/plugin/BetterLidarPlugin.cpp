/*
 * Copyright (c) 2026 何昊阳(He Haoyang) <hehaoyang1124@outlook.com>
 * Licensed under MIT License
 *
 * BetterLidar — Gazebo plugin registration.
 */

#include "BetterLidar.hpp"
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(
    blgz::BetterLidar,
    gz::sim::System,
    blgz::BetterLidar::ISystemConfigure,
    blgz::BetterLidar::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    blgz::BetterLidar,
    "blgz::BetterLidar")