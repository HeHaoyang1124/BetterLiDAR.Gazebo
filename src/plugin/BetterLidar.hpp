/*
 * Copyright (c) 2026 何昊阳(He Haoyang) <hehaoyang1124@outlook.com>
 * Licensed under MIT License
 *
 * BetterLidar — Gazebo Harmonic system plugin that simulates a lidar
 * sensor using offscreen rendering.
 */

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/math/Pose3.hh>

#include "GlRenderer.hpp"
#include "StageTimer.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace blgz {

using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::EventManager;
using gz::sim::UpdateInfo;
using gz::sim::kNullEntity;
using gz::sim::System;
using gz::sim::ISystemConfigure;
using gz::sim::ISystemPreUpdate;
using gz::sim::worldPose;

class BetterLidar
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate {
public:
    BetterLidar() = default;
    ~BetterLidar() override = default;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override;

private:

    void GenerateAndPublish(const std::chrono::nanoseconds &sim_time);

    GlRenderer renderer_;
    Entity lidar_entity_ = kNullEntity;
    bool geometries_collected_ = false;

    int hz_samples_      = 0;
    int vt_samples_      = 0;
    double update_rate_  = 0.0;
    std::chrono::nanoseconds update_period_{0};
    std::chrono::nanoseconds last_publish_time_{0};
    double max_intensity_ = 0.0;
    double range_noise_std_    = 0.0;
    double intensity_noise_std_ = 0.0;
    double hz_min_angle_ = 0.0;
    double hz_max_angle_ = 0.0;
    double vt_min_angle_ = 0.0;
    double vt_max_angle_ = 0.0;
    double range_min_    = 0.0;
    double range_max_    = 0.0;
    uint32_t frame_counter_ = 0;
    std::string frame_id_;

    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_;
    std::string output_topic_ = "/lidar/points";

    std::vector<float> point_cloud_buffer_;
    gz::msgs::PointCloudPacked cached_msg_;
    bool msg_layout_initialized_ = false;
    std::chrono::nanoseconds prev_sim_time_{0};

    StageTimer timer_;
};

} // namespace blgz