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

    void GenerateAndPublish(const std::chrono::nanoseconds &simTime);

    GlRenderer renderer_;
    Entity lidarEntity_ = kNullEntity;
    bool geometriesCollected_ = false;

    int hzSamples_      = 0;
    int vtSamples_      = 0;
    double hzMinAngle_  = 0.0;
    double hzMaxAngle_  = 0.0;
    double vtMinAngle_  = 0.0;
    double vtMaxAngle_  = 0.0;
    double rangeMin_    = 0.0;
    double rangeMax_    = 0.0;
    double updateRate_  = 0.0;
    std::chrono::nanoseconds updatePeriod_{0};
    std::chrono::nanoseconds lastPublishTime_{0};
    double maxIntensity_ = 0.0;
    double reflectance_  = 0.0;
    double atmosAtten_   = 0.0;
    double sysEfficiency_ = 0.0;
    double rangeNoiseStd_    = 0.0;
    double intensityNoiseStd_ = 0.0;
    uint32_t frameCounter_ = 0;
    std::string frameId_;

    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_;
    std::string outputTopic_ = "/lidar/points";

    std::vector<float> resultBuffer_;
    gz::msgs::PointCloudPacked cachedMsg_;
    bool msgLayoutInitialized_ = false;
    std::chrono::nanoseconds prevSimTime_{0};

    StageTimer timer_;
};

} // namespace blgz