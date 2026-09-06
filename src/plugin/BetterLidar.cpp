/*
 * Copyright (c) 2026 何昊阳(He Haoyang) <hehaoyang1124@outlook.com>
 * Licensed under MIT License
 *
 * BetterLidar — member function implementations.
 */

#include "BetterLidar.hpp"

#include <gz/sim/components.hh>

#include <cmath>
#include <cstring>
#include <filesystem>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>

namespace blgz {
    void BetterLidar::Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager & /*_ecm*/,
                                EventManager & /*_eventMgr*/) {
        lidarEntity_ = _entity;

        // clang-format off
        // === 1. get sdf parameters ===
        if (_sdf->HasElement("hz_samples"))          hzSamples_         = _sdf->Get<int>("hz_samples");
        if (_sdf->HasElement("vt_samples"))          vtSamples_         = _sdf->Get<int>("vt_samples");
        if (_sdf->HasElement("hz_min_angle"))        hzMinAngle_        = _sdf->Get<double>("hz_min_angle");
        if (_sdf->HasElement("hz_max_angle"))        hzMaxAngle_        = _sdf->Get<double>("hz_max_angle");
        if (_sdf->HasElement("vt_min_angle"))        vtMinAngle_        = _sdf->Get<double>("vt_min_angle");
        if (_sdf->HasElement("vt_max_angle"))        vtMaxAngle_        = _sdf->Get<double>("vt_max_angle");
        if (_sdf->HasElement("range_min"))           rangeMin_          = _sdf->Get<double>("range_min");
        if (_sdf->HasElement("range_max"))           rangeMax_          = _sdf->Get<double>("range_max");
        if (_sdf->HasElement("update_rate"))         updateRate_        = _sdf->Get<double>("update_rate");
        if (_sdf->HasElement("max_intensity"))       maxIntensity_      = _sdf->Get<double>("max_intensity");
        if (_sdf->HasElement("reflectance"))         reflectance_       = _sdf->Get<double>("reflectance");
        if (_sdf->HasElement("range_noise_std"))     rangeNoiseStd_     = _sdf->Get<double>("range_noise_std");
        if (_sdf->HasElement("intensity_noise_std")) intensityNoiseStd_ = _sdf->Get<double>("intensity_noise_std");
        if (_sdf->HasElement("output_topic"))        outputTopic_       = _sdf->Get<std::string>("output_topic");
        if (_sdf->HasElement("frame_id"))            frameId_           = _sdf->Get<std::string>("frame_id");
        // clang-format on

        // === 2. initialize member variables ===
        // point cloud publisher
        pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(outputTopic_);

        // point cloud buffer
        pointCloudBuffer_.resize(static_cast<size_t>(hzSamples_ * vtSamples_) * 4);

        // update period
        updatePeriod_ = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / updateRate_));

        // === 3. initialize renderer ===
        if (!renderer_.init(hzSamples_, vtSamples_)) {
            gzerr << "BetterLidar: Failed to initialize renderer" << std::endl;
            return;
        }

        // Load shaders
        if (!renderer_.loadAndCompileShaders(SHADER_DIR "/")) {
            gzerr << "BetterLidar: Failed to compile shaders" << std::endl;
            return;
        }

        // Set renderer parameters
        renderer_.setParams(hzMinAngle_, hzMaxAngle_,
                            vtMinAngle_, vtMaxAngle_,
                            rangeMin_, rangeMax_);

        renderer_.setNoiseParams(rangeNoiseStd_, intensityNoiseStd_);

        // Load LiDAR channel table (optional)
        const std::string rel = _sdf->HasElement("channel_table_file")
                                    ? _sdf->Get<std::string>("channel_table_file")
                                    : "";
        const std::string abs = rel.empty()
                                    ? ""
                                    : CHANNEL_TABLE_DIR "/" + rel;
        renderer_.loadChannelTable(abs);
        


        gzmsg << "BetterLidar: initialized [" << hzSamples_ << "x" << vtSamples_ << "]"
                << " update_rate=" << updateRate_ << "Hz"
                << " range_noise_std=" << rangeNoiseStd_
                << " intensity_noise_std=" << intensityNoiseStd_
                << " publishing to [" << outputTopic_ << "]" << std::endl;
    }

    void BetterLidar::PreUpdate(const UpdateInfo &_info,
                                EntityComponentManager &_ecm) {
        // return if everything is not ready
        if (_info.paused) return;
        if (!renderer_.isInitialized()) return;
        if (lidarEntity_ == kNullEntity) return;

        timer_.Begin(Stage::Work);

        // === 0. Collect geometries (once) ===
        if (!geometriesCollected_) {
            timer_.Begin(Stage::Collect);
            renderer_.collectGeometries(_ecm, lidarEntity_);
            timer_.End(Stage::Collect);
            geometriesCollected_ = true;
            gzmsg << "BetterLidar: collected geometries" << std::endl;
        }

        // === 1. Get lidar pose ===
        timer_.Begin(Stage::Pose);
        const auto lidarPose = worldPose(lidarEntity_, _ecm);
        timer_.End(Stage::Pose);

        // === 2. Readback point cloud (from last frame) ===
        timer_.Begin(Stage::Readback);
        const bool hasData = renderer_.finishAsyncReadbackInto(pointCloudBuffer_, hzSamples_, vtSamples_);
        timer_.End(Stage::Readback);

        // === 3. Publish point cloud (if available) ===
        if (hasData && _info.simTime - lastPublishTime_ >= updatePeriod_) {
            lastPublishTime_ = _info.simTime;
            GenerateAndPublish(prevSimTime_);
            timer_.TickPublish();
        }

        // === 4. Render scene ===
        timer_.Begin(Stage::Render);
        renderer_.renderScene(lidarPose, _ecm, maxIntensity_, reflectance_,
                              frameCounter_++);
        timer_.End(Stage::Render);

        renderer_.startAsyncReadback(hzSamples_, vtSamples_);

        // === 5. End of work ===
        prevSimTime_ = _info.simTime;

        timer_.End(Stage::Work);
        timer_.TickFrame();
        if (timer_.ShouldPrint()) timer_.PrintAndReset();
    }

    void BetterLidar::GenerateAndPublish(const std::chrono::nanoseconds &simTime) {
        const int w = hzSamples_;
        const int h = vtSamples_;
        const int totalOutput = w * h;

        timer_.Begin(Stage::MsgBuild);

        auto &msg = cachedMsg_;

        // === 0. Initialize point cloud layout (once) ===
        if (!msgLayoutInitialized_) {
            auto add_field = [&](const std::string &name,
                                 const uint32_t offset,
                                 const gz::msgs::PointCloudPacked::Field::DataType dtype) {
                auto *f = msg.add_field();
                f->set_name(name);
                f->set_offset(offset);
                f->set_datatype(dtype);
                f->set_count(1);
            };

            add_field("x", 0, gz::msgs::PointCloudPacked::Field::FLOAT32);
            add_field("y", 4, gz::msgs::PointCloudPacked::Field::FLOAT32);
            add_field("z", 8, gz::msgs::PointCloudPacked::Field::FLOAT32);
            add_field("intensity", 12, gz::msgs::PointCloudPacked::Field::FLOAT32);

            msg.set_point_step(16);
            msg.set_is_bigendian(false);
            msg.set_is_dense(true);

            auto *dataField = msg.mutable_header()->add_data();
            dataField->set_key("frame_id");
            dataField->add_value(frameId_);

            msgLayoutInitialized_ = true;
        }

        // === 1. Set point cloud timestamp ===
        const auto sec = std::chrono::duration_cast<std::chrono::seconds>(simTime);
        const auto nsec = simTime - sec;
        msg.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(sec.count()));
        msg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>(nsec.count()));

        // === 2. Set point cloud data ===
        const size_t maxDataSize = static_cast<size_t>(totalOutput) * 16;
        msg.mutable_data()->resize(maxDataSize);
        auto *msgData = msg.mutable_data()->data();
        size_t countValidPoints = 0;

        for (int i = 0; i < totalOutput; ++i) {
            const size_t base = static_cast<size_t>(i) * 4;

            // skip invalid points
            //   0: x         (4B: 0~3)
            //   1: y         (4B: 4~7)
            //   2: z         (4B: 8~11)
            //   3: intensity (4B: 12~15)
            if (pointCloudBuffer_[base + 3] == 0.0f) continue;

            // copy valid point data
            const auto dst = msgData + countValidPoints * 16;
            const auto src = &pointCloudBuffer_[base];
            std::memcpy(dst, src, 16);

            ++countValidPoints;
        }

        msg.mutable_data()->resize(countValidPoints * 16);
        msg.set_height(static_cast<int>(countValidPoints));
        msg.set_width(1);
        msg.set_row_step(16);

        timer_.End(Stage::MsgBuild);

        // === 3. Publish point cloud ===
        timer_.Begin(Stage::Publish);
        pub_.Publish(msg);
        timer_.End(Stage::Publish);
    }
} // namespace blgz