/*
 * Copyright (c) 2026 何昊阳(He Haoyang) <hehaoyang1124@outlook.com>
 * Licensed under MIT License
 *
 * BetterLidar — member function implementations.
 */

#include "BetterLidar.hpp"

#include <gz/sim/components.hh>

#include <cstring>
#include <iomanip>
#include <numeric>

#include "../../../../../../usr/include/gz/common5/gz/common/Console.hh"

namespace blgz {
    void BetterLidar::Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager & /*_ecm*/,
                                EventManager & /*_eventMgr*/) {
        lidarEntity_ = _entity;

        // clang-format off
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
        if (_sdf->HasElement("atmos_atten"))         atmosAtten_        = _sdf->Get<double>("atmos_atten");
        if (_sdf->HasElement("sys_efficiency"))      sysEfficiency_     = _sdf->Get<double>("sys_efficiency");
        if (_sdf->HasElement("range_noise_std"))     rangeNoiseStd_     = _sdf->Get<double>("range_noise_std");
        if (_sdf->HasElement("intensity_noise_std")) intensityNoiseStd_ = _sdf->Get<double>("intensity_noise_std");
        if (_sdf->HasElement("output_topic"))        outputTopic_       = _sdf->Get<std::string>("output_topic");
        if (_sdf->HasElement("frame_id"))            frameId_           = _sdf->Get<std::string>("frame_id");
        // clang-format on

        pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(outputTopic_);

        if (!renderer_.Initialize()) {
            gzerr << "BetterLidar: Failed to initialize OpenGL" << std::endl;
            return;
        }

        fbWidth_ = hzSamples_;
        fbHeight_ = vtSamples_;
        if (!renderer_.CreateFramebuffer(fbWidth_, fbHeight_)) {
            gzerr << "BetterLidar: Failed to create FBO" << std::endl;
            return;
        }

        std::string shaderDir = std::string(SHADER_DIR) + "/";
        if (!renderer_.LoadAndCompileShaders(shaderDir)) {
            gzerr << "BetterLidar: Failed to compile shaders" << std::endl;
            return;
        }

        renderer_.SetParams(hzMinAngle_, hzMaxAngle_,
                            vtMinAngle_, vtMaxAngle_,
                            rangeMin_, rangeMax_);

        renderer_.SetNoiseParams(rangeNoiseStd_, intensityNoiseStd_);

        updatePeriod_ = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / updateRate_));

        gzmsg << "BetterLidar: initialized [" << hzSamples_ << "x" << vtSamples_
                << "] render [" << fbWidth_ << "x" << fbHeight_
                << "] update_rate=" << updateRate_ << "Hz"
                << " range_noise_std=" << rangeNoiseStd_
                << " intensity_noise_std=" << intensityNoiseStd_
                << "] publishing to [" << outputTopic_ << "]" << std::endl;
    }

    void BetterLidar::PreUpdate(const UpdateInfo &_info,
                                EntityComponentManager &_ecm) {
        if (!renderer_.IsInitialized()) return;
        if (_info.paused) return;
        if (lidarEntity_ == kNullEntity) return;

        if (_info.simTime - lastRenderTime_ < updatePeriod_) return;
        lastRenderTime_ = _info.simTime;

        timer_.Begin(Stage::Work);

        if (!geometriesCollected_) {
            timer_.Begin(Stage::Collect);
            renderer_.CollectGeometries(_ecm, lidarEntity_);
            timer_.End(Stage::Collect);
            geometriesCollected_ = true;
            gzmsg << "BetterLidar: collected geometries" << std::endl;
        }

        timer_.Begin(Stage::Pose);
        auto lidarPose = worldPose(lidarEntity_, _ecm);
        timer_.End(Stage::Pose);

        timer_.Begin(Stage::Render);
        renderer_.RenderScene(lidarPose, _ecm, maxIntensity_, reflectance_, atmosAtten_, sysEfficiency_,
                              frameCounter_++);
        timer_.End(Stage::Render);

        GenerateAndPublish(lidarPose, _info.simTime);

        timer_.End(Stage::Work);

        timer_.TickFrame();

        if (timer_.ShouldPrint()) {
            timer_.PrintAndReset();
        }
    }

    void BetterLidar::GenerateAndPublish(const gz::math::Pose3d & /*lidarPose*/,
                                         const std::chrono::nanoseconds &simTime) {
        const int w = fbWidth_;
        const int h = fbHeight_;
        const int totalOutput = w * h;

        timer_.Begin(Stage::Readback);
        std::vector<float> resultData = renderer_.ReadRenderResult(w, h);
        timer_.End(Stage::Readback);

        timer_.Begin(Stage::MsgBuild);

        gz::msgs::PointCloudPacked msg;

        auto *header = msg.mutable_header();
        auto *stamp = header->mutable_stamp();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(simTime);
        auto nsec = simTime - sec;
        stamp->set_sec(static_cast<int64_t>(sec.count()));
        stamp->set_nsec(static_cast<int32_t>(nsec.count()));
        auto *dataField = header->add_data();
        dataField->set_key("frame_id");
        dataField->add_value(frameId_);

        msg.set_height(h);
        msg.set_width(w);

        auto addField = [&](const std::string &name, uint32_t offset,
                            gz::msgs::PointCloudPacked::Field::DataType dtype) {
            auto *f = msg.add_field();
            f->set_name(name);
            f->set_offset(offset);
            f->set_datatype(dtype);
            f->set_count(1);
        };

        addField("x", 0, gz::msgs::PointCloudPacked::Field::FLOAT32);
        addField("y", 4, gz::msgs::PointCloudPacked::Field::FLOAT32);
        addField("z", 8, gz::msgs::PointCloudPacked::Field::FLOAT32);
        addField("intensity", 12, gz::msgs::PointCloudPacked::Field::FLOAT32);

        const uint32_t pointStep = 16;
        msg.set_point_step(pointStep);
        msg.set_row_step(pointStep * static_cast<uint32_t>(w));
        msg.set_is_bigendian(false);
        msg.set_is_dense(false);

        msg.mutable_data()->resize(static_cast<size_t>(totalOutput) * pointStep);
        auto *raw = msg.mutable_data()->data();

        for (int row = 0; row < h; ++row) {
            for (int col = 0; col < w; ++col) {
                const uint32_t base = static_cast<uint32_t>(row * w + col) * pointStep;

                const int idx = (row * w + col) * 4;
                const float posX = resultData[idx + 0];
                const float posY = resultData[idx + 1];
                const float posZ = resultData[idx + 2];
                const float intensity = resultData[idx + 3];

                if (posX == 0.0f && posY == 0.0f && posZ == 0.0f && intensity == 0.0f) {
                    std::memset(raw + base, 0, pointStep);
                    continue;
                }

                std::memcpy(raw + base + 0, &posX, sizeof(float));
                std::memcpy(raw + base + 4, &posY, sizeof(float));
                std::memcpy(raw + base + 8, &posZ, sizeof(float));
                std::memcpy(raw + base + 12, &intensity, sizeof(float));
            }
        }

        timer_.End(Stage::MsgBuild);

        timer_.Begin(Stage::Publish);
        pub_.Publish(msg);
        timer_.End(Stage::Publish);
    }
} // namespace blgz
