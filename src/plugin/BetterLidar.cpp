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
#include <iomanip>
#include <numeric>

#include <gz/common/Console.hh>

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

        if (!renderer_.CreatePBOs(fbWidth_, fbHeight_)) {
            gzerr << "BetterLidar: Failed to create PBOs" << std::endl;
            return;
        }

        resultBuffer_.resize(static_cast<size_t>(fbWidth_ * fbHeight_) * 4);

        std::string shaderDir = std::string(SHADER_DIR) + "/";
        if (!renderer_.LoadAndCompileShaders(shaderDir)) {
            gzerr << "BetterLidar: Failed to compile shaders" << std::endl;
            return;
        }

        renderer_.SetParams(hzMinAngle_, hzMaxAngle_,
                            vtMinAngle_, vtMaxAngle_,
                            rangeMin_, rangeMax_);

        renderer_.SetNoiseParams(rangeNoiseStd_, intensityNoiseStd_);

        constexpr float degToRad = static_cast<float>(M_PI) / 180.0f;
        renderer_.SetHzOffsets(
            2.4f  * degToRad,
           -0.65f * degToRad,
           -2.4f  * degToRad,
            0.65f * degToRad);

        {
            const int n = vtSamples_;
            std::vector<float> chData(static_cast<size_t>(n) * 4);
            for (int i = 0; i < n; ++i) {
                const int chNum = i + 1;
                const int block = i / 16;
                const int rem   = chNum % 4;

                int group;
                if (block % 2 == 0) {
                    group = (rem == 1) ? 0 : (rem == 2) ? 1 : (rem == 3) ? 2 : 3;
                } else {
                    group = (rem == 1) ? 2 : (rem == 2) ? 0 : (rem == 3) ? 3 : 1;
                }

                float rMin = (chNum % 4 == 3) ? 0.5f : 4.5f;
                float rMax = (chNum >= 33 && chNum <= 96) ? 260.0f : 90.0f;

                const size_t base = static_cast<size_t>(n - 1 - i) * 4;
                chData[base + 0] = rMin;
                chData[base + 1] = rMax;
                chData[base + 2] = static_cast<float>(group);
                chData[base + 3] = 0.0f;
            }
            renderer_.CreateChannelTexture(chData.data(), n);
        }

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

        timer_.Begin(Stage::Readback);
        bool hasData = renderer_.FinishAsyncReadbackInto(resultBuffer_, fbWidth_, fbHeight_);
        timer_.End(Stage::Readback);

        if (hasData && _info.simTime - lastPublishTime_ >= updatePeriod_) {
            lastPublishTime_ = _info.simTime;
            GenerateAndPublish(lidarPose, prevSimTime_);
            timer_.TickPublish();
        }

        timer_.Begin(Stage::Render);
        renderer_.RenderScene(lidarPose, _ecm, maxIntensity_, reflectance_,
                              frameCounter_++);
        timer_.End(Stage::Render);

        renderer_.StartAsyncReadback(fbWidth_, fbHeight_);
        prevSimTime_ = _info.simTime;

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

        timer_.Begin(Stage::MsgBuild);

        auto &msg = cachedMsg_;

        if (!msgLayoutInitialized_) {
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

            msg.set_point_step(16);
            msg.set_is_bigendian(false);
            msg.set_is_dense(true);

            auto *dataField = msg.mutable_header()->add_data();
            dataField->set_key("frame_id");
            dataField->add_value(frameId_);

            msgLayoutInitialized_ = true;
        }

        auto sec = std::chrono::duration_cast<std::chrono::seconds>(simTime);
        auto nsec = simTime - sec;
        msg.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(sec.count()));
        msg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>(nsec.count()));

        int validCount = 0;
        for (int i = 0; i < totalOutput; ++i) {
            if (resultBuffer_[i * 4 + 3] > 0.0f) ++validCount;
        }

        const size_t dataSize = static_cast<size_t>(validCount) * 16;
        msg.set_height(validCount);
        msg.set_width(1);
        msg.set_row_step(16);
        msg.mutable_data()->resize(dataSize);

        if (validCount > 0) {
            auto *dst = msg.mutable_data()->data();
            size_t outIdx = 0;
            for (int i = 0; i < totalOutput; ++i) {
                const size_t srcBase = static_cast<size_t>(i) * 4;
                if (resultBuffer_[srcBase + 3] > 0.0f) {
                    std::memcpy(dst + outIdx * 16, &resultBuffer_[srcBase], 16);
                    ++outIdx;
                }
            }
        }

        timer_.End(Stage::MsgBuild);

        timer_.Begin(Stage::Publish);
        pub_.Publish(msg);
        timer_.End(Stage::Publish);
    }
} // namespace blgz