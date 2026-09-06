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
        if (_sdf->HasElement("atmos_atten"))         atmosAtten_        = _sdf->Get<double>("atmos_atten");
        if (_sdf->HasElement("sys_efficiency"))      sysEfficiency_     = _sdf->Get<double>("sys_efficiency");
        if (_sdf->HasElement("range_noise_std"))     rangeNoiseStd_     = _sdf->Get<double>("range_noise_std");
        if (_sdf->HasElement("intensity_noise_std")) intensityNoiseStd_ = _sdf->Get<double>("intensity_noise_std");
        if (_sdf->HasElement("output_topic"))        outputTopic_       = _sdf->Get<std::string>("output_topic");
        if (_sdf->HasElement("frame_id"))            frameId_           = _sdf->Get<std::string>("frame_id");
        // clang-format on

        pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(outputTopic_);

        if (!renderer_.Configure(hzSamples_, vtSamples_, SHADER_DIR)) {
            gzerr << "BetterLidar: renderer configuration failed" << std::endl;
            return;
        }
        renderer_.SetNoiseParams(rangeNoiseStd_, intensityNoiseStd_);
        renderer_.SetParams(hzMinAngle_, hzMaxAngle_,
                            vtMinAngle_, vtMaxAngle_,
                            rangeMin_, rangeMax_);

        resultBuffer_.resize(static_cast<size_t>(hzSamples_ * vtSamples_) * 4);

        updatePeriod_ = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / updateRate_));

        gzmsg << "BetterLidar: initialized [" << hzSamples_ << "x" << vtSamples_
                << "] render [" << hzSamples_ << "x" << vtSamples_
                << "] update_rate=" << updateRate_ << "Hz"
                << " range_noise_std=" << rangeNoiseStd_
                << " intensity_noise_std=" << intensityNoiseStd_
                << "] publishing to [" << outputTopic_ << "]" << std::endl;
    }

    void BetterLidar::PreUpdate(const UpdateInfo &_info,
                                EntityComponentManager &_ecm) {
        if (!renderer_.IsInitialized()
            || _info.paused
            || lidarEntity_ == kNullEntity)
            return;

        // Begin work
        timer_.Begin(Stage::Work);

        // 0. collect geometries
        if (!geometriesCollected_) {
            renderer_.CollectGeometries(_ecm, lidarEntity_);
            geometriesCollected_ = true;
            gzmsg << "BetterLidar: collected geometries" << std::endl;
        }

        // 1. get lidar pose
        timer_.Begin(Stage::Pose);
        const auto lidarPose = worldPose(lidarEntity_, _ecm);
        timer_.End(Stage::Pose);

        // 2. render scene
        timer_.Begin(Stage::Render);
        renderer_.RenderScene(lidarPose, _ecm, maxIntensity_, reflectance_,
                              atmosAtten_, sysEfficiency_, frameCounter_++);
        renderer_.StartAsyncReadback(hzSamples_, vtSamples_);
        timer_.End(Stage::Render);

        // 3. wait for readback to finish
        timer_.Begin(Stage::Readback);
        const bool hasData = renderer_.FinishAsyncReadbackInto(resultBuffer_, hzSamples_, vtSamples_);
        timer_.End(Stage::Readback);

        // 4. publish PointCloud message
        if (hasData && _info.simTime - lastPublishTime_ >= updatePeriod_) {
            lastPublishTime_ = _info.simTime;
            GenerateAndPublish(prevSimTime_);
            timer_.TickPublish();
        }
        prevSimTime_ = _info.simTime;

        timer_.End(Stage::Work);

        timer_.TickFrame();
        if (timer_.ShouldPrint()) timer_.PrintAndReset();
    }

    void BetterLidar::GenerateAndPublish(const std::chrono::nanoseconds &simTime) {
        const int w = hzSamples_;
        const int h = vtSamples_;

        auto &msg = cachedMsg_;

        // 0. prepare message layout
        if (!msgLayoutInitialized_) {
            msg.set_height(h);
            msg.set_width(w);

            using Field = gz::msgs::PointCloudPacked::Field;
            auto addField = [&](const std::string &name,
                                const uint32_t offset,
                                const Field::DataType dataType) {
                auto *f = msg.add_field();
                f->set_name(name);
                f->set_offset(offset);
                f->set_datatype(dataType);
                f->set_count(1);
            };
            addField("x", 0, Field::FLOAT32);
            addField("y", 4, Field::FLOAT32);
            addField("z", 8, Field::FLOAT32);
            addField("intensity", 12, Field::FLOAT32);

            msg.set_point_step(16);
            msg.set_row_step(16 * static_cast<uint32_t>(w));
            msg.set_is_bigendian(false);
            msg.set_is_dense(false);

            auto *dataField = msg.mutable_header()->add_data();
            dataField->set_key("frame_id");
            dataField->add_value(frameId_);

            msgLayoutInitialized_ = true;
        }

        timer_.Begin(Stage::MsgBuild);

        // 1. set timestamp
        const auto sec = std::chrono::duration_cast<std::chrono::seconds>(simTime);
        const auto nsec = simTime - sec;
        msg.mutable_header()->mutable_stamp()->set_sec(sec.count());
        msg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>(nsec.count()));

        // 2. set data
        const size_t dataSize = static_cast<size_t>(w * h) * 16;
        msg.mutable_data()->resize(dataSize);
        std::memcpy(msg.mutable_data()->data(), resultBuffer_.data(), dataSize);

        timer_.End(Stage::MsgBuild);

        // 3. publish message
        timer_.Begin(Stage::Publish);
        pub_.Publish(msg);
        timer_.End(Stage::Publish);
    }
} // namespace blgz
