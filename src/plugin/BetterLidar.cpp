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
        lidar_entity_ = _entity;

        // clang-format off
        // === 1. get sdf parameters ===
        if (_sdf->HasElement("hz_samples"))          hz_samples_          = _sdf->Get<int>("hz_samples");
        if (_sdf->HasElement("vt_samples"))          vt_samples_          = _sdf->Get<int>("vt_samples");
        double hz_min_angle = 0.0, hz_max_angle = 0.0;
        double vt_min_angle = 0.0, vt_max_angle = 0.0;
        double range_min = 0.0, range_max = 0.0;
        if (_sdf->HasElement("hz_min_angle"))        hz_min_angle         = _sdf->Get<double>("hz_min_angle");
        if (_sdf->HasElement("hz_max_angle"))        hz_max_angle         = _sdf->Get<double>("hz_max_angle");
        if (_sdf->HasElement("vt_min_angle"))        vt_min_angle         = _sdf->Get<double>("vt_min_angle");
        if (_sdf->HasElement("vt_max_angle"))        vt_max_angle         = _sdf->Get<double>("vt_max_angle");
        if (_sdf->HasElement("range_min"))           range_min            = _sdf->Get<double>("range_min");
        if (_sdf->HasElement("range_max"))           range_max            = _sdf->Get<double>("range_max");
        if (_sdf->HasElement("update_rate"))         update_rate_         = _sdf->Get<double>("update_rate");
        if (_sdf->HasElement("max_intensity"))       max_intensity_       = _sdf->Get<double>("max_intensity");
        if (_sdf->HasElement("reflectance"))         reflectance_         = _sdf->Get<double>("reflectance");
        if (_sdf->HasElement("range_noise_std"))     range_noise_std_     = _sdf->Get<double>("range_noise_std");
        if (_sdf->HasElement("intensity_noise_std")) intensity_noise_std_ = _sdf->Get<double>("intensity_noise_std");
        if (_sdf->HasElement("output_topic"))        output_topic_        = _sdf->Get<std::string>("output_topic");
        if (_sdf->HasElement("frame_id"))            frame_id_            = _sdf->Get<std::string>("frame_id");
        // clang-format on

        // === 2. initialize member variables ===
        // point cloud publisher
        pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(output_topic_);

        // point cloud buffer
        point_cloud_buffer_.resize(static_cast<size_t>(hz_samples_ * vt_samples_) * 4);

        // update period
        update_period_ = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / update_rate_));

        // === 3. initialize renderer ===
        if (!renderer_.init(hz_samples_, vt_samples_)) {
            gzerr << "BetterLidar: Failed to initialize renderer" << std::endl;
            return;
        }

        // Load shaders
        if (!renderer_.load_and_compile_shaders(SHADER_DIR "/")) {
            gzerr << "BetterLidar: Failed to compile shaders" << std::endl;
            return;
        }

        // Set renderer parameters
        renderer_.set_params(hz_min_angle, hz_max_angle,
                            vt_min_angle, vt_max_angle,
                            range_min, range_max);

        renderer_.set_noise_params(range_noise_std_, intensity_noise_std_);

        // Load LiDAR channel table (optional)
        const auto path = _sdf->HasElement("channel_table_file")
                              ? CHANNEL_TABLE_DIR "/" + _sdf->Get<std::string>("channel_table_file")
                              : ""; // default is empty
        if (!renderer_.load_channel_table(path)) {
            gzerr << "BetterLidar: failed to load channel table, aborting" << std::endl;
            return;
        }


        gzmsg << "BetterLidar: initialized [" << hz_samples_ << "x" << vt_samples_ << "]"
                << " update_rate=" << update_rate_ << "Hz"
                << " range_noise_std=" << range_noise_std_
                << " intensity_noise_std=" << intensity_noise_std_
                << " publishing to [" << output_topic_ << "]" << std::endl;
    }

    void BetterLidar::PreUpdate(const UpdateInfo &_info,
                                EntityComponentManager &_ecm) {
        // return if everything is not ready
        if (_info.paused) return;
        if (!renderer_.is_initialized()) return;
        if (lidar_entity_ == kNullEntity) return;

        timer_.Begin(Stage::Work);

        // === 0. Collect geometries (once) ===
        if (!geometries_collected_) {
            timer_.Begin(Stage::Collect);
            renderer_.collect_and_upload_geometries(_ecm, lidar_entity_);
            timer_.End(Stage::Collect);
            geometries_collected_ = true;
            gzmsg << "BetterLidar: collected geometries" << std::endl;
        }

        // === 1. Get lidar pose ===
        timer_.Begin(Stage::Pose);
        const auto lidar_pose = worldPose(lidar_entity_, _ecm);
        timer_.End(Stage::Pose);

        // === 2. Readback point cloud (from last frame) ===
        timer_.Begin(Stage::Readback);
        const bool has_data = renderer_.finish_async_readback_into(point_cloud_buffer_, hz_samples_, vt_samples_);
        timer_.End(Stage::Readback);

        // === 3. Publish point cloud (if available) ===
        if (has_data && _info.simTime - last_publish_time_ >= update_period_) {
            last_publish_time_ = _info.simTime;
            generate_and_publish(prev_sim_time_);
            timer_.TickPublish();
        }

        // === 4. Render scene ===
        timer_.Begin(Stage::Render);
        renderer_.render_scene(lidar_pose, _ecm, max_intensity_, reflectance_,
                              frame_counter_++);
        timer_.End(Stage::Render);

        renderer_.start_async_readback(hz_samples_, vt_samples_);

        // === 5. End of work ===
        prev_sim_time_ = _info.simTime;

        timer_.End(Stage::Work);
        timer_.TickFrame();
        if (timer_.ShouldPrint()) timer_.PrintAndReset();
    }

    void BetterLidar::generate_and_publish(const std::chrono::nanoseconds &sim_time) {
        const int w = hz_samples_;
        const int h = vt_samples_;
        const int total_output = w * h;

        timer_.Begin(Stage::MsgBuild);

        auto &msg = cached_msg_;

        // === 0. Initialize point cloud layout (once) ===
        if (!msg_layout_initialized_) {
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

            auto *data_field = msg.mutable_header()->add_data();
            data_field->set_key("frame_id");
            data_field->add_value(frame_id_);

            msg_layout_initialized_ = true;
        }

        // === 1. Set point cloud timestamp ===
        const auto sec = std::chrono::duration_cast<std::chrono::seconds>(sim_time);
        const auto nsec = sim_time - sec;
        msg.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(sec.count()));
        msg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>(nsec.count()));

        // === 2. Set point cloud data ===
        const size_t max_data_size = static_cast<size_t>(total_output) * 16;
        msg.mutable_data()->resize(max_data_size);
        auto *msg_data = msg.mutable_data()->data();
        size_t count_valid_points = 0;

        for (int i = 0; i < total_output; ++i) {
            const size_t base = static_cast<size_t>(i) * 4;

            // skip invalid points
            //   0: x         (4B: 0~3)
            //   1: y         (4B: 4~7)
            //   2: z         (4B: 8~11)
            //   3: intensity (4B: 12~15)
            if (point_cloud_buffer_[base + 3] == 0.0f) continue;

            // copy valid point data
            const auto dst = msg_data + count_valid_points * 16;
            const auto src = &point_cloud_buffer_[base];
            std::memcpy(dst, src, 16);

            ++count_valid_points;
        }

        msg.mutable_data()->resize(count_valid_points * 16);
        msg.set_height(static_cast<int>(count_valid_points));
        msg.set_width(1);
        msg.set_row_step(16);

        timer_.End(Stage::MsgBuild);

        // === 3. Publish point cloud ===
        timer_.Begin(Stage::Publish);
        pub_.Publish(msg);
        timer_.End(Stage::Publish);
    }
} // namespace blgz