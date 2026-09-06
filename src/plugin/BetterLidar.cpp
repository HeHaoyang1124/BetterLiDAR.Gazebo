/*
 * Copyright (c) 2026 何昊阳(He Haoyang) <hehaoyang1124@outlook.com>
 * Licensed under MIT License
 *
 * BetterLidar — member function implementations.
 */

#include "BetterLidar.hpp"

#include <cstring>

#include <gz/common/Console.hh>

namespace blgz {
    void BetterLidar::Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager & /*_ecm*/,
                                EventManager & /*_eventMgr*/) {
        lidar_entity_ = _entity;
        // === 1. get sdf parameters ===
        auto GetElement = [&]<typename T>
        (const std::string &name, T &var) {
            if (_sdf->HasElement(name)) var = _sdf->Get<T>(name);
        };
        // clang-format off
        GetElement("hz_samples",          hz_samples_);
        GetElement("vt_samples",          vt_samples_);
        GetElement("range_min",           range_min_);
        GetElement("range_max",           range_max_);
        GetElement("hz_min_angle",        hz_min_angle_);
        GetElement("hz_max_angle",        hz_max_angle_);
        GetElement("vt_min_angle",        vt_min_angle_);
        GetElement("vt_max_angle",        vt_max_angle_);
        GetElement("max_intensity",       max_intensity_);
        GetElement("range_noise_std",     range_noise_std_);
        GetElement("intensity_noise_std", intensity_noise_std_);
        GetElement("update_rate",         update_rate_);
        GetElement("output_topic",        output_topic_);
        GetElement("frame_id",            frame_id_);
        // clang-format on

        // === 2. initialize member variables ===
        // point cloud publisher
        pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(output_topic_);

        // point cloud buffer
        point_cloud_buffer_.resize(static_cast<size_t>(hz_samples_ * vt_samples_) * 4);

        // update period
        update_period_ = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / update_rate_));

        // === 3. initialize renderer ===
        // clang-format off
        const RenderParams params{
            .width               = hz_samples_,
            .height              = vt_samples_,
            .hz_min              = hz_min_angle_,
            .hz_max              = hz_max_angle_,
            .vt_min              = vt_min_angle_,
            .vt_max              = vt_max_angle_,
            .range_min           = range_min_,
            .range_max           = range_max_,
            .range_noise_std     = range_noise_std_,
            .intensity_noise_std = intensity_noise_std_,
            .shader_dir          = SHADER_DIR "/",
            .channel_table_path  = _sdf->HasElement("channel_table_file")
                                      ? CHANNEL_TABLE_DIR "/" + _sdf->Get<std::string>("channel_table_file")
                                      : "" // default is empty
        };
        // clang-format on

        if (!renderer_.Init(params)) {
            gzerr << "BetterLidar: failed to initialize renderer, aborting" << std::endl;
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
            renderer_.CollectAndUploadGeometries(_ecm, lidar_entity_);
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
        renderer_.FinishAsyncReadbackInto(point_cloud_buffer_);
        timer_.End(Stage::Readback);

        // === 3. Publish point cloud (if available) ===
        if (!point_cloud_buffer_.empty() && _info.simTime - last_publish_time_ >= update_period_) {
            last_publish_time_ = _info.simTime;
            GenerateAndPublish(prev_sim_time_);
            timer_.TickPublish();
        }

        // === 4. Render scene ===
        timer_.Begin(Stage::Render);
        renderer_.RenderScene(lidar_pose, _ecm, max_intensity_, frame_counter_++);
        timer_.End(Stage::Render);

        renderer_.StartAsyncReadback();

        // === 5. End of work ===
        prev_sim_time_ = _info.simTime;

        timer_.End(Stage::Work);
        timer_.TickFrame();
        if (timer_.ShouldPrint()) timer_.PrintAndReset();
    }

    void BetterLidar::GenerateAndPublish(const std::chrono::nanoseconds &sim_time) {
        const int w = hz_samples_;
        const int h = vt_samples_;
        const int total_points = w * h;

        timer_.Begin(Stage::MsgBuild);

        auto &msg = cached_msg_;

        // === 0. Initialize point cloud layout (once) ===
        if (!msg_layout_initialized_) {
            int point_step_bytes = 0;

            const auto add_field = [&](const std::string &name) {
                auto &field = *msg.add_field();
                field.set_name(name);
                field.set_offset(point_step_bytes);
                field.set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
                field.set_count(1);
                point_step_bytes += sizeof(float);
            };

            add_field("x");
            add_field("y");
            add_field("z");
            add_field("intensity");

            msg.set_point_step(point_step_bytes);
            msg.set_is_bigendian(false);
            msg.set_is_dense(true);

            auto *data_field = msg.mutable_header()->add_data();
            data_field->set_key("frame_id");
            data_field->add_value(frame_id_);

            msg_layout_initialized_ = true;
        }

        const size_t point_step_bytes = msg.point_step();

        // === 1. Set point cloud timestamp ===
        const auto sec = std::chrono::duration_cast<std::chrono::seconds>(sim_time);
        const auto nsec = sim_time - sec;
        msg.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(sec.count()));
        msg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>(nsec.count()));

        // === 2. Set point cloud data ===
        msg.mutable_data()->resize(total_points * point_step_bytes);
        char *msg_data = msg.mutable_data()->data();

        int valid_points = 0;

        for (int i = 0; i < total_points; ++i) {
            const int base = i * msg.field_size();
            // skip invalid points (intensity is the 4th float of each point)
            //   0: x         (4B: 0~3)
            //   1: y         (4B: 4~7)
            //   2: z         (4B: 8~11)
            //   3: intensity (4B: 12~15)
            if (point_cloud_buffer_[base + 3] == 0.0f) continue;

            // copy valid point data

            const auto dst = &msg_data[valid_points * point_step_bytes];
            const auto src = &point_cloud_buffer_[base];
            std::memcpy(dst, src, point_step_bytes);

            ++valid_points;
        }

        msg.mutable_data()->resize(valid_points * point_step_bytes);
        msg.set_height(valid_points);
        msg.set_width(1);
        msg.set_row_step(point_step_bytes);

        timer_.End(Stage::MsgBuild);

        // === 3. Publish point cloud ===
        timer_.Begin(Stage::Publish);
        pub_.Publish(msg);
        timer_.End(Stage::Publish);
    }
} // namespace blgz
