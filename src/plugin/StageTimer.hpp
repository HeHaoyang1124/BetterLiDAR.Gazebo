#pragma once

#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <array>
#include <cstring>

namespace blgz {
    enum class Stage {
        Pose,
        Render,
        Readback,
        MsgBuild,
        Publish,
        Work,
        Count
    };

    inline const char *StageName(Stage s) {
        static const char *names[] = {
            "pose", "render",
            "readback", "msgBuild", "publish", "work"
        };
        return names[static_cast<int>(s)];
    }

    class StageTimer {
        using Clock = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;

        static constexpr int N = static_cast<int>(Stage::Count);

        std::array<double, N> accum_{};
        std::array<TimePoint, N> start_{};
        std::array<bool, N> active_{};
        int frameCount_ = 0;
        int publishCount_ = 0;

        TimePoint epoch_ = Clock::now();
        double printInterval_ = 3.0;

    public:
        void Begin(Stage s) {
            int i = static_cast<int>(s);
            start_[i] = Clock::now();
            active_[i] = true;
        }

        void End(Stage s) {
            int i = static_cast<int>(s);
            if (!active_[i]) return;
            accum_[i] += std::chrono::duration<double, std::milli>(Clock::now() - start_[i]).count();
            active_[i] = false;
        }

        void TickFrame() {
            ++frameCount_;
        }

        void TickPublish() {
            ++publishCount_;
        }

        void SetPrintInterval(double seconds) {
            printInterval_ = seconds;
        }

        bool ShouldPrint() const {
            if (frameCount_ == 0) return false;
            double elapsed = std::chrono::duration<double>(Clock::now() - epoch_).count();
            return elapsed >= printInterval_;
        }

        void PrintAndReset() {
            if (frameCount_ == 0) return;

            double nRender = static_cast<double>(frameCount_);
            double nPublish = static_cast<double>(publishCount_ > 0 ? publishCount_ : 1);

            double sec = std::chrono::duration<double>(Clock::now() - epoch_).count();
            double renderFps = sec > 0 ? nRender / sec : 0.0;
            double publishFps = sec > 0 ? static_cast<double>(publishCount_) / sec : 0.0;

            std::cerr << "\n=== BetterLidar Profile ===\n";

            for (int i = 0; i < N; ++i) {
                double n = (static_cast<Stage>(i) == Stage::MsgBuild ||
                            static_cast<Stage>(i) == Stage::Publish) ? nPublish : nRender;
                std::cerr
                        << std::left << std::setw(10) << StageName(static_cast<Stage>(i)) << std::right
                        << std::fixed << std::setprecision(3) << (accum_[i] / n) << " ms" << std::endl;
            }

            std::cerr << "render:  " << frameCount_ << " frames / "
                    << std::fixed << std::setprecision(2) << sec << "s -> "
                    << std::setprecision(1) << renderFps << " fps\n"
                    << "publish: " << publishCount_ << " msgs / "
                    << std::setprecision(2) << sec << "s -> "
                    << std::setprecision(1) << publishFps << " Hz\n"
                    << "===============================\n";

            accum_.fill(0.0);
            frameCount_ = 0;
            publishCount_ = 0;
            epoch_ = Clock::now();
        }
    };
} // namespace blgz