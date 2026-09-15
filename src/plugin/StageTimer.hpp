#pragma once

#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <array>
#include <cstring>

namespace blgz {
    enum class Stage {
        Collect,
        Pose,
        Render,
        Readback,
        GpuTransfer,
        MsgBuild,
        Publish,
        Work,
        Count
    };

    inline const char *StageName(Stage s) {
        static const char *names[] = {
            "collect", "pose", "render",
            "readback", "gpuXfer", "msgBuild", "publish", "work"
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

            double n = static_cast<double>(frameCount_);
            auto avg = [&](int i) { return accum_[i] / n; };

            double sec = std::chrono::duration<double>(Clock::now() - epoch_).count();
            double fps = sec > 0 ? n / sec : 0.0;

            std::cerr << "\n=== BetterLidar Profile ===\n";

            for (int i = 0; i < N; ++i) {
                std::cerr
                        << std::left << std::setw(10) << StageName(static_cast<Stage>(i)) << std::right
                        << std::fixed << std::setprecision(3) << avg(i) << " ms" << std::endl;
            }

            std::cerr << "total:\t" << frameCount_ << " frames / "
                    << std::fixed << std::setprecision(2) << sec << "s -> "
                    << std::setprecision(1) << fps << " fps\n"
                    << "===============================\n";

            accum_.fill(0.0);
            frameCount_ = 0;
            epoch_ = Clock::now();
        }
    };
} // namespace blgz