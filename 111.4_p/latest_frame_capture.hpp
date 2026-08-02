#pragma once

// latest_frame_capture.hpp
// ============================================================================
// 摄像头采集线程：始终只保存“最新一帧”，不建立无限队列。
//
// OpenCV主循环若按“识别 -> imshow -> camera.read”串行执行，图像处理和等候
// 下一帧的时间会相加；MobaXterm/X11稍慢时还可能继续积压旧画面。本类让UVC
// 采集与识别并行，识别线程来不及时直接跳过旧帧，控制永远使用最新画面。
//
// VideoCapture只在本线程调用read()；主线程只在启动前设置参数和读取首帧，
// 避免同一个VideoCapture被两个线程同时访问。
// ============================================================================

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

namespace ball_stepper {

class LatestFrameCapture {
    cv::VideoCapture& camera_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::shared_ptr<const cv::Mat> latestFrame_;
    uint64_t latestSequence_ = 0;
    bool stopRequested_ = false;
    std::thread worker_;
    std::atomic<bool> failed_{false};
    std::atomic<double> measuredCaptureFps_{0.0};

    void threadMain()
    {
        using Clock = std::chrono::steady_clock;
        auto fpsStart = Clock::now();
        int fpsFrames = 0;

        while (true) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stopRequested_) break;
            }

            cv::Mat captured;
            if (!camera_.read(captured) || captured.empty()) {
                failed_.store(true);
                condition_.notify_all();
                break;
            }

            // 每次read使用新的Mat，并通过shared_ptr发布不可变帧。主线程即使
            // 正在识别上一帧，该像素缓冲也不会被下一次camera.read覆盖。
            auto published = std::make_shared<const cv::Mat>(
                std::move(captured));
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stopRequested_) break;
                latestFrame_ = std::move(published);
                ++latestSequence_;
            }
            condition_.notify_one();

            ++fpsFrames;
            const auto now = Clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - fpsStart).count();
            if (elapsed >= 1.0) {
                measuredCaptureFps_.store(fpsFrames / elapsed);
                fpsFrames = 0;
                fpsStart = now;
            }
        }
    }

public:
    explicit LatestFrameCapture(cv::VideoCapture& camera)
        : camera_(camera) {}

    LatestFrameCapture(const LatestFrameCapture&) = delete;
    LatestFrameCapture& operator=(const LatestFrameCapture&) = delete;

    ~LatestFrameCapture()
    {
        stop();
    }

    void start()
    {
        if (worker_.joinable()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopRequested_ = false;
            latestSequence_ = 0;
            latestFrame_.reset();
        }
        failed_.store(false);
        measuredCaptureFps_.store(0.0);
        worker_ = std::thread(&LatestFrameCapture::threadMain, this);
    }

    bool waitForNext(uint64_t& consumedSequence,
                     std::shared_ptr<const cv::Mat>& frame,
                     int timeoutMs = 100)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(
            lock,
            std::chrono::milliseconds(timeoutMs),
            [&]() {
                return latestSequence_ > consumedSequence ||
                       failed_.load() || stopRequested_;
            });

        if (latestSequence_ <= consumedSequence || !latestFrame_) {
            return false;
        }
        consumedSequence = latestSequence_;
        frame = latestFrame_;
        return true;
    }

    bool failed() const
    {
        return failed_.load();
    }

    double measuredFps() const
    {
        return measuredCaptureFps_.load();
    }

    void stop()
    {
        if (!worker_.joinable()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopRequested_ = true;
        }
        condition_.notify_all();
        worker_.join();
    }
};

} // namespace ball_stepper
