#pragma once

// preview_window.hpp
// ============================================================================
// MobaXterm/X11的imshow()可能因网络传输阻塞数百毫秒。如果它与视觉控制
// 在同一线程，名义上120 FPS的摄像头也可能只剩2 FPS处理速度。
//
// 本类把所有HighGUI操作放到独立线程，主线程只尝试发布一张预览副本。
// 队列永远最多只保留一张：当X11来不及显示时，丢掉的是预览帧，
// 而不是控制帧，也不会让摄像头缓冲越积越多。
// ============================================================================

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace ball_stepper {

class PreviewWindow {
    std::string windowName_;
    std::mutex mutex_;
    std::condition_variable condition_;
    cv::Mat pendingFrame_;
    bool hasPendingFrame_ = false;
    bool stopRequested_ = false;
    std::thread worker_;
    std::atomic<int> pendingKey_{-1};
    std::atomic<bool> failed_{false};

    void threadMain()
    {
        try {
            cv::namedWindow(windowName_, cv::WINDOW_AUTOSIZE);

            while (true) {
                cv::Mat displayFrame;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait_for(
                        lock, std::chrono::milliseconds(15), [this]() {
                            return stopRequested_ || hasPendingFrame_;
                        });

                    if (stopRequested_) break;
                    if (hasPendingFrame_) {
                        // Mat是引用计数容器，move后就能立即释放单槽队列。
                        displayFrame = std::move(pendingFrame_);
                        hasPendingFrame_ = false;
                    }
                }

                if (!displayFrame.empty()) {
                    cv::imshow(windowName_, displayFrame);
                }

                // waitKey必须与imshow在同一HighGUI线程执行。
                const int key = cv::waitKey(1);
                if (key >= 0) pendingKey_.store(key);
            }

            cv::destroyWindow(windowName_);
        } catch (const cv::Exception& error) {
            failed_.store(true);
            std::fprintf(stderr, "preview thread failed: %s\n", error.what());
        }
    }

public:
    explicit PreviewWindow(std::string windowName)
        : windowName_(std::move(windowName)) {}

    PreviewWindow(const PreviewWindow&) = delete;
    PreviewWindow& operator=(const PreviewWindow&) = delete;

    ~PreviewWindow()
    {
        stop();
    }

    void start()
    {
        if (worker_.joinable()) return;
        stopRequested_ = false;
        failed_.store(false);
        worker_ = std::thread(&PreviewWindow::threadMain, this);
    }

    // 主循环在克隆和绘制预览之前先调用本函数。X11线程正在显示或已有一帧
    // 排队时直接返回false，避免为一张注定被丢弃的画面做clone/putText。
    bool canAcceptFrame()
    {
        if (!worker_.joinable() || failed_.load()) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return !stopRequested_ && !hasPendingFrame_;
    }

    // frame由调用者独占，使用move把像素缓冲交给预览线程，不再二次clone。
    bool publish(cv::Mat&& frame)
    {
        if (!worker_.joinable() || frame.empty() || failed_.load()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (hasPendingFrame_) {
            // X11忙时不克隆更多画面，主控循环立即继续。
            return false;
        }
        pendingFrame_ = std::move(frame);
        hasPendingFrame_ = true;
        condition_.notify_one();
        return true;
    }

    int consumeKey()
    {
        return pendingKey_.exchange(-1);
    }

    bool failed() const
    {
        return failed_.load();
    }

    void stop()
    {
        if (!worker_.joinable()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopRequested_ = true;
            hasPendingFrame_ = false;
            pendingFrame_.release();
        }
        condition_.notify_one();
        worker_.join();
    }
};

} // namespace ball_stepper
