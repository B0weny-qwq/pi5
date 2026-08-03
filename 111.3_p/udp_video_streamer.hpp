#pragma once

// udp_video_streamer.hpp
// ============================================================================
// 将OpenCV画面异步编码为低延迟H.264/MPEG-TS并通过UDP发送。
// 队列只保留一帧；编码或无线链路来不及时丢弃显示帧，不阻塞视觉和控制循环。
// ============================================================================

#include <opencv2/opencv.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace ball_stepper {

class UdpVideoStreamer {
    std::string host_;
    int port_ = 0;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 0;
    int bitrateKbps_ = 0;

    cv::VideoWriter writer_;
    std::mutex mutex_;
    std::condition_variable condition_;
    cv::Mat pendingFrame_;
    bool hasPendingFrame_ = false;
    bool stopRequested_ = false;
    std::thread worker_;
    std::atomic<bool> failed_{false};
    std::atomic<uint64_t> sentFrames_{0};

    std::string pipeline() const
    {
        std::ostringstream text;
        text << "appsrc is-live=true format=time "
             << "! queue leaky=downstream max-size-buffers=1 "
             << "! videoconvert n-threads=2 "
             << "! video/x-raw,format=I420 "
             << "! x264enc tune=zerolatency speed-preset=ultrafast "
             << "bitrate=" << bitrateKbps_ << ' '
             << "key-int-max=" << fps_ << ' '
             << "bframes=0 threads=4 sliced-threads=true byte-stream=true "
             << "! h264parse config-interval=-1 "
             << "! mpegtsmux alignment=7 "
             << "! udpsink host=" << host_ << " port=" << port_
             << " sync=false async=false";
        return text.str();
    }

    void threadMain()
    {
        try {
            while (true) {
                cv::Mat frame;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait(lock, [this]() {
                        return stopRequested_ || hasPendingFrame_;
                    });

                    if (stopRequested_) break;
                    frame = std::move(pendingFrame_);
                    hasPendingFrame_ = false;
                }

                if (!frame.empty()) {
                    cv::Mat encodedFrame;
                    if (frame.channels() == 4) {
                        cv::cvtColor(frame, encodedFrame, cv::COLOR_BGRA2BGR);
                    } else if (frame.channels() == 1) {
                        cv::cvtColor(frame, encodedFrame, cv::COLOR_GRAY2BGR);
                    } else {
                        encodedFrame = std::move(frame);
                    }
                    writer_.write(encodedFrame);
                    ++sentFrames_;
                }
            }
        } catch (const cv::Exception& error) {
            failed_.store(true);
            std::fprintf(stderr, "UDP video stream failed: %s\n", error.what());
        }
    }

public:
    UdpVideoStreamer(std::string host,
                     int port,
                     int width,
                     int height,
                     int fps,
                     int bitrateKbps)
        : host_(std::move(host)),
          port_(port),
          width_(width),
          height_(height),
          fps_(fps),
          bitrateKbps_(bitrateKbps) {}

    UdpVideoStreamer(const UdpVideoStreamer&) = delete;
    UdpVideoStreamer& operator=(const UdpVideoStreamer&) = delete;

    ~UdpVideoStreamer()
    {
        stop();
    }

    bool start()
    {
        if (worker_.joinable()) return true;

        failed_.store(false);
        sentFrames_.store(0);
        stopRequested_ = false;
        hasPendingFrame_ = false;
        pendingFrame_.release();

        const std::string gstPipeline = pipeline();
        if (!writer_.open(gstPipeline,
                          cv::CAP_GSTREAMER,
                          0,
                          static_cast<double>(fps_),
                          cv::Size(width_, height_),
                          true)) {
            std::fprintf(stderr,
                "cannot open GStreamer UDP video pipeline; "
                "check OpenCV GStreamer support and x264enc/mpegtsmux\n");
            return false;
        }

        worker_ = std::thread(&UdpVideoStreamer::threadMain, this);
        std::fprintf(stderr,
            "UDP video ready: grayscale camera with color overlays, "
            "h264/mpegts %dx%d@%d %.1f Mbps -> %s:%d\n",
            width_, height_, fps_, bitrateKbps_ / 1000.0,
            host_.c_str(), port_);
        return true;
    }

    bool canAcceptFrame()
    {
        if (!worker_.joinable() || failed_.load()) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return !stopRequested_ && !hasPendingFrame_;
    }

    bool publish(cv::Mat frame)
    {
        if (!worker_.joinable() || frame.empty() || failed_.load()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (stopRequested_ || hasPendingFrame_) return false;
        pendingFrame_ = std::move(frame);
        hasPendingFrame_ = true;
        condition_.notify_one();
        return true;
    }

    bool failed() const
    {
        return failed_.load();
    }

    uint64_t sentFrames() const
    {
        return sentFrames_.load();
    }

    void stop()
    {
        if (worker_.joinable()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                stopRequested_ = true;
                hasPendingFrame_ = false;
                pendingFrame_.release();
            }
            condition_.notify_one();
            worker_.join();
        }
        writer_.release();
    }
};

} // namespace ball_stepper
