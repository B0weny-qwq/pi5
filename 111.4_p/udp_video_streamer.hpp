#pragma once

// udp_video_streamer.hpp
// ============================================================================
// 将OpenCV画面异步编码为低延迟H.264/MPEG-TS并通过UDP发送。
// 队列只保留一帧；编码或无线链路来不及时丢弃显示帧，不阻塞视觉和控制循环。
// ============================================================================

#include <opencv2/opencv.hpp>

#include <atomic>
#include <csignal>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

    std::FILE* encoderPipe_ = nullptr;
    std::mutex mutex_;
    std::condition_variable condition_;
    cv::Mat pendingFrame_;
    bool hasPendingFrame_ = false;
    bool stopRequested_ = false;
    std::thread worker_;
    std::atomic<bool> failed_{false};
    std::atomic<uint64_t> sentFrames_{0};

    std::string encoderCommand() const
    {
        std::ostringstream text;
        text << "ffmpeg -hide_banner -loglevel warning -nostdin "
             << "-f rawvideo -pixel_format bgr24 "
             << "-video_size " << width_ << 'x' << height_ << ' '
             << "-framerate " << fps_ << " -i pipe:0 -an "
             << "-c:v libx264 -preset ultrafast -tune zerolatency "
             << "-pix_fmt yuv420p -b:v " << bitrateKbps_ << "k "
             << "-maxrate " << bitrateKbps_ << "k "
             << "-bufsize " << bitrateKbps_ * 2 << "k "
             << "-g " << fps_ << " -keyint_min " << fps_ << " -bf 0 "
             << "-f mpegts -muxdelay 0 -muxpreload 0 -flush_packets 1 "
             << "\"udp://" << host_ << ':' << port_
             << "?pkt_size=1316\"";
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
                    if (!encodedFrame.isContinuous()) {
                        encodedFrame = encodedFrame.clone();
                    }
                    const std::size_t bytes = encodedFrame.total() *
                        encodedFrame.elemSize();
                    const std::size_t written = std::fwrite(
                        encodedFrame.data, 1, bytes, encoderPipe_);
                    std::fflush(encoderPipe_);
                    if (written != bytes) {
                        failed_.store(true);
                        std::fprintf(stderr,
                            "UDP video ffmpeg pipe stopped after %llu frames\n",
                            static_cast<unsigned long long>(sentFrames_.load()));
                        break;
                    }
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

        if (std::system("command -v ffmpeg >/dev/null 2>&1") != 0) {
            std::fprintf(stderr,
                "cannot start UDP video: ffmpeg is not installed\n");
            return false;
        }

        std::signal(SIGPIPE, SIG_IGN);
        const std::string command = encoderCommand();
        encoderPipe_ = ::popen(command.c_str(), "w");
        if (!encoderPipe_) {
            std::fprintf(stderr, "cannot open ffmpeg UDP video pipe\n");
            return false;
        }
        std::setvbuf(encoderPipe_, nullptr, _IONBF, 0);

        worker_ = std::thread(&UdpVideoStreamer::threadMain, this);
        std::fprintf(stderr,
            "UDP video ready: ffmpeg h264/mpegts %dx%d@%d "
            "%.1f Mbps -> %s:%d\n",
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
        if (encoderPipe_) {
            std::fflush(encoderPipe_);
            ::pclose(encoderPipe_);
            encoderPipe_ = nullptr;
        }
    }
};

} // namespace ball_stepper
