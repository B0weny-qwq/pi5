#pragma once

// balance_control.hpp
// ============================================================================
// 本文件负责把钢球像素圆心转换为厘米位置，估算钢球速度，用PD得到水管目标
// 倾角，再通过实测标定表或曲柄连杆近似公式换算成目标电机轴位脉冲。
// 它不访问摄像头、不打开串口，也不直接驱动电机。
// ============================================================================

#include "system_config.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

namespace ball_stepper {

inline constexpr double PI = 3.14159265358979323846;

class PipeAxis {
    // left_是水管一维坐标0 cm的位置。
    cv::Point2f left_{};
    // unit_是axisLeft->axisRight的单位向量，摄像头倾斜时仍沿真实水管方向。
    cv::Point2f unit_{1.0f, 0.0f};
    float pixelLength_ = 1.0f;
    double pipeLengthCm_ = 25.0;

    // 三点分段标定专门处理“O左边5cm”和“O右边5cm”像素比例不同的情况。
    bool useThreePointCalibration_ = false;
    cv::Point2f minus5Point_{};
    cv::Point2f centerPoint_{};
    cv::Point2f plus5Point_{};
    cv::Point2f calibrationUnit_{1.0f, 0.0f};
    float leftFivePixels_ = 1.0f;
    float rightFivePixels_ = 1.0f;
    double calibrationOffsetCm_ = 5.0;
    double centerCoordinateCm_ = 12.5;

public:
    explicit PipeAxis(const AppConfig& config)
        : left_(config.axisLeft),
          pipeLengthCm_(config.pipeLengthCm),
          useThreePointCalibration_(
              config.useThreePointPositionCalibration),
          minus5Point_(config.minus5CalibrationPoint),
          centerPoint_(config.centerCalibrationPoint),
          plus5Point_(config.plus5CalibrationPoint),
          calibrationOffsetCm_(config.task3OffsetCm),
          centerCoordinateCm_(config.pipeLengthCm * 0.5)
    {
        const cv::Point2f difference =
            config.axisRight - config.axisLeft;
        pixelLength_ = std::max(
            1.0f, static_cast<float>(cv::norm(difference)));
        unit_ = difference * (1.0f / pixelLength_);

        if (useThreePointCalibration_) {
            const cv::Point2f calibrationDifference =
                plus5Point_ - minus5Point_;
            const float calibrationLength = std::max(
                1.0f,
                static_cast<float>(cv::norm(calibrationDifference)));
            calibrationUnit_ =
                calibrationDifference * (1.0f / calibrationLength);

            const cv::Point2f leftDifference =
                centerPoint_ - minus5Point_;
            const cv::Point2f rightDifference =
                plus5Point_ - centerPoint_;
            leftFivePixels_ = std::max(
                1.0f,
                leftDifference.x * calibrationUnit_.x +
                leftDifference.y * calibrationUnit_.y);
            rightFivePixels_ = std::max(
                1.0f,
                rightDifference.x * calibrationUnit_.x +
                rightDifference.y * calibrationUnit_.y);
        }
    }

    double toCentimeters(const cv::Point2f& point) const
    {
        if (useThreePointCalibration_) {
            // 将圆心投影到“-5 -> O -> +5”方向。垂直于水管的小抖动不参与位置计算。
            const cv::Point2f fromCenter = point - centerPoint_;
            const double projectedPixels =
                fromCenter.x * calibrationUnit_.x +
                fromCenter.y * calibrationUnit_.y;

            // 左右两侧分别使用自己的像素/cm比例：
            // x=110 -> O-5cm，x=220 -> O，x=370 -> O+5cm。
            const double offsetCm = projectedPixels < 0.0 ?
                projectedPixels * calibrationOffsetCm_ / leftFivePixels_ :
                projectedPixels * calibrationOffsetCm_ / rightFivePixels_;
            return centerCoordinateCm_ + offsetCm;
        }

        // 将二维圆心投影到水管轴线。垂直于水管的小幅识别抖动不会直接变成位置误差。
        const cv::Point2f difference = point - left_;
        const double projectedPixels =
            difference.x * unit_.x + difference.y * unit_.y;
        // 不强制截断到0～管长，越界数值可以帮助发现ROI或轴线标错。
        return projectedPixels * pipeLengthCm_ / pixelLength_;
    }

    cv::Point targetPoint(double centimeters) const
    {
        if (useThreePointCalibration_) {
            const double offsetCm = centimeters - centerCoordinateCm_;
            cv::Point2f point = centerPoint_;
            if (offsetCm < 0.0) {
                const float ratio = static_cast<float>(
                    -offsetCm / calibrationOffsetCm_);
                point += (minus5Point_ - centerPoint_) * ratio;
            } else {
                const float ratio = static_cast<float>(
                    offsetCm / calibrationOffsetCm_);
                point += (plus5Point_ - centerPoint_) * ratio;
            }
            return {cvRound(point.x), cvRound(point.y)};
        }

        // 厘米位置的逆变换，只用于预览画紫色目标十字，不参与控制计算。
        const float pixels = static_cast<float>(
            centimeters / pipeLengthCm_ * pixelLength_);
        return {
            cvRound(left_.x + unit_.x * pixels),
            cvRound(left_.y + unit_.y * pixels)
        };
    }
};

class BallStateEstimator {
    struct Sample {
        double positionCm = 0.0;
        double timestamp = 0.0;
    };

    // Speed uses only real detections.  The derivative baseline is exactly two
    // measured frames, which is long enough to suppress one-pixel jitter while
    // still reacting in about 17 ms at 120 FPS.
    std::deque<Sample> samples_;
    int differenceFrames_ = 2;
    double rawTwoFrameSpeedCmS_ = 0.0;
    double filteredSpeedCmS_ = 0.0;
    double filterSeconds_ = 0.055;

public:
    BallStateEstimator(double filterSeconds, int differenceFrames)
        : differenceFrames_(differenceFrames),
          filterSeconds_(std::max(0.005, filterSeconds)) {}

    void reset()
    {
        // Each task start and every detection gap starts a new real-frame
        // history.  A stale position must never generate a fake D impulse.
        samples_.clear();
        rawTwoFrameSpeedCmS_ = 0.0;
        filteredSpeedCmS_ = 0.0;
    }

    void onMeasurementLost() { reset(); }

    void update(double positionCm, double timestamp)
    {
        if (!samples_.empty() &&
            (timestamp <= samples_.back().timestamp ||
             timestamp - samples_.back().timestamp > 0.12)) {
            reset();
        }

        samples_.push_back({positionCm, timestamp});
        if (samples_.size() <= static_cast<std::size_t>(differenceFrames_)) {
            rawTwoFrameSpeedCmS_ = 0.0;
            filteredSpeedCmS_ = 0.0;
            return;
        }

        const Sample& twoFramesAgo = samples_[
            samples_.size() - 1 - static_cast<std::size_t>(differenceFrames_)];
        const double dt = std::clamp(
            timestamp - twoFramesAgo.timestamp, 0.004, 0.10);
        rawTwoFrameSpeedCmS_ =
            (positionCm - twoFramesAgo.positionCm) / dt;

        // The low-pass input remains the two-frame difference above.  It only
        // removes vision noise; a larger measured speed still creates a larger
        // signed D term in the outer controller.
        const double alpha = dt / (filterSeconds_ + dt);
        filteredSpeedCmS_ +=
            alpha * (rawTwoFrameSpeedCmS_ - filteredSpeedCmS_);

        while (samples_.size() >
               static_cast<std::size_t>(differenceFrames_ + 1)) {
            samples_.pop_front();
        }
    }

    double rawTwoFrameSpeedCmS() const { return rawTwoFrameSpeedCmS_; }
    double speedCmS() const { return filteredSpeedCmS_; }
};

class MechanismModel {
    const AppConfig& config_;

    int interpolateCalibration(double angleDeg) const
    {
        const auto& table = config_.calibrationPoints;

        // 超过实测表两端时固定在端点脉冲，形成第二层软件角度限位。
        if (angleDeg <= table.front().pipeAngleDeg) {
            return table.front().motorSteps;
        }
        if (angleDeg >= table.back().pipeAngleDeg) {
            return table.back().motorSteps;
        }

        // 在包住目标角度的两个实测点之间做线性插值。
        for (std::size_t index = 1; index < table.size(); ++index) {
            const CalibrationPoint& low = table[index - 1];
            const CalibrationPoint& high = table[index];
            if (angleDeg <= high.pipeAngleDeg) {
                const double ratio =
                    (angleDeg - low.pipeAngleDeg) /
                    (high.pipeAngleDeg - low.pipeAngleDeg);
                return static_cast<int>(std::lround(
                    low.motorSteps +
                    ratio * (high.motorSteps - low.motorSteps)));
            }
        }
        return 0;
    }

public:
    explicit MechanismModel(const AppConfig& config) : config_(config) {}

    bool usesCalibrationTable() const
    {
        return config_.calibrationPoints.size() >= 2;
    }

    int angleToSteps(double pipeAngleDeg) const
    {
        // 有实测表时优先查表。motorSign只负责最终翻转电机正负方向。
        if (usesCalibrationTable()) {
            return config_.motorSign *
                   interpolateCalibration(pipeAngleDeg);
        }

        // 没有标定表时使用opencv8原来的曲柄连杆近似：
        // 水管右端高度 h=A*sin(theta)，曲柄销高度 h=r*sin(phi)，
        // 所以电机角phi=asin(A*sin(theta)/r)。
        // 该模型忽略连杆偏斜、安装偏心和间隙，只适合第一次检查方向与量级。
        const double pipeRadians = pipeAngleDeg * PI / 180.0;
        const double heightMm =
            config_.actuatorDistanceMm * std::sin(pipeRadians);

        // 避免asin输入越界，并禁止目标落在曲柄接近竖直死点的位置。
        const double ratio = std::clamp(
            heightMm / config_.crankRadiusMm, -0.98, 0.98);
        const double motorRadians = std::asin(ratio);
        const double revolutions = motorRadians / (2.0 * PI);
        const int steps = static_cast<int>(std::lround(
            revolutions * config_.pulsesPerRevolution));
        return config_.motorSign * steps;
    }
};

} // namespace ball_stepper
