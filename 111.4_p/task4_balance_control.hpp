#pragma once

// 第4问钢球外环：小车行驶时持续把钢球保持在中心O。
//
//   pipeAngle = Kp * positionError
//             + Kd * twoFrameBallSpeed
//             + Ki * integralError
//             + accelerationFeedforward
//
// 正角度抬高水管右端，使钢球向左加速。errorCm和speedCmS向右为正，
// 因此P、D、I三项极性相同；D直接阻挡钢球当前运动方向。

#include "system_config.hpp"

#include <algorithm>
#include <cmath>

namespace ball_stepper {

struct Task4ControlOutput {
    double angleDeg = 0.0;
    double rawAngleDeg = 0.0;
    double driveAngleDeg = 0.0;
    double pTermDeg = 0.0;
    double dTermDeg = 0.0;
    double iTermDeg = 0.0;
    double feedforwardTermDeg = 0.0;
    bool integralWindowActive = false;
    bool deadband = false;
    bool driveSaturated = false;
    bool saturated = false;
};

class Task4BalanceController {
    const AppConfig& config_;
    double integralAngleDeg_ = 0.0;
    double previousTime_ = -1.0;

    double driveSaturationDistance(double angleDeg) const
    {
        return std::abs(angleDeg - std::clamp(
            angleDeg,
            -config_.task4DriveAngleLimitDeg,
             config_.task4DriveAngleLimitDeg));
    }

public:
    explicit Task4BalanceController(const AppConfig& config)
        : config_(config) {}

    void reset(double now = -1.0)
    {
        integralAngleDeg_ = 0.0;
        previousTime_ = now;
    }

    void onMeasurementLost()
    {
        // 短漏帧期间保持真实测量历史；重捕后的第一帧只恢复时间基准。
        previousTime_ = -1.0;
    }

    Task4ControlOutput update(double errorCm,
                              double twoFrameSpeedCmS,
                              double feedforwardAngleDeg,
                              double now)
    {
        Task4ControlOutput output;
        double dt = 0.0;
        if (previousTime_ >= 0.0) {
            dt = std::clamp(now - previousTime_, 0.002, 0.05);
        }
        previousTime_ = now;

        output.pTermDeg = config_.task4Kp * errorCm;
        output.dTermDeg = config_.task4Kd * twoFrameSpeedCmS;
        output.feedforwardTermDeg = std::clamp(
            feedforwardAngleDeg,
            -config_.task4VehicleFeedforwardLimitDeg,
             config_.task4VehicleFeedforwardLimitDeg);
        output.deadband =
            std::abs(errorCm) <= config_.task4DeadbandCm &&
            std::abs(twoFrameSpeedCmS) <= config_.task4StopSpeedCmS;

        const double pForOutput = output.deadband ? 0.0 : output.pTermDeg;
        const double dForOutput = output.deadband ? 0.0 : output.dTermDeg;

        output.integralWindowActive =
            std::abs(errorCm) <= config_.task4IntegralZoneCm &&
            std::abs(twoFrameSpeedCmS) <=
                config_.task4IntegralSpeedLimitCmS;

        if (dt > 0.0) {
            // 换边后旧积分极性一定错误，立即清零，避免把球继续推出中心。
            if (integralAngleDeg_ * errorCm < 0.0) {
                integralAngleDeg_ = 0.0;
            }

            const double leakedIntegral = integralAngleDeg_ * std::exp(
                -dt / config_.task4IntegralLeakSeconds);
            double candidateIntegral = leakedIntegral;
            if (output.integralWindowActive) {
                candidateIntegral += config_.task4Ki * errorCm * dt;
            }
            candidateIntegral = std::clamp(
                candidateIntegral,
                -config_.task4IntegralLimitDeg,
                 config_.task4IntegralLimitDeg);

            // I只在不会让P+I推进项进一步顶住限幅时更新。
            const double fixedDriveAngleDeg =
                config_.task4LevelTrimDeg +
                output.feedforwardTermDeg + pForOutput;
            if (driveSaturationDistance(
                    fixedDriveAngleDeg + candidateIntegral) <=
                driveSaturationDistance(
                    fixedDriveAngleDeg + integralAngleDeg_)) {
                integralAngleDeg_ = candidateIntegral;
            }
        }

        output.iTermDeg = integralAngleDeg_;
        const double rawDriveAngleDeg =
            config_.task4LevelTrimDeg + output.feedforwardTermDeg +
            pForOutput + output.iTermDeg;
        output.driveAngleDeg = std::clamp(
            rawDriveAngleDeg,
            -config_.task4DriveAngleLimitDeg,
             config_.task4DriveAngleLimitDeg);
        output.driveSaturated =
            std::abs(rawDriveAngleDeg - output.driveAngleDeg) > 1e-9;

        // D在P/I推进限幅之后叠加，允许使用更大的制动包络，但仍受总机械限幅。
        output.rawAngleDeg = output.driveAngleDeg + dForOutput;
        output.angleDeg = std::clamp(
            output.rawAngleDeg,
            -config_.task4BrakeAngleLimitDeg,
             config_.task4BrakeAngleLimitDeg);
        output.saturated =
            std::abs(output.rawAngleDeg - output.angleDeg) > 1e-9;
        return output;
    }
};

} // namespace ball_stepper
