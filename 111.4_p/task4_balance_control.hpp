#pragma once

// 第4问钢球外环：小车行驶时持续把钢球保持在中心O。
// 正角度抬高水管右端，使钢球产生向左加速度；
// errorCm = 当前位置 - O，speedCmS为沿水管向右的钢球速度。

#include "system_config.hpp"

#include <algorithm>
#include <cmath>

namespace ball_stepper {

struct Task4ControlOutput {
    double angleDeg = 0.0;
    double rawAngleDeg = 0.0;
    double pTermDeg = 0.0;
    double dTermDeg = 0.0;
    double iTermDeg = 0.0;
    bool fineMode = false;
    bool deadband = false;
    bool saturated = false;
};

class Task4BalanceController {
    const AppConfig& config_;
    double integralAngleDeg_ = 0.0;
    double previousTime_ = -1.0;

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
        // 丢球时冻结积分，重新识别后的第一帧只恢复时间基准。
        previousTime_ = -1.0;
    }

    Task4ControlOutput update(double errorCm,
                              double speedCmS,
                              double now)
    {
        Task4ControlOutput output;
        double dt = 0.0;
        if (previousTime_ >= 0.0) {
            dt = std::clamp(now - previousTime_, 0.002, 0.05);
        }
        previousTime_ = now;

        output.fineMode =
            std::abs(errorCm) <= config_.task4FineZoneCm &&
            std::abs(speedCmS) <= config_.task4FineSpeedCmS;
        const double kp = output.fineMode ?
            config_.task4FineKp : config_.task4Kp;
        const double kd = output.fineMode ?
            config_.task4FineKd : config_.task4Kd;

        output.pTermDeg = kp * errorCm;
        output.dTermDeg = kd * speedCmS;
        output.deadband =
            std::abs(errorCm) <= config_.task4DeadbandCm &&
            std::abs(speedCmS) <= config_.task4StopSpeedCmS;

        double pForOutput = output.deadband ? 0.0 : output.pTermDeg;
        double dForOutput = output.deadband ? 0.0 : output.dTermDeg;

        if (dt > 0.0) {
            // 泄漏积分用于估计小车持续加速、坡面安装偏差和机构静差。
            integralAngleDeg_ *= std::exp(
                -dt / config_.task4IntegralLeakSeconds);

            if (std::abs(errorCm) <=
                config_.task4IntegralEnableErrorCm) {
                integralAngleDeg_ += config_.task4Ki * errorCm * dt;
            }
            integralAngleDeg_ = std::clamp(
                integralAngleDeg_,
                -config_.task4IntegralLimitDeg,
                 config_.task4IntegralLimitDeg);
        }

        output.rawAngleDeg =
            config_.task4LevelTrimDeg +
            pForOutput + dForOutput + integralAngleDeg_;
        output.angleDeg = std::clamp(
            output.rawAngleDeg,
            -config_.task4MaximumAngleDeg,
             config_.task4MaximumAngleDeg);
        output.saturated =
            std::abs(output.rawAngleDeg - output.angleDeg) > 1e-9;

        if (output.saturated && dt > 0.0) {
            // 饱和回算直接移除无法执行的积分，换向时不会释放旧积分猛冲。
            integralAngleDeg_ = std::clamp(
                output.angleDeg - config_.task4LevelTrimDeg -
                    pForOutput - dForOutput,
                -config_.task4IntegralLimitDeg,
                 config_.task4IntegralLimitDeg);
        }

        output.iTermDeg = integralAngleDeg_;
        return output;
    }
};

} // namespace ball_stepper
