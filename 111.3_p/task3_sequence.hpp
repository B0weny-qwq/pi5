#pragma once

// task3_sequence.hpp
// ============================================================================
// 本文件只负责第3题的自动流程：
//
//   等待启动 -> O+5 cm -> O-5 cm -> 在O-5 cm稳定保持
//
// 它不识别钢球、不计算水管角度、不发送电机命令。
// update()每次接收视觉测得的实际位置和速度，然后只输出当前目标位置。
// ============================================================================

#include "system_config.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>

namespace ball_stepper {

enum class Task3Phase {
    Ready,
    MoveToPositive,
    MoveToNegative,
    HoldNegative
};

class Task3Sequence {
    const AppConfig& config_;
    Task3Phase phase_ = Task3Phase::Ready;

    bool started_ = false;
    bool timedOut_ = false;
    double targetCm_ = 0.0;
    double startTime_ = 0.0;
    double finishTime_ = -1.0;
    double finalStableStartTime_ = -1.0;
    int positiveArrivalFrames_ = 0;

    double centerCm() const
    {
        return config_.pipeLengthCm * 0.5;
    }

    void resetToReady()
    {
        phase_ = Task3Phase::Ready;
        targetCm_ = centerCm();
        timedOut_ = false;
        finishTime_ = -1.0;
        finalStableStartTime_ = -1.0;
        positiveArrivalFrames_ = 0;
    }

public:
    explicit Task3Sequence(const AppConfig& config)
        : config_(config), targetCm_(config.pipeLengthCm * 0.5) {}

    bool started() const { return started_; }
    bool timedOut() const { return timedOut_; }
    bool finished() const { return phase_ == Task3Phase::HoldNegative; }
    Task3Phase phase() const { return phase_; }
    bool movingPositive() const
    {
        return phase_ == Task3Phase::MoveToPositive;
    }
    bool movingNegative() const
    {
        return phase_ == Task3Phase::MoveToNegative;
    }
    bool traveling() const
    {
        return phase_ == Task3Phase::MoveToPositive ||
               phase_ == Task3Phase::MoveToNegative;
    }
    // 折返到O-5和到达后的保持阶段都启用返回专用的驱动/制动参数。
    bool negativeTargetActive() const
    {
        return phase_ == Task3Phase::MoveToNegative ||
               phase_ == Task3Phase::HoldNegative;
    }
    double targetCm() const { return targetCm_; }

    void start(double now)
    {
        started_ = true;
        phase_ = Task3Phase::MoveToPositive;
        targetCm_ = centerCm() + config_.task3OffsetCm;
        startTime_ = now;
        finishTime_ = -1.0;
        finalStableStartTime_ = -1.0;
        positiveArrivalFrames_ = 0;
        timedOut_ = false;

        std::fprintf(stderr,
            "TASK 3 START: O -> O+%.2f cm -> O-%.2f cm\n",
            config_.task3OffsetCm, config_.task3OffsetCm);
    }

    void abortAndReset()
    {
        started_ = false;
        resetToReady();
    }

    void onMeasurementLost()
    {
        // “连续8帧到达+5”和“在-5稳定200 ms”都不允许跨越漏检帧累计。
        // 否则间歇误识可以把不连续的几帧拼成“到达”，导致过早折返。
        positiveArrivalFrames_ = 0;
        finalStableStartTime_ = -1.0;
    }

    void update(double positionCm, double speedCmS, double now)
    {
        // PAUSED、已完成或尚未启动时，绝不推进第3题状态。
        if (!started_ || phase_ == Task3Phase::Ready ||
            phase_ == Task3Phase::HoldNegative) {
            return;
        }

        const double elapsedMs = (now - startTime_) * 1000.0;
        if (!timedOut_ && elapsedMs > config_.task3TimeLimitMs) {
            // 超时只记录本轮不合格，闭环仍继续将球送到O-5 cm。
            timedOut_ = true;
            std::fprintf(stderr,
                "TASK 3 TIMEOUT: exceeded %.3f s, control continues\n",
                config_.task3TimeLimitMs / 1000.0);
        }

        if (phase_ == Task3Phase::MoveToPositive) {
            // +5 cm只是折返点，不要求钢球在那里停稳。只要真实圆心到达
            // “+5减允许误差”并连续确认，就立刻切换到-5；使用单向越线
            // 判定还能避免钢球较快时一帧跨过整个对称容差区后永不折返。
            if (positionCm >=
                targetCm_ - config_.task3PositiveToleranceCm) {
                ++positiveArrivalFrames_;
            } else {
                positiveArrivalFrames_ = 0;
            }

            if (positiveArrivalFrames_ >=
                config_.task3ArrivalConfirmFrames) {
                // 真实测量连续进入O+5 cm区间后，才自动切换到O-5 cm。
                phase_ = Task3Phase::MoveToNegative;
                targetCm_ = centerCm() - config_.task3OffsetCm -
                    config_.task3NegativeTargetBiasCm;
                positiveArrivalFrames_ = 0;
                std::fprintf(stderr,
                    "TASK 3 TURN at %.3f s: +%.2f cm reached, "
                    "now go to -%.2f cm\n",
                    now - startTime_,
                    config_.task3OffsetCm, config_.task3OffsetCm);
            }
            return;
        }

        if (phase_ == Task3Phase::MoveToNegative) {
            const double finalErrorCm = positionCm - targetCm_;
            const bool positionStable =
                finalErrorCm >= -config_.task3FinalToleranceCm &&
                finalErrorCm <= config_.task3FinalRightToleranceCm;
            const bool speedStable =
                std::abs(speedCmS) <= config_.task3FinalSpeedCmS;

            if (positionStable && speedStable) {
                if (finalStableStartTime_ < 0.0) {
                    finalStableStartTime_ = now;
                }

                const double stableMs =
                    (now - finalStableStartTime_) * 1000.0;
                if (stableMs >= config_.task3FinalStableMs) {
                    phase_ = Task3Phase::HoldNegative;
                    finishTime_ = now;
                    std::fprintf(stderr,
                        "TASK 3 FINISHED in %.3f s%s\n",
                        finishTime_ - startTime_,
                        timedOut_ ? " (TIMEOUT)" : " (PASS TIME)");
                }
            } else {
                // 稳定时间必须连续，任何一帧超差都重新计时。
                finalStableStartTime_ = -1.0;
            }
        }
    }

    const char* phaseText() const
    {
        switch (phase_) {
        case Task3Phase::Ready:          return "READY";
        case Task3Phase::MoveToPositive: return "GO O+5";
        case Task3Phase::MoveToNegative: return "GO O-5";
        case Task3Phase::HoldNegative:   return "DONE/HOLD O-5";
        }
        return "UNKNOWN";
    }

    void formatStatus(char* output, std::size_t outputSize,
                      double now) const
    {
        const double elapsed = started_ ?
            ((finishTime_ >= 0.0 ? finishTime_ : now) - startTime_) : 0.0;
        std::snprintf(output, outputSize,
            "TASK3 %s time=%.2fs %s",
            phaseText(), elapsed, timedOut_ ? "TIMEOUT" : "");
    }
};

} // namespace ball_stepper
