#pragma once

// 第3题的运动控制器。
//
// 运动方向约定：
//   正角度：抬高右端，钢球向左运动；
//   负角度：降低右端，钢球向右运动；
//   errorCm = 当前位置 - 目标位置。
//
// 这里特意把 O+5 -> O-5 做成有阶段的控制：
//   1. O -> O+5：固定角度快速到达经过点；
//   2. O+5 -> O-5：在确认真正越过 O-5 以前，只允许向 O-5 驱动；
//   3. 越过 O-5 后：才允许按照速度做小角度制动和位置保持。
//
// 这样视觉坐标在 -5 cm 附近偶尔抖动、速度估计突然变大时，也不会
// 在目标前提前输出反向角，把钢球推回中心甚至推向 O+5。

#include "system_config.hpp"
#include "task3_sequence.hpp"

#include <algorithm>
#include <cmath>

namespace ball_stepper {

enum class Task3MotionMode {
    Level,
    PositiveDrive,
    ReturnDrive,
    ReturnBrake,
    Settle,
    Hold
};

struct Task3MotionCommand {
    double angleDeg = 0.0;
    double stoppingDistanceCm = 0.0;
    Task3MotionMode mode = Task3MotionMode::Level;
};

inline const char* task3MotionModeText(Task3MotionMode mode)
{
    switch (mode) {
    case Task3MotionMode::Level:         return "LEVEL";
    case Task3MotionMode::PositiveDrive: return "TO +5 DRIVE";
    case Task3MotionMode::ReturnDrive:   return "TO -5 DRIVE";
    case Task3MotionMode::ReturnBrake:   return "TO -5 BRAKE";
    case Task3MotionMode::Settle:        return "TO -5 SETTLE";
    case Task3MotionMode::Hold:          return "HOLD -5";
    }
    return "UNKNOWN";
}

class Task3MotionController {
    const AppConfig& config_;

    // 仅用很小的越线量锁存“已到目标”；主要减速现在由目标前的速度制动完成。
    static constexpr double kTargetCrossingMarginCm = 0.05;
    static constexpr double kApproachStallSpeedCmS = 0.25;

    Task3Phase previousPhase_ = Task3Phase::Ready;
    bool returnBraking_ = false;

    // 目标穿越锁存：必须连续两帧确认球已越过 -5 cm，才允许制动。
    bool returnTargetReached_ = false;
    int targetCrossingFrames_ = 0;

    // -5 附近的有界小角度控制。Kd 只在已经越过目标后使用，避免目标前
    // 的速度噪声提前抵消返程驱动。
    Task3MotionCommand smallAngleControl(
        double errorCm, double speedCmS, Task3MotionMode mode) const
    {
        Task3MotionCommand command;
        command.mode = mode;

        // 误差为正表示球在目标右侧，需要正角度把球推向左；
        // 速度项用于抵消惯性，避免越过目标后反复摆动。
        double angle =
            config_.kp * errorCm + config_.task3ReturnKd * speedCmS;

        // 目标右侧的停稳区必须与更严格的完成容差一致，避免完成后又停回
        // -5右边；左侧仍沿用普通死区，吸收返程后的轻微过冲。
        const double deadbandCm = errorCm > 0.0 ?
            std::min(config_.centerDeadbandCm,
                     config_.task3FinalRightToleranceCm) :
            config_.centerDeadbandCm;
        const double creepErrorCm = errorCm > 0.0 ?
            std::min(config_.task3CreepErrorCm,
                     config_.task3FinalRightToleranceCm) :
            config_.task3CreepErrorCm;

        // 目标附近且球已经基本停止时直接回到水平，避免电机在终点抖动。
        if (std::abs(errorCm) <= deadbandCm &&
            std::abs(speedCmS) <= config_.stopSpeedCmS) {
            angle = 0.0;
        } else if (std::abs(errorCm) > creepErrorCm &&
                   std::abs(speedCmS) <= config_.task3CreepSpeedCmS &&
                   std::abs(angle) < config_.task3CreepAngleDeg) {
            // 机构有静摩擦或间隙时，给一个很小的爬行角。
            angle = std::copysign(config_.task3CreepAngleDeg, errorCm);
        }

        command.angleDeg = std::clamp(
            angle,
            -config_.task3SettleAngleLimitDeg,
             config_.task3SettleAngleLimitDeg);
        return command;
    }

public:
    explicit Task3MotionController(const AppConfig& config)
        : config_(config) {}

    void reset()
    {
        previousPhase_ = Task3Phase::Ready;
        returnBraking_ = false;
        returnTargetReached_ = false;
        targetCrossingFrames_ = 0;
    }

    Task3MotionCommand update(Task3Phase phase,
                              double errorCm,
                              double speedCmS)
    {
        if (phase != previousPhase_) {
            // 每次进入新阶段都清除上一阶段的刹车和目标锁存状态。
            returnBraking_ = false;
            returnTargetReached_ = false;
            targetCrossingFrames_ = 0;
            previousPhase_ = phase;
        }

        if (phase == Task3Phase::Ready) {
            return {};
        }

        if (phase == Task3Phase::MoveToPositive) {
            Task3MotionCommand command;
            command.mode = Task3MotionMode::PositiveDrive;

            // O+5 是经过点，不在这里刹停。固定角度可以避免速度项把
            // 行程角抵消，导致钢球只向右走一小段就折返。
            command.angleDeg = -config_.task3MinimumTravelAngleDeg;
            return command;
        }

        if (phase == Task3Phase::HoldNegative) {
            // 完成后只保留 -5 附近的小幅保持，不再使用返程大角度。
            returnBraking_ = false;
            return smallAngleControl(
                errorCm, speedCmS, Task3MotionMode::Hold);
        }

        // ---------------- O+5 -> O-5 ----------------
        const double distanceCm = std::abs(errorCm);
        const double speedMagnitude = std::abs(speedCmS);
        const double towardTargetSpeedCmS =
            std::max(0.0, -speedCmS);
        const bool movingTowardTarget = errorCm * speedCmS < -0.02;
        const double preTargetStoppingDistanceCm = movingTowardTarget ?
            speedMagnitude * speedMagnitude /
                (2.0 * config_.task3BrakingAccelerationCmS2) :
            0.0;

        // 目标穿越锁存只保留0.05 cm小余量；仍要求连续两帧过滤视觉跳点。
        if (!returnTargetReached_) {
            if (errorCm <= -kTargetCrossingMarginCm) {
                ++targetCrossingFrames_;
                if (targetCrossingFrames_ >= 2) {
                    returnTargetReached_ = true;
                }
            } else {
                targetCrossingFrames_ = 0;
            }

            if (!returnTargetReached_) {
                if (errorCm > 0.0 &&
                    errorCm <= config_.task3ReturnApproachZoneCm) {
                    // 已接近O-5时使用位置+速度控制提前减速。速度较高时Kd可
                    // 输出小幅反向角；若真的停在目标右侧，则恢复返程最大角
                    // 克服静摩擦，避免再次停在中点或刻度右侧。
                    Task3MotionCommand command;
                    command.stoppingDistanceCm =
                        preTargetStoppingDistanceCm;
                    double approachAngle =
                        config_.kp * errorCm +
                        config_.task3ReturnKd * speedCmS;
                    if (towardTargetSpeedCmS <= kApproachStallSpeedCmS &&
                        errorCm > config_.task3FinalRightToleranceCm) {
                        approachAngle = config_.task3ReturnAngleLimitDeg;
                    }
                    command.angleDeg = std::clamp(
                        approachAngle,
                        -config_.task3ReturnBrakeAngleLimitDeg,
                         config_.task3ReturnAngleLimitDeg);
                    command.mode = command.angleDeg < 0.0 ?
                        Task3MotionMode::ReturnBrake :
                        Task3MotionMode::ReturnDrive;
                    return command;
                }

                // 远离O-5时闭合钢球速度环。目标速度为向左的负值；
                // 球向左过快时输出负角制动，速度不足时输出正角驱动。
                // 这样从+5折返后会先消除向右惯性，也不会在10 cm返程中
                // 持续加速到O-5后才开始制动。
                returnBraking_ = false;
                Task3MotionCommand command;
                command.stoppingDistanceCm =
                    preTargetStoppingDistanceCm;
                const double velocityErrorCmS =
                    speedCmS + config_.task3ReturnCruiseSpeedCmS;
                command.angleDeg = std::clamp(
                    config_.task3ReturnSpeedKpDegPerCmS *
                        velocityErrorCmS,
                    -config_.task3ReturnBrakeAngleLimitDeg,
                     config_.task3ReturnAngleLimitDeg);
                command.mode = command.angleDeg < 0.0 ?
                    Task3MotionMode::ReturnBrake :
                    Task3MotionMode::ReturnDrive;
                return command;
            }
        }

        // 锁存后才计算制动。此时即使位置误差瞬间抖回正侧，也不会把
        // “尚未到达”逻辑重新打开。
        const double stoppingDistanceCm =
            speedMagnitude * speedMagnitude /
                (2.0 * config_.task3BrakingAccelerationCmS2);
        const bool insideBrakeWindow =
            distanceCm <= config_.task3MaximumBrakeStartDistanceCm;
        const bool brakeNeeded =
            insideBrakeWindow &&
            speedMagnitude > config_.task3CreepSpeedCmS &&
            stoppingDistanceCm + config_.task3BrakingMarginCm >=
                distanceCm;

        if (!returnBraking_ && brakeNeeded) {
            returnBraking_ = true;
        }

        // 速度降低后解除制动，进入小角度位置控制。
        if (returnBraking_ &&
            speedMagnitude <= config_.task3CreepSpeedCmS) {
            returnBraking_ = false;
        }

        if (returnBraking_) {
            Task3MotionCommand command;
            command.mode = Task3MotionMode::ReturnBrake;
            command.stoppingDistanceCm = stoppingDistanceCm;

            // 制动角与当前速度同号：
            //   speed>0（向右） -> 正角度产生向左减速度；
            //   speed<0（向左） -> 负角度产生向右减速度。
            // 上限由 main.cpp 的 task3ReturnBrakeAngleLimitDeg 控制为0.14°。
            command.angleDeg = std::copysign(
                config_.task3ReturnBrakeAngleLimitDeg, speedCmS);
            return command;
        }

        // 已越过 -5 且速度不大时，只用有界的小角度控制修正位置。
        Task3MotionCommand command = smallAngleControl(
            errorCm, speedCmS, Task3MotionMode::Settle);
        command.stoppingDistanceCm = stoppingDistanceCm;
        return command;
    }
};

} // namespace ball_stepper
