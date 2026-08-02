#pragma once

// Task 3 ball-position controller.
//
// The task state machine only selects +5 cm, -5 cm, and completion.  It does
// not select a drive angle.  Every measured frame uses the same outer loop:
//
//   pipeAngle = Kp * positionError + Kd * ballVelocity + Ki * integralError
//
// Positive pipe angle raises axisRight, so a positive position error or a ball
// moving to the right both ask the ball to move back to the left.

#include "system_config.hpp"
#include "task3_sequence.hpp"

#include <algorithm>
#include <cmath>

namespace ball_stepper {

enum class Task3MotionMode {
    Level,
    TrackingPd,
    FinalPdi,
    HoldPdi
};

struct Task3MotionCommand {
    double angleDeg = 0.0;
    double proportionalAngleDeg = 0.0;
    double derivativeAngleDeg = 0.0;
    double integralAngleDeg = 0.0;
    bool integralWindowActive = false;
    Task3MotionMode mode = Task3MotionMode::Level;
};

inline const char* task3MotionModeText(Task3MotionMode mode)
{
    switch (mode) {
    case Task3MotionMode::Level:      return "LEVEL";
    case Task3MotionMode::TrackingPd: return "TRACK PD";
    case Task3MotionMode::FinalPdi:   return "FINAL PDI";
    case Task3MotionMode::HoldPdi:    return "HOLD PDI";
    }
    return "UNKNOWN";
}

class Task3MotionController {
    const AppConfig& config_;
    Task3Phase previousPhase_ = Task3Phase::Ready;
    double integralErrorCmSeconds_ = 0.0;

    void resetIntegral()
    {
        integralErrorCmSeconds_ = 0.0;
    }

public:
    explicit Task3MotionController(const AppConfig& config)
        : config_(config) {}

    void reset()
    {
        previousPhase_ = Task3Phase::Ready;
        resetIntegral();
    }

    Task3MotionCommand update(Task3Phase phase,
                              double errorCm,
                              double speedCmS,
                              double dtSeconds)
    {
        Task3MotionCommand command;
        if (phase != previousPhase_) {
            // The target changes at +5 cm.  Never carry an accumulated final
            // hold correction into the next target.
            resetIntegral();
            previousPhase_ = phase;
        }

        if (phase == Task3Phase::Ready) {
            return command;
        }

        const double dt = std::clamp(dtSeconds, 0.002, 0.05);
        command.proportionalAngleDeg =
            config_.task3PositionKpDegPerCm * errorCm;
        command.derivativeAngleDeg =
            config_.task3VelocityKdDegPerCmS * speedCmS;

        const bool negativeTargetActive =
            phase == Task3Phase::MoveToNegative ||
            phase == Task3Phase::HoldNegative;
        const bool insideIntegralPositionWindow =
            negativeTargetActive &&
            std::abs(errorCm) <= config_.task3IntegralZoneCm;
        const bool slowEnoughToIntegrate =
            std::abs(speedCmS) <= config_.task3IntegralSpeedLimitCmS;

        // I is deliberately limited to the final -5 cm window.  It removes
        // residual mechanical bias without becoming a long-distance drive term.
        if (!insideIntegralPositionWindow) {
            resetIntegral();
        } else if (slowEnoughToIntegrate) {
            const double candidateIntegral = std::clamp(
                integralErrorCmSeconds_ + errorCm * dt,
                -config_.task3IntegralLimitCmSeconds,
                 config_.task3IntegralLimitCmSeconds);
            const double currentSum = command.proportionalAngleDeg +
                command.derivativeAngleDeg +
                config_.task3IntegralKiDegPerCmSecond *
                    integralErrorCmSeconds_;
            const double candidateSum = command.proportionalAngleDeg +
                command.derivativeAngleDeg +
                config_.task3IntegralKiDegPerCmSecond * candidateIntegral;

            // Conditional integration prevents the small I term from winding
            // farther into the common output limiter.
            if (std::abs(candidateSum) <= config_.task3OutputAngleLimitDeg ||
                std::abs(candidateSum) < std::abs(currentSum)) {
                integralErrorCmSeconds_ = candidateIntegral;
            }
        }

        command.integralWindowActive = insideIntegralPositionWindow;
        command.integralAngleDeg =
            config_.task3IntegralKiDegPerCmSecond *
            integralErrorCmSeconds_;
        command.angleDeg = std::clamp(
            command.proportionalAngleDeg + command.derivativeAngleDeg +
                command.integralAngleDeg,
            -config_.task3OutputAngleLimitDeg,
             config_.task3OutputAngleLimitDeg);

        if (phase == Task3Phase::HoldNegative) {
            command.mode = Task3MotionMode::HoldPdi;
        } else if (insideIntegralPositionWindow) {
            command.mode = Task3MotionMode::FinalPdi;
        } else {
            command.mode = Task3MotionMode::TrackingPd;
        }
        return command;
    }
};

} // namespace ball_stepper
