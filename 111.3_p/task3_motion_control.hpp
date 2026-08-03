#pragma once

// Task 3 ball-position controller.
//
// The task state machine only selects +5 cm, -5 cm, and completion.  It does
// not select a drive angle.  Every measured frame uses the same outer loop:
//
//   pipeAngle = Kp * positionError + Kd * ballVelocity + Ki * integralError
//               + bounded feedback-triggered breakaway compensation
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
    double breakawayAngleDeg = 0.0;
    bool integralWindowActive = false;
    bool breakawayActive = false;
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
    double breakawayDwellSeconds_ = 0.0;
    double breakawayAngleDeg_ = 0.0;
    int breakawayDirection_ = 0;

    void resetIntegral()
    {
        integralErrorCmSeconds_ = 0.0;
    }

    void resetBreakaway()
    {
        breakawayDwellSeconds_ = 0.0;
        breakawayAngleDeg_ = 0.0;
        breakawayDirection_ = 0;
    }

public:
    explicit Task3MotionController(const AppConfig& config)
        : config_(config) {}

    void reset()
    {
        previousPhase_ = Task3Phase::Ready;
        resetIntegral();
        resetBreakaway();
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
            resetBreakaway();
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

        const bool targetActive = phase != Task3Phase::Ready;
        const bool insideIntegralPositionWindow =
            targetActive &&
            std::abs(errorCm) <= config_.task3IntegralZoneCm;
        const bool slowEnoughToIntegrate =
            std::abs(speedCmS) <= config_.task3IntegralSpeedLimitCmS;

        // I is deliberately limited to the final approach around either target.
        // A target change clears it, so it cannot carry bias across the reversal.
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

        const double baseAngleDeg = std::clamp(
            command.proportionalAngleDeg + command.derivativeAngleDeg +
                command.integralAngleDeg,
            -config_.task3OutputAngleLimitDeg,
             config_.task3OutputAngleLimitDeg);

        const int desiredDirection =
            command.proportionalAngleDeg > 1e-9 ? 1 :
            command.proportionalAngleDeg < -1e-9 ? -1 : 0;
        if (desiredDirection != 0 &&
            desiredDirection != breakawayDirection_) {
            breakawayDwellSeconds_ = 0.0;
            breakawayAngleDeg_ = 0.0;
            breakawayDirection_ = desiredDirection;
        }

        const bool movingPhase =
            phase == Task3Phase::MoveToPositive ||
            phase == Task3Phase::MoveToNegative;
        const bool largeUnresolvedError =
            std::abs(errorCm) >= config_.task3BreakawayErrorCm;
        const bool ballNearlyStopped =
            std::abs(speedCmS) <= config_.task3BreakawaySpeedCmS;
        const bool baseStillDrivesTowardTarget =
            desiredDirection != 0 &&
            baseAngleDeg * static_cast<double>(desiredDirection) > 0.0;
        const bool shouldBuildBreakaway =
            movingPhase && largeUnresolvedError && ballNearlyStopped &&
            baseStillDrivesTowardTarget;

        if (shouldBuildBreakaway) {
            breakawayDwellSeconds_ += dt;
            if (breakawayDwellSeconds_ >=
                config_.task3BreakawayDelaySeconds) {
                const double targetBreakaway =
                    static_cast<double>(desiredDirection) *
                    config_.task3BreakawayMaximumAngleDeg;
                breakawayAngleDeg_ = approach(
                    breakawayAngleDeg_, targetBreakaway,
                    config_.task3BreakawayRampDegPerSecond * dt);
            }
        } else {
            breakawayDwellSeconds_ = 0.0;
            breakawayAngleDeg_ = approach(
                breakawayAngleDeg_, 0.0,
                3.0 * config_.task3BreakawayRampDegPerSecond * dt);
        }

        command.breakawayAngleDeg = breakawayAngleDeg_;
        command.breakawayActive =
            shouldBuildBreakaway && std::abs(breakawayAngleDeg_) > 1e-9;
        const double breakawayOutputLimit =
            config_.task3OutputAngleLimitDeg +
            config_.task3BreakawayMaximumAngleDeg;
        command.angleDeg = std::clamp(
            baseAngleDeg + command.breakawayAngleDeg,
            -breakawayOutputLimit, breakawayOutputLimit);

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
