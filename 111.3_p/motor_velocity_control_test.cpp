#include "zdt_stepper_uart.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace ball_stepper;

namespace {

void testVelocityFrame()
{
    const auto frame = makeZdtVelocityFrame(0x01, -1500.0, 3000, 0x0A);
    const std::array<uint8_t, 8> expected = {
        0x01, 0xF6, 0x01, 0x05, 0xDC, 0x0A, 0x00, 0x6B
    };
    assert(frame == expected);

    const auto limited = makeZdtVelocityFrame(0x02, 4000.0, 25, 12);
    assert(limited[0] == 0x02);
    assert(limited[2] == 0x00);
    assert(limited[3] == 0x00 && limited[4] == 0x19);
    assert(limited[5] == 12 && limited[7] == 0x6B);
}

void simulateToTarget(VelocityModePositionController& controller,
                      const AppConfig& config,
                      int targetSteps,
                      double seconds,
                      double& positionSteps,
                      double& speedRpm)
{
    constexpr double dt = 0.02;
    double previousAcceleration = 0.0;
    const int iterations = static_cast<int>(seconds / dt);
    for (int index = 0; index < iterations; ++index) {
        const MotorLoopTelemetry state = controller.update(
            targetSteps, positionSteps, speedRpm, dt);

        assert(std::abs(state.commandSpeedRpm) <= config.motorRpm + 1e-9);
        assert(std::abs(state.commandAccelerationRpmS) <=
               config.motorMaximumAccelerationRpmS + 1e-9);
        assert(std::abs(state.commandAccelerationRpmS -
                        previousAcceleration) <=
               config.motorMaximumJerkRpmS3 * dt + 1e-9);
        previousAcceleration = state.commandAccelerationRpmS;

        // 简化的ZDT速度闭环：实测速度按硬件加速度档平滑追踪命令速度。
        speedRpm = approach(
            speedRpm,
            state.commandSpeedRpm,
            82.0 * dt);
        positionSteps += speedRpm / 60.0 *
            config.pulsesPerRevolution * dt;
    }
}

void testCascadedController()
{
    AppConfig config;
    config.pulsesPerRevolution = 6400;
    config.motorRpm = 6;
    config.motorPositionKpRpmPerStep = 0.0225;
    config.motorVelocityKpPerSecond = 8.0;
    config.motorMaximumAccelerationRpmS = 20.0;
    config.motorMaximumJerkRpmS3 = 300.0;
    config.motorBrakingAccelerationRpmS = 8.0;
    config.motorPositionToleranceSteps = 3.0;
    config.motorStopSpeedRpm = 1.0;
    config.motorSoftLimitSteps = 260;

    VelocityModePositionController controller(config);
    double positionSteps = 0.0;
    double speedRpm = 0.0;

    simulateToTarget(
        controller, config, 194, 2.0, positionSteps, speedRpm);
    std::fprintf(stderr, "forward position=%.3f speed=%.3f\n",
                 positionSteps, speedRpm);
    assert(std::abs(positionSteps - 194.0) < 5.0);
    assert(std::abs(speedRpm) < 2.0);

    simulateToTarget(
        controller, config, -120, 2.5, positionSteps, speedRpm);
    std::fprintf(stderr, "reverse position=%.3f speed=%.3f\n",
                 positionSteps, speedRpm);
    assert(std::abs(positionSteps + 120.0) < 5.0);
    assert(std::abs(speedRpm) < 2.0);

    const MotorLoopTelemetry limited = controller.update(
        500, positionSteps, speedRpm, 0.02);
    assert(limited.targetSteps == config.motorSoftLimitSteps);
}

void testSmallPipeAngleTarget()
{
    AppConfig config;
    config.pulsesPerRevolution = 6400;
    config.motorRpm = 6;
    config.motorPositionKpRpmPerStep = 0.0225;
    config.motorVelocityKpPerSecond = 8.0;
    config.motorMaximumAccelerationRpmS = 20.0;
    config.motorMaximumJerkRpmS3 = 300.0;
    config.motorBrakingAccelerationRpmS = 8.0;
    config.motorPositionToleranceSteps = 3.0;
    config.motorStopSpeedRpm = 1.0;
    config.motorSoftLimitSteps = 260;

    VelocityModePositionController controller(config);
    double positionSteps = 0.0;
    double speedRpm = 0.0;
    double maximumPositionSteps = 0.0;
    constexpr double dt = 0.02;

    for (int index = 0; index < 150; ++index) {
        const MotorLoopTelemetry state = controller.update(
            34, positionSteps, speedRpm, dt);
        speedRpm = approach(
            speedRpm, state.commandSpeedRpm, 82.0 * dt);
        positionSteps += speedRpm / 60.0 *
            config.pulsesPerRevolution * dt;
        maximumPositionSteps = std::max(
            maximumPositionSteps, positionSteps);
    }

    std::fprintf(stderr,
                 "small target position=%.3f peak=%.3f speed=%.3f\n",
                 positionSteps, maximumPositionSteps, speedRpm);
    assert(std::abs(positionSteps - 34.0) <=
            config.motorPositionToleranceSteps + 0.1);
    assert(maximumPositionSteps < 44.0);
    assert(std::abs(speedRpm) < 1.0);
}

void testLowSpeedFeedbackAndQuantization()
{
    AppConfig config;
    config.pulsesPerRevolution = 6400;
    config.motorRpm = 6;
    config.motorEncoderSpeedFilterSeconds = 0.04;

    EncoderSpeedEstimator estimator(config);
    constexpr double dt = 0.02;
    double positionSteps = 0.0;
    double estimatedSpeedRpm = estimator.update(
        positionSteps, 0.0, dt);
    for (int index = 0; index < 20; ++index) {
        positionSteps += 1.0 / 60.0 *
            config.pulsesPerRevolution * dt;
        estimatedSpeedRpm = estimator.update(
            positionSteps, 0.0, dt);
    }
    assert(std::abs(estimatedSpeedRpm - 1.0) < 0.02);

    VelocityCommandQuantizer quantizer;
    double transmittedRpmSum = 0.0;
    for (int index = 0; index < 100; ++index) {
        const double wireRpm = quantizer.update(0.40);
        assert(wireRpm == 0.0 || wireRpm == 1.0);
        transmittedRpmSum += wireRpm;
    }
    assert(std::abs(transmittedRpmSum - 40.0) <= 1.0);

    quantizer.reset();
    transmittedRpmSum = 0.0;
    for (int index = 0; index < 100; ++index) {
        const double wireRpm = quantizer.update(-0.40);
        assert(wireRpm == 0.0 || wireRpm == -1.0);
        transmittedRpmSum += wireRpm;
    }
    assert(std::abs(transmittedRpmSum + 40.0) <= 1.0);
}

void testSmallTargetWithIntegerZdtFeedback()
{
    AppConfig config;
    config.pulsesPerRevolution = 6400;
    config.motorRpm = 6;
    config.motorPositionKpRpmPerStep = 0.0225;
    config.motorVelocityKpPerSecond = 8.0;
    config.motorMaximumAccelerationRpmS = 20.0;
    config.motorMaximumJerkRpmS3 = 300.0;
    config.motorBrakingAccelerationRpmS = 8.0;
    config.motorPositionToleranceSteps = 3.0;
    config.motorStopSpeedRpm = 1.0;
    config.motorSoftLimitSteps = 260;
    config.motorEncoderSpeedFilterSeconds = 0.04;

    VelocityModePositionController controller(config);
    EncoderSpeedEstimator estimator(config);
    VelocityCommandQuantizer quantizer;
    constexpr double dt = 0.02;
    double positionSteps = 0.0;
    double physicalSpeedRpm = 0.0;
    double reportedSpeedRpm = 0.0;
    double maximumPositionSteps = 0.0;

    for (int index = 0; index < 250; ++index) {
        const double feedbackSpeedRpm = estimator.update(
            positionSteps, reportedSpeedRpm, dt);
        const MotorLoopTelemetry state = controller.update(
            34, positionSteps, feedbackSpeedRpm, dt);
        const double wireSpeedRpm = quantizer.update(
            state.commandSpeedRpm);
        assert(std::abs(wireSpeedRpm) <= config.motorRpm);

        physicalSpeedRpm = approach(
            physicalSpeedRpm, wireSpeedRpm, 82.0 * dt);
        positionSteps += physicalSpeedRpm / 60.0 *
            config.pulsesPerRevolution * dt;
        reportedSpeedRpm = std::round(physicalSpeedRpm);
        maximumPositionSteps = std::max(
            maximumPositionSteps, positionSteps);
    }

    std::fprintf(stderr,
                 "integer feedback target position=%.3f peak=%.3f speed=%.3f\n",
                 positionSteps, maximumPositionSteps, physicalSpeedRpm);
    assert(std::abs(positionSteps - 34.0) < 6.0);
    assert(maximumPositionSteps < 48.0);
    assert(std::abs(physicalSpeedRpm) < 1.0);
}

} // namespace

int main()
{
    testVelocityFrame();
    testCascadedController();
    testSmallPipeAngleTarget();
    testLowSpeedFeedbackAndQuantization();
    testSmallTargetWithIntegerZdtFeedback();
    std::puts("motor velocity controller tests passed");
    return 0;
}
