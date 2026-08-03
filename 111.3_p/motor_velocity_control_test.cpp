#include "zdt_stepper_uart.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace ball_stepper;

namespace {

void testVelocityFrame()
{
    const auto frame = makeEmmV5VelocityFrame(
        0x01, -1500.0, 3000, 200);
    const std::array<uint8_t, 8> expected = {
        0x01, 0xF6, 0x01, 0x05, 0xDC, 0x9C, 0x00, 0x6B
    };
    assert(frame == expected);

    const auto limited = makeEmmV5VelocityFrame(
        0x02, 4000.0, 25, 200);
    assert(limited[0] == 0x02);
    assert(limited[2] == 0x00);
    assert(limited[3] == 0x00 && limited[4] == 0x19);
    assert(limited[5] == 0x9C);
    assert(limited[6] == 0x00 && limited[7] == 0x6B);

    const auto precise = makeEmmV5VelocityFrame(0x01, 0.4, 6, 30);
    assert(precise[3] == 0x00 && precise[4] == 0x00);

    assert(makeEmmV5AccelerationLevel(200.0) == 156);
    assert(std::abs(emmV5AccelerationRpmS(156) - 200.0) < 1e-9);
}

void testRelativePositionFrame()
{
    const auto frame = makeEmmV5RelativePositionFrame(
        0x01, -5.0, 6.0, 200, 3200);
    const std::array<uint8_t, 13> expected = {
        0x01, 0xFD, 0x01, 0x00, 0x06, 0x9C,
        0x00, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x6B
    };
    assert(frame == expected);

    const auto positive = makeEmmV5RelativePositionFrame(
        0x02, 5.0, 6.0, 200, 6400);
    assert(positive[0] == 0x02);
    assert(positive[2] == 0x00);
    assert(positive[3] == 0x00 && positive[4] == 0x06);
    assert(positive[5] == 0x9C);
    assert(positive[6] == 0x00 && positive[7] == 0x00 &&
           positive[8] == 0x00 && positive[9] == 0x59);
    assert(positive[10] == 0x00 && positive[11] == 0x00 &&
           positive[12] == 0x6B);
}

void testDriverParameterResponse()
{
    const std::array<uint8_t, 33> response = {
        0x01, 0x42, 0x21, 0x15, 0x19, 0x02, 0x02, 0x02,
        0x00, 0x10, 0x01, 0x00, 0x03, 0xE8, 0x0B, 0xB8,
        0x0F, 0xA0, 0x05, 0x07, 0x01, 0x00, 0x01, 0x01,
        0x00, 0x28, 0x09, 0x60, 0x0F, 0xA0, 0x00, 0x01,
        0x6B
    };
    ZdtDriverParameters parameters;
    assert(decodeZdtDriverParametersResponse(response, 0x01, parameters));
    assert(parameters.motorType == 25);
    assert(parameters.pulseControlMode == 0x02);
    assert(parameters.serialPortFunction == 0x02);
    assert(parameters.enableMode == 0x02);
    assert(parameters.microstep == 0x10);
    assert(parameters.uartBaudIndex == 0x05);
    assert(parameters.checksumMode == 0x00);
    assert(parameters.responseMode == 0x01);

    auto invalid = response;
    invalid[2] = 0x20;
    assert(!decodeZdtDriverParametersResponse(invalid, 0x01, parameters));
}

void testAbsoluteHomingProtocol()
{
    ZdtHomingParameters parameters;
    parameters.mode = ZdtHomingMode::Nearest;
    parameters.direction = 0x00;
    parameters.velocityRpm = 6;
    parameters.timeoutMs = 5000;
    parameters.sensorlessSpeedRpm = 4000;
    parameters.sensorlessCurrentMilliamps = 800;
    parameters.sensorlessTimeMs = 60;
    parameters.powerOnAutomatic = false;

    const auto frame = makeEmmV5HomingParametersFrame(
        0x01, parameters, true);
    const std::array<uint8_t, 20> expectedFrame = {
        0x01, 0x4C, 0xAE, 0x01, 0x00, 0x00, 0x00, 0x06,
        0x00, 0x00, 0x13, 0x88, 0x0F, 0xA0, 0x03, 0x20,
        0x00, 0x3C, 0x00, 0x6B
    };
    assert(frame == expectedFrame);

    const std::array<uint8_t, 18> response = {
        0x01, 0x22, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
        0x13, 0x88, 0x0F, 0xA0, 0x03, 0x20, 0x00, 0x3C,
        0x00, 0x6B
    };
    ZdtHomingParameters decoded;
    assert(decodeEmmV5HomingParametersResponse(response, 0x01, decoded));
    assert(decoded.mode == ZdtHomingMode::Nearest);
    assert(decoded.velocityRpm == 6);
    assert(decoded.timeoutMs == 5000);
    assert(decoded.sensorlessSpeedRpm == 4000);
    assert(decoded.sensorlessCurrentMilliamps == 800);
    assert(decoded.sensorlessTimeMs == 60);
    assert(!decoded.powerOnAutomatic);
    assert(zdtResponseModeHasImmediateAck(0x01));
    assert(zdtResponseModeHasImmediateAck(0x03));
    assert(!zdtResponseModeHasImmediateAck(0x00));
}

void testIntegerRpmQuantizer()
{
    VelocityCommandQuantizer quantizer;
    double sum = 0.0;
    for (int index = 0; index < 8; ++index) {
        const double wireRpm = quantizer.update(0.25);
        assert(wireRpm == 0.0 || wireRpm == 1.0);
        sum += wireRpm;
    }
    assert(sum == 2.0);

    assert(quantizer.update(-0.5) == 0.0);
    assert(quantizer.update(-0.5) == -1.0);
    assert(quantizer.update(0.0) == 0.0);
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

void testLowSpeedFeedbackWithIntegerRpmProtocol()
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

}

void testSmallTargetWithIntegerRpmZdtFeedback()
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
                 "integer-RPM feedback target position=%.3f peak=%.3f speed=%.3f\n",
                 positionSteps, maximumPositionSteps, physicalSpeedRpm);
    assert(std::abs(positionSteps - 34.0) < 6.0);
    assert(maximumPositionSteps < 48.0);
    assert(std::abs(physicalSpeedRpm) < 1.0);
}

void testSubPulsePositionErrorIsNotDiscarded()
{
    AppConfig config;
    config.pulsesPerRevolution = 200;
    config.motorRpm = 6;
    config.motorPositionKpRpmPerStep = 0.72;
    config.motorPositionToleranceSteps = 0.35;

    VelocityModePositionController controller(config);
    const MotorLoopTelemetry state = controller.update(
        3, 2.3, 0.0, 0.02);
    assert(std::abs(state.positionErrorSteps - 0.7) < 1e-9);
    assert(state.targetSpeedRpm > 0.0);
    assert(!state.atTarget);
}

} // namespace

int main()
{
    testVelocityFrame();
    testRelativePositionFrame();
    testDriverParameterResponse();
    testAbsoluteHomingProtocol();
    testIntegerRpmQuantizer();
    testCascadedController();
    testSmallPipeAngleTarget();
    testLowSpeedFeedbackWithIntegerRpmProtocol();
    testSmallTargetWithIntegerRpmZdtFeedback();
    testSubPulsePositionErrorIsNotDiscarded();
    std::puts("motor velocity controller tests passed");
    return 0;
}
