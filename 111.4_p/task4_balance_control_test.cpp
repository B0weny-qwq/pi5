#include "task4_balance_control.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace ball_stepper;

namespace {

AppConfig testConfig()
{
    AppConfig config;
    config.task4Kp = 0.30;
    config.task4Kd = 0.12;
    config.task4FineKp = 0.15;
    config.task4FineKd = 0.055;
    config.task4FineZoneCm = 0.30;
    config.task4FineSpeedCmS = 1.8;
    config.task4Ki = 0.018;
    config.task4IntegralLimitDeg = 0.12;
    config.task4IntegralEnableErrorCm = 1.5;
    config.task4IntegralLeakSeconds = 6.0;
    config.task4DeadbandCm = 0.05;
    config.task4StopSpeedCmS = 0.35;
    config.task4MaximumAngleDeg = 0.60;
    return config;
}

void testStateFeedbackAndLimits()
{
    AppConfig config = testConfig();
    Task4BalanceController controller(config);

    controller.reset(0.0);
    const Task4ControlOutput right = controller.update(1.0, 0.0, 0.01);
    assert(right.angleDeg > 0.0);
    assert(std::abs(right.pTermDeg - 0.30) < 1e-9);

    controller.reset(0.0);
    const Task4ControlOutput left = controller.update(-1.0, 0.0, 0.01);
    assert(left.angleDeg < 0.0);

    controller.reset(0.0);
    const Task4ControlOutput movingRight =
        controller.update(0.0, 2.0, 0.01);
    assert(movingRight.dTermDeg > 0.0);
    assert(movingRight.angleDeg > 0.0);

    controller.reset(0.0);
    const Task4ControlOutput quiet = controller.update(0.02, 0.1, 0.01);
    assert(quiet.deadband);
    assert(std::abs(quiet.angleDeg) < 1e-3);

    controller.reset(0.0);
    const Task4ControlOutput saturated =
        controller.update(10.0, 10.0, 0.01);
    assert(saturated.saturated);
    assert(std::abs(saturated.angleDeg - 0.60) < 1e-9);
}

void testLeakyIntegralAndLossFreeze()
{
    AppConfig config = testConfig();
    Task4BalanceController controller(config);
    controller.reset(0.0);

    Task4ControlOutput output;
    for (int index = 1; index <= 300; ++index) {
        output = controller.update(0.20, 0.0, index * 0.01);
    }
    assert(output.iTermDeg > 0.005);
    const double beforeLoss = output.iTermDeg;

    controller.onMeasurementLost();
    output = controller.update(0.20, 0.0, 20.0);
    assert(std::abs(output.iTermDeg - beforeLoss) < 1e-9);
}

void testEightSecondDisturbanceModel()
{
    AppConfig config = testConfig();
    Task4BalanceController controller(config);
    controller.reset(0.0);

    constexpr double dt = 0.005;
    constexpr double angleAccelerationGain = 18.0; // cm/s^2 per degree
    constexpr double rollingDamping = 0.55;
    constexpr double angleSlewDegS = 18.0;

    double positionCm = 0.0;
    double speedCmS = 0.0;
    double appliedAngleDeg = 0.0;
    double maximumErrorCm = 0.0;

    for (int index = 1; index <= static_cast<int>(8.0 / dt); ++index) {
        const double now = index * dt;
        double carDisturbanceCmS2 = 0.0;
        if (now >= 0.40 && now < 1.40) {
            carDisturbanceCmS2 = 2.2;
        } else if (now >= 1.40 && now < 2.10) {
            carDisturbanceCmS2 = -1.8;
        } else if (now >= 2.10 && now < 6.50) {
            carDisturbanceCmS2 =
                0.8 * std::sin((now - 2.10) * 2.4);
        }

        const Task4ControlOutput output =
            controller.update(positionCm, speedCmS, now);
        appliedAngleDeg = approach(
            appliedAngleDeg,
            output.angleDeg,
            angleSlewDegS * dt);

        const double accelerationCmS2 =
            carDisturbanceCmS2 -
            angleAccelerationGain * appliedAngleDeg -
            rollingDamping * speedCmS;
        speedCmS += accelerationCmS2 * dt;
        positionCm += speedCmS * dt;
        maximumErrorCm = std::max(
            maximumErrorCm, std::abs(positionCm));
    }

    std::printf("task4 model max error: %.3f cm\n", maximumErrorCm);
    assert(maximumErrorCm < 1.0);
    assert(std::abs(positionCm) < 0.30);
}

} // namespace

int main()
{
    testStateFeedbackAndLimits();
    testLeakyIntegralAndLossFreeze();
    testEightSecondDisturbanceModel();
    std::puts("task4 velocity balance tests passed");
    return 0;
}
