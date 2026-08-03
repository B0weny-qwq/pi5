#include "task4_balance_control.hpp"
#include "vehicle_motion_feedforward.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace ball_stepper;

namespace {

AppConfig testConfig()
{
    AppConfig config;
    config.task4Kp = 0.11;
    config.task4Kd = 0.08;
    config.task4Ki = 0.15;
    config.task4IntegralZoneCm = 1.0;
    config.task4IntegralSpeedLimitCmS = 1.0;
    config.task4IntegralLimitDeg = 0.12;
    config.task4IntegralLeakSeconds = 4.0;
    config.task4DeadbandCm = 0.02;
    config.task4StopSpeedCmS = 0.15;
    config.task4DriveAngleLimitDeg = 0.55;
    config.task4BrakeAngleLimitDeg = 0.65;
    return config;
}

void testSymmetricPdPolarityAndLimits()
{
    AppConfig config = testConfig();
    Task4BalanceController controller(config);

    controller.reset(0.0);
    const Task4ControlOutput right =
        controller.update(1.0, 0.0, 0.0, 0.01);
    assert(right.angleDeg > 0.0);
    assert(std::abs(right.pTermDeg - 0.11) < 1e-9);

    controller.reset(0.0);
    const Task4ControlOutput left =
        controller.update(-1.0, 0.0, 0.0, 0.01);
    assert(left.angleDeg < 0.0);
    assert(std::abs(left.angleDeg + right.angleDeg) < 1e-9);

    controller.reset(0.0);
    const Task4ControlOutput movingRight =
        controller.update(0.0, 2.0, 0.0, 0.01);
    assert(movingRight.dTermDeg > 0.0);
    assert(movingRight.angleDeg > 0.0);

    controller.reset(0.0);
    const Task4ControlOutput quiet =
        controller.update(0.01, 0.1, 0.0, 0.01);
    assert(quiet.deadband);
    assert(std::abs(quiet.angleDeg) < 1e-3);

    controller.reset(0.0);
    const Task4ControlOutput saturated =
        controller.update(10.0, 10.0, 0.0, 0.01);
    assert(saturated.driveSaturated);
    assert(saturated.saturated);
    assert(std::abs(saturated.driveAngleDeg - 0.55) < 1e-9);
    assert(std::abs(saturated.angleDeg - 0.65) < 1e-9);
}

void testNearCenterIntegralAndSignReset()
{
    AppConfig config = testConfig();
    Task4BalanceController controller(config);
    controller.reset(0.0);

    Task4ControlOutput output;
    for (int index = 1; index <= 300; ++index) {
        output = controller.update(0.40, 0.0, 0.0, index * 0.01);
    }
    assert(output.integralWindowActive);
    assert(output.iTermDeg > 0.05);
    assert(output.iTermDeg <= config.task4IntegralLimitDeg + 1e-9);

    output = controller.update(-0.20, 0.0, 0.0, 3.01);
    assert(output.iTermDeg <= 0.0);

    const double afterReversal = output.iTermDeg;
    controller.onMeasurementLost();
    output = controller.update(-0.20, 0.0, 0.0, 20.0);
    assert(std::abs(output.iTermDeg - afterReversal) < 1e-9);
}

void testVehicleAccelerationFeedforwardInterfaceAndPolarity()
{
    AppConfig config = testConfig();
    config.task4DriveAngleLimitDeg = 0.80;
    config.task4BrakeAngleLimitDeg = 0.91;
    config.task4VehicleSpeedFilterSeconds = 0.001;
    config.task4VehicleAccelerationFilterSeconds = 0.001;
    config.task4VehicleAccelerationDecaySeconds = 0.050;
    config.task4VehicleAccelerationDeadbandUnitsS = 0.0;
    config.task4VehicleAccelerationLimitUnitsS = 2000.0;
    config.task4VehicleAccelerationAngleSign = -1.0;
    config.task4VehicleFeedforwardDegPerUnitS = 0.00040;
    config.task4VehicleFeedforwardLimitDeg = 0.65;

    VehicleMotionFeedforward feedforward(config);
    feedforward.reset(0.0);
    assert(feedforward.submitEncoderSample({0.0, 0.00, 1}));
    VehicleMotionState state = feedforward.update(0.00);
    assert(state.signalFresh);
    assert(std::abs(state.feedforwardAngleDeg) < 1e-9);

    // The 20 Hz encoder is cleared every sample. A 0 -> 80 transition in
    // 50 ms is +1600 raw speed-units/s and should nearly reach the FF limit.
    assert(feedforward.submitEncoderSample({80.0, 0.05, 2}));
    state = feedforward.update(0.05);
    assert(state.rawAccelerationUnitsS > 1590.0);
    assert(state.feedforwardAngleDeg < -0.63);

    Task4BalanceController controller(config);
    controller.reset(0.05);
    const Task4ControlOutput output = controller.update(
        0.0, 0.0, state.feedforwardAngleDeg, 0.06);
    assert(output.deadband);
    assert(output.feedforwardTermDeg < -0.55);
    assert(output.angleDeg < -0.55);

    // A second cruise sample at 80 is steady speed, not acceleration.
    assert(feedforward.submitEncoderSample({80.0, 0.10, 3}));
    state = feedforward.update(0.10);
    assert(std::abs(state.rawAccelerationUnitsS) < 1.0);
    assert(std::abs(state.feedforwardAngleDeg) < 0.01);

    // Braking the chassis reverses acceleration and must reverse FF polarity.
    assert(feedforward.submitEncoderSample({0.0, 0.15, 4}));
    state = feedforward.update(0.15);
    assert(state.rawAccelerationUnitsS < -1590.0);
    assert(state.feedforwardAngleDeg > 0.63);

    const double beforeStaleDecay =
        std::abs(state.filteredAccelerationUnitsS);
    state = feedforward.update(0.50);
    assert(!state.signalFresh);
    assert(std::abs(state.filteredAccelerationUnitsS) < beforeStaleDecay);
}

void testEightSecondBidirectionalDisturbanceModel()
{
    AppConfig config = testConfig();
    Task4BalanceController controller(config);
    controller.reset(0.0);

    constexpr double dt = 0.005;
    constexpr double angleAccelerationGain = 18.0;
    constexpr double rollingDamping = 0.55;
    constexpr double angleSlewDegS = 8.0;

    double positionCm = 0.0;
    double speedCmS = 0.0;
    double appliedAngleDeg = 0.0;
    double maximumErrorCm = 0.0;

    for (int index = 1; index <= static_cast<int>(8.0 / dt); ++index) {
        const double now = index * dt;
        double carDisturbanceCmS2 = 0.0;
        if (now >= 0.40 && now < 1.30) {
            carDisturbanceCmS2 = 2.0;
        } else if (now >= 1.30 && now < 2.10) {
            carDisturbanceCmS2 = -1.8;
        } else if (now >= 2.10 && now < 6.80) {
            carDisturbanceCmS2 =
                0.75 * std::sin((now - 2.10) * 2.4);
        }

        const Task4ControlOutput output =
            controller.update(positionCm, speedCmS, 0.0, now);
        appliedAngleDeg = approach(
            appliedAngleDeg, output.angleDeg, angleSlewDegS * dt);

        const double accelerationCmS2 =
            carDisturbanceCmS2 -
            angleAccelerationGain * appliedAngleDeg -
            rollingDamping * speedCmS;
        speedCmS += accelerationCmS2 * dt;
        positionCm += speedCmS * dt;
        maximumErrorCm = std::max(maximumErrorCm, std::abs(positionCm));
    }

    std::printf("task4 model max error: %.3f cm\n", maximumErrorCm);
    assert(maximumErrorCm < 1.0);
    assert(std::abs(positionCm) < 0.30);
}

} // namespace

int main()
{
    testSymmetricPdPolarityAndLimits();
    testNearCenterIntegralAndSignReset();
    testVehicleAccelerationFeedforwardInterfaceAndPolarity();
    testEightSecondBidirectionalDisturbanceModel();
    std::puts("task4 balance tests passed");
    return 0;
}
