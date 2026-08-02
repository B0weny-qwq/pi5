#include "task3_sequence.hpp"
#include "task3_motion_control.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace ball_stepper;

int main()
{
    AppConfig config;
    config.pipeLengthCm = 10.0;
    config.task3OffsetCm = 5.0;
    config.task3NegativeTargetBiasCm = 0.0;
    config.task3PositiveToleranceCm = 2.0;
    config.task3ArrivalConfirmFrames = 2;
    config.task3FinalToleranceCm = 0.20;
    config.task3FinalRightToleranceCm = 0.02;
    config.task3FinalSpeedCmS = 1.5;
    config.task3FinalStableMs = 80;

    Task3Sequence sequence(config);
    sequence.start(0.0);

    // 到达帧中间发生漏检，不能拼成一次有效折返确认。
    sequence.update(9.1, 2.0, 0.01);
    sequence.onMeasurementLost();
    sequence.update(9.2, 2.0, 0.04);
    assert(sequence.phase() == Task3Phase::MoveToPositive);

    // 第二个连续真实测量到达帧才切换到O-5。
    sequence.update(9.3, 2.0, 0.05);
    assert(sequence.phase() == Task3Phase::MoveToNegative);
    assert(std::abs(sequence.targetCm()) < 1e-9);

    // 即使球已低速停止，仍在-5右侧0.30 cm时也不能提前完成。
    sequence.update(0.30, 0.0, 0.10);
    sequence.update(0.30, 0.0, 0.20);
    assert(sequence.phase() == Task3Phase::MoveToNegative);

    // 进入收紧后的右侧0.05 cm容差并连续稳定80 ms后才允许完成。
    sequence.update(0.01, 0.0, 0.30);
    sequence.update(0.01, 0.0, 0.39);
    assert(sequence.phase() == Task3Phase::HoldNegative);

    AppConfig motionConfig;
    motionConfig.task3MinimumTravelAngleDeg = 0.35;
    motionConfig.task3ReturnAngleLimitDeg = 0.0728;
    motionConfig.task3ReturnCruiseSpeedCmS = 4.5;
    motionConfig.task3ReturnSpeedKpDegPerCmS = 0.05;
    motionConfig.task3ReturnApproachZoneCm = 0.80;
    motionConfig.task3FinalRightToleranceCm = 0.02;
    motionConfig.centerDeadbandCm = 0.12;
    motionConfig.stopSpeedCmS = 0.8;
    motionConfig.kp = 0.025;
    motionConfig.task3ReturnKd = 0.010;
    motionConfig.task3ReturnBrakeAngleLimitDeg = 0.10192;
    motionConfig.task3CreepAngleDeg = 0.015;
    motionConfig.task3CreepErrorCm = 0.15;
    motionConfig.task3CreepSpeedCmS = 0.25;
    motionConfig.task3SettleAngleLimitDeg = 0.060;

    Task3MotionController motion(motionConfig);
    const Task3MotionCommand positiveDrive = motion.update(
        Task3Phase::MoveToPositive, -5.0, 0.0);
    assert(positiveDrive.mode == Task3MotionMode::PositiveDrive);
    assert(std::abs(positiveDrive.angleDeg + 0.35) < 1e-9);

    const Task3MotionCommand returnStart = motion.update(
        Task3Phase::MoveToNegative, 5.0, 0.0);
    assert(returnStart.mode == Task3MotionMode::ReturnDrive);
    assert(std::abs(returnStart.angleDeg - 0.0728) < 1e-9);

    // 远距离返程固定0.14°，不因速度升高自动收小。
    const Task3MotionCommand returnCruise = motion.update(
        Task3Phase::MoveToNegative, 3.0, -0.5);
    assert(returnCruise.mode == Task3MotionMode::ReturnDrive);
    assert(std::abs(returnCruise.angleDeg - 0.0728) < 1e-9);

    const Task3MotionCommand returnFast = motion.update(
        Task3Phase::MoveToNegative, 2.0, -3.0);
    assert(std::abs(returnFast.angleDeg - 0.0728) < 1e-9);

    const Task3MotionCommand returnAtCruise = motion.update(
        Task3Phase::MoveToNegative, 2.0, -4.5);
    assert(std::abs(returnAtCruise.angleDeg) < 1e-9);

    const Task3MotionCommand returnTooFast = motion.update(
        Task3Phase::MoveToNegative, 2.0, -8.0);
    assert(returnTooFast.mode == Task3MotionMode::ReturnBrake);
    assert(returnTooFast.angleDeg < 0.0);

    // 距-5较近且仍高速向左时，必须在越线前输出反向角提前减速。
    const Task3MotionCommand preBrake = motion.update(
        Task3Phase::MoveToNegative, 0.20, -2.0);
    assert(preBrake.mode == Task3MotionMode::ReturnBrake);
    assert(preBrake.angleDeg < 0.0);

    // 若在目标右侧低速停住，则恢复正向驱动力继续送向-5。
    const Task3MotionCommand antiStall = motion.update(
        Task3Phase::MoveToNegative, 0.20, 0.0);
    assert(antiStall.mode == Task3MotionMode::ReturnDrive);
    assert(std::abs(antiStall.angleDeg - 0.0728) < 1e-9);

    // 完成后若又漂到-5右侧0.02 cm以外，必须继续向左爬行。
    const Task3MotionCommand rightCreep = motion.update(
        Task3Phase::HoldNegative, 0.03, 0.0);
    assert(rightCreep.mode == Task3MotionMode::Hold);
    assert(std::abs(rightCreep.angleDeg - 0.015) < 1e-9);

    // While the ball is still moving, hold mode must remain continuous PD and
    // must not jump to the static-friction creep angle.
    const Task3MotionCommand movingHold = motion.update(
        Task3Phase::HoldNegative, 0.50, 1.0);
    assert(std::abs(movingHold.angleDeg - 0.0225) < 1e-9);

    const Task3MotionCommand centeredHold = motion.update(
        Task3Phase::HoldNegative, 0.01, 0.0);
    assert(std::abs(centeredHold.angleDeg) < 1e-9);

    std::puts("task3 sequence tests passed");
    return 0;
}
