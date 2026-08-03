#pragma once

// system_config.hpp
// ============================================================================
// 本文件只定义模块之间共享的配置结构、时间函数和参数合法性检查。
// 所有需要现场修改的实际数值都集中在main.cpp的makeUserConfig()中，
// 正常使用时不要在本文件改默认值。
// ============================================================================

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace ball_stepper {

// 一个机构标定点表示：水管处于pipeAngleDeg角度时，
// ZDT相对于水平零位应该位于motorSteps目标轴位脉冲。
struct CalibrationPoint {
    double pipeAngleDeg = 0.0;
    int motorSteps = 0;
};

struct AppConfig {
    // ---------------- 摄像头 ----------------
    int cameraIndex = 0;
    int cameraWidth = 640;
    int cameraHeight = 480;
    int cameraFps = 120;
    std::string cameraFourcc = "MJPG";
    bool disableAutofocus = true;

    // false表示完全不改摄像头当前曝光设置。某些120 FPS UVC摄像头一旦被
    // OpenCV重新切换自动/手动曝光，就会退回低帧率模式，因此比赛默认false。
    bool configureExposure = false;
    // configureExposure=true时才读取本项。120 FPS每帧约8.3 ms，手动曝光
    // 必须短于该时间；V4L2的exposureAbsolute常以0.1 ms为单位。
    bool useManualExposure = false;
    double exposureAbsolute = 45.0;

    // ---------------- 钢球搜索区域和水管轴线 ----------------
    // ROI只限制内部识别范围，程序不会在预览画面绘制蓝框。
    cv::Rect roi{};
    cv::Point2f axisLeft{};
    cv::Point2f axisRight{};
    bool axisConfigured = false;
    bool drawPipeDetectionArea = true;
    cv::Rect pipeDisplayArea{};
    double pipeLengthCm = 25.0;
    double targetCm = 12.5;

    // ---------------- 三点位置标定（第3题专用） ----------------
    // 普通模式默认认为像素与厘米线性对应。摄像头透视或安装偏心时，
    // O左侧5 cm和右侧5 cm可能对应不同像素长度，此时启用三点分段换算。
    bool useThreePointPositionCalibration = false;
    cv::Point2f minus5CalibrationPoint{};
    cv::Point2f centerCalibrationPoint{};
    cv::Point2f plus5CalibrationPoint{};

    // ---------------- 第3题目标流程 ----------------
    // 钢球从O出发，先到O+task3OffsetCm，再自动去O-task3OffsetCm。
    double task3OffsetCm = 5.0;
    // O-5终点的实机补偿量。正值表示控制目标略向左移，用于修正最终总停在
    // -5刻度右侧的机构滞后；视觉标定点本身仍保持真实的-5位置。
    double task3NegativeTargetBiasCm = 0.0;

    // 到达+5 cm附近连续确认多少帧后折返。120 FPS下3帧约25 ms。
    double task3PositiveToleranceCm = 0.35;
    int task3ArrivalConfirmFrames = 8;

    // 到达-5 cm后，位置和速度同时满足条件并持续一段时间，才判定完成。
    // 右侧误差单独收紧，防止球尚未真正到达-5就被提前判定为完成；
    // 左侧仍保留较宽容差，用于吸收返程惯性和视觉测量抖动。
    double task3FinalToleranceCm = 0.50;
    double task3FinalRightToleranceCm = 0.15;
    double task3FinalSpeedCmS = 1.0;
    int task3FinalStableMs = 200;
    int task3TimeLimitMs = 5000;

    // ---------------- 第3题钢球位置 PDI 外环 ----------------
    // error = position - target. Negative pipe angle moves the ball right;
    // positive pipe angle moves it left. The linkage is not symmetric, so P,
    // the normal output limit, and breakaway authority are tuned per direction.
    double task3MoveRightPositionKpDegPerCm = 0.300;
    double task3MoveLeftPositionKpDegPerCm = 0.035;
    double task3VelocityKdDegPerCmS = 0.020;
    // I is active only near either target.  It is reset whenever the target
    // changes and is capped by the two values below.
    double task3IntegralKiDegPerCmSecond = 0.120;
    double task3IntegralZoneCm = 0.80;
    double task3IntegralSpeedLimitCmS = 1.2;
    double task3IntegralLimitCmSeconds = 0.90;

    // If a large position error remains while the measured ball speed stays
    // near zero, slowly add a bounded breakaway angle.  This compensates
    // linkage play and static friction from feedback instead of hard-coding a
    // minimum motor angle.  The term decays as soon as the ball responds.
    double task3BreakawayErrorCm = 0.35;
    double task3BreakawaySpeedCmS = 0.35;
    double task3BreakawayDelaySeconds = 0.08;
    double task3BreakawayRampDegPerSecond = 0.80;
    double task3MoveRightBreakawayMaximumAngleDeg = 0.16;
    double task3MoveLeftBreakawayMaximumAngleDeg = 0.12;

    // These are directional safety/output scales, never fixed travel commands.
    // The PDI sum is continuously calculated and clamped to this asymmetric
    // interval: [-moveRightLimit, +moveLeftLimit].
    double task3MoveRightOutputAngleLimitDeg = 0.75;
    double task3MoveLeftOutputAngleLimitDeg = 0.40;
    double speedFilterSeconds = 0.020;
    int speedDifferenceFrames = 2;
    double maximumPipeAngleDeg = 1.0;
    double angleSlewDegS = 10.0;

    // ---------------- 曲柄连杆与脉冲换算 ----------------
    double crankRadiusMm = 15.0;
    double actuatorDistanceMm = 250.0;
    int pulsesPerRevolution = 200;
    int motorSign = 1;

    // 有两个以上点时优先查表插值；为空时使用曲柄连杆近似公式。
    // 实测标定表的实际数值也必须写在main.cpp中。
    std::vector<CalibrationPoint> calibrationPoints;

    // ---------------- 树莓派GPIO14/15与ZDT驱动器 ----------------
    bool motorEnabled = false;
    std::string serialPort = "/dev/serial0";
    int serialBaud = 115200;
    int motorAddress = 1;
    // Physical acceleration request.  The Emm V5 layer converts RPM/s to its
    // 0-255 acceleration slot before sending 0xF6.
    int motorRpm = 8;
    int motorSpeedSlopeRpmS = 60;
    int motorCommandHz = 30;

    // 电机串级环：目标水管角度先换算为目标电机轴位，位置环输出目标RPM，
    // 速度误差再输出目标加速度，最后经过加速度和跃度限制发送0xF6。
    double motorPositionKpRpmPerStep = 0.25;
    double motorVelocityKpPerSecond = 10.0;
    double motorMaximumAccelerationRpmS = 60.0;
    double motorMaximumJerkRpmS3 = 600.0;
    // 制动速度上限v=sqrt(2as)使用更保守的等效减速度，补偿跃度建立时间。
    double motorBrakingAccelerationRpmS = 40.0;
    double motorPositionToleranceSteps = 1.5;
    double motorStopSpeedRpm = 1.5;
    // Emm V5 0x35 reports integer RPM.  Below 1 RPM, use 0x36 encoder-position
    // changes to estimate a filtered motor speed for the inner loop.
    double motorEncoderSpeedFilterSeconds = 0.04;

    // 当前位置在启动时清零，所以软限位也是相对水平零位的正负脉冲范围。
    int motorSoftLimitSteps = 130;
    int motorReplyTimeoutMs = 15;
    bool motorExpectCommandAck = true;

    // true表示启动时把“当前电机位置”清为绝对0脉冲。
    // 只有水管已经真实水平、机构没有顶限位时才允许设为true。
    // Preferred startup path: return to the persistent single-turn absolute
    // encoder origin before defining the runtime 0x36 position as zero.
    bool absoluteEncoderHomeOnStart = false;
    int absoluteEncoderHomeRpm = 6;
    int absoluteEncoderHomeTimeoutMs = 5000;
    int absoluteEncoderHomePollMs = 40;

    // Legacy path. This declares the arbitrary startup position to be zero and
    // must not be enabled together with absoluteEncoderHomeOnStart.
    bool zeroOnStart = false;

    // ZDT启动/清零/退出时的等待时间。通常不改，电机很慢时可适当增加退出等待。
    int enableSettleMs = 80;
    int stopSettleMs = 30;
    int zeroSettleMs = 50;
    int exitReturnTimeoutMs = 1500;

    // ---------------- 丢球保护 ----------------
    // lostHoldMs内保持最后一次真实测量产生的角度；随后逐渐回到水平；
    // 达到lostNeutralMs后目标角度严格等于0。
    int lostHoldMs = 80;
    int lostNeutralMs = 300;

    // ---------------- 启动、显示和记录 ----------------
    bool startArmed = false;
    bool gui = true;
    bool terminalKeys = true;
    bool csv = false;
    bool runtimeLogEnabled = true;
    std::string runtimeLogDirectory = "logs";
    std::string runtimeLogEvent = "task3_start";
    int runtimeLogIntervalMs = 200;
    int previewEveryNFrames = 2;

    // ---------------- E611网络图传 ----------------
    // 视觉仍按cameraFps处理；这里只对叠加后的显示画面限帧、编码并通过UDP发送。
    bool videoStreamEnabled = false;
    std::string videoStreamHost = "192.168.137.1";
    int videoStreamPort = 5600;
    int videoStreamFps = 30;
    int videoStreamBitrateKbps = 1000;
};

using ControlClock = std::chrono::steady_clock;

// Ctrl+C和SIGTERM只修改该标志；串口回零与停机由主循环在普通线程完成。
inline std::atomic<bool> running{true};

inline void onSignal(int)
{
    running.store(false);
}

inline double secondsNow()
{
    return std::chrono::duration<double>(
        ControlClock::now().time_since_epoch()).count();
}

inline int64_t millisecondsNow()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        ControlClock::now().time_since_epoch()).count();
}

inline double approach(double current, double target, double maximumChange)
{
    // 每次最多改变maximumChange，防止目标水管角度在一帧内突然跳变。
    if (current < target) return std::min(current + maximumChange, target);
    if (current > target) return std::max(current - maximumChange, target);
    return current;
}

inline bool validateConfig(const AppConfig& config)
{
    if (config.cameraWidth < 160 || config.cameraHeight < 120 ||
        config.cameraFps < 1 || config.cameraFps > 240 ||
        config.cameraFourcc.size() != 4 ||
        !std::isfinite(config.exposureAbsolute) ||
        (config.configureExposure && config.useManualExposure &&
         config.exposureAbsolute <= 0.0)) {
        std::fprintf(stderr, "invalid camera configuration in main.cpp\n");
        return false;
    }

    const double axisLengthPx = cv::norm(
        config.axisRight - config.axisLeft);
    if (!config.axisConfigured || axisLengthPx < 30.0 ||
        config.roi.width <= 0 || config.roi.height <= 0) {
        std::fprintf(stderr,
            "ROI and axis endpoints must be configured in main.cpp\n");
        return false;
    }

    if (!std::isfinite(config.pipeLengthCm) ||
        !std::isfinite(config.targetCm) ||
        config.pipeLengthCm <= 1.0 ||
        config.targetCm < 0.0 || config.targetCm > config.pipeLengthCm) {
        std::fprintf(stderr, "invalid pipe length or target position\n");
        return false;
    }

    if (config.useThreePointPositionCalibration) {
        const cv::Point2f calibrationDirection =
            config.plus5CalibrationPoint - config.minus5CalibrationPoint;
        const double calibrationLength = cv::norm(calibrationDirection);
        if (calibrationLength < 40.0) {
            std::fprintf(stderr,
                "three-point calibration endpoints are too close\n");
            return false;
        }

        const cv::Point2f calibrationUnit =
            calibrationDirection * static_cast<float>(1.0 / calibrationLength);
        const cv::Point2f leftDifference =
            config.centerCalibrationPoint - config.minus5CalibrationPoint;
        const cv::Point2f rightDifference =
            config.plus5CalibrationPoint - config.centerCalibrationPoint;
        const double leftPixels =
            leftDifference.x * calibrationUnit.x +
            leftDifference.y * calibrationUnit.y;
        const double rightPixels =
            rightDifference.x * calibrationUnit.x +
            rightDifference.y * calibrationUnit.y;
        if (leftPixels < 20.0 || rightPixels < 20.0) {
            std::fprintf(stderr,
                "invalid minus5/center/plus5 calibration order\n");
            return false;
        }
    }

    const double halfPipeCm = config.pipeLengthCm * 0.5;
    if (!std::isfinite(config.task3OffsetCm) ||
        config.task3OffsetCm <= 0.0 ||
        // Task 3 allows a target exactly at the visible rolling endpoint.
        config.task3OffsetCm > halfPipeCm ||
        !std::isfinite(config.task3NegativeTargetBiasCm) ||
        config.task3NegativeTargetBiasCm < 0.0 ||
        config.task3NegativeTargetBiasCm > 0.5 ||
        !std::isfinite(config.task3PositiveToleranceCm) ||
        config.task3PositiveToleranceCm <= 0.0 ||
        config.task3ArrivalConfirmFrames < 1 ||
        !std::isfinite(config.task3FinalToleranceCm) ||
        config.task3FinalToleranceCm <= 0.0 ||
        !std::isfinite(config.task3FinalRightToleranceCm) ||
        config.task3FinalRightToleranceCm <= 0.0 ||
        config.task3FinalRightToleranceCm >
            config.task3FinalToleranceCm ||
        !std::isfinite(config.task3FinalSpeedCmS) ||
        config.task3FinalSpeedCmS < 0.0 ||
        config.task3FinalStableMs < 0 ||
        config.task3TimeLimitMs < 1000) {
        std::fprintf(stderr,
            "invalid TASK 3 target-flow parameter in main.cpp\n");
        return false;
    }

    if (!std::isfinite(config.task3MoveRightPositionKpDegPerCm) ||
        config.task3MoveRightPositionKpDegPerCm <= 0.0 ||
        !std::isfinite(config.task3MoveLeftPositionKpDegPerCm) ||
        config.task3MoveLeftPositionKpDegPerCm <= 0.0 ||
        !std::isfinite(config.task3VelocityKdDegPerCmS) ||
        config.task3VelocityKdDegPerCmS < 0.0 ||
        !std::isfinite(config.task3IntegralKiDegPerCmSecond) ||
        config.task3IntegralKiDegPerCmSecond <= 0.0 ||
        !std::isfinite(config.task3IntegralZoneCm) ||
        config.task3IntegralZoneCm <= 0.0 ||
        !std::isfinite(config.task3IntegralSpeedLimitCmS) ||
        config.task3IntegralSpeedLimitCmS <= 0.0 ||
        !std::isfinite(config.task3IntegralLimitCmSeconds) ||
        config.task3IntegralLimitCmSeconds <= 0.0 ||
        !std::isfinite(config.task3BreakawayErrorCm) ||
        config.task3BreakawayErrorCm <= 0.0 ||
        !std::isfinite(config.task3BreakawaySpeedCmS) ||
        config.task3BreakawaySpeedCmS <= 0.0 ||
        !std::isfinite(config.task3BreakawayDelaySeconds) ||
        config.task3BreakawayDelaySeconds < 0.0 ||
        !std::isfinite(config.task3BreakawayRampDegPerSecond) ||
        config.task3BreakawayRampDegPerSecond <= 0.0 ||
        !std::isfinite(
            config.task3MoveRightBreakawayMaximumAngleDeg) ||
        config.task3MoveRightBreakawayMaximumAngleDeg < 0.0 ||
        !std::isfinite(
            config.task3MoveLeftBreakawayMaximumAngleDeg) ||
        config.task3MoveLeftBreakawayMaximumAngleDeg < 0.0 ||
        !std::isfinite(config.task3MoveRightOutputAngleLimitDeg) ||
        config.task3MoveRightOutputAngleLimitDeg <= 0.0 ||
        !std::isfinite(config.task3MoveLeftOutputAngleLimitDeg) ||
        config.task3MoveLeftOutputAngleLimitDeg <= 0.0 ||
        config.speedFilterSeconds < 0.005 ||
        config.speedDifferenceFrames != 2 ||
        config.maximumPipeAngleDeg <= 0.0 ||
        config.maximumPipeAngleDeg > 10.0 ||
        config.task3MoveRightOutputAngleLimitDeg +
                config.task3MoveRightBreakawayMaximumAngleDeg >
            config.maximumPipeAngleDeg ||
        config.task3MoveLeftOutputAngleLimitDeg +
                config.task3MoveLeftBreakawayMaximumAngleDeg >
            config.maximumPipeAngleDeg ||
        config.angleSlewDegS <= 0.0) {
        std::fprintf(stderr, "invalid TASK 3 PDI or angle-limit parameter\n");
        return false;
    }

    if (config.crankRadiusMm <= 1.0 ||
        config.actuatorDistanceMm <= 10.0 ||
        config.pulsesPerRevolution <= 0 ||
        (config.motorSign != 1 && config.motorSign != -1)) {
        std::fprintf(stderr, "invalid mechanism parameter\n");
        return false;
    }

    // 标定表必须严格按角度从小到大排列，禁止重复角度造成插值除以0。
    for (std::size_t index = 1;
         index < config.calibrationPoints.size(); ++index) {
        if (config.calibrationPoints[index].pipeAngleDeg <=
            config.calibrationPoints[index - 1].pipeAngleDeg) {
            std::fprintf(stderr,
                "calibrationPoints must be strictly sorted by angle\n");
            return false;
        }
    }
    if (config.calibrationPoints.size() == 1) {
        std::fprintf(stderr,
            "calibrationPoints needs zero points or at least two points\n");
        return false;
    }

    if (config.serialBaud != 9600 && config.serialBaud != 57600 &&
        config.serialBaud != 115200 && config.serialBaud != 230400) {
        std::fprintf(stderr, "unsupported serial baud\n");
        return false;
    }
    if (config.motorAddress < 0 || config.motorAddress > 255 ||
        config.motorRpm < 1 || config.motorRpm > 3000 ||
        config.motorSpeedSlopeRpmS < 1 ||
        config.motorSpeedSlopeRpmS > 65535 ||
        config.motorCommandHz < 1 || config.motorCommandHz > 120 ||
        !std::isfinite(config.motorPositionKpRpmPerStep) ||
        config.motorPositionKpRpmPerStep <= 0.0 ||
        !std::isfinite(config.motorVelocityKpPerSecond) ||
        config.motorVelocityKpPerSecond <= 0.0 ||
        !std::isfinite(config.motorMaximumAccelerationRpmS) ||
        config.motorMaximumAccelerationRpmS <= 0.0 ||
        !std::isfinite(config.motorMaximumJerkRpmS3) ||
        config.motorMaximumJerkRpmS3 <= 0.0 ||
        !std::isfinite(config.motorBrakingAccelerationRpmS) ||
        config.motorBrakingAccelerationRpmS <= 0.0 ||
        config.motorBrakingAccelerationRpmS >
            config.motorMaximumAccelerationRpmS ||
        !std::isfinite(config.motorPositionToleranceSteps) ||
        config.motorPositionToleranceSteps <= 0.0 ||
        !std::isfinite(config.motorStopSpeedRpm) ||
        config.motorStopSpeedRpm < 0.0 ||
        !std::isfinite(config.motorEncoderSpeedFilterSeconds) ||
        config.motorEncoderSpeedFilterSeconds < 0.005 ||
        config.motorEncoderSpeedFilterSeconds > 0.5 ||
        config.motorSoftLimitSteps < 1 ||
        config.motorReplyTimeoutMs < 2 ||
        config.motorReplyTimeoutMs > 100) {
        std::fprintf(stderr, "invalid ZDT motor parameter\n");
        return false;
    }

    if (config.motorMaximumAccelerationRpmS >
        static_cast<double>(config.motorSpeedSlopeRpmS)) {
        std::fprintf(stderr,
            "software acceleration exceeds ZDT 0xF6 speed slope\n");
        return false;
    }

    if (config.motorEnabled && config.serialPort.empty()) {
        std::fprintf(stderr, "motor enabled but serialPort is empty\n");
        return false;
    }
    if (config.absoluteEncoderHomeOnStart && config.zeroOnStart) {
        std::fprintf(stderr,
            "absolute homing and legacy zeroOnStart cannot both be enabled\n");
        return false;
    }
    if (config.motorEnabled && !config.absoluteEncoderHomeOnStart &&
        !config.zeroOnStart) {
        std::fprintf(stderr,
            "motor mode requires absolute homing or explicit legacy zeroing\n");
        return false;
    }
    if (config.absoluteEncoderHomeOnStart &&
        (config.absoluteEncoderHomeRpm < 1 ||
         config.absoluteEncoderHomeRpm > config.motorRpm ||
         config.absoluteEncoderHomeTimeoutMs < 200 ||
         config.absoluteEncoderHomeTimeoutMs > 60000 ||
         config.absoluteEncoderHomePollMs < 10 ||
         config.absoluteEncoderHomePollMs > 500)) {
        std::fprintf(stderr, "invalid absolute encoder homing parameter\n");
        return false;
    }
    if (!config.gui && !config.terminalKeys &&
        config.motorEnabled && !config.startArmed) {
        std::fprintf(stderr,
            "headless motor mode requires terminalKeys or startArmed=true\n");
        return false;
    }
    if (config.runtimeLogEnabled && config.runtimeLogDirectory.empty()) {
        std::fprintf(stderr, "runtime log directory is empty\n");
        return false;
    }
    if (config.runtimeLogIntervalMs < 50 ||
        config.runtimeLogIntervalMs > 5000) {
        std::fprintf(stderr, "invalid runtime log interval\n");
        return false;
    }

    if (config.videoStreamEnabled &&
        (config.videoStreamHost.empty() ||
         config.videoStreamPort < 1 || config.videoStreamPort > 65535 ||
         config.videoStreamFps < 1 ||
         config.videoStreamFps > config.cameraFps ||
         config.videoStreamBitrateKbps < 100 ||
         config.videoStreamBitrateKbps > 12000)) {
        std::fprintf(stderr, "invalid UDP video stream configuration\n");
        return false;
    }

    if (config.lostHoldMs < 0 ||
        config.lostNeutralMs <= config.lostHoldMs ||
        config.previewEveryNFrames < 1 ||
        config.enableSettleMs < 0 || config.stopSettleMs < 0 ||
        config.zeroSettleMs < 0 || config.exitReturnTimeoutMs < 100) {
        std::fprintf(stderr, "invalid timing parameter\n");
        return false;
    }
    return true;
}

} // namespace ball_stepper
