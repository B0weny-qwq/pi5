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

    // ---------------- 三点位置标定 ----------------
    // 普通模式默认认为像素与厘米线性对应。摄像头透视或安装偏心时，
    // O左侧5 cm和右侧5 cm可能对应不同像素长度，此时启用三点分段换算。
    bool useThreePointPositionCalibration = false;
    cv::Point2f minus5CalibrationPoint{};
    cv::Point2f centerCalibrationPoint{};
    cv::Point2f plus5CalibrationPoint{};
    double positionCalibrationOffsetCm = 5.0;

    // ---------------- 第4问：小车运动时钢球保持在O点 ----------------
    double task4TargetCm = 12.5;
    double task4EvaluationSeconds = 8.0;
    double task4AllowedErrorCm = 1.0;
    double task4StartToleranceCm = 0.50;
    double task4StartSpeedCmS = 1.0;
    int task4StartConfirmFrames = 6;

    // 对称PDI外环。P/I负责把球推回O点，D使用两帧速度提前制动。
    double task4Kp = 0.11;
    double task4Kd = 0.08;
    double task4Ki = 0.15;
    double task4IntegralZoneCm = 1.0;
    double task4IntegralSpeedLimitCmS = 1.0;
    double task4IntegralLimitDeg = 0.12;
    double task4IntegralLeakSeconds = 4.0;
    double task4LevelTrimDeg = 0.0;
    double task4DeadbandCm = 0.02;
    double task4StopSpeedCmS = 0.15;
    double task4DriveAngleLimitDeg = 0.55;
    double task4BrakeAngleLimitDeg = 0.65;
    double task4AngleSlewDegS = 8.0;
    int task4LossFailureMs = 180;

    // The chassis encoder is cleared every 50 ms. Each 20 Hz sample is already
    // a signed speed quantity; normal cruise is about 80 raw units.
    double task4VehicleEncoderCruiseValue = 80.0;
    double task4VehicleEncoderMaximumAbsValue = 150.0;
    double task4VehicleEncoderDirectionSign = 1.0;
    double task4VehicleSpeedFilterSeconds = 0.035;
    double task4VehicleAccelerationFilterSeconds = 0.060;
    double task4VehicleAccelerationDecaySeconds = 0.120;
    double task4VehicleAccelerationDeadbandUnitsS = 15.0;
    double task4VehicleAccelerationLimitUnitsS = 2000.0;
    double task4VehicleFeedforwardDegPerUnitS = 0.00040;
    double task4VehicleAccelerationAngleSign = -1.0;
    double task4VehicleFeedforwardLimitDeg = 0.50;
    int task4VehicleInputTimeoutMs = 140;
    int task4VehicleSampleMaximumGapMs = 120;

    // 钢球测速滤波和机构总安全倾角。
    double speedFilterSeconds = 0.020;
    int speedDifferenceFrames = 2;
    double maximumPipeAngleDeg = 1.0;

    // ---------------- 曲柄连杆与脉冲换算 ----------------
    double crankRadiusMm = 15.0;
    double actuatorDistanceMm = 250.0;
    int pulsesPerRevolution = 3200;
    int motorSign = 1;

    // 有两个以上点时优先查表插值；为空时使用曲柄连杆近似公式。
    // 实测标定表的实际数值也必须写在main.cpp中。
    std::vector<CalibrationPoint> calibrationPoints;

    // ---------------- 树莓派GPIO14/15与ZDT驱动器 ----------------
    bool motorEnabled = false;
    std::string serialPort = "/dev/serial0";
    int serialBaud = 115200;
    int motorAddress = 1;
    // Emm V5 0xF6物理加速度请求，单位RPM/s。
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
    // 0x35 reports integer RPM. Below 1 RPM, derive a filtered speed from the
    // high-resolution 0x36 encoder position instead of treating it as zero.
    double motorEncoderSpeedFilterSeconds = 0.04;

    // 当前位置在启动时清零，所以软限位也是相对水平零位的正负脉冲范围。
    int motorSoftLimitSteps = 130;
    int motorReplyTimeoutMs = 15;
    int motorMaximumConsecutiveFailures = 3;
    bool motorExpectCommandAck = true;

    // 首选：回到已保存的单圈绝对编码器零点，再清运行时坐标。
    bool absoluteEncoderHomeOnStart = false;
    int absoluteEncoderHomeRpm = 6;
    int absoluteEncoderHomeTimeoutMs = 5000;
    int absoluteEncoderHomePollMs = 40;

    // 旧路径：把任意启动位置声明为零点，不能与绝对归位同时开启。
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
    std::string runtimeLogEvent = "task4_balance";
    int runtimeLogIntervalMs = 100;
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

    if (!std::isfinite(config.positionCalibrationOffsetCm) ||
        config.positionCalibrationOffsetCm <= 0.0 ||
        config.positionCalibrationOffsetCm > config.pipeLengthCm * 0.5 ||
        !std::isfinite(config.task4TargetCm) ||
        config.task4TargetCm < 0.0 ||
        config.task4TargetCm > config.pipeLengthCm ||
        !std::isfinite(config.task4EvaluationSeconds) ||
        config.task4EvaluationSeconds < 1.0 ||
        !std::isfinite(config.task4AllowedErrorCm) ||
        config.task4AllowedErrorCm <= 0.0 ||
        !std::isfinite(config.task4StartToleranceCm) ||
        config.task4StartToleranceCm <= 0.0 ||
        config.task4StartToleranceCm > config.task4AllowedErrorCm ||
        !std::isfinite(config.task4StartSpeedCmS) ||
        config.task4StartSpeedCmS < 0.0 ||
        config.task4StartConfirmFrames < 1 ||
        !std::isfinite(config.task4Kp) || config.task4Kp < 0.0 ||
        !std::isfinite(config.task4Kd) || config.task4Kd < 0.0 ||
        !std::isfinite(config.task4Ki) || config.task4Ki < 0.0 ||
        !std::isfinite(config.task4IntegralZoneCm) ||
        config.task4IntegralZoneCm <= config.task4DeadbandCm ||
        !std::isfinite(config.task4IntegralSpeedLimitCmS) ||
        config.task4IntegralSpeedLimitCmS <= config.task4StopSpeedCmS ||
        !std::isfinite(config.task4IntegralLimitDeg) ||
        config.task4IntegralLimitDeg < 0.0 ||
        !std::isfinite(config.task4IntegralLeakSeconds) ||
        config.task4IntegralLeakSeconds <= 0.0 ||
        !std::isfinite(config.task4LevelTrimDeg) ||
        !std::isfinite(config.task4DeadbandCm) ||
        config.task4DeadbandCm < 0.0 ||
        !std::isfinite(config.task4StopSpeedCmS) ||
        config.task4StopSpeedCmS < 0.0 ||
        !std::isfinite(config.task4DriveAngleLimitDeg) ||
        config.task4DriveAngleLimitDeg <= 0.0 ||
        !std::isfinite(config.task4BrakeAngleLimitDeg) ||
        config.task4BrakeAngleLimitDeg < config.task4DriveAngleLimitDeg ||
        !std::isfinite(config.task4AngleSlewDegS) ||
        config.task4AngleSlewDegS <= 0.0 ||
        config.task4LossFailureMs < config.lostHoldMs ||
        !std::isfinite(config.task4VehicleEncoderCruiseValue) ||
        config.task4VehicleEncoderCruiseValue <= 0.0 ||
        !std::isfinite(config.task4VehicleEncoderMaximumAbsValue) ||
        config.task4VehicleEncoderMaximumAbsValue <
            config.task4VehicleEncoderCruiseValue ||
        !std::isfinite(config.task4VehicleEncoderDirectionSign) ||
        std::abs(config.task4VehicleEncoderDirectionSign) != 1.0 ||
        !std::isfinite(config.task4VehicleSpeedFilterSeconds) ||
        config.task4VehicleSpeedFilterSeconds <= 0.0 ||
        !std::isfinite(config.task4VehicleAccelerationFilterSeconds) ||
        config.task4VehicleAccelerationFilterSeconds <= 0.0 ||
        !std::isfinite(config.task4VehicleAccelerationDecaySeconds) ||
        config.task4VehicleAccelerationDecaySeconds <= 0.0 ||
        !std::isfinite(config.task4VehicleAccelerationDeadbandUnitsS) ||
        config.task4VehicleAccelerationDeadbandUnitsS < 0.0 ||
        !std::isfinite(config.task4VehicleAccelerationLimitUnitsS) ||
        config.task4VehicleAccelerationLimitUnitsS <= 0.0 ||
        !std::isfinite(config.task4VehicleFeedforwardDegPerUnitS) ||
        config.task4VehicleFeedforwardDegPerUnitS < 0.0 ||
        !std::isfinite(config.task4VehicleAccelerationAngleSign) ||
        std::abs(config.task4VehicleAccelerationAngleSign) != 1.0 ||
        !std::isfinite(config.task4VehicleFeedforwardLimitDeg) ||
        config.task4VehicleFeedforwardLimitDeg < 0.0 ||
        config.task4VehicleFeedforwardLimitDeg >
            config.task4DriveAngleLimitDeg ||
        config.task4VehicleInputTimeoutMs < 1 ||
        config.task4VehicleSampleMaximumGapMs < 2 ||
        config.speedFilterSeconds < 0.005 ||
        config.speedDifferenceFrames < 1 ||
        config.speedDifferenceFrames > 8 ||
        config.maximumPipeAngleDeg <= 0.0 ||
        config.maximumPipeAngleDeg > 10.0 ||
        config.task4BrakeAngleLimitDeg > config.maximumPipeAngleDeg ||
        std::abs(config.task4LevelTrimDeg) >
            config.task4DriveAngleLimitDeg) {
        std::fprintf(stderr, "invalid TASK 4 control parameter in main.cpp\n");
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
        config.motorReplyTimeoutMs > 100 ||
        config.motorMaximumConsecutiveFailures < 1 ||
        config.motorMaximumConsecutiveFailures > 20) {
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
    if (config.runtimeLogIntervalMs < 20 ||
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
