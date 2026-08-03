#pragma once

#ifndef BALL_TASK_LABEL
#define BALL_TASK_LABEL "TASK4"
#endif

#ifndef BALL_TASK_PREVIEW_NAME
#define BALL_TASK_PREVIEW_NAME "ball2-task4-velocity"
#endif

#ifndef BALL_TASK_LOG_EVENT
#define BALL_TASK_LOG_EVENT "task4_balance"
#endif

#ifndef BALL_TASK_QUESTION_NUMBER
#define BALL_TASK_QUESTION_NUMBER 4
#endif

// task4_app.hpp
// ============================================================================
// 第4问完整主循环：高速摄像头 -> 钢球位置/速度外环 -> 目标水管角度
// -> 目标编码器轴位 -> 电机位置/速度/加速度串级环 -> ZDT 0xF6速度命令。
// ============================================================================

#include "steel_ball_vision.hpp"
#include "latest_frame_capture.hpp"
#include "balance_control.hpp"
#include "task4_balance_control.hpp"
#include "vehicle_motion_feedforward.hpp"
#include "preview_window.hpp"
#include "terminal_key_input.hpp"
#include "udp_video_streamer.hpp"
#include "zdt_stepper_uart.hpp"
#include "contest_control_ipc.hpp"

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace ball_stepper {

class Task4ControlInbox final :
    public contest_control::EventHandler,
    public contest_control::TelemetryHandler,
    public VehicleEncoderSource {
    std::vector<contest_control::Frame> pendingEvents_;
    std::deque<VehicleEncoderSample> encoderSamples_;

    bool pushEvent(const contest_control::Frame& frame)
    {
        if (frame.question != BALL_TASK_QUESTION_NUMBER) return false;
        pendingEvents_.push_back(frame);
        return true;
    }

public:
    bool selectQuestion(const contest_control::Frame& frame) override
    {
        return pushEvent(frame);
    }
    bool executeQuestion(const contest_control::Frame& frame) override
    {
        return pushEvent(frame);
    }
    bool setMotorEnabled(
        bool, const contest_control::Frame& frame) override
    {
        return pushEvent(frame);
    }
    bool returnToOrigin(const contest_control::Frame& frame) override
    {
        return pushEvent(frame);
    }
    bool setCurrentPositionAsOrigin(
        const contest_control::Frame& frame) override
    {
        return pushEvent(frame);
    }

    bool handleTelemetry(
        const contest_control::TelemetryFrame& frame) override
    {
        if (frame.question != BALL_TASK_QUESTION_NUMBER) return false;
        VehicleEncoderSample sample;
        sample.speedUnits =
            (static_cast<double>(frame.leftSpeed50ms) +
             static_cast<double>(frame.rightSpeed50ms)) * 0.5;
        sample.sampleTime = secondsNow();
        sample.sequence = frame.sequence;
        encoderSamples_.push_back(sample);
        while (encoderSamples_.size() > 64) encoderSamples_.pop_front();
        return true;
    }

    bool poll(VehicleEncoderSample& sample) override
    {
        if (encoderSamples_.empty()) return false;
        sample = encoderSamples_.front();
        encoderSamples_.pop_front();
        return true;
    }

    std::vector<contest_control::Frame> takeEvents()
    {
        std::vector<contest_control::Frame> events;
        events.swap(pendingEvents_);
        return events;
    }
};

inline bool validateTask4Config(const AppConfig& config)
{
    if (config.cameraWidth <= 0 || config.cameraHeight <= 0 ||
        config.cameraFps <= 0 || config.roi.width <= 0 ||
        config.roi.height <= 0 || !config.axisConfigured ||
        cv::norm(config.axisRight - config.axisLeft) < 30.0 ||
        config.pipeLengthCm <= 0.0) {
        std::fprintf(stderr, "invalid camera, ROI or pipe-axis parameter\n");
        return false;
    }

    if (config.useThreePointPositionCalibration &&
        (cv::norm(config.centerCalibrationPoint -
                  config.minus5CalibrationPoint) < 10.0 ||
         cv::norm(config.plus5CalibrationPoint -
                  config.centerCalibrationPoint) < 10.0 ||
         config.positionCalibrationOffsetCm <= 0.0)) {
        std::fprintf(stderr, "invalid three-point position calibration\n");
        return false;
    }

    if (!std::isfinite(config.task4TargetCm) ||
        config.task4EvaluationSeconds < 1.0 ||
        config.task4AllowedErrorCm <= 0.0 ||
        config.task4StartToleranceCm <= 0.0 ||
        config.task4StartToleranceCm > config.task4AllowedErrorCm ||
        config.task4StartSpeedCmS < 0.0 ||
        config.task4StartConfirmFrames < 1 ||
        config.task4Kp < 0.0 || config.task4Kd < 0.0 ||
        config.task4Ki < 0.0 ||
        config.task4IntegralLimitDeg < 0.0 ||
        config.task4IntegralZoneCm <= config.task4DeadbandCm ||
        config.task4IntegralSpeedLimitCmS <=
            config.task4StopSpeedCmS ||
        config.task4IntegralLeakSeconds <= 0.0 ||
        config.task4DeadbandCm < 0.0 ||
        config.task4StopSpeedCmS < 0.0 ||
        config.task4DriveAngleLimitDeg <= 0.0 ||
        config.task4BrakeAngleLimitDeg <
            config.task4DriveAngleLimitDeg ||
        config.task4BrakeAngleLimitDeg > config.maximumPipeAngleDeg ||
        std::abs(config.task4LevelTrimDeg) >
            config.task4DriveAngleLimitDeg ||
        config.task4AngleSlewDegS <= 0.0 ||
        config.task4LossFailureMs < config.lostHoldMs ||
        config.task4VehicleEncoderCruiseValue <= 0.0 ||
        config.task4VehicleEncoderMaximumAbsValue <
            config.task4VehicleEncoderCruiseValue ||
        std::abs(config.task4VehicleEncoderDirectionSign) != 1.0 ||
        config.task4VehicleSpeedFilterSeconds <= 0.0 ||
        config.task4VehicleAccelerationFilterSeconds <= 0.0 ||
        config.task4VehicleAccelerationDecaySeconds <= 0.0 ||
        config.task4VehicleAccelerationDeadbandUnitsS < 0.0 ||
        config.task4VehicleAccelerationLimitUnitsS <= 0.0 ||
        config.task4VehicleFeedforwardDegPerUnitS < 0.0 ||
        std::abs(config.task4VehicleAccelerationAngleSign) != 1.0 ||
        config.task4VehicleFeedforwardLimitDeg < 0.0 ||
        config.task4VehicleFeedforwardLimitDeg >
            config.task4DriveAngleLimitDeg ||
        config.task4VehicleInputTimeoutMs < 1 ||
        config.task4VehicleSampleMaximumGapMs < 2) {
        std::fprintf(stderr,
            "invalid " BALL_TASK_LABEL " balance parameter in main.cpp\n");
        return false;
    }

    if (config.speedFilterSeconds < 0.005 ||
        config.speedDifferenceFrames < 1 ||
        config.maximumPipeAngleDeg <= 0.0 ||
        config.crankRadiusMm <= 1.0 ||
        config.actuatorDistanceMm <= 10.0 ||
        config.pulsesPerRevolution <= 0 ||
        (config.motorSign != 1 && config.motorSign != -1)) {
        std::fprintf(stderr, "invalid estimator or mechanism parameter\n");
        return false;
    }

    for (std::size_t index = 1;
         index < config.calibrationPoints.size(); ++index) {
        if (config.calibrationPoints[index].pipeAngleDeg <=
            config.calibrationPoints[index - 1].pipeAngleDeg) {
            std::fprintf(stderr,
                "calibrationPoints angles must be strictly increasing\n");
            return false;
        }
    }

    if (config.motorEnabled &&
        ((!config.zeroOnStart && !config.absoluteEncoderHomeOnStart) ||
         config.serialPort.empty() || config.serialBaud <= 0 ||
         config.motorRpm <= 0 || config.motorSpeedSlopeRpmS < 1 ||
         config.motorCommandHz <= 0)) {
        std::fprintf(stderr, "invalid ZDT/UART safety parameter\n");
        return false;
    }
    return true;
}

inline bool task4FrameMatchesConfig(const cv::Mat& frame,
                                    const AppConfig& config)
{
    return !frame.empty() &&
           frame.cols == config.cameraWidth &&
           frame.rows == config.cameraHeight;
}

inline std::string task4FourccText(double rawFourcc)
{
    const int code = static_cast<int>(rawFourcc);
    char text[5] = {
        static_cast<char>(code & 0xFF),
        static_cast<char>((code >> 8) & 0xFF),
        static_cast<char>((code >> 16) & 0xFF),
        static_cast<char>((code >> 24) & 0xFF),
        '\0'
    };
    for (int index = 0; index < 4; ++index) {
        if (text[index] < 32 || text[index] > 126) text[index] = '?';
    }
    return std::string(text);
}

struct Task4LogPaths {
    std::string textPath;
    std::string csvPath;
};

inline Task4LogPaths makeTask4LogPaths(const AppConfig& config)
{
    namespace filesystem = std::filesystem;
    std::error_code error;
    const filesystem::path directory(config.runtimeLogDirectory);
    filesystem::create_directories(directory, error);
    if (error) return {};

    std::string event;
    for (const unsigned char character : config.runtimeLogEvent) {
        event.push_back(std::isalnum(character) || character == '_' ||
                        character == '-' ? static_cast<char>(character) : '_');
    }
    if (event.empty()) event = BALL_TASK_LOG_EVENT;

    const auto now = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<
        std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    const std::time_t calendarTime =
        std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#if defined(_WIN32)
    if (localtime_s(&localTime, &calendarTime) != 0) return {};
#else
    if (localtime_r(&calendarTime, &localTime) == nullptr) return {};
#endif

    char timestamp[40];
    std::snprintf(
        timestamp, sizeof(timestamp), "%04d%02d%02d_%02d%02d%02d_%03lld",
        localTime.tm_year + 1900, localTime.tm_mon + 1, localTime.tm_mday,
        localTime.tm_hour, localTime.tm_min, localTime.tm_sec,
        static_cast<long long>(milliseconds));
    const std::string stem = event + "_" + timestamp;
    return {
        (directory / (stem + ".log")).string(),
        (directory / (stem + ".csv")).string()
    };
}

class Task4RuntimeLog {
    bool enabled_ = false;
    std::FILE* textFile_ = nullptr;
    std::FILE* csvFile_ = nullptr;
    std::string textPath_;
    std::string csvPath_;
    double startTime_ = secondsNow();

public:
    explicit Task4RuntimeLog(const AppConfig& config)
        : enabled_(config.runtimeLogEnabled)
    {
        if (!enabled_) return;
        const Task4LogPaths paths = makeTask4LogPaths(config);
        textPath_ = paths.textPath;
        csvPath_ = paths.csvPath;
        if (!textPath_.empty()) textFile_ = std::fopen(textPath_.c_str(), "w");
        if (!csvPath_.empty()) csvFile_ = std::fopen(csvPath_.c_str(), "w");
        if (!textFile_ || !csvFile_) {
            std::fprintf(stderr,
                "WARNING: cannot create " BALL_TASK_LABEL " runtime logs\n");
        }
        if (csvFile_) {
            std::fputs(
                "elapsed_s,armed,finished,measured,locked,confidence,pixel_x,"
                "pixel_y,position_cm,error_cm,speed_cm_s,two_frame_speed_cm_s,"
                "vehicle_speed_raw_units,vehicle_speed_filtered_units,"
                "vehicle_accel_raw_units_s,vehicle_accel_filtered_units_s,"
                "vehicle_ff_direction,vehicle_ff_segment,"
                "vehicle_ff_map_input_units_s,vehicle_ff_mapped_deg,"
                "vehicle_ff_unclamped_deg,vehicle_sample_age_ms,"
                "vehicle_signal_fresh,ff_deg,p_deg,d_deg,i_deg,drive_deg,"
                "request_deg,applied_deg,"
                "motor_target_steps,motor_actual_steps,motor_target_rpm,"
                "motor_actual_rpm,motor_command_rpm,motor_wire_rpm,"
                "motor_acceleration_rpm_s,max_abs_error_cm,longest_lost_ms\n",
                csvFile_);
            std::fflush(csvFile_);
        }
    }

    ~Task4RuntimeLog()
    {
        if (textFile_) std::fclose(textFile_);
        if (csvFile_) std::fclose(csvFile_);
    }

    const std::string& textPath() const { return textPath_; }
    const std::string& csvPath() const { return csvPath_; }

    void write(const char* format, ...)
    {
        if (!enabled_) return;
        char message[900];
        va_list arguments;
        va_start(arguments, format);
        std::vsnprintf(message, sizeof(message), format, arguments);
        va_end(arguments);

        char line[1024];
        std::snprintf(line, sizeof(line), "[diag +%.3fs] %s\n",
                      secondsNow() - startTime_, message);
        std::fputs(line, stderr);
        std::fflush(stderr);
        if (textFile_) {
            std::fputs(line, textFile_);
            std::fflush(textFile_);
        }
    }

    void writeSample(bool armed,
                     bool finished,
                     const Result& result,
                     double positionCm,
                     double errorCm,
                     double speedCmS,
                     double rawTwoFrameSpeedCmS,
                     const VehicleMotionState& vehicle,
                     const Task4ControlOutput& control,
                     double requestedAngleDeg,
                     double appliedAngleDeg,
                     int motorTargetSteps,
                     const MotorLoopTelemetry& motor,
                     double maximumAbsErrorCm,
                     double longestLostMs)
    {
        if (!enabled_ || !csvFile_) return;
        const double pixelX = result.measured ? result.center.x : -1.0;
        const double pixelY = result.measured ? result.center.y : -1.0;
        std::fprintf(
            csvFile_,
            "%.6f,%d,%d,%d,%d,%.5f,%.3f,%.3f,%.5f,%+.5f,%+.5f,%+.5f,"
            "%+.6f,%+.6f,%+.6f,%+.6f,%d,%d,%+.6f,%+.6f,%+.6f,"
            "%.2f,%d,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,"
            "%d,%+.3f,%+.3f,"
            "%+.3f,%+.3f,%+.3f,%+.3f,%.5f,%.1f\n",
            secondsNow() - startTime_, armed ? 1 : 0, finished ? 1 : 0,
            result.measured ? 1 : 0, result.locked ? 1 : 0,
            result.confidence, pixelX, pixelY, positionCm, errorCm,
            speedCmS, rawTwoFrameSpeedCmS, vehicle.rawSpeedUnits,
            vehicle.filteredSpeedUnits, vehicle.rawAccelerationUnitsS,
            vehicle.filteredAccelerationUnitsS,
            vehicle.feedforwardMapDirection,
            vehicle.feedforwardMapSegment,
            vehicle.feedforwardMapInputUnitsS,
            vehicle.feedforwardMappedMagnitudeDeg,
            vehicle.feedforwardUnclampedAngleDeg,
            vehicle.sampleAgeMs,
            vehicle.signalFresh ? 1 : 0, control.feedforwardTermDeg,
            control.pTermDeg,
            control.dTermDeg, control.iTermDeg, control.driveAngleDeg,
            requestedAngleDeg, appliedAngleDeg, motorTargetSteps,
            motor.actualSteps, motor.targetSpeedRpm, motor.actualSpeedRpm,
            motor.commandSpeedRpm, motor.wireCommandSpeedRpm,
            motor.commandAccelerationRpmS, maximumAbsErrorCm, longestLostMs);
        std::fflush(csvFile_);
    }
};

inline int runTask4VelocityApp(
    const AppConfig& config,
    VehicleEncoderSource* vehicleEncoderSource = nullptr)
{
    if (!validateConfig(config) || !validateTask4Config(config)) return 1;

    Task4RuntimeLog diagnostics(config);
    diagnostics.write("RUN_LOG text=%s csv=%s",
        diagnostics.textPath().empty() ? "<unavailable>" :
            diagnostics.textPath().c_str(),
        diagnostics.csvPath().empty() ? "<unavailable>" :
            diagnostics.csvPath().c_str());
    diagnostics.write(
        "START motor=%d ppr=%d command_hz=%d max_rpm=%d slope=%d "
        "P=%.3f D=%.3f I=%.3f drive=%.3f brake=%.3f "
        "encoder_source=%d ff_maps=%zu/%zu interpolate=%d "
        "fallback_gain=%.6f ff_limit=%.3f sign=%+.0f cruise=%.1f",
        config.motorEnabled ? 1 : 0, config.pulsesPerRevolution,
        config.motorCommandHz, config.motorRpm, config.motorSpeedSlopeRpmS,
        config.task4Kp, config.task4Kd, config.task4Ki,
        config.task4DriveAngleLimitDeg, config.task4BrakeAngleLimitDeg,
        (config.controlIpcEnabled || vehicleEncoderSource != nullptr) ? 1 : 0,
        config.task4VehicleAccelerationFeedforwardMap.size(),
        config.task4VehicleBrakingFeedforwardMap.size(),
        config.task4VehicleFeedforwardInterpolate ? 1 : 0,
        config.task4VehicleFeedforwardDegPerUnitS,
        config.task4VehicleFeedforwardLimitDeg,
        config.task4VehicleAccelerationAngleSign,
        config.task4VehicleEncoderCruiseValue);
    if (config.cameraWidth != CAMERA_WIDTH ||
        config.cameraHeight != CAMERA_HEIGHT ||
        config.cameraFps != CAMERA_FPS) {
        std::fprintf(stderr,
            "camera config does not match BALL_CFG_CAMERA_* in main.cpp\n");
        return 1;
    }

    // ---------------- 摄像头初始化 ----------------
    cv::VideoCapture camera(config.cameraIndex, cv::CAP_V4L2);
    if (!camera.isOpened()) {
        std::fprintf(stderr, "cannot open camera %d\n", config.cameraIndex);
        return 1;
    }

    camera.set(cv::CAP_PROP_FOURCC,
        cv::VideoWriter::fourcc(
            config.cameraFourcc[0], config.cameraFourcc[1],
            config.cameraFourcc[2], config.cameraFourcc[3]));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, config.cameraWidth);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, config.cameraHeight);
    camera.set(cv::CAP_PROP_FPS, config.cameraFps);
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (config.disableAutofocus) camera.set(cv::CAP_PROP_AUTOFOCUS, 0.0);

    if (config.configureExposure && config.useManualExposure) {
        const bool modeSet = camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0);
        const bool exposureSet = camera.set(
            cv::CAP_PROP_EXPOSURE, config.exposureAbsolute);
        const double actualExposure = camera.get(cv::CAP_PROP_EXPOSURE);
        std::fprintf(stderr,
            "camera exposure: manual requested=%.1f actual=%.1f%s\n",
            config.exposureAbsolute, actualExposure,
            modeSet && exposureSet ? "" : " (driver rejected setting)");
    } else if (config.configureExposure) {
        const bool autoExposureSet =
            camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 3.0);
        std::fprintf(stderr, "camera exposure: automatic%s\n",
            autoExposureSet ? "" : " (driver rejected setting)");
    } else {
        std::fprintf(stderr,
            "camera exposure: preserved (OpenCV did not change it)\n");
    }

    std::fprintf(stderr, "actual camera: %.0fx%.0f %s %.2f fps\n",
        camera.get(cv::CAP_PROP_FRAME_WIDTH),
        camera.get(cv::CAP_PROP_FRAME_HEIGHT),
        task4FourccText(camera.get(cv::CAP_PROP_FOURCC)).c_str(),
        camera.get(cv::CAP_PROP_FPS));
    diagnostics.write(
        "CAMERA actual=%.0fx%.0f fourcc=%s fps=%.2f exposure=%.1f",
        camera.get(cv::CAP_PROP_FRAME_WIDTH),
        camera.get(cv::CAP_PROP_FRAME_HEIGHT),
        task4FourccText(camera.get(cv::CAP_PROP_FOURCC)).c_str(),
        camera.get(cv::CAP_PROP_FPS),
        camera.get(cv::CAP_PROP_EXPOSURE));

    cv::Mat frame;
    if (!camera.read(frame) || !task4FrameMatchesConfig(frame, config)) {
        std::fprintf(stderr,
            "camera output is not configured %dx%d; control refused\n",
            config.cameraWidth, config.cameraHeight);
        return 1;
    }

    LatestFrameCapture capture(camera);
    capture.start();
    uint64_t consumedCaptureSequence = 0;
    std::shared_ptr<const cv::Mat> capturedFrame;

    std::unique_ptr<UdpVideoStreamer> videoStreamer;
    if (config.videoStreamEnabled) {
        videoStreamer = std::make_unique<UdpVideoStreamer>(
            config.videoStreamHost,
            config.videoStreamPort,
            config.cameraWidth,
            config.cameraHeight,
            config.videoStreamFps,
            config.videoStreamBitrateKbps);
        if (!videoStreamer->start()) return 1;
    }

    TerminalKeyInput terminalKeys;
    if (config.terminalKeys) {
        if (terminalKeys.start()) {
            std::fprintf(stderr,
                "terminal keys ready: SPACE=start/abort, F=finish, "
                "R=reset, Q=exit\n");
        } else if (!config.gui && config.motorEnabled && !config.startArmed &&
                   !config.controlIpcEnabled) {
            std::fprintf(stderr,
                "headless paused mode needs an interactive terminal for keys\n");
            return 1;
        }
    }

    Task4ControlInbox controlInbox;
    contest_control::UdpFrameReceiver controlIpc;
    bool controlIpcActive = false;
    if (config.controlIpcEnabled) {
        if (controlIpc.openPort(config.controlIpcPort)) {
            controlIpcActive = true;
            std::fprintf(stderr,
                BALL_TASK_LABEL " control IPC ready: 127.0.0.1:%u\n",
                static_cast<unsigned>(config.controlIpcPort));
            diagnostics.write(
                "CONTROL_IPC ready port=%u",
                static_cast<unsigned>(config.controlIpcPort));
        } else {
            std::fprintf(stderr,
                BALL_TASK_LABEL " control IPC open failed: %s\n",
                controlIpc.lastError().c_str());
            diagnostics.write(
                "ERROR control IPC open failed: %s",
                controlIpc.lastError().c_str());
            if (config.controlIpcRequired) return 1;
        }
    }
    VehicleEncoderSource* activeVehicleEncoderSource =
        controlIpcActive ? static_cast<VehicleEncoderSource*>(&controlInbox) :
                           vehicleEncoderSource;

    // ---------------- ZDT启动 ----------------
    SerialPort serial;
    std::unique_ptr<EmmV5Motor> motor;
    std::unique_ptr<MotorCommander> commander;
    bool communicationOk = true;

    if (config.motorEnabled) {
        if (!serial.openPort(config.serialPort, config.serialBaud)) return 1;
        motor = std::make_unique<EmmV5Motor>(serial, config);

        ZdtDriverParameters driverParameters;
        if (!motor->readDriverParameters(driverParameters) ||
            !motor->configureDriverParameters(driverParameters)) {
            std::fprintf(stderr,
                "ZDT driver parameter query failed before startup\n");
            diagnostics.write("ERROR ZDT driver parameter query failed");
            return 1;
        }
        const bool immediateCommandAck =
            zdtResponseModeHasImmediateAck(driverParameters.responseMode);
        motor->setExpectCommandAck(immediateCommandAck);
        diagnostics.write(
            "ZDT_CONFIG motor_type=%u mstep=%u response=%u ack=%d "
            "command_ppr=%u",
            static_cast<unsigned>(driverParameters.motorType),
            static_cast<unsigned>(driverParameters.microstep),
            static_cast<unsigned>(driverParameters.responseMode),
            immediateCommandAck ? 1 : 0,
            static_cast<unsigned>(
                motor->positionCommandPulsesPerRevolution()));

        if (!motor->enable()) {
            std::fprintf(stderr, "ZDT enable command failed\n");
            diagnostics.write("ERROR ZDT enable command failed");
            return 1;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.enableSettleMs));
        if (!motor->stop()) {
            std::fprintf(stderr, "ZDT startup stop command failed\n");
            diagnostics.write("ERROR ZDT startup stop command failed");
            return 1;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.stopSettleMs));

        if (config.absoluteEncoderHomeOnStart) {
            ZdtHomingParameters homingParameters;
            if (!motor->readHomingParameters(homingParameters)) {
                std::fprintf(stderr,
                    "ZDT absolute homing parameters could not be read\n");
                diagnostics.write("ERROR cannot read absolute home config");
                motor->stop();
                return 1;
            }
            if (homingParameters.mode != ZdtHomingMode::Nearest ||
                homingParameters.velocityRpm !=
                    static_cast<uint16_t>(config.absoluteEncoderHomeRpm) ||
                homingParameters.powerOnAutomatic) {
                std::fprintf(stderr,
                    "ZDT absolute origin is unsafe; run "
                    "./motor_cli origin-set at LEVEL\n");
                diagnostics.write("ERROR unsafe absolute home configuration");
                motor->stop();
                return 1;
            }

            std::fprintf(stderr,
                "returning to stored absolute encoder zero at %d RPM...\n",
                config.absoluteEncoderHomeRpm);
            diagnostics.write("EVENT absolute encoder homing started");
            if (!motor->homeToStoredSingleTurnOrigin(
                    config.absoluteEncoderHomeTimeoutMs,
                    config.absoluteEncoderHomePollMs)) {
                std::fprintf(stderr,
                    "ZDT absolute encoder homing failed; control refused\n");
                diagnostics.write("ERROR absolute encoder homing failed");
                motor->stop();
                return 1;
            }
            diagnostics.write("EVENT absolute encoder homing completed");
        }

        if (!motor->clearPosition()) {
            std::fprintf(stderr, "ZDT clear-position command failed\n");
            diagnostics.write("ERROR ZDT clear-position command failed");
            motor->stop();
            return 1;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.zeroSettleMs));
        commander = std::make_unique<MotorCommander>(*motor, config);
        if (!commander->force(0)) {
            std::fprintf(stderr,
                "ZDT speed-mode initialization failed; check UART settings\n");
            diagnostics.write("ERROR initial 0 RPM cycle failed");
            motor->stop();
            return 1;
        }
        std::fprintf(stderr,
            "ZDT velocity mode ready: %s %d baud, address=%d; "
            "stored LEVEL is zero\n",
            config.serialPort.c_str(), config.serialBaud,
            config.motorAddress);
        diagnostics.write("ZDT_READY stored absolute LEVEL is logical zero");
    } else {
        std::fprintf(stderr,
            "DRY-RUN: motor disabled; commands are display only\n");
        diagnostics.write("DRY_RUN motor disabled");
    }

    if (capture.waitForNext(
            consumedCaptureSequence, capturedFrame, 300)) {
        frame = *capturedFrame;
    } else if (capture.failed()) {
        std::fprintf(stderr, "camera capture thread failed during startup\n");
        if (motor) motor->stop();
        return 1;
    }
    if (!task4FrameMatchesConfig(frame, config)) {
        std::fprintf(stderr,
            "camera resolution changed before control start\n");
        if (motor) motor->stop();
        return 1;
    }

    // ---------------- 视觉与中心保持控制 ----------------
    SteelBallDetector detector(
        config.roi,
        config.axisLeft,
        config.axisRight,
        config.axisConfigured,
        config.centerCalibrationPoint,
        config.useThreePointPositionCalibration);
    const PipeAxis pipeAxis(config);
    BallStateEstimator estimator(
        config.speedFilterSeconds, config.speedDifferenceFrames);
    Task4BalanceController controller(config);
    VehicleMotionFeedforward vehicleFeedforward(config);
    const MechanismModel mechanism(config);
    PreviewWindow preview(BALL_TASK_PREVIEW_NAME);
    if (config.gui) preview.start();

    if (config.csv) {
        std::printf(
            "frame,time,measured,position_cm,error_cm,speed_cm_s,"
            "vehicle_speed_units,vehicle_accel_units_s,ff_deg,"
            "p_deg,d_deg,i_deg,request_deg,applied_deg,motor_steps,"
            "motor_pos,motor_rpm,target_rpm,command_rpm,command_acc\n");
    }

    const double initialControlTime = secondsNow();
    bool armed = config.startArmed;
    bool hadMeasurement = false;
    bool measuredNow = false;
    bool startPositionReady = false;
    bool startSpeedReady = false;
    bool evaluationFinished = false;
    bool evaluationPassed = false;
    bool lossFailure = false;
    int readyFrames = 0;
    double runStartTime = armed ? initialControlTime : -1.0;
    double finishTime = -1.0;
    double lastMeasurementTime = -1.0;
    double previousLoopTime = initialControlTime;
    double positionCm = config.task4TargetCm;
    double errorCm = 0.0;
    double speedCmS = 0.0;
    double rawTwoFrameSpeedCmS = 0.0;
    double requestedAngleDeg = 0.0;
    double appliedAngleDeg = 0.0;
    double lastMeasuredFeedbackAngleDeg = 0.0;
    double maximumAbsErrorCm = 0.0;
    double longestLostMs = 0.0;
    int motorSteps = 0;
    MotorLoopTelemetry motorTelemetry;
    Task4ControlOutput controlOutput;
    VehicleMotionState vehicleMotion;
    vehicleFeedforward.reset(initialControlTime);

    if (armed) {
        controller.reset(initialControlTime);
        estimator.reset();
    }

    uint64_t sequence = 0;
    double nextVideoStreamTime = secondsNow();
    double nextRuntimeLogTime = secondsNow();
    double nextVehicleStatusTime = secondsNow();
    double nextPausedStatusTime = secondsNow();
    bool videoStreamFailureReported = false;
    bool chassisMotorEnabled = false;
    bool haveControlSequence = false;
    uint16_t lastControlSequence = 0;
    int controlFrames = 0;
    double controlFps = 0.0;
    double fpsStart = secondsNow();

    auto pauseTask4FromControl = [&](const char* reason) {
        armed = false;
        requestedAngleDeg = 0.0;
        controlOutput = {};
        controller.reset();
        estimator.reset();
        vehicleFeedforward.reset(secondsNow());
        readyFrames = 0;
        hadMeasurement = false;
        lastMeasurementTime = -1.0;
        speedCmS = 0.0;
        rawTwoFrameSpeedCmS = 0.0;
        diagnostics.write(
            "CONTROL_IPC " BALL_TASK_LABEL " paused reason=%s", reason);
    };

    auto returnOriginFromControl = [&]() {
        pauseTask4FromControl("return origin");
        requestedAngleDeg = 0.0;
        appliedAngleDeg = 0.0;
        motorSteps = 0;
        if (!motor || !commander) {
            diagnostics.write("CONTROL_IPC dry-run return origin");
            return true;
        }
        bool ok = commander->returnToZero(config.exitReturnTimeoutMs);
        if (!motor->stop()) ok = false;
        if (ok) {
            motorTelemetry = commander->telemetry();
            std::fprintf(stderr,
                BALL_TASK_LABEL " returned pipe to origin\n");
            diagnostics.write("CONTROL_IPC return origin completed");
        }
        return ok;
    };

    auto setOriginFromControl = [&]() {
        pauseTask4FromControl("set origin");
        requestedAngleDeg = 0.0;
        appliedAngleDeg = 0.0;
        motorSteps = 0;
        if (!motor) {
            diagnostics.write("CONTROL_IPC dry-run set origin");
            return true;
        }
        if (!motor->stop() || !motor->setSingleTurnOrigin(true)) return false;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.zeroSettleMs));
        if (!motor->clearPosition()) return false;
        if (commander && !commander->force(0)) return false;
        if (commander) motorTelemetry = commander->telemetry();
        std::fprintf(stderr,
            BALL_TASK_LABEL " stored current pipe position as origin\n");
        diagnostics.write("CONTROL_IPC set origin completed");
        return true;
    };

    if (armed) {
        std::fprintf(stderr,
            BALL_TASK_LABEL " AUTO BALANCE active: control starts on first ball "
            "measurement; SPACE=abort, F=finish, Q/ESC=exit\n");
    } else {
        std::fprintf(stderr,
            BALL_TASK_LABEL " PAUSED: put ball at O; SPACE=start/abort, F=finish, "
            "R=reset vision, Q/ESC=exit\n");
    }

    while (running.load()) {
        ++sequence;
        ++controlFrames;
        const double now = secondsNow();
        const double loopDt = std::clamp(
            now - previousLoopTime, 0.002, 0.05);
        previousLoopTime = now;

        bool serialExecuteRequested = false;
        bool serialFinishRequested = false;
        bool controlActionFailed = false;
        if (controlIpcActive) {
            std::size_t dispatchedEvents = 0;
            std::size_t dispatchedTelemetry = 0;
            if (!controlIpc.receiveAndDispatch(
                    controlInbox, &dispatchedEvents,
                    &controlInbox, &dispatchedTelemetry)) {
                std::fprintf(stderr,
                    BALL_TASK_LABEL " control IPC failed: %s\n",
                    controlIpc.lastError().c_str());
                diagnostics.write(
                    "ERROR control IPC receive/dispatch failed: %s",
                    controlIpc.lastError().c_str());
                communicationOk = false;
                break;
            }
        }

        for (const contest_control::Frame& controlFrame :
             controlInbox.takeEvents()) {
            if (haveControlSequence) {
                const uint16_t forward = static_cast<uint16_t>(
                    controlFrame.sequence - lastControlSequence);
                if (forward == 0) {
                    diagnostics.write(
                        "CONTROL_IPC duplicate sequence=%u ignored",
                        static_cast<unsigned>(controlFrame.sequence));
                    continue;
                }
                if (forward > 0x8000u) {
                    const bool controllerRestart =
                        controlFrame.event == contest_control::Event::Select &&
                        controlFrame.state == contest_control::State::Ready;
                    if (!controllerRestart) {
                        diagnostics.write(
                            "CONTROL_IPC stale sequence=%u last=%u ignored",
                            static_cast<unsigned>(controlFrame.sequence),
                            static_cast<unsigned>(lastControlSequence));
                        continue;
                    }
                }
            }
            haveControlSequence = true;
            lastControlSequence = controlFrame.sequence;

            diagnostics.write(
                "CONTROL_IPC RX event=%s state=%s sequence=%u flags=0x%02X",
                contest_control::eventName(controlFrame.event),
                contest_control::stateName(controlFrame.state),
                static_cast<unsigned>(controlFrame.sequence),
                static_cast<unsigned>(controlFrame.flags));

            switch (controlFrame.event) {
            case contest_control::Event::Select:
                pauseTask4FromControl("question selected");
                chassisMotorEnabled = controlFrame.motorEnabled();
                break;

            case contest_control::Event::Execute:
                if (controlFrame.state != contest_control::State::Running) {
                    diagnostics.write(
                        "CONTROL_IPC " BALL_TASK_LABEL
                        " EXECUTE rejected state=%s",
                        contest_control::stateName(controlFrame.state));
                } else if (armed) {
                    diagnostics.write(
                        "CONTROL_IPC " BALL_TASK_LABEL
                        " EXECUTE ignored; already running");
                } else {
                    chassisMotorEnabled = controlFrame.motorEnabled();
                    serialExecuteRequested = true;
                }
                break;

            case contest_control::Event::MotorToggle:
                chassisMotorEnabled = controlFrame.motorEnabled();
                diagnostics.write(
                    "CONTROL_IPC chassis_motor_enabled=%d",
                    chassisMotorEnabled ? 1 : 0);
                if (!chassisMotorEnabled &&
                    controlFrame.state == contest_control::State::Sent) {
                    serialFinishRequested = true;
                }
                break;

            case contest_control::Event::ReturnOrigin:
                if (!returnOriginFromControl()) controlActionFailed = true;
                break;

            case contest_control::Event::SetOrigin:
                if (!controlFrame.originSet()) {
                    diagnostics.write(
                        "CONTROL_IPC SET_ORIGIN rejected: flag is 0");
                } else {
                    if (!setOriginFromControl()) controlActionFailed = true;
                }
                break;
            }
            if (controlActionFailed) break;
        }
        if (controlActionFailed) {
            std::fprintf(stderr,
                BALL_TASK_LABEL
                " control action failed; stopping safely\n");
            communicationOk = false;
            break;
        }

        if (activeVehicleEncoderSource) {
            VehicleEncoderSample sample;
            for (int sampleCount = 0; sampleCount < 32; ++sampleCount) {
                if (!activeVehicleEncoderSource->poll(sample)) break;
                vehicleFeedforward.submitEncoderSample(sample);
            }
        }
        vehicleMotion = vehicleFeedforward.update(now);

        const Result result = detector.update(frame);
        measuredNow = result.measured;

        if (result.measured) {
            positionCm = pipeAxis.toCentimeters(result.center);
            estimator.update(positionCm, now);
            speedCmS = estimator.speedCmS();
            rawTwoFrameSpeedCmS = estimator.rawTwoFrameSpeedCmS();
            errorCm = positionCm - config.task4TargetCm;
            lastMeasurementTime = now;
            hadMeasurement = true;
            startPositionReady =
                std::abs(errorCm) <= config.task4StartToleranceCm;
            startSpeedReady =
                std::abs(speedCmS) <= config.task4StartSpeedCmS;

            if (!armed) {
                if (startPositionReady && startSpeedReady) {
                    ++readyFrames;
                } else {
                    readyFrames = 0;
                }
                requestedAngleDeg = 0.0;
                controlOutput = {};
            } else {
                controlOutput = controller.update(
                    errorCm, speedCmS,
                    vehicleMotion.feedforwardAngleDeg, now);
                requestedAngleDeg = controlOutput.angleDeg;
                lastMeasuredFeedbackAngleDeg =
                    requestedAngleDeg - controlOutput.feedforwardTermDeg;

                if (!evaluationFinished) {
                    maximumAbsErrorCm = std::max(
                        maximumAbsErrorCm, std::abs(errorCm));
                }
            }
        } else {
            controlOutput.feedforwardTermDeg =
                vehicleMotion.feedforwardAngleDeg;
            readyFrames = 0;
            startPositionReady = false;
            startSpeedReady = false;
            controller.onMeasurementLost();

            if (armed && hadMeasurement && lastMeasurementTime >= 0.0) {
                const double lostMs =
                    (now - lastMeasurementTime) * 1000.0;
                if (!evaluationFinished) {
                    longestLostMs = std::max(longestLostMs, lostMs);
                    if (lostMs >= config.task4LossFailureMs) {
                        lossFailure = true;
                    }
                }

                if (lostMs <= config.lostHoldMs) {
                    requestedAngleDeg = std::clamp(
                        vehicleMotion.feedforwardAngleDeg +
                            lastMeasuredFeedbackAngleDeg,
                        -config.task4BrakeAngleLimitDeg,
                         config.task4BrakeAngleLimitDeg);
                } else if (lostMs < config.lostNeutralMs) {
                    const double ratio =
                        (lostMs - config.lostHoldMs) /
                        std::max(1, config.lostNeutralMs -
                                       config.lostHoldMs);
                    requestedAngleDeg = std::clamp(
                        vehicleMotion.feedforwardAngleDeg +
                            lastMeasuredFeedbackAngleDeg * (1.0 - ratio),
                        -config.task4BrakeAngleLimitDeg,
                         config.task4BrakeAngleLimitDeg);
                } else {
                    requestedAngleDeg =
                        vehicleMotion.feedforwardAngleDeg;
                    speedCmS = 0.0;
                    rawTwoFrameSpeedCmS = 0.0;
                    estimator.onMeasurementLost();
                }
            } else {
                requestedAngleDeg = 0.0;
            }
        }

        if (!armed) requestedAngleDeg = 0.0;

        appliedAngleDeg = approach(
            appliedAngleDeg,
            requestedAngleDeg,
            config.task4AngleSlewDegS * loopDt);
        motorSteps = mechanism.angleToSteps(appliedAngleDeg);

        if (commander &&
            !commander->update(motorSteps, millisecondsNow())) {
            std::fprintf(stderr,
                "ZDT communication failed; stopping control loop\n");
            communicationOk = false;
            running.store(false);
        } else if (commander) {
            motorTelemetry = commander->telemetry();
        }

        if (armed && !evaluationFinished &&
            now - runStartTime >= config.task4EvaluationSeconds) {
            finishTime = now;
            evaluationFinished = true;
            evaluationPassed =
                maximumAbsErrorCm <= config.task4AllowedErrorCm &&
                !lossFailure;
            std::fprintf(stderr,
                BALL_TASK_LABEL " %.0fs CHECK: %s, max_error=%.3f cm, "
                "longest_lost=%.0f ms\n",
                config.task4EvaluationSeconds,
                evaluationPassed ? "PASS" : "FAIL",
                maximumAbsErrorCm, longestLostMs);
            diagnostics.write(
                "RESULT %s elapsed=%.3f max_error_cm=%.3f longest_lost_ms=%.0f",
                evaluationPassed ? "PASS" : "FAIL",
                finishTime - runStartTime, maximumAbsErrorCm, longestLostMs);
        }

        if (now >= nextRuntimeLogTime) {
            diagnostics.writeSample(
                armed, evaluationFinished, result, positionCm, errorCm,
                speedCmS, rawTwoFrameSpeedCmS, vehicleMotion, controlOutput,
                requestedAngleDeg, appliedAngleDeg, motorSteps,
                motorTelemetry, maximumAbsErrorCm, longestLostMs);
            nextRuntimeLogTime = now +
                config.runtimeLogIntervalMs / 1000.0;
        }

        if (now >= nextVehicleStatusTime) {
            diagnostics.write(
                "CAR_ENCODER source=%d fresh=%d age=%.0fms "
                "raw_v=%+.2f filt_v=%+.2f raw_a=%+.1f filt_a=%+.1f "
                "dir=%+d segment=%d map_input=%.1f mapped=%.3fdeg "
                "ff_raw=%+.3fdeg ff=%+.3fdeg request=%+.3fdeg",
                activeVehicleEncoderSource ? 1 : 0,
                vehicleMotion.signalFresh ? 1 : 0,
                vehicleMotion.sampleAgeMs,
                vehicleMotion.rawSpeedUnits,
                vehicleMotion.filteredSpeedUnits,
                vehicleMotion.rawAccelerationUnitsS,
                vehicleMotion.filteredAccelerationUnitsS,
                vehicleMotion.feedforwardMapDirection,
                vehicleMotion.feedforwardMapSegment,
                vehicleMotion.feedforwardMapInputUnitsS,
                vehicleMotion.feedforwardMappedMagnitudeDeg,
                vehicleMotion.feedforwardUnclampedAngleDeg,
                vehicleMotion.feedforwardAngleDeg,
                requestedAngleDeg);
            nextVehicleStatusTime = now + 0.5;
        }

        if (!armed && now >= nextPausedStatusTime) {
            const char* reason = !measuredNow ? "BALL_NOT_FOUND" :
                !startPositionReady ? "OUTSIDE_START_WINDOW" :
                !startSpeedReady ? "BALL_MOVING" :
                readyFrames < config.task4StartConfirmFrames ?
                    "CONFIRMING" : "READY_PRESS_SPACE";
            diagnostics.write(
                "WAIT reason=%s measured=%d error=%+.3fcm allowed=+/-%.3fcm "
                "speed=%+.3fcm/s speed_limit=%.3f ready=%d/%d "
                "encoder_source=%d encoder_fresh=%d car_v=%+.2funit "
                "car_a=%+.1funit/s seg=%d map=%.3fdeg ff=%+.3fdeg "
                "video_sent=%llu video_failed=%d",
                reason, measuredNow ? 1 : 0, errorCm,
                config.task4StartToleranceCm, speedCmS,
                config.task4StartSpeedCmS, readyFrames,
                config.task4StartConfirmFrames,
                activeVehicleEncoderSource ? 1 : 0,
                vehicleMotion.signalFresh ? 1 : 0,
                vehicleMotion.filteredSpeedUnits,
                vehicleMotion.filteredAccelerationUnitsS,
                vehicleMotion.feedforwardMapSegment,
                vehicleMotion.feedforwardMappedMagnitudeDeg,
                vehicleMotion.feedforwardAngleDeg,
                static_cast<unsigned long long>(
                    videoStreamer ? videoStreamer->sentFrames() : 0),
                videoStreamer && videoStreamer->failed() ? 1 : 0);
            nextPausedStatusTime = now + 1.0;
        }

        if (config.csv) {
            std::printf(
                "%llu,%.6f,%d,%.4f,%+.4f,%+.4f,%+.5f,%+.5f,%+.5f,"
                "%+.5f,%+.5f,%+.5f,%+.5f,%+.5f,%d,%.3f,%.3f,"
                "%.3f,%.3f,%.3f\n",
                static_cast<unsigned long long>(sequence), now,
                result.measured ? 1 : 0,
                positionCm, errorCm, speedCmS,
                vehicleMotion.filteredSpeedUnits,
                vehicleMotion.filteredAccelerationUnitsS,
                controlOutput.feedforwardTermDeg,
                controlOutput.pTermDeg,
                controlOutput.dTermDeg,
                controlOutput.iTermDeg,
                requestedAngleDeg, appliedAngleDeg, motorSteps,
                motorTelemetry.actualSteps,
                motorTelemetry.actualSpeedRpm,
                motorTelemetry.targetSpeedRpm,
                motorTelemetry.commandSpeedRpm,
                motorTelemetry.commandAccelerationRpmS);
            std::fflush(stdout);
        }

        if (now - fpsStart >= 1.0) {
            controlFps = controlFrames / (now - fpsStart);
            controlFrames = 0;
            fpsStart = now;
        }

        int key = config.gui ? preview.consumeKey() : -1;
        if (key < 0 && config.terminalKeys) {
            key = terminalKeys.consumeKey();
        }
        if (serialExecuteRequested) key = ' ';
        if (serialFinishRequested) key = 'f';

        const bool previewRequested =
            config.gui &&
            sequence % static_cast<uint64_t>(config.previewEveryNFrames) == 0 &&
            preview.canAcceptFrame();

        bool streamRequested = false;
        if (videoStreamer && now >= nextVideoStreamTime) {
            const double streamInterval =
                1.0 / static_cast<double>(config.videoStreamFps);
            nextVideoStreamTime = now + streamInterval;
            streamRequested = videoStreamer->canAcceptFrame();
        }

        if (videoStreamer && videoStreamer->failed() &&
            !videoStreamFailureReported) {
            videoStreamFailureReported = true;
            std::fprintf(stderr,
                "WARNING: UDP video stopped; control remains active\n");
        }

        if (previewRequested || streamRequested) {
            cv::Mat displayFrame = frame.clone();

            if (config.drawPipeDetectionArea) {
                const cv::Rect pipeArea = boundedVisionRect(
                    config.pipeDisplayArea, displayFrame.size());
                if (pipeArea.width > 0 && pipeArea.height > 0) {
                    cv::rectangle(displayFrame, pipeArea,
                                  {0, 220, 255}, 2, cv::LINE_AA);
                }
            }

            if (result.measured) {
                drawBall(displayFrame, result.center, result.radius,
                         cv::Scalar(0, 255, 0));
            } else {
                char visionText[160];
                std::snprintf(visionText, sizeof(visionText),
                    "%s H=%d D=%d V=%d MISS=%d",
                    result.locked ? "BALL LOST" : "SEARCHING",
                    result.houghCandidates,
                    result.darkBlobCandidates,
                    result.validCandidates,
                    result.missStreak);
                cv::putText(displayFrame, visionText, {10, 28},
                            cv::FONT_HERSHEY_SIMPLEX, 0.62,
                            {0, 0, 255}, 2, cv::LINE_AA);
            }

            cv::drawMarker(displayFrame,
                           pipeAxis.targetPoint(config.task4TargetCm),
                           {255, 0, 255}, cv::MARKER_CROSS,
                           18, 2, cv::LINE_AA);
            const cv::Point leftLimit = pipeAxis.targetPoint(
                config.task4TargetCm - config.task4AllowedErrorCm);
            const cv::Point rightLimit = pipeAxis.targetPoint(
                config.task4TargetCm + config.task4AllowedErrorCm);
            cv::line(displayFrame,
                     {leftLimit.x, leftLimit.y - 16},
                     {leftLimit.x, leftLimit.y + 16},
                     {0, 200, 255}, 1, cv::LINE_AA);
            cv::line(displayFrame,
                     {rightLimit.x, rightLimit.y - 16},
                     {rightLimit.x, rightLimit.y + 16},
                     {0, 200, 255}, 1, cv::LINE_AA);

            const double elapsed = armed ?
                ((evaluationFinished ? finishTime : now) - runStartTime) : 0.0;
            char line[220];
            if (!armed) {
                const char* waitText = !measuredNow ? "WAIT BALL" :
                    !startPositionReady ? "MOVE BALL INTO +/-1CM" :
                    !startSpeedReady ? "WAIT BALL STOP" :
                    readyFrames < config.task4StartConfirmFrames ?
                        "CONFIRMING" : "READY - PRESS SPACE";
                std::snprintf(line, sizeof(line),
                    BALL_TASK_LABEL " %s ready=%d/%d",
                    waitText, readyFrames, config.task4StartConfirmFrames);
            } else {
                std::snprintf(line, sizeof(line),
                    BALL_TASK_LABEL " %s time=%.2fs",
                    evaluationFinished ?
                        (evaluationPassed ? "PASS/HOLD" : "FAIL/HOLD") :
                        "BALANCE",
                    elapsed);
            }
            cv::putText(displayFrame, line, {10, 52},
                        cv::FONT_HERSHEY_SIMPLEX, 0.50,
                        !armed && readyFrames >=
                            config.task4StartConfirmFrames ?
                                cv::Scalar(0, 255, 0) :
                                (armed ? cv::Scalar(0, 255, 255)
                                       : cv::Scalar(0, 165, 255)),
                        2, cv::LINE_AA);

            std::snprintf(line, sizeof(line),
                "pos=%.3f err=%+.3fcm v=%+.2fcm/s max=%.3fcm",
                positionCm, errorCm, speedCmS, maximumAbsErrorCm);
            cv::putText(displayFrame, line, {10, 76},
                        cv::FONT_HERSHEY_SIMPLEX, 0.46,
                        std::abs(errorCm) <= config.task4AllowedErrorCm
                            ? cv::Scalar(0, 255, 0)
                            : cv::Scalar(0, 0, 255),
                        1, cv::LINE_AA);

            std::snprintf(line, sizeof(line),
                "%s FF=%+.3f P=%+.3f D=%+.3f I=%+.3f req=%+.3f pipe=%+.3f",
                controlOutput.integralWindowActive ? "PDI" : "PD",
                controlOutput.feedforwardTermDeg,
                controlOutput.pTermDeg,
                controlOutput.dTermDeg,
                controlOutput.iTermDeg,
                requestedAngleDeg, appliedAngleDeg);
            cv::putText(displayFrame, line, {10, 100},
                        cv::FONT_HERSHEY_SIMPLEX, 0.44,
                        {0, 255, 255}, 1, cv::LINE_AA);

            std::snprintf(line, sizeof(line),
                "CAR src=%d fresh=%d v=%+.1f a=%+.1f seg=%d map=%.3f",
                activeVehicleEncoderSource ? 1 : 0,
                vehicleMotion.signalFresh ? 1 : 0,
                vehicleMotion.filteredSpeedUnits,
                vehicleMotion.filteredAccelerationUnitsS,
                vehicleMotion.feedforwardMapSegment,
                vehicleMotion.feedforwardMappedMagnitudeDeg);
            cv::putText(displayFrame, line, {10, 124},
                        cv::FONT_HERSHEY_SIMPLEX, 0.43,
                        vehicleMotion.signalFresh ?
                            cv::Scalar(0, 255, 0) :
                            cv::Scalar(0, 165, 255),
                        1, cv::LINE_AA);

            std::snprintf(line, sizeof(line),
                "M tgt=%+d pos=%+.1f rpm=%+.1f cmd=%+.1f acc=%+.1f",
                motorSteps,
                motorTelemetry.actualSteps,
                motorTelemetry.actualSpeedRpm,
                motorTelemetry.commandSpeedRpm,
                motorTelemetry.commandAccelerationRpmS);
            cv::putText(displayFrame, line, {10, 148},
                        cv::FONT_HERSHEY_SIMPLEX, 0.43,
                        {220, 220, 220}, 1, cv::LINE_AA);

            char sourceText[190];
            const char* visionSource = !result.measured ? "HOLD" :
                (result.fused ? "FUSED" :
                 (result.contourFallback ? "BLOB" : "HOUGH"));
            std::snprintf(sourceText, sizeof(sourceText),
                "VISION=%s CTRL=%.0f CAP=%.0f",
                visionSource,
                controlFps,
                capture.measuredFps());
            cv::putText(displayFrame, sourceText, {10, 172},
                        cv::FONT_HERSHEY_SIMPLEX, 0.43,
                        result.measured ? cv::Scalar(0, 255, 0)
                                        : cv::Scalar(0, 165, 255),
                        1, cv::LINE_AA);

            if (previewRequested && streamRequested) {
                cv::Mat streamFrame = displayFrame;
                preview.publish(std::move(displayFrame));
                videoStreamer->publish(std::move(streamFrame));
            } else if (previewRequested) {
                preview.publish(std::move(displayFrame));
            } else {
                videoStreamer->publish(std::move(displayFrame));
            }
        }

        if (key == 27 || key == 'q' || key == 'Q') break;

        if (key == ' ') {
            if (armed) {
                armed = false;
                requestedAngleDeg = 0.0;
                controller.reset();
                estimator.reset();
                readyFrames = 0;
                std::fprintf(stderr,
                    BALL_TASK_LABEL
                    " aborted; PAUSED; pipe returning to level\n");
            } else if (!measuredNow ||
                       readyFrames < config.task4StartConfirmFrames) {
                std::fprintf(stderr,
                    BALL_TASK_LABEL
                    " start refused: measured=%d error=%+.3fcm "
                    "allowed=+/-%.3fcm speed=%+.3fcm/s limit=%.3f "
                    "ready=%d/%d\n",
                    measuredNow ? 1 : 0, errorCm,
                    config.task4StartToleranceCm, speedCmS,
                    config.task4StartSpeedCmS, readyFrames,
                    config.task4StartConfirmFrames);
            } else {
                armed = true;
                evaluationFinished = false;
                evaluationPassed = false;
                lossFailure = false;
                maximumAbsErrorCm = std::abs(errorCm);
                longestLostMs = 0.0;
                runStartTime = now;
                finishTime = -1.0;
                controller.reset(now);
                estimator.reset();
                lastMeasurementTime = now;
                lastMeasuredFeedbackAngleDeg = 0.0;
                std::fprintf(stderr,
                    BALL_TASK_LABEL
                    " BALANCE started; start the car now\n");
            }
        }

        if (armed && !evaluationFinished &&
            (key == 'f' || key == 'F')) {
            finishTime = now;
            evaluationFinished = true;
            const double elapsed = finishTime - runStartTime;
            evaluationPassed =
                elapsed <= config.task4EvaluationSeconds &&
                maximumAbsErrorCm <= config.task4AllowedErrorCm &&
                !lossFailure;
            std::fprintf(stderr,
                BALL_TASK_LABEL
                " manual finish: %s time=%.3fs max_error=%.3fcm\n",
                evaluationPassed ? "PASS" : "FAIL",
                elapsed, maximumAbsErrorCm);
        }

        if (!armed && (key == 'r' || key == 'R')) {
            detector.reset(
                config.centerCalibrationPoint,
                config.useThreePointPositionCalibration);
            estimator.reset();
            controller.reset();
            hadMeasurement = false;
            lastMeasurementTime = -1.0;
            readyFrames = 0;
            std::fprintf(stderr,
                "vision reset; put ball at O and wait for green circle\n");
        }

        if (!running.load()) break;
        while (running.load() &&
               !capture.waitForNext(
                   consumedCaptureSequence, capturedFrame, 100)) {
            if (capture.failed()) {
                std::fprintf(stderr, "camera capture thread failed\n");
                communicationOk = false;
                running.store(false);
                break;
            }
        }
        if (!running.load()) break;
        frame = *capturedFrame;
        if (!task4FrameMatchesConfig(frame, config)) {
            std::fprintf(stderr,
                "camera resolution changed; motor control stopped\n");
            communicationOk = false;
            break;
        }
    }

    // ---------------- 安全退出 ----------------
    capture.stop();
    if (commander && motor) {
        std::fprintf(stderr, "returning pipe to LEVEL zero...\n");
        if (!commander->returnToZero(config.exitReturnTimeoutMs)) {
            communicationOk = false;
        }
        if (!motor->stop()) communicationOk = false;
    }

    preview.stop();
    terminalKeys.stop();
    if (controlIpcActive) {
        diagnostics.write(
            "CONTROL_IPC stats frames=%llu telemetry=%llu rejected=%llu",
            static_cast<unsigned long long>(controlIpc.acceptedFrames()),
            static_cast<unsigned long long>(
                controlIpc.acceptedTelemetryFrames()),
            static_cast<unsigned long long>(controlIpc.rejectedDatagrams()));
        controlIpc.closeSocket();
    }
    if (videoStreamer) {
        const uint64_t sentFrames = videoStreamer->sentFrames();
        videoStreamer->stop();
        std::fprintf(stderr,
            "UDP video stopped after %llu frames\n",
            static_cast<unsigned long long>(sentFrames));
    }
    if (!config.motorEnabled) {
        std::fprintf(stderr, "DRY-RUN finished; no ZDT command was sent\n");
    } else {
        std::fprintf(stderr,
            BALL_TASK_LABEL " ZDT velocity control finished\n");
    }
    return communicationOk ? 0 : 1;
}

} // namespace ball_stepper
