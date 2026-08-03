#pragma once

// stepper_app.hpp
// ============================================================================
// 本文件是整个“opencv8直接控制ZDT步进电机方案”的总调度器。
// 正常使用时不修改本文件，所有现场参数都在main.cpp。
//
// 完整数据流：
// 摄像头采集线程 -> SteelBallDetector -> PipeAxis厘米位置 -> BallStateEstimator速度
// -> Task3MotionController位置PDI角度 -> 倾角变化率限制 -> MechanismModel目标轴位
// -> MotorCommander位置/速度/加速度串级环 -> ZDT 0xF6速度命令 -> GPIO14/TXD。
// ============================================================================

#include "steel_ball_vision.hpp"
#include "balance_control.hpp"
#include "task3_sequence.hpp"
#include "task3_motion_control.hpp"
#include "latest_frame_capture.hpp"
#include "preview_window.hpp"
#include "terminal_key_input.hpp"
#include "udp_video_streamer.hpp"
#include "zdt_stepper_uart.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

namespace ball_stepper {

inline bool frameMatchesConfig(const cv::Mat& frame,
                               const AppConfig& config)
{
    // ROI、轴线和钢球像素尺寸都按固定分辨率标定，分辨率变化后不能继续控制。
    return !frame.empty() &&
           frame.cols == config.cameraWidth &&
           frame.rows == config.cameraHeight;
}

inline std::string cameraFourccText(double rawFourcc)
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

struct RuntimeLogPaths {
    std::string textPath;
    std::string csvPath;
};

inline std::string runtimeLogToken(const std::string& value)
{
    std::string token;
    token.reserve(value.size());
    for (const unsigned char character : value) {
        if (std::isalnum(character) || character == '_' ||
            character == '-') {
            token.push_back(static_cast<char>(character));
        } else {
            token.push_back('_');
        }
    }
    return token.empty() ? "run" : token;
}

inline RuntimeLogPaths makeRuntimeLogPaths(const AppConfig& config)
{
    namespace filesystem = std::filesystem;

    const filesystem::path directory(config.runtimeLogDirectory);
    std::error_code error;
    filesystem::create_directories(directory, error);
    if (error) return {};

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
    const std::string stem = runtimeLogToken(config.runtimeLogEvent) +
        "_" + timestamp;

    for (int suffix = 0; suffix < 1000; ++suffix) {
        const std::string suffixText = suffix == 0 ? "" :
            "_" + std::to_string(suffix);
        const filesystem::path textPath = directory /
            (stem + suffixText + ".log");
        const filesystem::path csvPath = directory /
            (stem + suffixText + ".csv");
        std::error_code existsError;
        const bool textExists = filesystem::exists(textPath, existsError);
        if (existsError) return {};
        const bool csvExists = filesystem::exists(csvPath, existsError);
        if (existsError) return {};
        if (!textExists && !csvExists) {
            return {textPath.string(), csvPath.string()};
        }
    }
    return {};
}

class RuntimeDiagnosticLog {
    bool enabled_ = false;
    std::FILE* textFile_ = nullptr;
    std::FILE* csvFile_ = nullptr;
    std::string textPath_;
    std::string csvPath_;
    double startTime_ = 0.0;

public:
    explicit RuntimeDiagnosticLog(const AppConfig& config)
        : enabled_(config.runtimeLogEnabled),
          startTime_(secondsNow())
    {
        if (!enabled_) return;
        const RuntimeLogPaths paths = makeRuntimeLogPaths(config);
        textPath_ = paths.textPath;
        csvPath_ = paths.csvPath;
        if (textPath_.empty() || csvPath_.empty()) {
            std::fprintf(stderr,
                "WARNING: cannot create runtime log directory %s\n",
                config.runtimeLogDirectory.c_str());
            return;
        }

        textFile_ = std::fopen(textPath_.c_str(), "w");
        if (!textFile_) {
            std::fprintf(stderr, "WARNING: cannot open runtime log %s\n",
                         textPath_.c_str());
        }
        csvFile_ = std::fopen(csvPath_.c_str(), "w");
        if (!csvFile_) {
            std::fprintf(stderr, "WARNING: cannot open runtime CSV %s\n",
                         csvPath_.c_str());
        } else {
            std::fputs(
                "elapsed_s,armed,center_ready_frames,ball_measured,"
                "ball_locked,confidence,pixel_x,pixel_y,position_cm,"
                "target_cm,error_cm,speed_cm_s,two_frame_speed_cm_s,"
                "p_angle_deg,d_angle_deg,i_angle_deg,base_limit_deg,"
                "breakaway_limit_deg,"
                "requested_angle_deg,breakaway_angle_deg,applied_angle_deg,"
                "motor_target_steps,"
                "motor_actual_steps,motor_target_rpm,motor_actual_rpm,"
                "motor_command_rpm,motor_wire_rpm,"
                "motor_acceleration_rpm_s\n",
                csvFile_);
            std::fflush(csvFile_);
        }
    }

    ~RuntimeDiagnosticLog()
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

    void writeControlSample(
        bool armed,
        int centerReadyFrames,
        bool ballMeasured,
        bool ballLocked,
        double confidence,
        double pixelX,
        double pixelY,
        double positionCm,
        double targetCm,
        double errorCm,
        double speedCmS,
        double rawTwoFrameSpeedCmS,
        double requestedAngleDeg,
        const Task3MotionCommand& motionCommand,
        double appliedAngleDeg,
        int motorTargetSteps,
        const MotorLoopTelemetry& motorTelemetry)
    {
        if (!enabled_ || !csvFile_) return;
        std::fprintf(
            csvFile_,
            "%.6f,%d,%d,%d,%d,%.5f,%.3f,%.3f,%.5f,%.5f,%+.5f,%+.5f,"
            "%+.5f,%+.6f,%+.6f,%+.6f,%.6f,%.6f,"
            "%+.6f,%+.6f,%+.6f,%d,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f\n",
            secondsNow() - startTime_, armed ? 1 : 0, centerReadyFrames,
            ballMeasured ? 1 : 0, ballLocked ? 1 : 0, confidence, pixelX,
            pixelY, positionCm, targetCm, errorCm, speedCmS,
            rawTwoFrameSpeedCmS,
            motionCommand.proportionalAngleDeg,
            motionCommand.derivativeAngleDeg,
            motionCommand.integralAngleDeg,
            motionCommand.baseOutputLimitDeg,
            motionCommand.breakawayOutputLimitDeg,
            requestedAngleDeg, motionCommand.breakawayAngleDeg,
            appliedAngleDeg,
            motorTargetSteps, motorTelemetry.actualSteps,
            motorTelemetry.targetSpeedRpm, motorTelemetry.actualSpeedRpm,
            motorTelemetry.commandSpeedRpm,
            motorTelemetry.wireCommandSpeedRpm,
            motorTelemetry.commandAccelerationRpmS);
        std::fflush(csvFile_);
    }
};

inline int runTask3App(const AppConfig& config)
{
    if (!validateConfig(config)) return 1;

    RuntimeDiagnosticLog diagnostics(config);
    diagnostics.write(
        "RUN_LOG text=%s csv=%s",
        diagnostics.textPath().empty() ? "<unavailable>" :
            diagnostics.textPath().c_str(),
        diagnostics.csvPath().empty() ? "<unavailable>" :
            diagnostics.csvPath().c_str());
    diagnostics.write(
        "START motor=%d port=%s baud=%d address=%d ppr=%d cmd_hz=%d "
        "max_rpm=%d slope=%d armed_at_start=%d",
        config.motorEnabled ? 1 : 0, config.serialPort.c_str(),
        config.serialBaud, config.motorAddress, config.pulsesPerRevolution,
        config.motorCommandHz, config.motorRpm,
        config.motorSpeedSlopeRpmS, config.startArmed ? 1 : 0);

    // 视觉头文件的编译期尺寸也来自main.cpp宏；这里再次防止两套数值不一致。
    if (config.cameraWidth != CAMERA_WIDTH ||
        config.cameraHeight != CAMERA_HEIGHT ||
        config.cameraFps != CAMERA_FPS) {
        std::fprintf(stderr,
            "camera config does not match BALL_CFG_CAMERA_* in main.cpp\n");
        diagnostics.write("ERROR camera compile-time configuration mismatch");
        return 1;
    }

    // ---------------- 摄像头初始化 ----------------
    cv::VideoCapture camera(config.cameraIndex, cv::CAP_V4L2);
    if (!camera.isOpened()) {
        std::fprintf(stderr, "cannot open camera %d\n", config.cameraIndex);
        diagnostics.write("ERROR cannot open camera index=%d", config.cameraIndex);
        return 1;
    }

    // 像素格式由main.cpp配置；缓冲区1减少控制使用过时画面的延迟。
    const bool fourccSet = camera.set(
        cv::CAP_PROP_FOURCC,
        cv::VideoWriter::fourcc(
            config.cameraFourcc[0], config.cameraFourcc[1],
            config.cameraFourcc[2], config.cameraFourcc[3]));
    const bool widthSet = camera.set(
        cv::CAP_PROP_FRAME_WIDTH, config.cameraWidth);
    const bool heightSet = camera.set(
        cv::CAP_PROP_FRAME_HEIGHT, config.cameraHeight);
    const bool fpsSet = camera.set(cv::CAP_PROP_FPS, config.cameraFps);
    const bool bufferSet = camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (!fourccSet || !widthSet || !heightSet || !fpsSet) {
        std::fprintf(stderr,
            "WARNING: camera driver rejected format/size/fps setting; "
            "actual values are checked below\n");
    }
    if (!bufferSet) {
        std::fprintf(stderr,
            "camera driver does not expose buffer-size control\n");
    }
    if (config.disableAutofocus) {
        camera.set(cv::CAP_PROP_AUTOFOCUS, 0.0);
    }

    if (config.configureExposure && config.useManualExposure) {
        // V4L2_CID_EXPOSURE_AUTO=1表示手动曝光；很多UVC摄像头的
        // CAP_PROP_EXPOSURE单位为0.1 ms。先关闭自动曝光，再设置具体时间。
        const bool manualModeSet =
            camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0);
        const bool exposureSet = camera.set(
            cv::CAP_PROP_EXPOSURE, config.exposureAbsolute);
        const double actualExposure = camera.get(cv::CAP_PROP_EXPOSURE);

        std::fprintf(stderr,
            "camera exposure: manual requested=%.1f actual=%.1f%s\n",
            config.exposureAbsolute, actualExposure,
            (manualModeSet && exposureSet) ? "" : " (driver rejected setting)");
    } else if (config.configureExposure) {
        // 只有main.cpp明确要求配置曝光时才切换自动模式。部分高速UVC
        // 摄像头在这一步会退回低帧率，因此比赛默认不进入本分支。
        const bool autoExposureSet =
            camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 3.0);
        std::fprintf(stderr, "camera exposure: automatic%s\n",
            autoExposureSet ? "" : " (driver rejected setting)");
    } else {
        // 保留摄像头在v4l2-ctl或出厂配置中的高速曝光状态。
        std::fprintf(stderr,
            "camera exposure: preserved (OpenCV did not change it)\n");
    }

    const std::string actualFourcc = cameraFourccText(
        camera.get(cv::CAP_PROP_FOURCC));
    std::fprintf(stderr, "actual camera: %.0fx%.0f %s %.2f fps\n",
                 camera.get(cv::CAP_PROP_FRAME_WIDTH),
                 camera.get(cv::CAP_PROP_FRAME_HEIGHT),
                 actualFourcc.c_str(),
                 camera.get(cv::CAP_PROP_FPS));

    cv::Mat frame;
    if (!camera.read(frame) || !frameMatchesConfig(frame, config)) {
        std::fprintf(stderr,
            "camera output is not configured %dx%d; motor control refused\n",
            config.cameraWidth, config.cameraHeight);
        return 1;
    }

    // 首帧验证通过后，后续camera.read全部交给独立采集线程。它只保留
    // 最新一帧，识别或X11显示偶尔变慢时不会继续控制旧画面。
    LatestFrameCapture capture(camera);
    capture.start();
    uint64_t consumedCaptureSequence = 0;
    std::shared_ptr<const cv::Mat> capturedFrame;

    std::unique_ptr<UdpVideoStreamer> videoStreamer;
    cv::Rect videoStreamArea;
    if (config.videoStreamEnabled) {
        const cv::Rect requestedStreamArea =
            config.pipeDisplayArea.width > 0 &&
            config.pipeDisplayArea.height > 0 ?
                config.pipeDisplayArea : config.roi;
        videoStreamArea = boundedVisionRect(requestedStreamArea, frame.size());
        const int alignedWidth = videoStreamArea.width & ~15;
        const int alignedHeight = videoStreamArea.height & ~15;
        videoStreamArea.x += (videoStreamArea.width - alignedWidth) / 2;
        videoStreamArea.y += (videoStreamArea.height - alignedHeight) / 2;
        videoStreamArea.width = alignedWidth;
        videoStreamArea.height = alignedHeight;
        if (videoStreamArea.width < 16 || videoStreamArea.height < 16) {
            std::fprintf(stderr, "video stream crop area is empty\n");
            return 1;
        }
        videoStreamer = std::make_unique<UdpVideoStreamer>(
            config.videoStreamHost,
            config.videoStreamPort,
            videoStreamArea.width,
            videoStreamArea.height,
            config.videoStreamFps,
            config.videoStreamBitrateKbps);
        if (!videoStreamer->start()) return 1;
    }

    TerminalKeyInput terminalKeys;
    if (config.terminalKeys) {
        if (terminalKeys.start()) {
            std::fprintf(stderr,
                "terminal keys ready: SPACE=start/abort, R=reset, Q=exit\n");
        } else if (!config.gui && config.motorEnabled && !config.startArmed) {
            std::fprintf(stderr,
                "headless paused mode needs an interactive terminal for keys\n");
            diagnostics.write(
                "ERROR terminal key input unavailable while startArmed=0");
            return 1;
        }
    }

    // ---------------- ZDT串口与电机启动 ----------------
    SerialPort serial;
    std::unique_ptr<EmmV5Motor> motor;
    std::unique_ptr<MotorCommander> commander;

    if (config.motorEnabled) {
        // 启动顺序：打开串口 -> 使能 -> 停止旧动作 -> 当前位置清零
        // -> 读取编码器位置/速度 -> 发送0 RPM速度命令。
        if (!serial.openPort(config.serialPort, config.serialBaud)) {
            diagnostics.write("ERROR cannot open ZDT serial port");
            return 1;
        }
        motor = std::make_unique<EmmV5Motor>(serial, config);

        ZdtDriverParameters driverParameters;
        if (!motor->readDriverParameters(driverParameters) ||
            !motor->configureDriverParameters(driverParameters)) {
            std::fprintf(stderr,
                "ZDT driver parameter query failed before startup\n");
            diagnostics.write(
                "ERROR ZDT driver parameter query failed before startup");
            return 1;
        }
        const bool immediateCommandAck =
            zdtResponseModeHasImmediateAck(driverParameters.responseMode);
        motor->setExpectCommandAck(immediateCommandAck);
        diagnostics.write(
            "ZDT_CONFIG motor_type=%u mstep=%u response=%u ack=%d command_ppr=%u",
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

        // 驱动器刚使能后先停止，防止继续执行断电前或上次程序留下的运动。
        if (!motor->stop()) {
            std::fprintf(stderr, "ZDT startup stop command failed\n");
            diagnostics.write("ERROR ZDT startup stop command failed");
            return 1;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.stopSettleMs));

        // validateConfig已要求motorEnabled时zeroOnStart=true。
        // 执行这一行之前，使用者必须保证水管真的处于机械水平位置。
        if (config.absoluteEncoderHomeOnStart) {
            ZdtHomingParameters homingParameters;
            if (!motor->readHomingParameters(homingParameters)) {
                std::fprintf(stderr,
                    "ZDT absolute homing parameters could not be read\n");
                diagnostics.write(
                    "ERROR ZDT absolute homing parameters could not be read");
                motor->stop();
                return 1;
            }
            diagnostics.write(
                "ABS_HOME_CONFIG mode=%u rpm=%u timeout_ms=%u power_on=%d",
                static_cast<unsigned>(homingParameters.mode),
                static_cast<unsigned>(homingParameters.velocityRpm),
                static_cast<unsigned>(homingParameters.timeoutMs),
                homingParameters.powerOnAutomatic ? 1 : 0);
            if (homingParameters.mode != ZdtHomingMode::Nearest ||
                homingParameters.velocityRpm !=
                    static_cast<uint16_t>(config.absoluteEncoderHomeRpm) ||
                homingParameters.powerOnAutomatic) {
                std::fprintf(stderr,
                    "ZDT absolute origin is not configured for safe startup; "
                    "run ./motor_cli origin-set at LEVEL\n");
                diagnostics.write(
                    "ERROR unsafe absolute home configuration; run origin-set");
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

        // Clear the high-resolution runtime coordinate only after the stored
        // absolute origin has been reached. Legacy startup zeroing remains an
        // explicit fallback and is disabled in the competition configuration.
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
                "ZDT speed-mode initialization failed; check UART, ID, baud and checksum\n");
            diagnostics.write(
                "ERROR ZDT initial 0 RPM query/command cycle failed");
            motor->stop();
            return 1;
        }

        uint8_t motorStatus = 0;
        uint8_t originStatus = 0;
        const bool motorStatusOk = motor->readMotorStatus(motorStatus);
        const bool originStatusOk = motor->readOriginStatus(originStatus);
        const MotorLoopTelemetry startupTelemetry = commander->telemetry();
        diagnostics.write(
            "ZDT_READY pulse=%.1f rpm=%.1f cmd=%.1f wire=%.1f "
            "motor_status=%s0x%02X enabled=%d arrived=%d stall=%d protect=%d "
            "origin_status=%s0x%02X encoder_ready=%d calibration_ready=%d",
            startupTelemetry.actualSteps, startupTelemetry.actualSpeedRpm,
            startupTelemetry.commandSpeedRpm,
            startupTelemetry.wireCommandSpeedRpm,
            motorStatusOk ? "" : "ERR:",
            static_cast<unsigned>(motorStatus),
            motorStatusOk && (motorStatus & 0x01) ? 1 : 0,
            motorStatusOk && (motorStatus & 0x02) ? 1 : 0,
            motorStatusOk && (motorStatus & 0x04) ? 1 : 0,
            motorStatusOk && (motorStatus & 0x08) ? 1 : 0,
            originStatusOk ? "" : "ERR:",
            static_cast<unsigned>(originStatus),
            originStatusOk && (originStatus & 0x01) ? 1 : 0,
            originStatusOk && (originStatus & 0x02) ? 1 : 0);

        std::fprintf(stderr,
            "ZDT velocity mode ready: %s %d baud, address=%d; stored LEVEL is zero\n",
            config.serialPort.c_str(), config.serialBaud,
            config.motorAddress);
        diagnostics.write(
            "ZDT velocity mode ready; stored absolute LEVEL is logical zero");
    } else {
        // dry-run仍计算并显示角度与脉冲，但不会打开UART或发送任何字节。
        std::fprintf(stderr,
            "DRY-RUN: motor disabled; angle and steps are display only\n");
        diagnostics.write("DRY_RUN motor disabled");
    }

    // 串口启动等待期间摄像头仍在持续采集；正式建立视觉模块前取一张
    // 最新帧，避免第一轮控制使用数百毫秒前的初始化画面。
    if (capture.waitForNext(
            consumedCaptureSequence, capturedFrame, 300)) {
        frame = *capturedFrame;
    } else if (capture.failed()) {
        std::fprintf(stderr, "camera capture thread failed during startup\n");
        if (motor) motor->stop();
        return 1;
    }
    if (!frameMatchesConfig(frame, config)) {
        std::fprintf(stderr,
            "camera resolution changed before control start\n");
        if (motor) motor->stop();
        return 1;
    }

    // ---------------- 视觉与控制模块 ----------------
    // 灰度霍夫识别器同时使用第3题轴线，圆心离y=237过远时直接拒绝。
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
    const MechanismModel mechanism(config);
    Task3Sequence task3(config);
    Task3MotionController motionController(config);

    PreviewWindow preview("ball2-task3");
    if (config.gui) preview.start();

    std::fprintf(stderr, "mechanism conversion: %s\n",
        mechanism.usesCalibrationTable() ?
        "measured calibration table" :
        "estimated crank-link formula (not final calibration)");

    if (config.csv) {
        std::printf(
            "frame,time,measured,target_cm,position_cm,error_cm,speed_cm_s,"
            "speed_two_frame_cm_s,"
            "p_angle_deg,d_angle_deg,i_angle_deg,base_limit_deg,"
            "breakaway_limit_deg,"
            "requested_angle_deg,breakaway_angle_deg,applied_angle_deg,"
            "motor_target_steps,"
            "motor_actual_steps,motor_target_rpm,motor_actual_rpm,"
            "motor_command_rpm,motor_acceleration_rpm_s,confidence\n");
    }

    // ---------------- 主循环状态 ----------------
    bool armed = config.startArmed;
    if (armed) task3.start(secondsNow());
    bool hadMeasurement = false;
    bool communicationOk = true;
    int centerReadyFrames = 0;
    double targetCm = task3.targetCm();
    double positionCm = 0.0;
    double errorCm = 0.0;
    double speedCmS = 0.0;
    double rawTwoFrameSpeedCmS = 0.0;
    double rawPositionCm = 0.0;

    // requested是PD或丢球保护要求的角度，applied是变化率限制后真正发给机构的角度。
    double requestedAngleDeg = 0.0;
    double appliedAngleDeg = 0.0;
    double angleAtLastMeasurementDeg = 0.0;
    Task3MotionCommand motionCommand;
    double lastMeasurementTime = -1.0;
    double previousLoopTime = secondsNow();
    int motorSteps = 0;
    MotorLoopTelemetry motorTelemetry;
    if (commander) motorTelemetry = commander->telemetry();

    uint64_t sequence = 0;
    double nextVideoStreamTime = secondsNow();
    double nextRuntimeLogTime = secondsNow();
    bool videoStreamFailureReported = false;
    std::fprintf(stderr,
        "%s: SPACE=start/abort TASK3, R=reset vision while paused, Q/ESC=exit\n",
        armed ? "CONTROL ARMED" : "PAUSED");

        // ---------------- 钢球视觉外环 ----------------
    while (running.load()) {
        ++sequence;
        const double now = secondsNow();

        // 按真实循环间隔执行角度变化率限制；2～50ms边界抑制重复时间戳和卡帧尖峰。
        const double loopDt = std::clamp(
            now - previousLoopTime, 0.002, 0.05);
        previousLoopTime = now;

        // Use the archived pre detector exactly as it was: image-only tracking
        // without the later direction, color-reference or center-anchor paths.
        const Result result = detector.update(frame);
        if (!armed && result.measured &&
            cv::norm(result.center - config.centerCalibrationPoint) <=
                config.task3StartCenterGatePx) {
            centerReadyFrames = std::min(centerReadyFrames + 1, 1000);
        } else if (!armed) {
            centerReadyFrames = 0;
        }
        if (result.measured) {
            rawPositionCm = pipeAxis.toCentimeters(result.center);
            positionCm = rawPositionCm;
            estimator.update(positionCm, now);
            speedCmS = estimator.speedCmS();
            rawTwoFrameSpeedCmS = estimator.rawTwoFrameSpeedCmS();
            task3.update(positionCm, speedCmS, now);
            targetCm = task3.targetCm();
            errorCm = positionCm - targetCm;

            motionCommand = motionController.update(
                task3.phase(), errorCm, speedCmS, loopDt);
            requestedAngleDeg = motionCommand.angleDeg;
            angleAtLastMeasurementDeg = requestedAngleDeg;
            lastMeasurementTime = now;
            hadMeasurement = true;
        } else if (hadMeasurement && lastMeasurementTime >= 0.0) {
            task3.onMeasurementLost();
            estimator.onMeasurementLost();
            rawTwoFrameSpeedCmS = 0.0;
            const double lostMs = (now - lastMeasurementTime) * 1000.0;
            if (lostMs <= config.lostHoldMs) {
                requestedAngleDeg = angleAtLastMeasurementDeg;
            } else if (lostMs < config.lostNeutralMs) {
                const double ratio =
                    (lostMs - config.lostHoldMs) /
                    std::max(1, config.lostNeutralMs - config.lostHoldMs);
                requestedAngleDeg = angleAtLastMeasurementDeg * (1.0 - ratio);
            } else {
                requestedAngleDeg = 0.0;
                speedCmS = 0.0;
                rawTwoFrameSpeedCmS = 0.0;
                motionCommand = {};
            }
        } else {
            task3.onMeasurementLost();
            estimator.onMeasurementLost();
            requestedAngleDeg = 0.0;
            speedCmS = 0.0;
            rawTwoFrameSpeedCmS = 0.0;
            motionCommand = {};
        }

        // 暂停时无论视觉位置如何，水管都回到水平。
        if (!armed) {
            requestedAngleDeg = 0.0;
            motionCommand = {};
        }

        appliedAngleDeg = approach(
            appliedAngleDeg,
            requestedAngleDeg,
            config.angleSlewDegS * loopDt);
        motorSteps = mechanism.angleToSteps(appliedAngleDeg);

        // dry-run时commander为空；实际模式以motorCommandHz读取编码器位置/速度，
        // 运行位置、速度、加速度和跃度限制后发送0xF6速度命令。
        if (commander) {
            if (!commander->update(motorSteps, millisecondsNow())) {
                std::fprintf(stderr,
                    "ZDT velocity feedback/command failed; stopping control loop\n");
                diagnostics.write(
                    "ERROR ZDT cycle failed tgt=%d actual=%.1f rpm=%.1f "
                    "cmd=%.1f wire=%.1f",
                    motorSteps, motorTelemetry.actualSteps,
                    motorTelemetry.actualSpeedRpm,
                    motorTelemetry.commandSpeedRpm,
                    motorTelemetry.wireCommandSpeedRpm);
                communicationOk = false;
                running.store(false);
            } else {
                motorTelemetry = commander->telemetry();
            }
        }

        if (now >= nextRuntimeLogTime) {
            diagnostics.write(
                "LOOP armed=%d phase=%s mode=%s "
                "center_ready=%d/6 ball=%d locked=%d "
                "confidence=%.2f px=(%.1f,%.1f) cm=%.3f target=%.3f "
                "error=%+.3f v=%.3f v2=%.3f "
                "P=%+.4f D=%+.4f I=%+.4f request=%+.4f "
                "breakaway=%+.4f lim=%.3f+%.3f applied=%+.4f "
                "M[tgt=%d pos=%.1f target_rpm=%+.2f actual_rpm=%+.2f "
                "cmd=%+.2f wire=%+.2f acc=%+.2f]",
                armed ? 1 : 0, task3.phaseText(),
                task3MotionModeText(motionCommand.mode),
                std::min(centerReadyFrames, 6),
                result.measured ? 1 : 0, result.locked ? 1 : 0,
                result.confidence,
                result.measured ? result.center.x : -1.0f,
                result.measured ? result.center.y : -1.0f,
                positionCm, targetCm, errorCm, speedCmS, rawTwoFrameSpeedCmS,
                motionCommand.proportionalAngleDeg,
                motionCommand.derivativeAngleDeg,
                motionCommand.integralAngleDeg,
                requestedAngleDeg, motionCommand.breakawayAngleDeg,
                motionCommand.baseOutputLimitDeg,
                motionCommand.breakawayOutputLimitDeg,
                appliedAngleDeg, motorSteps,
                motorTelemetry.actualSteps, motorTelemetry.targetSpeedRpm,
                motorTelemetry.actualSpeedRpm,
                motorTelemetry.commandSpeedRpm,
                motorTelemetry.wireCommandSpeedRpm,
                motorTelemetry.commandAccelerationRpmS);
            diagnostics.writeControlSample(
                armed, std::min(centerReadyFrames, 6), result.measured,
                result.locked, result.confidence,
                result.measured ? result.center.x : -1.0f,
                result.measured ? result.center.y : -1.0f,
                positionCm, targetCm, errorCm, speedCmS, rawTwoFrameSpeedCmS,
                requestedAngleDeg, motionCommand,
                appliedAngleDeg, motorSteps,
                motorTelemetry);
            nextRuntimeLogTime = now +
                config.runtimeLogIntervalMs / 1000.0;
        }

        if (config.csv) {
            std::printf(
                "%llu,%.6f,%d,%.4f,%.4f,%+.4f,%+.4f,%+.4f,"
                "%+.5f,%+.5f,%+.5f,%.5f,%.5f,%+.5f,%+.5f,%+.5f,%d,"
                "%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%.3f\n",
                static_cast<unsigned long long>(sequence),
                now,
                result.measured ? 1 : 0,
                targetCm,
                positionCm,
                errorCm,
                speedCmS,
                rawTwoFrameSpeedCmS,
                motionCommand.proportionalAngleDeg,
                motionCommand.derivativeAngleDeg,
                motionCommand.integralAngleDeg,
                motionCommand.baseOutputLimitDeg,
                motionCommand.breakawayOutputLimitDeg,
                requestedAngleDeg,
                motionCommand.breakawayAngleDeg,
                appliedAngleDeg,
                motorSteps,
                motorTelemetry.actualSteps,
                motorTelemetry.targetSpeedRpm,
                motorTelemetry.actualSpeedRpm,
                motorTelemetry.commandSpeedRpm,
                motorTelemetry.commandAccelerationRpmS,
                result.confidence);
            std::fflush(stdout);
        }

        int key = config.gui ? preview.consumeKey() : -1;
        if (key < 0 && config.terminalKeys) {
            key = terminalKeys.consumeKey();
        }

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
            cv::Mat grayscaleBackground;
            if (displayFrame.channels() == 3) {
                cv::cvtColor(displayFrame, grayscaleBackground,
                             cv::COLOR_BGR2GRAY);
                cv::cvtColor(grayscaleBackground, displayFrame,
                             cv::COLOR_GRAY2BGR);
            } else if (displayFrame.channels() == 4) {
                cv::cvtColor(displayFrame, grayscaleBackground,
                             cv::COLOR_BGRA2GRAY);
                cv::cvtColor(grayscaleBackground, displayFrame,
                             cv::COLOR_GRAY2BGR);
            }
            // 黄框框出当前画面的整段可见水管；第3题的实际霍夫搜索ROI更窄。
            // 该框在detector.update()之后才绘制，不会被算法当成画面边缘。
            if (config.drawPipeDetectionArea) {
                const cv::Rect requestedPipeArea =
                    config.pipeDisplayArea.width > 0 &&
                    config.pipeDisplayArea.height > 0 ?
                        config.pipeDisplayArea : config.roi;
                const cv::Rect pipeArea =
                    boundedVisionRect(requestedPipeArea, displayFrame.size());
                if (pipeArea.width > 0 && pipeArea.height > 0) {
                    cv::rectangle(displayFrame, pipeArea,
                                  {0, 220, 255}, 2, cv::LINE_AA);
                }
            }

            // 识别到钢球就直接画绿色，不再用额外状态改变圆圈颜色。
            if (result.measured) {
                drawBall(displayFrame, result.center, result.radius,
                         cv::Scalar(0, 255, 0));
            } else {
                char visionText[128];
                std::snprintf(visionText, sizeof(visionText),
                              "%s H=%d D=%d V=%d MISS=%d",
                              result.locked ? "BALL LOST" : "SEARCHING",
                              result.houghCandidates,
                              result.darkBlobCandidates,
                              result.validCandidates,
                              result.missStreak);
                cv::putText(displayFrame, visionText,
                            {10, 28}, cv::FONT_HERSHEY_SIMPLEX,
                            0.62, {0, 0, 255}, 2, cv::LINE_AA);
            }

            // 紫色十字是当前厘米目标位置；本程序不会绘制蓝色ROI框。
            cv::drawMarker(displayFrame, pipeAxis.targetPoint(targetCm),
                           {255, 0, 255}, cv::MARKER_CROSS,
                           18, 2, cv::LINE_AA);

            char controlText[190];
            std::snprintf(
                controlText, sizeof(controlText),
                "%s x=%.1f raw=%.2f pos=%.2f err=%+.2f v=%+.1f target=%.1f",
                armed ? "CONTROL" : "PAUSED",
                result.measured ? result.center.x : -1.0f,
                rawPositionCm, positionCm, errorCm, speedCmS, targetCm);
            cv::putText(displayFrame, controlText, {10, 56},
                        cv::FONT_HERSHEY_SIMPLEX, 0.47,
                        armed ? cv::Scalar(0, 255, 255)
                              : cv::Scalar(0, 128, 255),
                        1, cv::LINE_AA);

            // 第一行显示钢球外环角度，第二行显示电机位置、速度和加速度串级环。
            char motorText[160];
            std::snprintf(
                motorText, sizeof(motorText),
                "request=%+.3fdeg brk=%+.3fdeg pipe=%+.3fdeg",
                requestedAngleDeg, motionCommand.breakawayAngleDeg,
                appliedAngleDeg);
            cv::putText(displayFrame, motorText, {10, 80},
                        cv::FONT_HERSHEY_SIMPLEX, 0.46,
                        {0, 255, 255}, 1, cv::LINE_AA);

            char motorLoopText[190];
            std::snprintf(
                motorLoopText, sizeof(motorLoopText),
                "M tgt=%+d pos=%+.1f rpm=%+.1f cmd=%+.1f acc=%+.0f",
                motorSteps,
                motorTelemetry.actualSteps,
                motorTelemetry.actualSpeedRpm,
                motorTelemetry.commandSpeedRpm,
                motorTelemetry.commandAccelerationRpmS);
            cv::putText(displayFrame, motorLoopText, {10, 104},
                        cv::FONT_HERSHEY_SIMPLEX, 0.43,
                        motorTelemetry.atTarget
                            ? cv::Scalar(0, 255, 0)
                            : cv::Scalar(0, 220, 255),
                        1, cv::LINE_AA);

            // 单独显示第3题当前阶段和计时；OpenCV原生字体仅支持英文。
            char taskText[190];
            task3.formatStatus(taskText, sizeof(taskText), now);
            cv::putText(displayFrame, taskText, {10, 128},
                        cv::FONT_HERSHEY_SIMPLEX, 0.50,
                        task3.timedOut()
                            ? cv::Scalar(0, 0, 255)
                            : cv::Scalar(255, 255, 0),
                        1, cv::LINE_AA);

            char sourceText[128];
            const char* visionSource = !result.measured ? "HOLD" :
                (result.fused ? "FUSED" :
                 (result.contourFallback ? "BLOB" : "HOUGH"));
            std::snprintf(sourceText, sizeof(sourceText),
                          "VISION=%s", visionSource);
            cv::putText(displayFrame, sourceText, {10, 152},
                        cv::FONT_HERSHEY_SIMPLEX, 0.43,
                        result.measured ? cv::Scalar(0, 255, 0)
                                        : cv::Scalar(0, 165, 255),
                        1, cv::LINE_AA);

            char motionText[128];
            std::snprintf(
                motionText, sizeof(motionText),
                "MODE=%s P=%+.3f D=%+.3f I=%+.3f v2=%+.2f",
                task3MotionModeText(motionCommand.mode),
                motionCommand.proportionalAngleDeg,
                motionCommand.derivativeAngleDeg,
                motionCommand.integralAngleDeg,
                rawTwoFrameSpeedCmS);
            cv::putText(displayFrame, motionText, {10, 176},
                        cv::FONT_HERSHEY_SIMPLEX, 0.43,
                        motionCommand.integralWindowActive
                            ? cv::Scalar(0, 80, 255)
                            : cv::Scalar(220, 220, 220),
                        1, cv::LINE_AA);

            // The network encoder receives only the pipe ROI.
            if (previewRequested && streamRequested) {
                cv::Mat streamFrame = displayFrame(videoStreamArea).clone();
                preview.publish(std::move(displayFrame));
                videoStreamer->publish(std::move(streamFrame));
            } else if (previewRequested) {
                preview.publish(std::move(displayFrame));
            } else {
                cv::Mat streamFrame = displayFrame(videoStreamArea).clone();
                videoStreamer->publish(std::move(streamFrame));
            }
        }

        // ---------------- 快捷键 ----------------
        if (key == 27 || key == 'q' || key == 'Q') break;

        if (key == ' ') {
            if (armed) {
                // 运行中按空格代表中止本轮测试。水管平滑回水平；下一次启动重新计时。
                armed = false;
                task3.abortAndReset();
                motionController.reset();
                estimator.reset();
                hadMeasurement = false;
                rawTwoFrameSpeedCmS = 0.0;
                lastMeasurementTime = -1.0;
                angleAtLastMeasurementDeg = 0.0;
                targetCm = task3.targetCm();
                requestedAngleDeg = 0.0;
                std::fprintf(stderr,
                    "TASK aborted; PAUSED; pipe returning to level\n");
                diagnostics.write("EVENT task aborted; paused and returning level");
            } else {
                // 必须连续确认钢球确实位于O点后才能启动。视频中的失败
                // 正是BALL LOST状态仍允许开始，随后单个错误候选推进了状态机。
                if (centerReadyFrames < 6) {
                    std::fprintf(stderr,
                        "TASK start blocked: wait for stable green ball at O "
                        "(%d/6 frames)\n",
                        centerReadyFrames);
                    diagnostics.write(
                        "EVENT start blocked: center_ready=%d/6 ball=%d "
                        "position=%.3f",
                        std::min(centerReadyFrames, 6),
                        result.measured ? 1 : 0, positionCm);
                } else {
                    estimator.reset();
                    motionController.reset();
                    task3.start(now);
                    targetCm = task3.targetCm();
                    armed = true;
                    centerReadyFrames = 0;
                    hadMeasurement = false;
                    rawTwoFrameSpeedCmS = 0.0;
                    lastMeasurementTime = -1.0;
                    angleAtLastMeasurementDeg = 0.0;
                    std::fprintf(stderr, "control ARMED for TASK 3\n");
                    diagnostics.write("EVENT control armed for TASK 3");
                }
            }
        }

        if (!armed && (key == 'r' || key == 'R')) {
            // 球放回O点后可按R清除旧轨迹。首次捕获仍只在O点附近进行，
            // 不会重新锁到远端螺丝或固定阴影。
            detector.reset(
                config.centerCalibrationPoint,
                config.useThreePointPositionCalibration);
            estimator.reset();
            hadMeasurement = false;
            rawTwoFrameSpeedCmS = 0.0;
            lastMeasurementTime = -1.0;
            std::fprintf(stderr,
                "vision tracker reset; put ball at O and wait for green circle\n");
            diagnostics.write("EVENT vision tracker reset while paused");
        }

        // 等待采集线程发布下一张“最新画面”。若识别落后，采集线程已经
        // 自动覆盖中间旧帧；这里绝不会从队列中逐张追赶过时图像。
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
        if (!frameMatchesConfig(frame, config)) {
            std::fprintf(stderr,
                "camera resolution changed; motor control stopped\n");
            communicationOk = false;
            break;
        }
    }

    // ---------------- 安全退出 ----------------
    // 先结束摄像头read线程，再执行可能持续数百毫秒的电机回零等待。
    capture.stop();

    // Q/ESC、Ctrl+C、摄像头失败或串口失败都会先用速度模式串级环回编码器0位，
    // 到位或超时后再发送立即停止，避免退出后继续推拉机构。
    if (commander && motor) {
        std::fprintf(stderr, "returning pipe to LEVEL zero...\n");
        diagnostics.write("EVENT exiting; returning pipe to logical zero");
        if (!commander->returnToZero(config.exitReturnTimeoutMs)) {
            communicationOk = false;
            std::fprintf(stderr,
                "WARNING: velocity-mode return to zero failed or timed out\n");
            diagnostics.write(
                "WARNING velocity-mode return to zero failed or timed out");
        }

        // 无论回零命令是否成功，最后都发送立即停止，避免程序退出后继续运动。
        if (!motor->stop()) {
            communicationOk = false;
            std::fprintf(stderr, "WARNING: final ZDT stop failed\n");
            diagnostics.write("WARNING final ZDT stop failed");
        }
    }

    preview.stop();
    terminalKeys.stop();
    if (videoStreamer) {
        const uint64_t sentFrames = videoStreamer->sentFrames();
        videoStreamer->stop();
        std::fprintf(stderr,
            "UDP video stopped after %llu frames\n",
            static_cast<unsigned long long>(sentFrames));
    }
    if (!config.motorEnabled) {
        std::fprintf(stderr, "DRY-RUN finished; no ZDT command was sent\n");
        diagnostics.write("EXIT dry-run communication_ok=%d", communicationOk ? 1 : 0);
    } else {
        std::fprintf(stderr, "ZDT control finished\n");
        diagnostics.write("EXIT motor control communication_ok=%d",
                          communicationOk ? 1 : 0);
    }
    return communicationOk ? 0 : 1;
}

} // namespace ball_stepper
