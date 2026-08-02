#pragma once

// task4_app.hpp
// ============================================================================
// 第4问完整主循环：高速摄像头 -> 钢球位置/速度外环 -> 目标水管角度
// -> 目标编码器轴位 -> 电机位置/速度/加速度串级环 -> ZDT 0xF6速度命令。
// ============================================================================

#include "steel_ball_vision.hpp"
#include "latest_frame_capture.hpp"
#include "balance_control.hpp"
#include "task4_balance_control.hpp"
#include "preview_window.hpp"
#include "terminal_key_input.hpp"
#include "udp_video_streamer.hpp"
#include "zdt_stepper_uart.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace ball_stepper {

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
        config.task4FineKp < 0.0 || config.task4FineKd < 0.0 ||
        config.task4FineKp > config.task4Kp ||
        config.task4FineKd > config.task4Kd ||
        config.task4FineZoneCm <= config.task4DeadbandCm ||
        config.task4FineSpeedCmS <= config.task4StopSpeedCmS ||
        config.task4Ki < 0.0 ||
        config.task4IntegralLimitDeg < 0.0 ||
        config.task4IntegralEnableErrorCm <= 0.0 ||
        config.task4IntegralLeakSeconds <= 0.0 ||
        config.task4DeadbandCm < 0.0 ||
        config.task4StopSpeedCmS < 0.0 ||
        config.task4MaximumAngleDeg <= 0.0 ||
        config.task4MaximumAngleDeg > config.maximumPipeAngleDeg ||
        std::abs(config.task4LevelTrimDeg) >
            config.task4MaximumAngleDeg ||
        config.task4AngleSlewDegS <= 0.0 ||
        config.task4LossFailureMs < config.lostHoldMs) {
        std::fprintf(stderr, "invalid TASK 4 balance parameter in main.cpp\n");
        return false;
    }

    if (config.speedFilterSeconds < 0.005 ||
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
        (!config.zeroOnStart || config.serialPort.empty() ||
         config.serialBaud <= 0 || config.motorRpm <= 0 ||
          config.motorAcceleration < 1 || config.motorCommandHz <= 0)) {
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

inline int runTask4VelocityApp(const AppConfig& config)
{
    if (!validateConfig(config) || !validateTask4Config(config)) return 1;
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
               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, config.cameraWidth);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, config.cameraHeight);
    camera.set(cv::CAP_PROP_FPS, config.cameraFps);
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (config.disableAutofocus) camera.set(cv::CAP_PROP_AUTOFOCUS, 0.0);

    if (config.configureExposure && config.useManualExposure) {
        const bool modeSet = camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0);
        const bool exposureSet = camera.set(
            cv::CAP_PROP_EXPOSURE, config.exposureAbsolute);
        std::fprintf(stderr,
            "camera exposure: manual %.1f%s\n",
            config.exposureAbsolute,
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
        } else if (!config.gui && config.motorEnabled && !config.startArmed) {
            std::fprintf(stderr,
                "headless paused mode needs an interactive terminal for keys\n");
            return 1;
        }
    }

    // ---------------- ZDT启动 ----------------
    SerialPort serial;
    std::unique_ptr<EmmV5Motor> motor;
    std::unique_ptr<MotorCommander> commander;
    bool communicationOk = true;

    if (config.motorEnabled) {
        if (!serial.openPort(config.serialPort, config.serialBaud)) return 1;
        motor = std::make_unique<EmmV5Motor>(serial, config);
        if (!motor->enable()) {
            std::fprintf(stderr, "ZDT enable command failed\n");
            return 1;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.enableSettleMs));
        if (!motor->stop()) {
            std::fprintf(stderr, "ZDT startup stop command failed\n");
            return 1;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.stopSettleMs));
        if (!motor->clearPosition()) {
            std::fprintf(stderr, "ZDT clear-position command failed\n");
            motor->stop();
            return 1;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.zeroSettleMs));
        commander = std::make_unique<MotorCommander>(*motor, config);
        if (!commander->force(0)) {
            std::fprintf(stderr,
                "ZDT speed-mode initialization failed; "
                "check Response=Receive/Both\n");
            motor->stop();
            return 1;
        }
        std::fprintf(stderr,
            "ZDT velocity mode ready: %s %d baud, address=%d; "
            "0x36 position + 0x35 speed + 0xF6 command\n",
            config.serialPort.c_str(), config.serialBaud,
            config.motorAddress);
    } else {
        std::fprintf(stderr,
            "DRY-RUN: motor disabled; commands are display only\n");
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
    BallStateEstimator estimator(config.speedFilterSeconds);
    Task4BalanceController controller(config);
    const MechanismModel mechanism(config);
    PreviewWindow preview("ball2-task4-velocity");
    if (config.gui) preview.start();

    if (config.csv) {
        std::printf(
            "frame,time,measured,position_cm,error_cm,speed_cm_s,"
            "p_deg,d_deg,i_deg,request_deg,applied_deg,motor_steps,"
            "motor_pos,motor_rpm,target_rpm,command_rpm,command_acc\n");
    }

    const double initialControlTime = secondsNow();
    bool armed = config.startArmed;
    bool hadMeasurement = false;
    bool measuredNow = false;
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
    double requestedAngleDeg = 0.0;
    double appliedAngleDeg = 0.0;
    double lastMeasuredAngleDeg = 0.0;
    double maximumAbsErrorCm = 0.0;
    double longestLostMs = 0.0;
    int motorSteps = 0;
    MotorLoopTelemetry motorTelemetry;
    Task4ControlOutput controlOutput;

    if (armed) {
        controller.reset(initialControlTime);
        estimator.reset();
    }

    uint64_t sequence = 0;
    double nextVideoStreamTime = secondsNow();
    bool videoStreamFailureReported = false;
    int controlFrames = 0;
    double controlFps = 0.0;
    double fpsStart = secondsNow();

    if (armed) {
        std::fprintf(stderr,
            "TASK4 AUTO BALANCE active: control starts on first ball "
            "measurement; SPACE=abort, F=finish, Q/ESC=exit\n");
    } else {
        std::fprintf(stderr,
            "TASK4 PAUSED: put ball at O; SPACE=start/abort, F=finish, "
            "R=reset vision, Q/ESC=exit\n");
    }

    while (running.load()) {
        ++sequence;
        ++controlFrames;
        const double now = secondsNow();
        const double loopDt = std::clamp(
            now - previousLoopTime, 0.002, 0.05);
        previousLoopTime = now;

        const Result result = detector.update(frame);
        measuredNow = result.measured;

        if (result.measured) {
            positionCm = pipeAxis.toCentimeters(result.center);
            estimator.update(positionCm, now);
            speedCmS = estimator.speedCmS();
            errorCm = positionCm - config.task4TargetCm;
            lastMeasurementTime = now;
            hadMeasurement = true;

            if (!armed) {
                if (std::abs(errorCm) <= config.task4StartToleranceCm &&
                    std::abs(speedCmS) <= config.task4StartSpeedCmS) {
                    ++readyFrames;
                } else {
                    readyFrames = 0;
                }
                requestedAngleDeg = 0.0;
                controlOutput = {};
            } else {
                controlOutput = controller.update(errorCm, speedCmS, now);
                requestedAngleDeg = controlOutput.angleDeg;
                lastMeasuredAngleDeg = requestedAngleDeg;

                if (!evaluationFinished) {
                    maximumAbsErrorCm = std::max(
                        maximumAbsErrorCm, std::abs(errorCm));
                }
            }
        } else {
            readyFrames = 0;
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
                    requestedAngleDeg = lastMeasuredAngleDeg;
                } else if (lostMs < config.lostNeutralMs) {
                    const double ratio =
                        (lostMs - config.lostHoldMs) /
                        std::max(1, config.lostNeutralMs -
                                       config.lostHoldMs);
                    requestedAngleDeg =
                        lastMeasuredAngleDeg * (1.0 - ratio);
                } else {
                    requestedAngleDeg = 0.0;
                    speedCmS = 0.0;
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
                "TASK4 8s CHECK: %s, max_error=%.3f cm, "
                "longest_lost=%.0f ms\n",
                evaluationPassed ? "PASS" : "FAIL",
                maximumAbsErrorCm, longestLostMs);
        }

        if (config.csv) {
            std::printf(
                "%llu,%.6f,%d,%.4f,%+.4f,%+.4f,%+.5f,%+.5f,%+.5f,"
                "%+.5f,%+.5f,%d,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                static_cast<unsigned long long>(sequence), now,
                result.measured ? 1 : 0,
                positionCm, errorCm, speedCmS,
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
                         result.confidence, cv::Scalar(0, 255, 0));
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
            std::snprintf(line, sizeof(line),
                "TASK4 %s time=%.2fs ready=%d/%d",
                armed ? (evaluationFinished ?
                    (evaluationPassed ? "PASS/HOLD" : "FAIL/HOLD") :
                    "BALANCE") : "PAUSED",
                elapsed, readyFrames, config.task4StartConfirmFrames);
            cv::putText(displayFrame, line, {10, 52},
                        cv::FONT_HERSHEY_SIMPLEX, 0.50,
                        armed ? cv::Scalar(0, 255, 255)
                              : cv::Scalar(0, 165, 255),
                        1, cv::LINE_AA);

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
                "%s P=%+.3f D=%+.3f I=%+.3f req=%+.3f pipe=%+.3f",
                controlOutput.fineMode ? "FINE" : "RECOVER",
                controlOutput.pTermDeg,
                controlOutput.dTermDeg,
                controlOutput.iTermDeg,
                requestedAngleDeg, appliedAngleDeg);
            cv::putText(displayFrame, line, {10, 100},
                        cv::FONT_HERSHEY_SIMPLEX, 0.44,
                        {0, 255, 255}, 1, cv::LINE_AA);

            std::snprintf(line, sizeof(line),
                "M tgt=%+d pos=%+.1f rpm=%+.1f cmd=%+.1f acc=%+.1f",
                motorSteps,
                motorTelemetry.actualSteps,
                motorTelemetry.actualSpeedRpm,
                motorTelemetry.commandSpeedRpm,
                motorTelemetry.commandAccelerationRpmS);
            cv::putText(displayFrame, line, {10, 124},
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
            cv::putText(displayFrame, sourceText, {10, 148},
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
                    "TASK4 aborted; PAUSED; pipe returning to level\n");
            } else if (!measuredNow ||
                       readyFrames < config.task4StartConfirmFrames) {
                std::fprintf(stderr,
                    "TASK4 start refused: wait for green ball at O and "
                    "ready counter\n");
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
                lastMeasuredAngleDeg = 0.0;
                std::fprintf(stderr,
                    "TASK4 BALANCE started; start the car now\n");
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
                "TASK4 manual finish: %s time=%.3fs max_error=%.3fcm\n",
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
        std::fprintf(stderr, "TASK4 ZDT velocity control finished\n");
    }
    return communicationOk ? 0 : 1;
}

} // namespace ball_stepper
