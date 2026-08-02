#pragma once

// zdt_stepper_uart.hpp
// ============================================================================
// ZDT Emm V5.0速度模式执行层。
//
// 控制链路：
//   目标水管角度 -> 目标电机脉冲位置
//   -> 编码器位置外环 -> 目标RPM
//   -> 速度误差环 -> 目标加速度
//   -> 加速度/跃度限制 -> 0xF6速度模式
//   -> ZDT内部20kHz速度闭环
//
// 0x35和0x36分别读取实时转速与实时位置。速度模式不再依靠软件积分猜位置，
// 因而反向间隙、负载扰动和闭环步进的追赶动作都能反映到下一次控制计算中。
// ============================================================================

#include "system_config.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <thread>

#if defined(__linux__) || defined(BALL_STEPPER_SYNTAX_CHECK)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace ball_stepper {

inline constexpr uint8_t ZDT_TAIL = 0x6B;
inline constexpr double ZDT_POSITION_UNITS_PER_REVOLUTION = 65536.0;

inline std::array<uint8_t, 8> makeZdtVelocityFrame(
    uint8_t address,
    double signedRpm,
    uint16_t maximumRpm,
    uint8_t acceleration)
{
    const double limited = std::clamp(
        signedRpm,
        -static_cast<double>(maximumRpm),
         static_cast<double>(maximumRpm));
    const uint16_t magnitude = static_cast<uint16_t>(std::lround(
        std::abs(limited)));
    return {
        address,
        0xF6,
        limited >= 0.0 ? uint8_t{0x00} : uint8_t{0x01},
        static_cast<uint8_t>((magnitude >> 8) & 0xFF),
        static_cast<uint8_t>(magnitude & 0xFF),
        acceleration,
        0x00,
        ZDT_TAIL
    };
}

#if defined(__linux__) || defined(BALL_STEPPER_SYNTAX_CHECK)

class SerialPort {
    int fileDescriptor_ = -1;

public:
    ~SerialPort() { closePort(); }

    bool openPort(const std::string& device, int baud)
    {
        fileDescriptor_ = ::open(
            device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fileDescriptor_ < 0) {
            std::fprintf(stderr, "open %s failed: %s\n",
                         device.c_str(), std::strerror(errno));
            return false;
        }

        termios terminal{};
        if (tcgetattr(fileDescriptor_, &terminal) != 0) {
            std::fprintf(stderr, "tcgetattr failed: %s\n",
                         std::strerror(errno));
            closePort();
            return false;
        }

        speed_t speed = B115200;
        if (baud == 9600) speed = B9600;
        else if (baud == 57600) speed = B57600;
        else if (baud == 230400) speed = B230400;
        cfsetispeed(&terminal, speed);
        cfsetospeed(&terminal, speed);

        terminal.c_cflag =
            (terminal.c_cflag & static_cast<tcflag_t>(~CSIZE)) | CS8;
        terminal.c_iflag &= static_cast<tcflag_t>(
            ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
              INLCR | IGNCR | ICRNL | IXON));
        terminal.c_lflag = 0;
        terminal.c_oflag = 0;
        terminal.c_cc[VMIN] = 0;
        terminal.c_cc[VTIME] = 0;
        terminal.c_cflag |= CLOCAL | CREAD;
        terminal.c_cflag &= static_cast<tcflag_t>(
            ~(PARENB | PARODD | CSTOPB | CRTSCTS));

        if (tcsetattr(fileDescriptor_, TCSANOW, &terminal) != 0) {
            std::fprintf(stderr, "tcsetattr failed: %s\n",
                         std::strerror(errno));
            closePort();
            return false;
        }
        tcflush(fileDescriptor_, TCIOFLUSH);
        return true;
    }

    void closePort()
    {
        if (fileDescriptor_ >= 0) {
            ::close(fileDescriptor_);
            fileDescriptor_ = -1;
        }
    }

    bool discardInput()
    {
        return fileDescriptor_ >= 0 &&
               tcflush(fileDescriptor_, TCIFLUSH) == 0;
    }

    bool writeBytes(const uint8_t* data, std::size_t size)
    {
        if (fileDescriptor_ < 0) return false;

        std::size_t written = 0;
        while (written < size) {
            const ssize_t count = ::write(
                fileDescriptor_, data + written, size - written);
            if (count < 0) {
                if (errno == EINTR) continue;
                std::fprintf(stderr, "serial write failed: %s\n",
                             std::strerror(errno));
                return false;
            }
            if (count == 0) return false;
            written += static_cast<std::size_t>(count);
        }
        return tcdrain(fileDescriptor_) == 0;
    }

    bool readBytes(uint8_t* data, std::size_t size, int timeoutMs)
    {
        if (fileDescriptor_ < 0) return false;

        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeoutMs);
        std::size_t received = 0;
        while (received < size &&
               std::chrono::steady_clock::now() < deadline) {
            const ssize_t count = ::read(
                fileDescriptor_, data + received, size - received);
            if (count > 0) {
                received += static_cast<std::size_t>(count);
                continue;
            }
            if (count < 0 && errno != EINTR &&
                errno != EAGAIN && errno != EWOULDBLOCK) {
                std::fprintf(stderr, "serial read failed: %s\n",
                             std::strerror(errno));
                return false;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }

        if (received != size) {
            std::fprintf(stderr,
                "serial response timeout: expected=%zu received=%zu\n",
                size, received);
            return false;
        }
        return true;
    }
};

#else

class SerialPort {
public:
    bool openPort(const std::string&, int)
    {
        std::fprintf(stderr,
            "ZDT UART is only available on Raspberry Pi/Linux\n");
        return false;
    }
    void closePort() {}
    bool discardInput() { return false; }
    bool writeBytes(const uint8_t*, std::size_t) { return false; }
    bool readBytes(uint8_t*, std::size_t, int) { return false; }
};

#endif

struct ZdtMotionState {
    double positionSteps = 0.0;
    double speedRpm = 0.0;
};

class EmmV5Motor {
    SerialPort& serial_;
    uint8_t address_ = 1;
    uint16_t maximumRpm_ = 8;
    uint8_t acceleration_ = 5;
    int pulsesPerRevolution_ = 3200;
    int replyTimeoutMs_ = 15;
    bool expectCommandAck_ = true;

    static int64_t decodeSignedMagnitude(
        uint8_t sign, const uint8_t* magnitude, std::size_t size)
    {
        uint64_t value = 0;
        for (std::size_t index = 0; index < size; ++index) {
            value = (value << 8) | magnitude[index];
        }
        const int64_t signedValue = static_cast<int64_t>(value);
        return sign == 0x01 ? -signedValue : signedValue;
    }

    bool readResponse(uint8_t expectedCommand,
                      uint8_t* response,
                      std::size_t responseSize)
    {
        if (responseSize < 4) return false;
        if (!serial_.readBytes(response, 2, replyTimeoutMs_)) return false;

        if (response[0] != address_) {
            std::fprintf(stderr,
                "ZDT response address mismatch: expected=%u received=%u\n",
                static_cast<unsigned>(address_),
                static_cast<unsigned>(response[0]));
            return false;
        }

        if (response[1] == 0x00) {
            if (!serial_.readBytes(response + 2, 2, replyTimeoutMs_)) {
                return false;
            }
            std::fprintf(stderr,
                "ZDT rejected command 0x%02X: error=0x%02X\n",
                static_cast<unsigned>(expectedCommand),
                static_cast<unsigned>(response[2]));
            return false;
        }

        if (response[1] != expectedCommand) {
            std::fprintf(stderr,
                "ZDT response command mismatch: expected=0x%02X received=0x%02X\n",
                static_cast<unsigned>(expectedCommand),
                static_cast<unsigned>(response[1]));
            return false;
        }
        if (!serial_.readBytes(
                response + 2, responseSize - 2, replyTimeoutMs_)) {
            return false;
        }
        if (response[responseSize - 1] != ZDT_TAIL) {
            std::fprintf(stderr, "ZDT response checksum/tail mismatch\n");
            return false;
        }
        return true;
    }

    bool exchange(const uint8_t* command,
                  std::size_t commandSize,
                  uint8_t expectedCommand,
                  uint8_t* response,
                  std::size_t responseSize,
                  bool expectResponse)
    {
        if (!serial_.discardInput()) return false;
        if (!serial_.writeBytes(command, commandSize)) return false;
        if (!expectResponse) return true;
        return readResponse(expectedCommand, response, responseSize);
    }

    bool sendControlCommand(const uint8_t* command,
                            std::size_t commandSize,
                            uint8_t commandCode)
    {
        uint8_t response[4]{};
        if (!exchange(command, commandSize, commandCode,
                      response, sizeof(response), expectCommandAck_)) {
            return false;
        }
        if (expectCommandAck_ && response[2] != 0x02) {
            std::fprintf(stderr,
                "ZDT command 0x%02X returned status 0x%02X\n",
                static_cast<unsigned>(commandCode),
                static_cast<unsigned>(response[2]));
            return false;
        }
        return true;
    }

public:
    EmmV5Motor(SerialPort& serial, const AppConfig& config)
        : serial_(serial),
          address_(static_cast<uint8_t>(
              std::clamp(config.motorAddress, 0, 255))),
          maximumRpm_(static_cast<uint16_t>(
              std::clamp(config.motorRpm, 1, 3000))),
          acceleration_(static_cast<uint8_t>(
              std::clamp(config.motorAcceleration, 1, 255))),
          pulsesPerRevolution_(config.pulsesPerRevolution),
          replyTimeoutMs_(config.motorReplyTimeoutMs),
          expectCommandAck_(config.motorExpectCommandAck) {}

    bool enable()
    {
        const uint8_t frame[] = {
            address_, 0xF3, 0xAB, 0x01, 0x00, ZDT_TAIL
        };
        return sendControlCommand(frame, sizeof(frame), 0xF3);
    }

    bool stop()
    {
        const uint8_t frame[] = {
            address_, 0xFE, 0x98, 0x00, ZDT_TAIL
        };
        return sendControlCommand(frame, sizeof(frame), 0xFE);
    }

    bool clearPosition()
    {
        const uint8_t frame[] = {
            address_, 0x0A, 0x6D, ZDT_TAIL
        };
        return sendControlCommand(frame, sizeof(frame), 0x0A);
    }

    bool setVelocityRpm(double signedRpm)
    {
        const auto frame = makeZdtVelocityFrame(
            address_, signedRpm, maximumRpm_, acceleration_);
        return sendControlCommand(frame.data(), frame.size(), 0xF6);
    }

    bool readRealtimeSpeedRpm(double& signedRpm)
    {
        const uint8_t command[] = {address_, 0x35, ZDT_TAIL};
        uint8_t response[6]{};
        if (!exchange(command, sizeof(command), 0x35,
                      response, sizeof(response), true)) {
            return false;
        }
        signedRpm = static_cast<double>(decodeSignedMagnitude(
            response[2], response + 3, 2));
        return true;
    }

    bool readRealtimePositionSteps(double& positionSteps)
    {
        const uint8_t command[] = {address_, 0x36, ZDT_TAIL};
        uint8_t response[8]{};
        if (!exchange(command, sizeof(command), 0x36,
                      response, sizeof(response), true)) {
            return false;
        }
        const int64_t positionUnits = decodeSignedMagnitude(
            response[2], response + 3, 4);
        positionSteps = static_cast<double>(positionUnits) *
            static_cast<double>(pulsesPerRevolution_) /
            ZDT_POSITION_UNITS_PER_REVOLUTION;
        return true;
    }

    bool readMotionState(ZdtMotionState& state)
    {
        return readRealtimePositionSteps(state.positionSteps) &&
               readRealtimeSpeedRpm(state.speedRpm);
    }
};

struct MotorLoopTelemetry {
    bool initialized = false;
    bool atTarget = false;
    int targetSteps = 0;
    double actualSteps = 0.0;
    double positionErrorSteps = 0.0;
    double targetSpeedRpm = 0.0;
    double actualSpeedRpm = 0.0;
    double commandSpeedRpm = 0.0;
    double commandAccelerationRpmS = 0.0;
};

class VelocityModePositionController {
    const AppConfig& config_;
    double commandSpeedRpm_ = 0.0;
    double commandAccelerationRpmS_ = 0.0;

public:
    explicit VelocityModePositionController(const AppConfig& config)
        : config_(config) {}

    void reset()
    {
        commandSpeedRpm_ = 0.0;
        commandAccelerationRpmS_ = 0.0;
    }

    MotorLoopTelemetry update(int requestedTargetSteps,
                              double actualSteps,
                              double actualSpeedRpm,
                              double dt)
    {
        MotorLoopTelemetry result;
        result.initialized = true;
        result.targetSteps = std::clamp(
            requestedTargetSteps,
            -config_.motorSoftLimitSteps,
             config_.motorSoftLimitSteps);
        result.actualSteps = actualSteps;
        result.actualSpeedRpm = actualSpeedRpm;
        result.positionErrorSteps =
            static_cast<double>(result.targetSteps) - actualSteps;

        double targetSpeedRpm = 0.0;
        if (std::abs(result.positionErrorSteps) >
            config_.motorPositionToleranceSteps) {
            const double proportionalSpeed =
                config_.motorPositionKpRpmPerStep *
                std::abs(result.positionErrorSteps);

            // v^2=2as。这里把脉冲距离换算为转数，把RPM/s换算为转/s^2，
            // 得到不会在目标轴位前来不及减速的速度上限。
            const double brakingSpeed = std::sqrt(std::max(
                0.0,
                120.0 * config_.motorBrakingAccelerationRpmS *
                std::abs(result.positionErrorSteps) /
                static_cast<double>(config_.pulsesPerRevolution)));
            const double magnitude = std::min({
                static_cast<double>(config_.motorRpm),
                proportionalSpeed,
                brakingSpeed
            });
            targetSpeedRpm = std::copysign(
                magnitude, result.positionErrorSteps);
        }
        result.targetSpeedRpm = targetSpeedRpm;

        const double boundedDt = std::clamp(dt, 0.001, 0.10);
        const double desiredAcceleration = std::clamp(
            config_.motorVelocityKpPerSecond *
                (targetSpeedRpm - actualSpeedRpm),
            -config_.motorMaximumAccelerationRpmS,
             config_.motorMaximumAccelerationRpmS);
        commandAccelerationRpmS_ = approach(
            commandAccelerationRpmS_,
            desiredAcceleration,
            config_.motorMaximumJerkRpmS3 * boundedDt);

        double nextCommandSpeed = commandSpeedRpm_ +
            commandAccelerationRpmS_ * boundedDt;
        const double before = targetSpeedRpm - commandSpeedRpm_;
        const double after = targetSpeedRpm - nextCommandSpeed;
        if (std::abs(before) < 1e-9 || before * after <= 0.0) {
            nextCommandSpeed = targetSpeedRpm;
            commandAccelerationRpmS_ = approach(
                commandAccelerationRpmS_,
                0.0,
                config_.motorMaximumJerkRpmS3 * boundedDt);
        }
        commandSpeedRpm_ = std::clamp(
            nextCommandSpeed,
            -static_cast<double>(config_.motorRpm),
             static_cast<double>(config_.motorRpm));
        if (std::abs(commandSpeedRpm_) < 0.5 &&
            std::abs(targetSpeedRpm) < 1e-9) {
            commandSpeedRpm_ = 0.0;
        }

        result.commandSpeedRpm = commandSpeedRpm_;
        result.commandAccelerationRpmS = commandAccelerationRpmS_;
        result.atTarget =
            std::abs(result.positionErrorSteps) <=
                config_.motorPositionToleranceSteps &&
            std::abs(actualSpeedRpm) <= config_.motorStopSpeedRpm;
        return result;
    }
};

class MotorCommander {
    EmmV5Motor& motor_;
    VelocityModePositionController controller_;
    int minimumIntervalMs_ = 33;
    int targetSteps_ = 0;
    int64_t lastCycleMs_ = 0;
    MotorLoopTelemetry telemetry_{};

    bool runCycle(int64_t nowMs)
    {
        ZdtMotionState state;
        if (!motor_.readMotionState(state)) return false;

        const double dt = lastCycleMs_ == 0 ?
            minimumIntervalMs_ / 1000.0 :
            std::clamp((nowMs - lastCycleMs_) / 1000.0, 0.001, 0.10);
        telemetry_ = controller_.update(
            targetSteps_, state.positionSteps, state.speedRpm, dt);
        if (!motor_.setVelocityRpm(telemetry_.commandSpeedRpm)) {
            return false;
        }
        lastCycleMs_ = nowMs;
        return true;
    }

public:
    MotorCommander(EmmV5Motor& motor, const AppConfig& config)
        : motor_(motor),
          controller_(config),
          minimumIntervalMs_(std::max(
              1, 1000 / config.motorCommandHz)) {}

    bool update(int targetSteps, int64_t nowMs)
    {
        targetSteps_ = targetSteps;
        if (lastCycleMs_ != 0 &&
            nowMs - lastCycleMs_ < minimumIntervalMs_) {
            return true;
        }
        return runCycle(nowMs);
    }

    bool force(int targetSteps)
    {
        targetSteps_ = targetSteps;
        return runCycle(millisecondsNow());
    }

    bool returnToZero(int timeoutMs)
    {
        targetSteps_ = 0;
        const int64_t deadline = millisecondsNow() + timeoutMs;
        while (millisecondsNow() <= deadline) {
            if (!runCycle(millisecondsNow())) return false;
            if (telemetry_.atTarget) {
                controller_.reset();
                return motor_.setVelocityRpm(0.0);
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(minimumIntervalMs_));
        }
        motor_.setVelocityRpm(0.0);
        return false;
    }

    const MotorLoopTelemetry& telemetry() const { return telemetry_; }
};

} // namespace ball_stepper
