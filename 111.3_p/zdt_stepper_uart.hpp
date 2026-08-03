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
// 0x35和0x36分别读取实时转速与编码器位置。速度模式不再依靠软件积分猜位置，
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
#include <limits>
#include <thread>

#if defined(__linux__) || defined(BALL_STEPPER_SYNTAX_CHECK)
#include <fcntl.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace ball_stepper {

inline constexpr uint8_t ZDT_TAIL = 0x6B;
inline constexpr double EMM_V5_POSITION_UNITS_PER_REVOLUTION = 65536.0;
inline constexpr uint8_t ZDT_STATUS_ENABLED = 0x01;
inline constexpr uint8_t ZDT_STATUS_ARRIVED = 0x02;
inline constexpr uint8_t ZDT_STATUS_STALL = 0x04;
inline constexpr uint8_t ZDT_STATUS_STALL_PROTECTION = 0x08;

inline uint8_t makeEmmV5AccelerationLevel(double rpmPerSecond)
{
    if (!std::isfinite(rpmPerSecond) || rpmPerSecond <= 0.0) return 0;
    const double level = 256.0 - 20000.0 / rpmPerSecond;
    return static_cast<uint8_t>(std::clamp(
        static_cast<int>(std::lround(level)), 1, 255));
}

inline double emmV5AccelerationRpmS(uint8_t level)
{
    if (level == 0) return std::numeric_limits<double>::infinity();
    return 20000.0 / (256.0 - static_cast<double>(level));
}

inline std::array<uint8_t, 8> makeEmmV5VelocityFrame(
    uint8_t address,
    double signedRpm,
    uint16_t maximumRpm,
    uint16_t accelerationRpmS)
{
    const double limited = std::clamp(
        signedRpm,
        -static_cast<double>(maximumRpm),
         static_cast<double>(maximumRpm));
    const uint16_t magnitudeRpm = static_cast<uint16_t>(std::lround(
        std::abs(limited)));
    return {
        address,
        0xF6,
        limited >= 0.0 ? uint8_t{0x00} : uint8_t{0x01},
        static_cast<uint8_t>((magnitudeRpm >> 8) & 0xFF),
        static_cast<uint8_t>(magnitudeRpm & 0xFF),
        makeEmmV5AccelerationLevel(accelerationRpmS),
        0x00,
        ZDT_TAIL
    };
}

inline std::array<uint8_t, 13> makeEmmV5RelativePositionFrame(
    uint8_t address,
    double signedDegrees,
    double maximumRpm,
    uint16_t accelerationRpmS,
    uint32_t commandPulsesPerRevolution)
{
    const double maximumDegrees =
        static_cast<double>(std::numeric_limits<uint32_t>::max()) * 360.0 /
        static_cast<double>(std::max(uint32_t{1},
                                     commandPulsesPerRevolution));
    const double limitedDegrees = std::clamp(
        signedDegrees, -maximumDegrees, maximumDegrees);
    const uint32_t magnitudePulses = static_cast<uint32_t>(std::llround(
        std::abs(limitedDegrees) *
        static_cast<double>(commandPulsesPerRevolution) / 360.0));
    const uint16_t magnitudeRpm = static_cast<uint16_t>(std::lround(
        std::clamp(maximumRpm, 1.0, 3000.0)));

    return {
        address,
        0xFD,
        limitedDegrees >= 0.0 ? uint8_t{0x00} : uint8_t{0x01},
        static_cast<uint8_t>((magnitudeRpm >> 8) & 0xFF),
        static_cast<uint8_t>(magnitudeRpm & 0xFF),
        makeEmmV5AccelerationLevel(accelerationRpmS),
        static_cast<uint8_t>((magnitudePulses >> 24) & 0xFF),
        static_cast<uint8_t>((magnitudePulses >> 16) & 0xFF),
        static_cast<uint8_t>((magnitudePulses >> 8) & 0xFF),
        static_cast<uint8_t>(magnitudePulses & 0xFF),
        0x00,
        0x00,
        ZDT_TAIL
    };
}

#if defined(__linux__) || defined(BALL_STEPPER_SYNTAX_CHECK)

class SerialPort {
    int fileDescriptor_ = -1;
    int lockDescriptor_ = -1;

public:
    ~SerialPort() { closePort(); }

    bool openPort(const std::string& device, int baud)
    {
        closePort();

        lockDescriptor_ = ::open(
            "/tmp/ball_stepper_zdt_uart.lock", O_CREAT | O_RDWR, 0666);
        if (lockDescriptor_ < 0) {
            std::fprintf(stderr, "open UART lock failed: %s\n",
                         std::strerror(errno));
            return false;
        }
        if (::flock(lockDescriptor_, LOCK_EX | LOCK_NB) != 0) {
            std::fprintf(stderr,
                "ZDT UART is busy; stop the main controller or other motor_cli first\n");
            ::close(lockDescriptor_);
            lockDescriptor_ = -1;
            return false;
        }

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
        if (lockDescriptor_ >= 0) {
            ::flock(lockDescriptor_, LOCK_UN);
            ::close(lockDescriptor_);
            lockDescriptor_ = -1;
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

struct ZdtPositionState {
    double targetDegrees = 0.0;
    double realtimeTargetDegrees = 0.0;
    double actualDegrees = 0.0;
};

struct ZdtDriverParameters {
    uint8_t motorType = 0;
    uint8_t pulseControlMode = 0;
    uint8_t serialPortFunction = 0;
    uint8_t enableMode = 0;
    uint8_t directionMode = 0;
    uint8_t microstep = 0;
    uint8_t uartBaudIndex = 0;
    uint8_t checksumMode = 0;
    uint8_t responseMode = 0;
};

inline bool decodeZdtDriverParametersResponse(
    const std::array<uint8_t, 33>& response,
    uint8_t expectedAddress,
    ZdtDriverParameters& parameters)
{
    if (response[0] != expectedAddress || response[1] != 0x42 ||
        response[2] != 0x21 || response[3] != 0x15 ||
        response.back() != ZDT_TAIL) {
        return false;
    }
    parameters.motorType = response[4];
    parameters.pulseControlMode = response[5];
    parameters.serialPortFunction = response[6];
    parameters.enableMode = response[7];
    parameters.directionMode = response[8];
    parameters.microstep = response[9];
    parameters.uartBaudIndex = response[18];
    parameters.checksumMode = response[21];
    parameters.responseMode = response[22];
    return true;
}

class EmmV5Motor {
    SerialPort& serial_;
    uint8_t address_ = 1;
    uint16_t maximumRpm_ = 8;
    uint16_t speedSlopeRpmS_ = 60;
    int pulsesPerRevolution_ = 200;
    uint32_t positionCommandPulsesPerRevolution_ = 3200;
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

    bool readStatusByte(uint8_t commandCode, uint8_t& status)
    {
        const uint8_t command[] = {address_, commandCode, ZDT_TAIL};
        uint8_t response[4]{};
        if (!exchange(command, sizeof(command), commandCode,
                      response, sizeof(response), true)) {
            return false;
        }
        status = response[2];
        return true;
    }

    bool readPositionDegreesRegister(uint8_t commandCode, double& degrees)
    {
        const uint8_t command[] = {address_, commandCode, ZDT_TAIL};
        uint8_t response[8]{};
        if (!exchange(command, sizeof(command), commandCode,
                      response, sizeof(response), true)) {
            return false;
        }
        const int64_t positionUnits = decodeSignedMagnitude(
            response[2], response + 3, 4);
        degrees = static_cast<double>(positionUnits) * 360.0 /
            EMM_V5_POSITION_UNITS_PER_REVOLUTION;
        return true;
    }

    bool readUnsigned16Register(uint8_t commandCode, uint16_t& value)
    {
        const uint8_t command[] = {address_, commandCode, ZDT_TAIL};
        uint8_t response[5]{};
        if (!exchange(command, sizeof(command), commandCode,
                      response, sizeof(response), true)) {
            return false;
        }
        value = static_cast<uint16_t>(
            (static_cast<uint16_t>(response[2]) << 8) | response[3]);
        return true;
    }

public:
    EmmV5Motor(SerialPort& serial, const AppConfig& config)
        : serial_(serial),
          address_(static_cast<uint8_t>(
              std::clamp(config.motorAddress, 0, 255))),
          maximumRpm_(static_cast<uint16_t>(
              std::clamp(config.motorRpm, 1, 3000))),
          speedSlopeRpmS_(static_cast<uint16_t>(
              std::clamp(config.motorSpeedSlopeRpmS, 1, 65535))),
          pulsesPerRevolution_(config.pulsesPerRevolution),
          replyTimeoutMs_(config.motorReplyTimeoutMs),
          expectCommandAck_(config.motorExpectCommandAck) {}

    void setExpectCommandAck(bool expectCommandAck)
    {
        expectCommandAck_ = expectCommandAck;
    }

    bool readDriverParameters(ZdtDriverParameters& parameters)
    {
        const uint8_t command[] = {address_, 0x42, 0x6C, ZDT_TAIL};
        std::array<uint8_t, 33> response{};
        if (!exchange(command, sizeof(command), 0x42,
                      response.data(), response.size(), true)) {
            return false;
        }
        if (!decodeZdtDriverParametersResponse(
                response, address_, parameters)) {
            std::fprintf(stderr, "invalid ZDT driver parameter response\n");
            return false;
        }
        return true;
    }

    bool configureDriverParameters(const ZdtDriverParameters& parameters)
    {
        const uint32_t fullStepsPerRevolution =
            parameters.motorType == 25 ? 200u :
            parameters.motorType == 50 ? 400u : 0u;
        const uint32_t microsteps =
            parameters.microstep == 0 ? 256u : parameters.microstep;
        if (fullStepsPerRevolution == 0 || microsteps == 0) {
            std::fprintf(stderr,
                "unsupported Emm V5 motor type=%u microstep=%u\n",
                static_cast<unsigned>(parameters.motorType),
                static_cast<unsigned>(parameters.microstep));
            return false;
        }
        positionCommandPulsesPerRevolution_ =
            fullStepsPerRevolution * microsteps;
        return true;
    }

    uint32_t positionCommandPulsesPerRevolution() const
    {
        return positionCommandPulsesPerRevolution_;
    }

    bool readVersion(uint16_t& firmwareVersion, uint16_t& hardwareVersion)
    {
        const uint8_t command[] = {address_, 0x1F, ZDT_TAIL};
        // Emm V5 returns one byte each for firmware and hardware versions.
        uint8_t response[5]{};
        if (!exchange(command, sizeof(command), 0x1F,
                      response, sizeof(response), true)) {
            return false;
        }
        firmwareVersion = response[2];
        hardwareVersion = response[3];
        return true;
    }

    bool setEnabled(bool enabled)
    {
        const uint8_t frame[] = {
            address_, 0xF3, 0xAB,
            enabled ? uint8_t{0x01} : uint8_t{0x00},
            0x00, ZDT_TAIL
        };
        return sendControlCommand(frame, sizeof(frame), 0xF3);
    }

    bool enable() { return setEnabled(true); }
    bool disable() { return setEnabled(false); }

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

    double quantizeVelocityRpm(double signedRpm) const
    {
        const double limited = std::clamp(
            signedRpm,
            -static_cast<double>(maximumRpm_),
             static_cast<double>(maximumRpm_));
        return std::round(limited);
    }

    bool setVelocityRpm(double signedRpm)
    {
        const auto frame = makeEmmV5VelocityFrame(
            address_, quantizeVelocityRpm(signedRpm), maximumRpm_,
            speedSlopeRpmS_);
        return sendControlCommand(frame.data(), frame.size(), 0xF6);
    }

    bool moveRelativeDegrees(double signedDegrees,
                             double maximumRpm,
                             uint16_t accelerationRpmS)
    {
        const auto frame = makeEmmV5RelativePositionFrame(
            address_, signedDegrees, maximumRpm,
            accelerationRpmS, positionCommandPulsesPerRevolution_);
        return sendControlCommand(frame.data(), frame.size(), 0xFD);
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
            EMM_V5_POSITION_UNITS_PER_REVOLUTION;
        return true;
    }

    bool readTargetPositionDegrees(double& degrees)
    {
        return readPositionDegreesRegister(0x33, degrees);
    }

    bool readRealtimeTargetPositionDegrees(double& degrees)
    {
        return readPositionDegreesRegister(0x34, degrees);
    }

    bool readRealtimePositionDegrees(double& degrees)
    {
        return readPositionDegreesRegister(0x36, degrees);
    }

    bool readBusVoltageVolts(double& volts)
    {
        uint16_t millivolts = 0;
        if (!readUnsigned16Register(0x24, millivolts)) return false;
        volts = static_cast<double>(millivolts) / 1000.0;
        return true;
    }

    bool readPhaseCurrentMilliamps(uint16_t& milliamps)
    {
        return readUnsigned16Register(0x27, milliamps);
    }

    bool readMotorStatus(uint8_t& status)
    {
        return readStatusByte(0x3A, status);
    }

    bool readOriginStatus(uint8_t& status)
    {
        return readStatusByte(0x3B, status);
    }

    bool readMotionState(ZdtMotionState& state)
    {
        return readRealtimePositionSteps(state.positionSteps) &&
               readRealtimeSpeedRpm(state.speedRpm);
    }

    bool readPositionState(ZdtPositionState& state)
    {
        return readTargetPositionDegrees(state.targetDegrees) &&
               readRealtimeTargetPositionDegrees(
                   state.realtimeTargetDegrees) &&
               readRealtimePositionDegrees(state.actualDegrees);
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
    double wireCommandSpeedRpm = 0.0;
    double commandAccelerationRpmS = 0.0;
};

class EncoderSpeedEstimator {
    const AppConfig& config_;
    bool initialized_ = false;
    double previousPositionSteps_ = 0.0;
    double filteredSpeedRpm_ = 0.0;

public:
    explicit EncoderSpeedEstimator(const AppConfig& config)
        : config_(config) {}

    void reset()
    {
        initialized_ = false;
        previousPositionSteps_ = 0.0;
        filteredSpeedRpm_ = 0.0;
    }

    double update(double positionSteps,
                  double reportedSpeedRpm,
                  double dt)
    {
        const double boundedDt = std::clamp(dt, 0.001, 0.10);
        if (!initialized_) {
            initialized_ = true;
            previousPositionSteps_ = positionSteps;
            filteredSpeedRpm_ = reportedSpeedRpm;
            return filteredSpeedRpm_;
        }

        const double encoderSpeedRpm =
            (positionSteps - previousPositionSteps_) * 60.0 /
            (static_cast<double>(config_.pulsesPerRevolution) * boundedDt);
        previousPositionSteps_ = positionSteps;

        // Emm V5 0x35 has integer-RPM resolution. Use encoder position
        // changes below 1 RPM so the inner loop can still see slow motion.
        const double speedSampleRpm = std::clamp(
            std::abs(reportedSpeedRpm) >= 0.5 ?
                reportedSpeedRpm : encoderSpeedRpm,
            -static_cast<double>(config_.motorRpm),
             static_cast<double>(config_.motorRpm));
        const double alpha = boundedDt /
            (config_.motorEncoderSpeedFilterSeconds + boundedDt);
        filteredSpeedRpm_ += alpha *
            (speedSampleRpm - filteredSpeedRpm_);
        return filteredSpeedRpm_;
    }
};

class VelocityCommandQuantizer {
    double fractionalRpm_ = 0.0;
    int direction_ = 0;

public:
    void reset()
    {
        fractionalRpm_ = 0.0;
        direction_ = 0;
    }

    double update(double requestedRpm)
    {
        if (std::abs(requestedRpm) < 1e-9) {
            reset();
            return 0.0;
        }

        const int direction = requestedRpm > 0.0 ? 1 : -1;
        if (direction != direction_) {
            fractionalRpm_ = 0.0;
            direction_ = direction;
        }

        const double magnitude = std::abs(requestedRpm);
        double integerRpm = std::floor(magnitude);
        fractionalRpm_ += magnitude - integerRpm;
        if (fractionalRpm_ >= 1.0) {
            integerRpm += 1.0;
            fractionalRpm_ -= 1.0;
        }
        return static_cast<double>(direction) * integerRpm;
    }
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
    EncoderSpeedEstimator speedEstimator_;
    VelocityCommandQuantizer velocityQuantizer_;
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
        const double feedbackSpeedRpm = speedEstimator_.update(
            state.positionSteps, state.speedRpm, dt);
        telemetry_ = controller_.update(
            targetSteps_, state.positionSteps, feedbackSpeedRpm, dt);
        telemetry_.wireCommandSpeedRpm = velocityQuantizer_.update(
            telemetry_.commandSpeedRpm);
        if (!motor_.setVelocityRpm(telemetry_.wireCommandSpeedRpm)) {
            return false;
        }
        lastCycleMs_ = nowMs;
        return true;
    }

public:
    MotorCommander(EmmV5Motor& motor, const AppConfig& config)
        : motor_(motor),
          controller_(config),
          speedEstimator_(config),
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
                velocityQuantizer_.reset();
                return motor_.setVelocityRpm(0.0);
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(minimumIntervalMs_));
        }
        velocityQuantizer_.reset();
        motor_.setVelocityRpm(0.0);
        return false;
    }

    const MotorLoopTelemetry& telemetry() const { return telemetry_; }
};

} // namespace ball_stepper
