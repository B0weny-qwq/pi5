#include "zdt_stepper_uart.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace {

using namespace ball_stepper;

constexpr double MANUAL_MINIMUM_DEGREES = 1.0;
constexpr double MANUAL_MAXIMUM_DEGREES = 10.0;
constexpr double MANUAL_MAXIMUM_RPM = 6.0;
constexpr uint16_t MANUAL_ACCELERATION_RPM_S = 200;
constexpr int COMMAND_SETTLE_MS = 80;
constexpr int POLL_INTERVAL_MS = 40;

std::atomic<bool> interrupted{false};

void handleSignal(int)
{
    interrupted.store(true);
}

AppConfig makeMotorConfig()
{
    AppConfig config;
    config.serialPort = "/dev/ttyAMA0";
    config.serialBaud = 115200;
    config.motorAddress = 1;
    config.pulsesPerRevolution = 200;
    config.motorSign = 1;
    config.motorRpm = static_cast<int>(MANUAL_MAXIMUM_RPM);
    config.motorSpeedSlopeRpmS = MANUAL_ACCELERATION_RPM_S;
    config.motorReplyTimeoutMs = 30;
    // The driver is configured with Response=None. Read commands still reply.
    config.motorExpectCommandAck = false;
    return config;
}

void printUsage(const char* program)
{
    std::fprintf(stderr,
        "Usage:\n"
        "  %s status\n"
        "  %s up DEGREES\n"
        "  %s down DEGREES\n"
        "  %s move SIGNED_DEGREES\n"
        "  %s zero\n"
        "  %s stop\n"
        "  %s enable\n"
        "  %s disable\n"
        "\n"
        "Moves are relative motor-shaft angles, limited to %.1f..%.1f degrees.\n"
        "Default motion: %.1f RPM, %u RPM/s acceleration and deceleration.\n",
        program, program, program, program, program, program, program, program,
        MANUAL_MINIMUM_DEGREES, MANUAL_MAXIMUM_DEGREES,
        MANUAL_MAXIMUM_RPM,
        static_cast<unsigned>(MANUAL_ACCELERATION_RPM_S));
}

bool parseDegrees(const char* text, double& value)
{
    if (!text || *text == '\0') return false;
    char* end = nullptr;
    errno = 0;
    value = std::strtod(text, &end);
    return errno == 0 && end && *end == '\0' && std::isfinite(value);
}

bool readStatus(EmmV5Motor& motor,
                const AppConfig& config,
                ZdtMotionState& motion,
                uint8_t& motorStatus,
                uint8_t& originStatus)
{
    if (!motor.readMotionState(motion) ||
        !motor.readMotorStatus(motorStatus) ||
        !motor.readOriginStatus(originStatus)) {
        std::fprintf(stderr, "ERROR: failed to read ZDT status\n");
        return false;
    }

    const double motorDegrees = motion.positionSteps * 360.0 /
        static_cast<double>(config.pulsesPerRevolution);
    std::printf(
        "position_steps=%+.1f estimated_motor_deg=%+.2f speed_rpm=%+.1f "
        "enabled=%d arrived=%d stall=%d protect=%d "
        "encoder_ready=%d calibration_ready=%d\n",
        motion.positionSteps, motorDegrees, motion.speedRpm,
        (motorStatus & ZDT_STATUS_ENABLED) ? 1 : 0,
        (motorStatus & ZDT_STATUS_ARRIVED) ? 1 : 0,
        (motorStatus & ZDT_STATUS_STALL) ? 1 : 0,
        (motorStatus & ZDT_STATUS_STALL_PROTECTION) ? 1 : 0,
        (originStatus & 0x01) ? 1 : 0,
        (originStatus & 0x02) ? 1 : 0);
    return true;
}

int runMove(EmmV5Motor& motor,
            const AppConfig& config,
            double signedDegrees)
{
    ZdtMotionState initialMotion;
    uint8_t initialStatus = 0;
    uint8_t originStatus = 0;
    if (!readStatus(
            motor, config, initialMotion, initialStatus, originStatus)) {
        return 1;
    }
    if (initialStatus & (ZDT_STATUS_STALL | ZDT_STATUS_STALL_PROTECTION)) {
        std::fprintf(stderr,
            "ERROR: motor is stalled or protected; movement refused\n");
        return 1;
    }

    if (!motor.enable()) {
        std::fprintf(stderr, "ERROR: enable command failed\n");
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(COMMAND_SETTLE_MS));

    uint8_t enabledStatus = 0;
    if (!motor.readMotorStatus(enabledStatus) ||
        !(enabledStatus & ZDT_STATUS_ENABLED)) {
        std::fprintf(stderr, "ERROR: motor did not enter enabled state\n");
        return 1;
    }

    if (!motor.stop()) {
        std::fprintf(stderr, "ERROR: pre-move stop command failed\n");
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    if (!motor.moveRelativeDegrees(
            signedDegrees,
            MANUAL_MAXIMUM_RPM,
            MANUAL_ACCELERATION_RPM_S,
            MANUAL_ACCELERATION_RPM_S)) {
        std::fprintf(stderr, "ERROR: 0xFD position command failed\n");
        return 1;
    }

    std::printf(
        "move_sent relative_motor_deg=%+.1f max_rpm=%.1f accel_rpm_s=%u\n",
        signedDegrees, MANUAL_MAXIMUM_RPM,
        static_cast<unsigned>(MANUAL_ACCELERATION_RPM_S));

    const double nominalSeconds = std::abs(signedDegrees) /
        (MANUAL_MAXIMUM_RPM * 6.0);
    const double timeoutSeconds = std::max(3.0, nominalSeconds * 4.0 + 1.0);
    const auto started = std::chrono::steady_clock::now();
    bool observedMoving = false;

    while (!interrupted.load()) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(POLL_INTERVAL_MS));

        uint8_t status = 0;
        if (!motor.readMotorStatus(status)) {
            motor.stop();
            return 1;
        }
        if (status & (ZDT_STATUS_STALL | ZDT_STATUS_STALL_PROTECTION)) {
            motor.stop();
            std::fprintf(stderr,
                "ERROR: stall/protection detected; immediate stop sent\n");
            return 1;
        }
        if (!(status & ZDT_STATUS_ARRIVED)) observedMoving = true;

        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        if ((status & ZDT_STATUS_ARRIVED) &&
            (observedMoving || elapsed >= nominalSeconds + 0.20)) {
            ZdtMotionState finalMotion;
            uint8_t finalStatus = 0;
            uint8_t finalOriginStatus = 0;
            if (!readStatus(motor, config, finalMotion,
                            finalStatus, finalOriginStatus)) {
                return 1;
            }

            const double expectedSteps = std::abs(signedDegrees) /
                360.0 * config.pulsesPerRevolution;
            const double actualSteps = std::abs(
                finalMotion.positionSteps - initialMotion.positionSteps);
            const double minimumProgress = std::max(0.5, expectedSteps * 0.4);
            if (actualSteps < minimumProgress) {
                std::fprintf(stderr,
                    "ERROR: command reported arrived but position changed only %.1f steps\n",
                    actualSteps);
                return 1;
            }
            std::printf(
                "move_complete requested_deg=%+.1f delta_steps=%+.1f elapsed_s=%.3f\n",
                signedDegrees,
                finalMotion.positionSteps - initialMotion.positionSteps,
                elapsed);
            return 0;
        }
        if (elapsed >= timeoutSeconds) {
            motor.stop();
            std::fprintf(stderr,
                "ERROR: movement timed out after %.2f s; immediate stop sent\n",
                elapsed);
            return 1;
        }
    }

    motor.stop();
    std::fprintf(stderr, "Interrupted: immediate stop sent\n");
    return 130;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (argc < 2 || std::string(argv[1]) == "--help" ||
        std::string(argv[1]) == "-h") {
        printUsage(argv[0]);
        return argc < 2 ? 2 : 0;
    }

    const std::string command(argv[1]);
    double signedDegrees = 0.0;
    if (command == "move" || command == "up" || command == "down") {
        double degrees = 0.0;
        if (argc != 3 || !parseDegrees(argv[2], degrees)) {
            printUsage(argv[0]);
            return 2;
        }
        if (command != "move" && degrees <= 0.0) {
            std::fprintf(stderr, "ERROR: up/down requires a positive angle\n");
            return 2;
        }
        signedDegrees = command == "down" ? -degrees : degrees;
        if (std::abs(signedDegrees) < MANUAL_MINIMUM_DEGREES ||
            std::abs(signedDegrees) > MANUAL_MAXIMUM_DEGREES) {
            std::fprintf(stderr,
                "ERROR: manual movement must be between %.1f and %.1f degrees\n",
                MANUAL_MINIMUM_DEGREES, MANUAL_MAXIMUM_DEGREES);
            return 2;
        }
    } else if (argc != 2 ||
               (command != "status" && command != "zero" &&
                command != "stop" && command != "enable" &&
                command != "disable")) {
        printUsage(argv[0]);
        return 2;
    }

    AppConfig config = makeMotorConfig();
    if (command == "up") signedDegrees *= config.motorSign;
    if (command == "down") signedDegrees *= config.motorSign;

    SerialPort serial;
    if (!serial.openPort(config.serialPort, config.serialBaud)) return 1;
    EmmV5Motor motor(serial, config);

    if (command == "move" || command == "up" || command == "down") {
        return runMove(motor, config, signedDegrees);
    }

    if (command == "status") {
        ZdtMotionState motion;
        uint8_t motorStatus = 0;
        uint8_t originStatus = 0;
        return readStatus(
            motor, config, motion, motorStatus, originStatus) ? 0 : 1;
    }
    if (command == "enable") {
        if (!motor.enable()) return 1;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(COMMAND_SETTLE_MS));
    } else if (command == "stop") {
        if (!motor.stop()) return 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    } else if (command == "zero") {
        if (!motor.stop()) return 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        if (!motor.clearPosition()) return 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } else if (command == "disable") {
        if (!motor.stop()) return 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        if (!motor.disable()) return 1;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(COMMAND_SETTLE_MS));
    }

    ZdtMotionState motion;
    uint8_t motorStatus = 0;
    uint8_t originStatus = 0;
    return readStatus(
        motor, config, motion, motorStatus, originStatus) ? 0 : 1;
}
