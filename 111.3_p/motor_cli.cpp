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
constexpr uint16_t ABSOLUTE_HOME_RPM = 6;
constexpr int ABSOLUTE_HOME_TIMEOUT_MS = 5000;
constexpr int ABSOLUTE_HOME_POLL_MS = 40;
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
    config.motorSign = -1;
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
        "  %s origin-set\n"
        "  %s home\n"
        "  %s zero                 (volatile runtime coordinate only)\n"
        "  %s stop\n"
        "  %s enable\n"
        "  %s disable\n"
        "\n"
        "Moves are relative motor-shaft angles, limited to %.1f..%.1f degrees.\n"
        "Default motion: %.1f RPM, %u RPM/s physical acceleration.\n",
        program, program, program, program, program, program, program, program,
        program, program,
        MANUAL_MINIMUM_DEGREES, MANUAL_MAXIMUM_DEGREES,
        MANUAL_MAXIMUM_RPM,
        static_cast<unsigned>(MANUAL_ACCELERATION_RPM_S));
}

template <std::size_t Size>
void printFrame(const std::array<uint8_t, Size>& frame)
{
    std::printf("tx_frame=");
    for (std::size_t index = 0; index < frame.size(); ++index) {
        std::printf("%s%02X", index == 0 ? "" : " ",
                    static_cast<unsigned>(frame[index]));
    }
    std::printf("\n");
}

const char* pulseControlModeName(uint8_t mode)
{
    if (mode == 0x00) return "PUL_OPEN";
    if (mode == 0x01) return "PUL_FOC";
    if (mode == 0x02) return "PUL_FOC";
    return "UNKNOWN";
}

const char* serialFunctionName(uint8_t mode)
{
    if (mode == 0x00) return "RxTx_OFF";
    if (mode == 0x01) return "ESI_ALO";
    if (mode == 0x02) return "UART_FUN";
    if (mode == 0x03) return "CAN1_MAP";
    return "UNKNOWN";
}

const char* responseModeName(uint8_t mode)
{
    if (mode == 0x00) return "None";
    if (mode == 0x01) return "Receive";
    if (mode == 0x02) return "Reached";
    if (mode == 0x03) return "Both";
    if (mode == 0x04) return "Other";
    return "UNKNOWN";
}

bool readAndPrintDriverInfo(EmmV5Motor& motor)
{
    ZdtDriverParameters parameters;
    uint16_t firmwareVersion = 0;
    uint16_t hardwareVersion = 0;
    if (!motor.readVersion(firmwareVersion, hardwareVersion)) {
        std::fprintf(stderr,
            "ERROR: failed to read Emm V5 firmware version\n");
        return false;
    }
    if (!motor.readDriverParameters(parameters)) {
        std::fprintf(stderr,
            "ERROR: failed to read Emm V5 driver configuration\n");
        return false;
    }

    const bool immediateAck =
        zdtResponseModeHasImmediateAck(parameters.responseMode);
    if (!motor.configureDriverParameters(parameters)) return false;
    motor.setExpectCommandAck(immediateAck);

    const double motorStepDegrees = parameters.motorType == 25 ? 1.8 :
                                    parameters.motorType == 50 ? 0.9 : 0.0;

    std::printf(
        "driver firmware=%u hardware=%u motor_type=%u(%.1f_deg) "
        "pulse_mode=%u(%s) "
        "p_serial=%u(%s) en_mode=%u mstep=%u uart_baud_index=%u "
        "checksum=%u response=%u(%s) command_ppr=%u ack_expected=%d\n",
        static_cast<unsigned>(firmwareVersion),
        static_cast<unsigned>(hardwareVersion),
        static_cast<unsigned>(parameters.motorType), motorStepDegrees,
        static_cast<unsigned>(parameters.pulseControlMode),
        pulseControlModeName(parameters.pulseControlMode),
        static_cast<unsigned>(parameters.serialPortFunction),
        serialFunctionName(parameters.serialPortFunction),
        static_cast<unsigned>(parameters.enableMode),
        static_cast<unsigned>(parameters.microstep),
        static_cast<unsigned>(parameters.uartBaudIndex),
        static_cast<unsigned>(parameters.checksumMode),
        static_cast<unsigned>(parameters.responseMode),
        responseModeName(parameters.responseMode),
        static_cast<unsigned>(
            motor.positionCommandPulsesPerRevolution()),
        immediateAck ? 1 : 0);
    return true;
}

void printHomingParameters(const ZdtHomingParameters& parameters)
{
    std::printf(
        "home_mode=%u home_rpm=%u timeout_ms=%u power_on_auto=%d\n",
        static_cast<unsigned>(parameters.mode),
        static_cast<unsigned>(parameters.velocityRpm),
        static_cast<unsigned>(parameters.timeoutMs),
        parameters.powerOnAutomatic ? 1 : 0);
}

bool saveCurrentPositionAsAbsoluteOrigin(EmmV5Motor& motor)
{
    if (!motor.stop()) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    ZdtHomingParameters parameters;
    if (!motor.readHomingParameters(parameters)) {
        std::fprintf(stderr, "ERROR: cannot read ZDT homing parameters\n");
        return false;
    }
    parameters.mode = ZdtHomingMode::Nearest;
    parameters.velocityRpm = ABSOLUTE_HOME_RPM;
    parameters.timeoutMs = ABSOLUTE_HOME_TIMEOUT_MS;
    // The application triggers and monitors homing. Do not let the mechanism
    // move immediately when driver power is applied.
    parameters.powerOnAutomatic = false;
    if (!motor.writeHomingParameters(parameters, true)) {
        std::fprintf(stderr, "ERROR: cannot store ZDT homing parameters\n");
        return false;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(COMMAND_SETTLE_MS));

    ZdtHomingParameters verified;
    if (!motor.readHomingParameters(verified)) return false;
    printHomingParameters(verified);
    if (verified.mode != ZdtHomingMode::Nearest ||
        verified.velocityRpm != ABSOLUTE_HOME_RPM ||
        verified.timeoutMs != ABSOLUTE_HOME_TIMEOUT_MS ||
        verified.powerOnAutomatic) {
        std::fprintf(stderr,
            "ERROR: driver did not retain the requested safe homing settings\n");
        return false;
    }

    if (!motor.setSingleTurnOrigin(true)) {
        std::fprintf(stderr,
            "ERROR: persistent single-turn absolute origin write failed\n");
        return false;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(COMMAND_SETTLE_MS));
    if (!motor.clearPosition()) {
        std::fprintf(stderr, "ERROR: runtime position clear failed\n");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::printf(
        "absolute_origin_saved=1 current_position_is_persistent_level_zero\n");
    return true;
}

bool returnToAbsoluteOrigin(EmmV5Motor& motor)
{
    ZdtHomingParameters parameters;
    if (!motor.readHomingParameters(parameters)) return false;
    printHomingParameters(parameters);
    if (parameters.mode != ZdtHomingMode::Nearest ||
        parameters.velocityRpm != ABSOLUTE_HOME_RPM ||
        parameters.powerOnAutomatic) {
        std::fprintf(stderr,
            "ERROR: unsafe homing configuration; run origin-set at LEVEL\n");
        return false;
    }
    if (!motor.enable()) return false;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(COMMAND_SETTLE_MS));
    if (!motor.stop()) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    std::printf("absolute_home_started=1 rpm=%u\n",
                static_cast<unsigned>(parameters.velocityRpm));
    if (!motor.homeToStoredSingleTurnOrigin(
            ABSOLUTE_HOME_TIMEOUT_MS, ABSOLUTE_HOME_POLL_MS)) {
        return false;
    }
    if (!motor.clearPosition()) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::printf("absolute_home_completed=1 runtime_position_zeroed=1\n");
    return true;
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
                uint8_t& originStatus,
                ZdtPositionState* positionState = nullptr)
{
    ZdtPositionState positions;
    double busVoltage = 0.0;
    uint16_t phaseCurrentMilliamps = 0;
    if (!motor.readMotionState(motion) ||
        !motor.readPositionState(positions) ||
        !motor.readBusVoltageVolts(busVoltage) ||
        !motor.readPhaseCurrentMilliamps(phaseCurrentMilliamps) ||
        !motor.readMotorStatus(motorStatus) ||
        !motor.readOriginStatus(originStatus)) {
        std::fprintf(stderr, "ERROR: failed to read ZDT status\n");
        return false;
    }

    if (positionState) *positionState = positions;

    const double motorDegrees = motion.positionSteps * 360.0 /
        static_cast<double>(config.pulsesPerRevolution);
    std::printf(
        "position_steps=%+.1f pulse_est_deg=%+.2f actual_deg=%+.1f "
        "target_deg=%+.1f trajectory_deg=%+.1f speed_rpm=%+.1f "
        "bus_v=%.2f phase_ma=%u "
        "enabled=%d arrived=%d stall=%d protect=%d "
        "encoder_ready=%d calibration_ready=%d\n",
        motion.positionSteps, motorDegrees, positions.actualDegrees,
        positions.targetDegrees, positions.realtimeTargetDegrees,
        motion.speedRpm, busVoltage,
        static_cast<unsigned>(phaseCurrentMilliamps),
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
    ZdtPositionState initialPosition;
    if (!readStatus(
            motor, config, initialMotion, initialStatus, originStatus,
            &initialPosition)) {
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

    const uint32_t commandPpr =
        motor.positionCommandPulsesPerRevolution();
    const uint32_t commandPulses = static_cast<uint32_t>(std::llround(
        std::abs(signedDegrees) * static_cast<double>(commandPpr) / 360.0));
    const uint8_t accelerationLevel =
        makeEmmV5AccelerationLevel(MANUAL_ACCELERATION_RPM_S);
    const auto frame = makeEmmV5RelativePositionFrame(
        static_cast<uint8_t>(config.motorAddress), signedDegrees,
        MANUAL_MAXIMUM_RPM, MANUAL_ACCELERATION_RPM_S, commandPpr);
    printFrame(frame);

    if (!motor.moveRelativeDegrees(
            signedDegrees,
            MANUAL_MAXIMUM_RPM,
            MANUAL_ACCELERATION_RPM_S)) {
        std::fprintf(stderr, "ERROR: 0xFD position command failed\n");
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    ZdtPositionState acceptedPosition;
    if (!motor.readPositionState(acceptedPosition)) {
        std::fprintf(stderr,
            "ERROR: failed to read position registers after 0xFD\n");
        return 1;
    }

    const double expectedTargetDegrees =
        initialPosition.actualDegrees + signedDegrees;

    std::printf(
        "move_sent relative_motor_deg=%+.1f command_pulses=%u "
        "command_ppr=%u max_rpm=%.1f accel_rpm_s=%u accel_level=%u "
        "target_before=%+.1f target_after=%+.1f expected_target=%+.1f "
        "trajectory_after=%+.1f actual_after=%+.1f\n",
        signedDegrees, static_cast<unsigned>(commandPulses),
        static_cast<unsigned>(commandPpr), MANUAL_MAXIMUM_RPM,
        static_cast<unsigned>(MANUAL_ACCELERATION_RPM_S),
        static_cast<unsigned>(accelerationLevel),
        initialPosition.targetDegrees, acceptedPosition.targetDegrees,
        expectedTargetDegrees, acceptedPosition.realtimeTargetDegrees,
        acceptedPosition.actualDegrees);

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
            ZdtPositionState finalPosition;
            if (!readStatus(motor, config, finalMotion,
                            finalStatus, finalOriginStatus,
                            &finalPosition)) {
                return 1;
            }

            const double actualDegrees = std::abs(
                finalPosition.actualDegrees - initialPosition.actualDegrees);
            const double minimumProgressDegrees =
                std::max(0.2, std::abs(signedDegrees) * 0.4);
            if (actualDegrees < minimumProgressDegrees) {
                std::fprintf(stderr,
                    "ERROR: arrived without motion: target_delta=%+.1f deg "
                    "actual_delta=%+.1f deg pulse_delta=%+.1f\n",
                    acceptedPosition.targetDegrees -
                        initialPosition.targetDegrees,
                    finalPosition.actualDegrees -
                        initialPosition.actualDegrees,
                    finalMotion.positionSteps - initialMotion.positionSteps);
                return 1;
            }
            std::printf(
                "move_complete requested_deg=%+.1f actual_delta_deg=%+.1f "
                "pulse_delta=%+.1f elapsed_s=%.3f\n",
                signedDegrees,
                finalPosition.actualDegrees - initialPosition.actualDegrees,
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
                command != "origin-set" && command != "home" &&
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

    if (!readAndPrintDriverInfo(motor)) return 1;

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
    if (command == "origin-set") {
        if (!saveCurrentPositionAsAbsoluteOrigin(motor)) return 1;
    } else if (command == "home") {
        if (!returnToAbsoluteOrigin(motor)) return 1;
    } else
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
