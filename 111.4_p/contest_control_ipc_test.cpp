#include "contest_control_ipc.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {

class RecordingHandler final : public contest_control::EventHandler {
public:
    contest_control::Frame lastFrame{};
    int count = 0;

    bool record(const contest_control::Frame& frame)
    {
        lastFrame = frame;
        ++count;
        return true;
    }

    bool selectQuestion(const contest_control::Frame& frame) override
    {
        return record(frame);
    }
    bool executeQuestion(const contest_control::Frame& frame) override
    {
        return record(frame);
    }
    bool setMotorEnabled(
        bool, const contest_control::Frame& frame) override
    {
        return record(frame);
    }
    bool returnToOrigin(const contest_control::Frame& frame) override
    {
        return record(frame);
    }
    bool setCurrentPositionAsOrigin(
        const contest_control::Frame& frame) override
    {
        return record(frame);
    }
};

class RecordingTelemetryHandler final :
    public contest_control::TelemetryHandler {
public:
    contest_control::TelemetryFrame lastFrame{};
    int count = 0;

    bool handleTelemetry(
        const contest_control::TelemetryFrame& frame) override
    {
        lastFrame = frame;
        ++count;
        return true;
    }
};

void require(bool condition, const char* message)
{
    if (condition) return;
    std::fprintf(stderr, "contest_control_ipc_test: %s\n", message);
    std::exit(1);
}

}  // namespace

int main()
{
    constexpr uint16_t testPort = 17993;
    contest_control::UdpFrameReceiver receiver;
    contest_control::UdpFrameSender sender;
    require(receiver.openPort(testPort), receiver.lastError().c_str());
    require(sender.openSocket(), sender.lastError().c_str());

    contest_control::Frame frame;
    require(
        contest_control::makeFrame(
            contest_control::Event::MotorToggle,
            3,
            contest_control::State::Ready,
            0x3412,
            contest_control::kFlagMotorEnabled |
                contest_control::kFlagOriginSet,
            frame) == contest_control::DecodeError::None,
        "could not create test frame");
    require(sender.send(frame, testPort), sender.lastError().c_str());

    RecordingHandler handler;
    for (int attempt = 0; attempt < 100 && handler.count == 0; ++attempt) {
        require(receiver.receiveAndDispatch(handler),
                receiver.lastError().c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    require(handler.count == 1, "loopback frame was not delivered");
    require(handler.lastFrame.event == contest_control::Event::MotorToggle,
            "event changed during IPC forwarding");
    require(handler.lastFrame.question == 3,
            "question changed during IPC forwarding");
    require(handler.lastFrame.sequence == 0x3412,
            "little-endian sequence changed during IPC forwarding");
    require(handler.lastFrame.flags ==
                (contest_control::kFlagMotorEnabled |
                 contest_control::kFlagOriginSet),
            "flags changed during IPC forwarding");

    contest_control::TelemetryFrame telemetry;
    telemetry.question = 4;
    telemetry.state = contest_control::State::Running;
    telemetry.flags = contest_control::kFlagPidEnabled |
        contest_control::kFlagMotorEnabled;
    telemetry.sequence = 0x3413;
    telemetry.leftSpeed50ms = -72;
    telemetry.rightSpeed50ms = 75;
    telemetry.leftTarget50ms = 80;
    telemetry.rightTarget50ms = 80;
    telemetry.distanceSteps = 12000;
    telemetry.elapsedMs = 8000;
    require(sender.send(telemetry, testPort), sender.lastError().c_str());

    RecordingTelemetryHandler telemetryHandler;
    for (int attempt = 0;
         attempt < 100 && telemetryHandler.count == 0; ++attempt) {
        require(receiver.receiveAndDispatch(
                    handler, nullptr, &telemetryHandler),
                receiver.lastError().c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(telemetryHandler.count == 1,
            "loopback telemetry was not delivered");
    require(telemetryHandler.lastFrame.leftSpeed50ms == -72,
            "signed telemetry changed during IPC forwarding");
    require(telemetryHandler.lastFrame.distanceSteps == 12000,
            "telemetry distance changed during IPC forwarding");

    std::puts("contest_control_ipc_test: PASS");
    return 0;
}
