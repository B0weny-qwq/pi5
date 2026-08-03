#include "contest_control_uart.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

#if defined(__linux__)
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#endif

using namespace contest_control;

namespace {

std::array<uint8_t, kFrameSize> encodeChecked(const Frame& frame)
{
    std::array<uint8_t, kFrameSize> bytes{};
    assert(encodeFrame(frame, bytes) == DecodeError::None);
    return bytes;
}

void testPublishedExample()
{
    Frame frame;
    assert(makeFrame(
        Event::Select, 1, State::Ready, 1, 0, frame) == DecodeError::None);

    const std::array<uint8_t, kFrameSize> expected = {
        0xA5, 0x5A, 0x01, 0x0C, 0x01, 0x01,
        0x00, 0x00, 0x01, 0x00, 0x00, 0xF3
    };
    assert(encodeChecked(frame) == expected);

    Frame decoded;
    assert(decodeFrame(expected.data(), expected.size(), decoded) ==
           DecodeError::None);
    assert(decoded.event == Event::Select);
    assert(decoded.question == 1);
    assert(decoded.mode == Mode::UartTask);
    assert(decoded.state == State::Ready);
    assert(decoded.sequence == 1);
    assert(decoded.flags == 0);
}

void testLittleEndianAndFlags()
{
    Frame frame;
    assert(makeFrame(
        Event::Execute,
        4,
        State::Running,
        0x1234,
        kFlagPidEnabled | kFlagMotorEnabled | kFlagOriginSet,
        frame) == DecodeError::None);

    const auto bytes = encodeChecked(frame);
    assert(bytes[6] == static_cast<uint8_t>(Mode::TrackingPid));
    assert(bytes[8] == 0x34);
    assert(bytes[9] == 0x12);
    assert(bytes[10] == 0x07);

    Frame decoded;
    assert(decodeFrame(bytes.data(), bytes.size(), decoded) ==
           DecodeError::None);
    assert(decoded.pidEnabled());
    assert(decoded.motorEnabled());
    assert(decoded.originSet());
}

void testValidation()
{
    Frame frame;
    assert(makeFrame(
        Event::Select, 0, State::Ready, 0, 0, frame) ==
        DecodeError::InvalidQuestion);
    assert(makeFrame(
        Event::MotorToggle, 2, State::Running, 0, kFlagMotorEnabled, frame) ==
        DecodeError::EventNotAllowedForQuestion);

    assert(makeFrame(
        Event::Select, 2, State::Ready, 0, 0, frame) == DecodeError::None);
    auto bytes = encodeChecked(frame);

    bytes[6] = static_cast<uint8_t>(Mode::UartTask);
    bytes.back() = xorChecksum(bytes.data(), bytes.size() - 1);
    assert(decodeFrame(bytes.data(), bytes.size(), frame) ==
           DecodeError::QuestionModeMismatch);

    bytes[6] = static_cast<uint8_t>(Mode::TrackingPid);
    bytes[10] = 0x80;
    bytes.back() = xorChecksum(bytes.data(), bytes.size() - 1);
    assert(decodeFrame(bytes.data(), bytes.size(), frame) ==
           DecodeError::ReservedFlagsSet);

    bytes[10] = 0;
    bytes.back() ^= 0x01;
    assert(decodeFrame(bytes.data(), bytes.size(), frame) ==
           DecodeError::ChecksumMismatch);
}

void testStreamingAndResynchronization()
{
    Frame first;
    Frame second;
    assert(makeFrame(
        Event::Select, 3, State::Ready, 10, 0, first) == DecodeError::None);
    assert(makeFrame(
        Event::Execute, 3, State::Sent, 11, kFlagMotorEnabled, second) ==
        DecodeError::None);

    const auto firstBytes = encodeChecked(first);
    const auto secondBytes = encodeChecked(second);

    StreamParser parser;
    const std::vector<uint8_t> prefix = {0x00, 0x7E, 0xA5};
    assert(parser.push(prefix).empty());

    std::vector<uint8_t> firstRemainder = {0x5A};
    firstRemainder.insert(
        firstRemainder.end(), firstBytes.begin() + 2, firstBytes.end());
    auto frames = parser.push(firstRemainder);
    assert(frames.size() == 1);
    assert(frames[0].sequence == 10);

    auto corrupt = firstBytes;
    corrupt.back() ^= 0x80;
    std::vector<uint8_t> combined(corrupt.begin(), corrupt.end());
    combined.insert(combined.end(), secondBytes.begin(), secondBytes.end());
    frames = parser.push(combined);
    assert(frames.size() == 1);
    assert(frames[0].event == Event::Execute);
    assert(frames[0].sequence == 11);
    assert(parser.stats().checksumErrors == 1);
    assert(parser.stats().framesAccepted == 2);
    assert(parser.stats().discardedBytes >= 3);
}

void testSequenceTrackingAndWrap()
{
    SequenceEncoder encoder(0xFFFF);
    Frame frame;
    std::array<uint8_t, kFrameSize> bytes{};
    assert(encoder.next(
        Event::Select, 1, State::Ready, 0, frame, bytes) ==
        DecodeError::None);
    assert(frame.sequence == 0xFFFF);
    assert(encoder.nextSequence() == 0);

    assert(encoder.next(
        Event::Select, 1, State::Ready, 0, frame, bytes) ==
        DecodeError::None);
    assert(frame.sequence == 0);
    assert(encoder.nextSequence() == 1);

    StreamParser parser;
    for (const uint16_t sequence : {uint16_t{0xFFFF}, uint16_t{0},
                                    uint16_t{0}, uint16_t{2}}) {
        assert(makeFrame(
            Event::Select, 1, State::Ready, sequence, 0, frame) ==
            DecodeError::None);
        const auto encoded = encodeChecked(frame);
        assert(parser.push(encoded.data(), encoded.size()).size() == 1);
    }
    assert(parser.stats().duplicateSequences == 1);
    assert(parser.stats().sequenceDiscontinuities == 1);
}

class RecordingEventHandler final : public EventHandler {
public:
    int selectCount = 0;
    int executeCount = 0;
    int motorCount = 0;
    int returnOriginCount = 0;
    int setOriginCount = 0;
    bool requestedMotorEnabled = false;
    bool rejectExecute = false;

    bool selectQuestion(const Frame&) override
    {
        ++selectCount;
        return true;
    }

    bool executeQuestion(const Frame&) override
    {
        ++executeCount;
        return !rejectExecute;
    }

    bool setMotorEnabled(bool enabled, const Frame&) override
    {
        ++motorCount;
        requestedMotorEnabled = enabled;
        return true;
    }

    bool returnToOrigin(const Frame&) override
    {
        ++returnOriginCount;
        return true;
    }

    bool setCurrentPositionAsOrigin(const Frame&) override
    {
        ++setOriginCount;
        return true;
    }
};

void testFunctionalDispatch()
{
    RecordingEventHandler handler;
    Frame frame;

    assert(makeFrame(
        Event::Select, 1, State::Ready, 1, 0, frame) == DecodeError::None);
    assert(dispatchFrame(frame, handler) == DispatchResult::Handled);

    assert(makeFrame(
        Event::Execute, 2, State::Running, 2, kFlagPidEnabled, frame) ==
        DecodeError::None);
    assert(dispatchFrame(frame, handler) == DispatchResult::Handled);

    assert(makeFrame(
        Event::MotorToggle,
        4,
        State::Running,
        3,
        kFlagPidEnabled | kFlagMotorEnabled,
        frame) == DecodeError::None);
    assert(dispatchFrame(frame, handler) == DispatchResult::Handled);
    assert(handler.requestedMotorEnabled);

    assert(makeFrame(
        Event::ReturnOrigin, 3, State::Sent, 4, kFlagOriginSet, frame) ==
        DecodeError::None);
    assert(dispatchFrame(frame, handler) == DispatchResult::Handled);

    assert(makeFrame(
        Event::SetOrigin, 5, State::Sent, 5, kFlagOriginSet, frame) ==
        DecodeError::None);
    assert(dispatchFrame(frame, handler) == DispatchResult::Handled);

    assert(handler.selectCount == 1);
    assert(handler.executeCount == 1);
    assert(handler.motorCount == 1);
    assert(handler.returnOriginCount == 1);
    assert(handler.setOriginCount == 1);

    handler.rejectExecute = true;
    assert(makeFrame(
        Event::Execute, 3, State::Sent, 6, 0, frame) == DecodeError::None);
    assert(dispatchFrame(frame, handler) == DispatchResult::HandlerRejected);

    frame = {
        Event::MotorToggle,
        2,
        Mode::TrackingPid,
        State::Running,
        7,
        kFlagMotorEnabled
    };
    assert(dispatchFrame(frame, handler) == DispatchResult::InvalidFrame);
}

#if defined(__linux__)
void testLinuxFullDuplexUart()
{
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0);
    assert(::grantpt(master) == 0);
    assert(::unlockpt(master) == 0);
    const char* slave = ::ptsname(master);
    assert(slave != nullptr);

    ControlUart uart;
    assert(uart.openPort(slave));
    termios configured{};
    assert(::tcgetattr(master, &configured) == 0);
    assert(::cfgetispeed(&configured) == B230400);
    assert(::cfgetospeed(&configured) == B230400);

    Frame incoming;
    assert(makeFrame(
        Event::Execute, 3, State::Sent, 42, kFlagMotorEnabled, incoming) ==
        DecodeError::None);
    const auto incomingBytes = encodeChecked(incoming);
    assert(::write(master, incomingBytes.data(), 5) == 5);
    assert(::write(master, incomingBytes.data() + 5, 7) == 7);

    RecordingEventHandler handler;
    std::size_t dispatched = 0;
    for (int attempt = 0; attempt < 50 && handler.executeCount == 0; ++attempt) {
        assert(uart.receiveAndDispatch(handler, &dispatched));
        if (handler.executeCount == 0) ::usleep(1000);
    }
    assert(dispatched == 1);
    assert(handler.executeCount == 1);

    uart.setNextTransmitSequence(0xFFFF);
    Frame sent;
    assert(uart.send(
        Event::MotorToggle,
        4,
        State::Sent,
        kFlagOriginSet,
        &sent));
    assert(sent.sequence == 0xFFFF);
    assert(uart.nextTransmitSequence() == 0);

    std::array<uint8_t, kFrameSize> transmitted{};
    std::size_t used = 0;
    for (int attempt = 0; attempt < 50 && used < transmitted.size(); ++attempt) {
        const ssize_t count = ::read(
            master, transmitted.data() + used, transmitted.size() - used);
        if (count > 0) {
            used += static_cast<std::size_t>(count);
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                   errno != EINTR) {
            assert(false);
        }
        if (used < transmitted.size()) ::usleep(1000);
    }
    assert(used == transmitted.size());

    Frame decoded;
    assert(decodeFrame(transmitted.data(), transmitted.size(), decoded) ==
           DecodeError::None);
    assert(decoded.event == Event::MotorToggle);
    assert(decoded.question == 4);
    assert(decoded.mode == Mode::TrackingPid);
    assert(decoded.sequence == 0xFFFF);
    assert(decoded.flags == kFlagOriginSet);

    uart.closePort();
    ::close(master);
}
#endif

} // namespace

int main()
{
    testPublishedExample();
    testLittleEndianAndFlags();
    testValidation();
    testStreamingAndResynchronization();
    testSequenceTrackingAndWrap();
    testFunctionalDispatch();
#if defined(__linux__)
    testLinuxFullDuplexUart();
#endif
    std::puts("contest_control_uart_test: PASS");
    return 0;
}
