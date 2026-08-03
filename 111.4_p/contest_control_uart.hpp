#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace contest_control {

inline constexpr std::size_t kFrameSize = 12;
inline constexpr std::size_t kTelemetryFrameSize = 28;
inline constexpr uint8_t kSof0 = 0xA5;
inline constexpr uint8_t kSof1 = 0x5A;
inline constexpr uint8_t kVersion = 0x01;
inline constexpr uint8_t kWireSize = 0x0C;
inline constexpr uint8_t kTelemetryWireSize = 0x1C;
inline constexpr uint8_t kTelemetryType = 0x10;
inline constexpr uint8_t kCrlf0 = 0x0D;
inline constexpr uint8_t kCrlf1 = 0x0A;
inline constexpr int kBaud = 230400;

inline constexpr uint8_t kFlagPidEnabled = 0x01;
inline constexpr uint8_t kFlagMotorEnabled = 0x02;
inline constexpr uint8_t kFlagOriginSet = 0x04;
inline constexpr uint8_t kKnownFlags =
    kFlagPidEnabled | kFlagMotorEnabled | kFlagOriginSet;

enum class Event : uint8_t {
    Select = 0x01,
    Execute = 0x02,
    MotorToggle = 0x03,
    ReturnOrigin = 0x04,
    SetOrigin = 0x05
};

enum class Mode : uint8_t {
    UartTask = 0x00,
    TrackingPid = 0x01
};

enum class State : uint8_t {
    Ready = 0x00,
    Sent = 0x01,
    Running = 0x02
};

enum class DecodeError {
    None,
    WrongLength,
    WrongHeader,
    UnsupportedVersion,
    WrongDeclaredSize,
    ChecksumMismatch,
    UnsupportedEvent,
    InvalidQuestion,
    EventNotAllowedForQuestion,
    QuestionModeMismatch,
    InvalidState,
    ReservedFlagsSet,
    UnsupportedTelemetryType,
    InvalidTelemetryQuestion,
    InvalidTelemetryState,
    NonzeroTelemetryReserved
};

struct Frame {
    Event event = Event::Select;
    uint8_t question = 1;
    Mode mode = Mode::UartTask;
    State state = State::Ready;
    uint16_t sequence = 0;
    uint8_t flags = 0;

    bool pidEnabled() const { return (flags & kFlagPidEnabled) != 0; }
    bool motorEnabled() const { return (flags & kFlagMotorEnabled) != 0; }
    bool originSet() const { return (flags & kFlagOriginSet) != 0; }
};

struct TelemetryFrame {
    uint8_t question = 4;
    State state = State::Sent;
    uint8_t flags = 0;
    uint16_t sequence = 0;
    int16_t leftSpeed50ms = 0;
    int16_t rightSpeed50ms = 0;
    int16_t leftTarget50ms = 0;
    int16_t rightTarget50ms = 0;
    uint32_t distanceSteps = 0;
    uint32_t elapsedMs = 0;

    bool pidEnabled() const { return (flags & kFlagPidEnabled) != 0; }
    bool motorEnabled() const { return (flags & kFlagMotorEnabled) != 0; }
    bool originSet() const { return (flags & kFlagOriginSet) != 0; }
};

inline const char* eventName(Event event)
{
    switch (event) {
    case Event::Select: return "SELECT";
    case Event::Execute: return "EXECUTE";
    case Event::MotorToggle: return "MOTOR_TOGGLE";
    case Event::ReturnOrigin: return "RETURN_ORIGIN";
    case Event::SetOrigin: return "SET_ORIGIN";
    }
    return "UNKNOWN";
}

inline const char* stateName(State state)
{
    switch (state) {
    case State::Ready: return "READY";
    case State::Sent: return "SENT";
    case State::Running: return "RUNNING";
    }
    return "UNKNOWN";
}

inline const char* decodeErrorName(DecodeError error)
{
    switch (error) {
    case DecodeError::None: return "none";
    case DecodeError::WrongLength: return "wrong length";
    case DecodeError::WrongHeader: return "wrong header";
    case DecodeError::UnsupportedVersion: return "unsupported version";
    case DecodeError::WrongDeclaredSize: return "wrong declared size";
    case DecodeError::ChecksumMismatch: return "checksum mismatch";
    case DecodeError::UnsupportedEvent: return "unsupported event";
    case DecodeError::InvalidQuestion: return "invalid question";
    case DecodeError::EventNotAllowedForQuestion:
        return "event not allowed for question";
    case DecodeError::QuestionModeMismatch: return "question/mode mismatch";
    case DecodeError::InvalidState: return "invalid state";
    case DecodeError::ReservedFlagsSet: return "reserved flags set";
    case DecodeError::UnsupportedTelemetryType:
        return "unsupported telemetry type";
    case DecodeError::InvalidTelemetryQuestion:
        return "invalid telemetry question";
    case DecodeError::InvalidTelemetryState:
        return "invalid telemetry state";
    case DecodeError::NonzeroTelemetryReserved:
        return "telemetry reserved byte is not zero";
    }
    return "unknown error";
}

inline bool modeForQuestion(uint8_t question, Mode& mode)
{
    switch (question) {
    case 1:
    case 3:
        mode = Mode::UartTask;
        return true;
    case 2:
    case 4:
    case 5:
        mode = Mode::TrackingPid;
        return true;
    default:
        return false;
    }
}

inline bool isKnownEvent(uint8_t value)
{
    return value >= static_cast<uint8_t>(Event::Select) &&
           value <= static_cast<uint8_t>(Event::SetOrigin);
}

inline bool isKnownState(uint8_t value)
{
    return value <= static_cast<uint8_t>(State::Running);
}

inline bool eventAllowedForQuestion(Event event, uint8_t question)
{
    if (question < 1 || question > 5) return false;
    if (event == Event::Select || event == Event::Execute) return true;
    return question >= 3;
}

inline uint8_t xorChecksum(const uint8_t* data, std::size_t size)
{
    uint8_t checksum = 0;
    for (std::size_t index = 0; index < size; ++index) checksum ^= data[index];
    return checksum;
}

inline uint16_t readUint16Le(const uint8_t* bytes)
{
    return static_cast<uint16_t>(bytes[0]) |
        (static_cast<uint16_t>(bytes[1]) << 8);
}

inline int16_t readInt16Le(const uint8_t* bytes)
{
    const uint16_t raw = readUint16Le(bytes);
    const int32_t signedValue = (raw & 0x8000u) != 0 ?
        static_cast<int32_t>(raw) - 0x10000 : static_cast<int32_t>(raw);
    return static_cast<int16_t>(signedValue);
}

inline uint32_t readUint32Le(const uint8_t* bytes)
{
    return static_cast<uint32_t>(bytes[0]) |
        (static_cast<uint32_t>(bytes[1]) << 8) |
        (static_cast<uint32_t>(bytes[2]) << 16) |
        (static_cast<uint32_t>(bytes[3]) << 24);
}

inline void writeUint16Le(uint8_t* bytes, uint16_t value)
{
    bytes[0] = static_cast<uint8_t>(value & 0xFFu);
    bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

inline void writeUint32Le(uint8_t* bytes, uint32_t value)
{
    bytes[0] = static_cast<uint8_t>(value & 0xFFu);
    bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    bytes[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    bytes[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

inline DecodeError validateFrame(const Frame& frame)
{
    if (!isKnownEvent(static_cast<uint8_t>(frame.event))) {
        return DecodeError::UnsupportedEvent;
    }
    Mode expectedMode = Mode::UartTask;
    if (!modeForQuestion(frame.question, expectedMode)) {
        return DecodeError::InvalidQuestion;
    }
    if (!eventAllowedForQuestion(frame.event, frame.question)) {
        return DecodeError::EventNotAllowedForQuestion;
    }
    if (frame.mode != expectedMode) {
        return DecodeError::QuestionModeMismatch;
    }
    if (!isKnownState(static_cast<uint8_t>(frame.state))) {
        return DecodeError::InvalidState;
    }
    if ((frame.flags & static_cast<uint8_t>(~kKnownFlags)) != 0) {
        return DecodeError::ReservedFlagsSet;
    }
    return DecodeError::None;
}

inline DecodeError decodeFrame(
    const uint8_t* bytes, std::size_t size, Frame& frame)
{
    if (bytes == nullptr || size != kFrameSize) return DecodeError::WrongLength;
    if (bytes[0] != kSof0 || bytes[1] != kSof1) {
        return DecodeError::WrongHeader;
    }
    if (bytes[2] != kVersion) return DecodeError::UnsupportedVersion;
    if (bytes[3] != kWireSize) return DecodeError::WrongDeclaredSize;
    if (xorChecksum(bytes, kFrameSize - 1) != bytes[kFrameSize - 1]) {
        return DecodeError::ChecksumMismatch;
    }

    Frame decoded;
    decoded.event = static_cast<Event>(bytes[4]);
    decoded.question = bytes[5];
    decoded.mode = static_cast<Mode>(bytes[6]);
    decoded.state = static_cast<State>(bytes[7]);
    decoded.sequence = static_cast<uint16_t>(bytes[8]) |
        (static_cast<uint16_t>(bytes[9]) << 8);
    decoded.flags = bytes[10];

    const DecodeError error = validateFrame(decoded);
    if (error != DecodeError::None) return error;
    frame = decoded;
    return DecodeError::None;
}

inline DecodeError validateTelemetryFrame(const TelemetryFrame& frame)
{
    if (frame.question != 4 && frame.question != 5) {
        return DecodeError::InvalidTelemetryQuestion;
    }
    if (frame.state != State::Sent && frame.state != State::Running) {
        return DecodeError::InvalidTelemetryState;
    }
    if ((frame.flags & static_cast<uint8_t>(~kKnownFlags)) != 0) {
        return DecodeError::ReservedFlagsSet;
    }
    return DecodeError::None;
}

inline DecodeError decodeTelemetryFrame(
    const uint8_t* bytes, std::size_t size, TelemetryFrame& frame)
{
    if (bytes == nullptr || size != kTelemetryFrameSize) {
        return DecodeError::WrongLength;
    }
    if (bytes[0] != kSof0 || bytes[1] != kSof1) {
        return DecodeError::WrongHeader;
    }
    if (bytes[2] != kVersion) return DecodeError::UnsupportedVersion;
    if (bytes[3] != kTelemetryWireSize) {
        return DecodeError::WrongDeclaredSize;
    }
    if (bytes[4] != kTelemetryType) {
        return DecodeError::UnsupportedTelemetryType;
    }
    if (xorChecksum(bytes, kTelemetryFrameSize - 1) !=
        bytes[kTelemetryFrameSize - 1]) {
        return DecodeError::ChecksumMismatch;
    }
    if (bytes[26] != 0) return DecodeError::NonzeroTelemetryReserved;

    TelemetryFrame decoded;
    decoded.question = bytes[5];
    decoded.state = static_cast<State>(bytes[6]);
    decoded.flags = bytes[7];
    decoded.sequence = readUint16Le(bytes + 8);
    decoded.leftSpeed50ms = readInt16Le(bytes + 10);
    decoded.rightSpeed50ms = readInt16Le(bytes + 12);
    decoded.leftTarget50ms = readInt16Le(bytes + 14);
    decoded.rightTarget50ms = readInt16Le(bytes + 16);
    decoded.distanceSteps = readUint32Le(bytes + 18);
    decoded.elapsedMs = readUint32Le(bytes + 22);

    const DecodeError error = validateTelemetryFrame(decoded);
    if (error != DecodeError::None) return error;
    frame = decoded;
    return DecodeError::None;
}

inline DecodeError encodeFrame(
    const Frame& frame, std::array<uint8_t, kFrameSize>& bytes)
{
    const DecodeError error = validateFrame(frame);
    if (error != DecodeError::None) return error;

    bytes = {
        kSof0, kSof1, kVersion, kWireSize,
        static_cast<uint8_t>(frame.event), frame.question,
        static_cast<uint8_t>(frame.mode), static_cast<uint8_t>(frame.state),
        static_cast<uint8_t>(frame.sequence & 0xFF),
        static_cast<uint8_t>((frame.sequence >> 8) & 0xFF),
        frame.flags, 0
    };
    bytes.back() = xorChecksum(bytes.data(), bytes.size() - 1);
    return DecodeError::None;
}

inline DecodeError encodeTelemetryFrame(
    const TelemetryFrame& frame,
    std::array<uint8_t, kTelemetryFrameSize>& bytes)
{
    const DecodeError error = validateTelemetryFrame(frame);
    if (error != DecodeError::None) return error;

    bytes.fill(0);
    bytes[0] = kSof0;
    bytes[1] = kSof1;
    bytes[2] = kVersion;
    bytes[3] = kTelemetryWireSize;
    bytes[4] = kTelemetryType;
    bytes[5] = frame.question;
    bytes[6] = static_cast<uint8_t>(frame.state);
    bytes[7] = frame.flags;
    writeUint16Le(bytes.data() + 8, frame.sequence);
    writeUint16Le(bytes.data() + 10,
                  static_cast<uint16_t>(frame.leftSpeed50ms));
    writeUint16Le(bytes.data() + 12,
                  static_cast<uint16_t>(frame.rightSpeed50ms));
    writeUint16Le(bytes.data() + 14,
                  static_cast<uint16_t>(frame.leftTarget50ms));
    writeUint16Le(bytes.data() + 16,
                  static_cast<uint16_t>(frame.rightTarget50ms));
    writeUint32Le(bytes.data() + 18, frame.distanceSteps);
    writeUint32Le(bytes.data() + 22, frame.elapsedMs);
    bytes[26] = 0;
    bytes[27] = xorChecksum(bytes.data(), bytes.size() - 1);
    return DecodeError::None;
}

inline DecodeError makeFrame(
    Event event,
    uint8_t question,
    State state,
    uint16_t sequence,
    uint8_t flags,
    Frame& frame)
{
    Mode mode = Mode::UartTask;
    if (!modeForQuestion(question, mode)) return DecodeError::InvalidQuestion;
    frame = {event, question, mode, state, sequence, flags};
    return validateFrame(frame);
}

struct ParserStats {
    uint64_t bytesReceived = 0;
    uint64_t framesAccepted = 0;
    uint64_t eventFramesAccepted = 0;
    uint64_t telemetryFramesAccepted = 0;
    uint64_t crlfTerminatorsConsumed = 0;
    uint64_t checksumErrors = 0;
    uint64_t formatErrors = 0;
    uint64_t discardedBytes = 0;
    uint64_t duplicateSequences = 0;
    uint64_t sequenceDiscontinuities = 0;
};

struct ParsedBatch {
    std::vector<Frame> events;
    std::vector<TelemetryFrame> telemetry;
};

class StreamParser {
    std::vector<uint8_t> buffer_;
    ParserStats stats_;
    bool haveLastSequence_ = false;
    uint16_t lastSequence_ = 0;

    void discardPrefix(std::size_t size)
    {
        size = std::min(size, buffer_.size());
        stats_.discardedBytes += size;
        buffer_.erase(buffer_.begin(), buffer_.begin() + size);
    }

    void recordSequence(uint16_t sequence)
    {
        if (haveLastSequence_) {
            const uint16_t expected = static_cast<uint16_t>(lastSequence_ + 1);
            if (sequence == lastSequence_) {
                ++stats_.duplicateSequences;
            } else if (sequence != expected) {
                ++stats_.sequenceDiscontinuities;
            }
        }
        lastSequence_ = sequence;
        haveLastSequence_ = true;
    }

public:
    ParsedBatch pushAll(const uint8_t* data, std::size_t size)
    {
        ParsedBatch batch;
        if (data == nullptr || size == 0) return batch;

        stats_.bytesReceived += size;
        buffer_.insert(buffer_.end(), data, data + size);

        while (true) {
            std::size_t header = buffer_.size();
            for (std::size_t index = 0; index + 1 < buffer_.size(); ++index) {
                if (buffer_[index] == kSof0 && buffer_[index + 1] == kSof1) {
                    header = index;
                    break;
                }
            }

            if (header == buffer_.size()) {
                const std::size_t keep =
                    !buffer_.empty() && buffer_.back() == kSof0 ? 1 : 0;
                discardPrefix(buffer_.size() - keep);
                break;
            }
            if (header > 0) discardPrefix(header);
            if (buffer_.size() < 4) break;

            const std::size_t declaredSize = buffer_[3];
            if (declaredSize != kFrameSize &&
                declaredSize != kTelemetryFrameSize) {
                ++stats_.formatErrors;
                discardPrefix(1);
                continue;
            }
            if (buffer_.size() < declaredSize) break;

            DecodeError error = DecodeError::WrongDeclaredSize;
            uint16_t sequence = 0;
            if (declaredSize == kFrameSize) {
                Frame frame;
                error = decodeFrame(buffer_.data(), declaredSize, frame);
                if (error == DecodeError::None) {
                    sequence = frame.sequence;
                    batch.events.push_back(frame);
                    ++stats_.eventFramesAccepted;
                }
            } else {
                TelemetryFrame frame;
                error = decodeTelemetryFrame(
                    buffer_.data(), declaredSize, frame);
                if (error == DecodeError::None) {
                    sequence = frame.sequence;
                    batch.telemetry.push_back(frame);
                    ++stats_.telemetryFramesAccepted;
                }
            }

            if (error == DecodeError::None) {
                ++stats_.framesAccepted;
                recordSequence(sequence);
                buffer_.erase(
                    buffer_.begin(), buffer_.begin() + declaredSize);
                if (buffer_.size() >= 2 &&
                    buffer_[0] == kCrlf0 && buffer_[1] == kCrlf1) {
                    buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
                    ++stats_.crlfTerminatorsConsumed;
                }
                continue;
            }

            if (error == DecodeError::ChecksumMismatch) {
                ++stats_.checksumErrors;
            } else {
                ++stats_.formatErrors;
            }
            discardPrefix(1);
        }
        return batch;
    }

    ParsedBatch pushAll(const std::vector<uint8_t>& data)
    {
        return pushAll(data.data(), data.size());
    }

    std::vector<Frame> push(const uint8_t* data, std::size_t size)
    {
        return pushAll(data, size).events;
    }

    std::vector<Frame> push(const std::vector<uint8_t>& data)
    {
        return push(data.data(), data.size());
    }

    const ParserStats& stats() const { return stats_; }

    void reset()
    {
        buffer_.clear();
        stats_ = {};
        haveLastSequence_ = false;
        lastSequence_ = 0;
    }
};

class SequenceEncoder {
    uint16_t nextSequence_ = 1;

public:
    explicit SequenceEncoder(uint16_t firstSequence = 1)
        : nextSequence_(firstSequence)
    {
    }

    DecodeError next(
        Event event,
        uint8_t question,
        State state,
        uint8_t flags,
        Frame& frame,
        std::array<uint8_t, kFrameSize>& bytes)
    {
        const DecodeError makeError = makeFrame(
            event, question, state, nextSequence_, flags, frame);
        if (makeError != DecodeError::None) return makeError;
        const DecodeError encodeError = encodeFrame(frame, bytes);
        if (encodeError == DecodeError::None) {
            nextSequence_ = static_cast<uint16_t>(nextSequence_ + 1);
        }
        return encodeError;
    }

    uint16_t nextSequence() const { return nextSequence_; }
    void setNextSequence(uint16_t sequence) { nextSequence_ = sequence; }
};

class EventHandler {
public:
    virtual ~EventHandler() = default;

    virtual bool selectQuestion(const Frame& frame) = 0;
    virtual bool executeQuestion(const Frame& frame) = 0;
    virtual bool setMotorEnabled(bool enabled, const Frame& frame) = 0;
    virtual bool returnToOrigin(const Frame& frame) = 0;
    virtual bool setCurrentPositionAsOrigin(const Frame& frame) = 0;
};

class TelemetryHandler {
public:
    virtual ~TelemetryHandler() = default;
    virtual bool handleTelemetry(const TelemetryFrame& frame) = 0;
};

enum class DispatchResult {
    Handled,
    InvalidFrame,
    HandlerRejected
};

inline const char* dispatchResultName(DispatchResult result)
{
    switch (result) {
    case DispatchResult::Handled: return "handled";
    case DispatchResult::InvalidFrame: return "invalid frame";
    case DispatchResult::HandlerRejected: return "handler rejected event";
    }
    return "unknown dispatch result";
}

inline DispatchResult dispatchFrame(const Frame& frame, EventHandler& handler)
{
    if (validateFrame(frame) != DecodeError::None) {
        return DispatchResult::InvalidFrame;
    }

    bool handled = false;
    switch (frame.event) {
    case Event::Select:
        handled = handler.selectQuestion(frame);
        break;
    case Event::Execute:
        handled = handler.executeQuestion(frame);
        break;
    case Event::MotorToggle:
        handled = handler.setMotorEnabled(frame.motorEnabled(), frame);
        break;
    case Event::ReturnOrigin:
        handled = handler.returnToOrigin(frame);
        break;
    case Event::SetOrigin:
        handled = handler.setCurrentPositionAsOrigin(frame);
        break;
    }
    return handled ? DispatchResult::Handled : DispatchResult::HandlerRejected;
}

inline DispatchResult dispatchTelemetryFrame(
    const TelemetryFrame& frame, TelemetryHandler& handler)
{
    if (validateTelemetryFrame(frame) != DecodeError::None) {
        return DispatchResult::InvalidFrame;
    }
    return handler.handleTelemetry(frame) ?
        DispatchResult::Handled : DispatchResult::HandlerRejected;
}

class ControlUart {
#if defined(__linux__)
    int fileDescriptor_ = -1;
    int lockDescriptor_ = -1;
#endif
    StreamParser parser_;
    SequenceEncoder encoder_;
    std::string lastError_;

    bool fail(const std::string& message)
    {
        lastError_ = message;
        return false;
    }

#if defined(__linux__)
    bool writeAll(const uint8_t* data, std::size_t size)
    {
        std::size_t written = 0;
        while (written < size) {
            const ssize_t count = ::write(
                fileDescriptor_, data + written, size - written);
            if (count > 0) {
                written += static_cast<std::size_t>(count);
                continue;
            }
            if (count < 0 && errno == EINTR) continue;
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd descriptor{fileDescriptor_, POLLOUT, 0};
                const int ready = ::poll(&descriptor, 1, 100);
                if (ready > 0) continue;
                if (ready < 0 && errno == EINTR) continue;
                return fail("control UART write timeout");
            }
            return fail(std::string("control UART write failed: ") +
                        std::strerror(errno));
        }
        if (::tcdrain(fileDescriptor_) != 0) {
            return fail(std::string("control UART drain failed: ") +
                        std::strerror(errno));
        }
        return true;
    }

    bool writePacketWithCrlf(const uint8_t* data, std::size_t size)
    {
        std::vector<uint8_t> wire(data, data + size);
        wire.push_back(kCrlf0);
        wire.push_back(kCrlf1);
        return writeAll(wire.data(), wire.size());
    }
#endif

public:
    ControlUart() = default;
    ControlUart(const ControlUart&) = delete;
    ControlUart& operator=(const ControlUart&) = delete;
    ~ControlUart() { closePort(); }

    bool openPort(
        const std::string& device,
        int baud = kBaud,
        bool flushPendingInput = true)
    {
        closePort();
        parser_.reset();
        lastError_.clear();
        if (baud != kBaud) {
            return fail("control UART protocol requires 230400 baud");
        }

#if defined(__linux__)
        struct stat deviceStatus{};
        if (::stat(device.c_str(), &deviceStatus) != 0) {
            return fail("stat " + device + " failed: " +
                        std::strerror(errno));
        }
        const std::string lockPath =
            "/tmp/ball_stepper_contest_control_uart_" +
            std::to_string(static_cast<unsigned>(major(deviceStatus.st_rdev))) +
            "_" +
            std::to_string(static_cast<unsigned>(minor(deviceStatus.st_rdev))) +
            ".lock";
        lockDescriptor_ = ::open(
            lockPath.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
        if (lockDescriptor_ < 0) {
            return fail(std::string("open control UART lock failed: ") +
                        std::strerror(errno));
        }
        if (::flock(lockDescriptor_, LOCK_EX | LOCK_NB) != 0) {
            closePort();
            return fail("control UART is already in use");
        }

        fileDescriptor_ = ::open(
            device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fileDescriptor_ < 0) {
            const std::string message =
                "open " + device + " failed: " + std::strerror(errno);
            closePort();
            return fail(message);
        }

        termios terminal{};
        if (::tcgetattr(fileDescriptor_, &terminal) != 0) {
            const std::string message =
                std::string("control UART tcgetattr failed: ") +
                std::strerror(errno);
            closePort();
            return fail(message);
        }

        ::cfmakeraw(&terminal);
        ::cfsetispeed(&terminal, B230400);
        ::cfsetospeed(&terminal, B230400);
        terminal.c_cflag =
            (terminal.c_cflag & static_cast<tcflag_t>(~CSIZE)) | CS8;
        terminal.c_cflag |= CLOCAL | CREAD;
        terminal.c_cflag &= static_cast<tcflag_t>(
            ~(PARENB | PARODD | CSTOPB | CRTSCTS));
        terminal.c_cc[VMIN] = 0;
        terminal.c_cc[VTIME] = 0;

        if (::tcsetattr(fileDescriptor_, TCSANOW, &terminal) != 0) {
            const std::string message =
                std::string("control UART tcsetattr failed: ") +
                std::strerror(errno);
            closePort();
            return fail(message);
        }
        const int flushQueue = flushPendingInput ? TCIOFLUSH : TCOFLUSH;
        if (::tcflush(fileDescriptor_, flushQueue) != 0) {
            const std::string message =
                std::string("control UART flush failed: ") +
                std::strerror(errno);
            closePort();
            return fail(message);
        }
        return true;
#else
        (void)device;
        return fail("control UART is only available on Linux");
#endif
    }

    void closePort()
    {
#if defined(__linux__)
        if (fileDescriptor_ >= 0) {
            ::close(fileDescriptor_);
            fileDescriptor_ = -1;
        }
        if (lockDescriptor_ >= 0) {
            ::flock(lockDescriptor_, LOCK_UN);
            ::close(lockDescriptor_);
            lockDescriptor_ = -1;
        }
#endif
    }

    bool isOpen() const
    {
#if defined(__linux__)
        return fileDescriptor_ >= 0;
#else
        return false;
#endif
    }

    bool receive(
        std::vector<Frame>& frames,
        std::vector<TelemetryFrame>& telemetry)
    {
#if defined(__linux__)
        if (fileDescriptor_ < 0) return fail("control UART is not open");
        std::array<uint8_t, 256> chunk{};
        while (true) {
            const ssize_t count =
                ::read(fileDescriptor_, chunk.data(), chunk.size());
            if (count > 0) {
                ParsedBatch decoded = parser_.pushAll(
                    chunk.data(), static_cast<std::size_t>(count));
                frames.insert(
                    frames.end(), decoded.events.begin(), decoded.events.end());
                telemetry.insert(
                    telemetry.end(), decoded.telemetry.begin(),
                    decoded.telemetry.end());
                continue;
            }
            if (count == 0) return true;
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
            return fail(std::string("control UART read failed: ") +
                        std::strerror(errno));
        }
#else
        (void)frames;
        (void)telemetry;
        return fail("control UART is only available on Linux");
#endif
    }

    bool receive(std::vector<Frame>& frames)
    {
        std::vector<TelemetryFrame> ignoredTelemetry;
        return receive(frames, ignoredTelemetry);
    }

    bool receiveAndDispatch(
        EventHandler& handler,
        std::size_t* dispatchedCount = nullptr)
    {
        std::vector<Frame> frames;
        if (!receive(frames)) return false;

        std::size_t dispatched = 0;
        for (const Frame& frame : frames) {
            const DispatchResult result = dispatchFrame(frame, handler);
            if (result != DispatchResult::Handled) {
                return fail(
                    std::string("control event ") + eventName(frame.event) +
                    " sequence=" + std::to_string(frame.sequence) +
                    " dispatch failed: " + dispatchResultName(result));
            }
            ++dispatched;
        }
        if (dispatchedCount != nullptr) *dispatchedCount = dispatched;
        return true;
    }

    bool sendFrame(const Frame& frame)
    {
        std::array<uint8_t, kFrameSize> bytes{};
        const DecodeError error = encodeFrame(frame, bytes);
        if (error != DecodeError::None) {
            return fail(std::string("invalid control frame: ") +
                        decodeErrorName(error));
        }
#if defined(__linux__)
        if (fileDescriptor_ < 0) return fail("control UART is not open");
        return writePacketWithCrlf(bytes.data(), bytes.size());
#else
        return fail("control UART is only available on Linux");
#endif
    }


    bool sendTelemetryFrame(const TelemetryFrame& frame)
    {
        std::array<uint8_t, kTelemetryFrameSize> bytes{};
        const DecodeError error = encodeTelemetryFrame(frame, bytes);
        if (error != DecodeError::None) {
            return fail(std::string("invalid telemetry frame: ") +
                        decodeErrorName(error));
        }
#if defined(__linux__)
        if (fileDescriptor_ < 0) return fail("control UART is not open");
        return writePacketWithCrlf(bytes.data(), bytes.size());
#else
        return fail("control UART is only available on Linux");
#endif
    }

    bool send(
        Event event,
        uint8_t question,
        State state,
        uint8_t flags,
        Frame* sentFrame = nullptr)
    {
#if !defined(__linux__)
        (void)event;
        (void)question;
        (void)state;
        (void)flags;
        (void)sentFrame;
        return fail("control UART is only available on Linux");
#else
        if (fileDescriptor_ < 0) return fail("control UART is not open");

        Frame frame;
        std::array<uint8_t, kFrameSize> bytes{};
        const DecodeError error =
            encoder_.next(event, question, state, flags, frame, bytes);
        if (error != DecodeError::None) {
            return fail(std::string("invalid control frame: ") +
                        decodeErrorName(error));
        }
        if (!writePacketWithCrlf(bytes.data(), bytes.size())) {
            encoder_.setNextSequence(frame.sequence);
            return false;
        }
        if (sentFrame != nullptr) *sentFrame = frame;
        return true;
#endif
    }

    const ParserStats& parserStats() const { return parser_.stats(); }
    const std::string& lastError() const { return lastError_; }
    uint16_t nextTransmitSequence() const { return encoder_.nextSequence(); }
    void setNextTransmitSequence(uint16_t sequence)
    {
        encoder_.setNextSequence(sequence);
    }
};

} // namespace contest_control
