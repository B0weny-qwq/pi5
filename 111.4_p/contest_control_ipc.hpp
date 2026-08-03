#pragma once

#include "contest_control_uart.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#if defined(__linux__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace contest_control {

inline constexpr uint16_t kIpcPortBase = 17300;

inline uint16_t ipcPortForQuestion(uint8_t question)
{
    return question >= 1 && question <= 5 ?
        static_cast<uint16_t>(kIpcPortBase + question) : 0;
}

class UdpFrameSender {
#if defined(__linux__)
    int socket_ = -1;
#endif
    std::string lastError_;

    bool fail(const std::string& message)
    {
        lastError_ = message;
        return false;
    }

    bool sendBytes(const uint8_t* bytes, std::size_t size, uint16_t port)
    {
#if defined(__linux__)
        if (socket_ < 0) return fail("control IPC socket is not open");
        if (port == 0) return fail("invalid control IPC destination port");

        sockaddr_in destination{};
        destination.sin_family = AF_INET;
        destination.sin_port = htons(port);
        destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        const ssize_t sent = ::sendto(
            socket_, bytes, size, 0,
            reinterpret_cast<const sockaddr*>(&destination),
            sizeof(destination));
        if (sent != static_cast<ssize_t>(size)) {
            return fail(std::string("send control IPC frame failed: ") +
                        std::strerror(errno));
        }
        return true;
#else
        (void)bytes;
        (void)size;
        (void)port;
        return fail("control IPC is only available on Linux");
#endif
    }

public:
    UdpFrameSender() = default;
    UdpFrameSender(const UdpFrameSender&) = delete;
    UdpFrameSender& operator=(const UdpFrameSender&) = delete;
    ~UdpFrameSender() { closeSocket(); }

    bool openSocket()
    {
        closeSocket();
        lastError_.clear();
#if defined(__linux__)
        socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (socket_ < 0) {
            return fail(std::string("create control IPC socket failed: ") +
                        std::strerror(errno));
        }
        return true;
#else
        return fail("control IPC is only available on Linux");
#endif
    }

    bool send(const Frame& frame, uint16_t port)
    {
#if defined(__linux__)
        std::array<uint8_t, kFrameSize> bytes{};
        const DecodeError encodeError = encodeFrame(frame, bytes);
        if (encodeError != DecodeError::None) {
            return fail(std::string("encode control IPC frame failed: ") +
                        decodeErrorName(encodeError));
        }

        return sendBytes(bytes.data(), bytes.size(), port);
#else
        (void)frame;
        (void)port;
        return fail("control IPC is only available on Linux");
#endif
    }

    bool send(const TelemetryFrame& frame, uint16_t port)
    {
#if defined(__linux__)
        std::array<uint8_t, kTelemetryFrameSize> bytes{};
        const DecodeError encodeError = encodeTelemetryFrame(frame, bytes);
        if (encodeError != DecodeError::None) {
            return fail(std::string("encode telemetry IPC frame failed: ") +
                        decodeErrorName(encodeError));
        }
        return sendBytes(bytes.data(), bytes.size(), port);
#else
        (void)frame;
        (void)port;
        return fail("control IPC is only available on Linux");
#endif
    }

    void closeSocket()
    {
#if defined(__linux__)
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
#endif
    }

    const std::string& lastError() const { return lastError_; }
};

class UdpFrameReceiver {
#if defined(__linux__)
    int socket_ = -1;
#endif
    uint16_t port_ = 0;
    uint64_t acceptedFrames_ = 0;
    uint64_t acceptedTelemetryFrames_ = 0;
    uint64_t rejectedDatagrams_ = 0;
    std::string lastError_;

    bool fail(const std::string& message)
    {
        lastError_ = message;
        return false;
    }

public:
    UdpFrameReceiver() = default;
    UdpFrameReceiver(const UdpFrameReceiver&) = delete;
    UdpFrameReceiver& operator=(const UdpFrameReceiver&) = delete;
    ~UdpFrameReceiver() { closeSocket(); }

    bool openPort(uint16_t port)
    {
        closeSocket();
        lastError_.clear();
        acceptedFrames_ = 0;
        acceptedTelemetryFrames_ = 0;
        rejectedDatagrams_ = 0;
        if (port == 0) return fail("invalid control IPC listen port");

#if defined(__linux__)
        socket_ = ::socket(
            AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket_ < 0) {
            return fail(std::string("create control IPC receiver failed: ") +
                        std::strerror(errno));
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(socket_, reinterpret_cast<const sockaddr*>(&address),
                   sizeof(address)) != 0) {
            const std::string message =
                std::string("bind control IPC port failed: ") +
                std::strerror(errno);
            closeSocket();
            return fail(message);
        }
        port_ = port;
        return true;
#else
        (void)port;
        return fail("control IPC is only available on Linux");
#endif
    }

    bool receiveAndDispatch(
        EventHandler& handler,
        std::size_t* dispatchedCount = nullptr,
        TelemetryHandler* telemetryHandler = nullptr,
        std::size_t* telemetryDispatchedCount = nullptr)
    {
#if defined(__linux__)
        if (socket_ < 0) return fail("control IPC receiver is not open");
        std::size_t dispatched = 0;
        std::size_t telemetryDispatched = 0;
        while (true) {
            std::array<uint8_t, 64> datagram{};
            const ssize_t count = ::recv(
                socket_, datagram.data(), datagram.size(), 0);
            if (count < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                return fail(std::string("receive control IPC frame failed: ") +
                            std::strerror(errno));
            }

            if (count == static_cast<ssize_t>(kFrameSize)) {
                Frame frame;
                const DecodeError decodeError = decodeFrame(
                    datagram.data(), static_cast<std::size_t>(count), frame);
                if (decodeError != DecodeError::None) {
                    ++rejectedDatagrams_;
                    continue;
                }
                if (dispatchFrame(frame, handler) != DispatchResult::Handled) {
                    return fail("control IPC handler rejected event frame");
                }
                ++acceptedFrames_;
                ++dispatched;
                continue;
            }

            if (count == static_cast<ssize_t>(kTelemetryFrameSize) &&
                telemetryHandler != nullptr) {
                TelemetryFrame frame;
                const DecodeError decodeError = decodeTelemetryFrame(
                    datagram.data(), static_cast<std::size_t>(count), frame);
                if (decodeError != DecodeError::None) {
                    ++rejectedDatagrams_;
                    continue;
                }
                if (dispatchTelemetryFrame(frame, *telemetryHandler) !=
                    DispatchResult::Handled) {
                    return fail(
                        "control IPC handler rejected telemetry frame");
                }
                ++acceptedFrames_;
                ++acceptedTelemetryFrames_;
                ++telemetryDispatched;
                continue;
            }

            ++rejectedDatagrams_;
        }
        if (dispatchedCount) *dispatchedCount = dispatched;
        if (telemetryDispatchedCount) {
            *telemetryDispatchedCount = telemetryDispatched;
        }
        return true;
#else
        (void)handler;
        (void)dispatchedCount;
        (void)telemetryHandler;
        (void)telemetryDispatchedCount;
        return fail("control IPC is only available on Linux");
#endif
    }

    void closeSocket()
    {
#if defined(__linux__)
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
#endif
        port_ = 0;
    }

    uint16_t port() const { return port_; }
    uint64_t acceptedFrames() const { return acceptedFrames_; }
    uint64_t acceptedTelemetryFrames() const
    {
        return acceptedTelemetryFrames_;
    }
    uint64_t rejectedDatagrams() const { return rejectedDatagrams_; }
    const std::string& lastError() const { return lastError_; }
};

}  // namespace contest_control
