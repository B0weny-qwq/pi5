#pragma once

// terminal_key_input.hpp
// ============================================================================
// 无X11运行时，从当前SSH/本地终端非阻塞读取SPACE、R、Q等单键。
// 仅关闭终端回显和行缓冲，保留Ctrl+C信号；退出时恢复原终端设置。
// ============================================================================

#if defined(__linux__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace ball_stepper {

class TerminalKeyInput {
#if defined(__linux__)
    termios originalTermios_{};
    int originalFlags_ = 0;
    bool active_ = false;
#endif

public:
    TerminalKeyInput() = default;
    TerminalKeyInput(const TerminalKeyInput&) = delete;
    TerminalKeyInput& operator=(const TerminalKeyInput&) = delete;

    ~TerminalKeyInput()
    {
        stop();
    }

    bool start()
    {
#if defined(__linux__)
        if (active_) return true;
        if (!::isatty(STDIN_FILENO)) return false;
        if (::tcgetattr(STDIN_FILENO, &originalTermios_) != 0) return false;

        originalFlags_ = ::fcntl(STDIN_FILENO, F_GETFL, 0);
        if (originalFlags_ < 0) return false;

        termios terminal = originalTermios_;
        terminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        terminal.c_cc[VMIN] = 0;
        terminal.c_cc[VTIME] = 0;

        if (::tcsetattr(STDIN_FILENO, TCSANOW, &terminal) != 0) return false;
        if (::fcntl(STDIN_FILENO, F_SETFL,
                    originalFlags_ | O_NONBLOCK) != 0) {
            ::tcsetattr(STDIN_FILENO, TCSANOW, &originalTermios_);
            return false;
        }

        active_ = true;
        return true;
#else
        return false;
#endif
    }

    bool active() const
    {
#if defined(__linux__)
        return active_;
#else
        return false;
#endif
    }

    int consumeKey()
    {
#if defined(__linux__)
        if (!active_) return -1;

        unsigned char key = 0;
        const ssize_t count = ::read(STDIN_FILENO, &key, 1);
        return count == 1 ? static_cast<int>(key) : -1;
#else
        return -1;
#endif
    }

    void stop()
    {
#if defined(__linux__)
        if (!active_) return;
        ::fcntl(STDIN_FILENO, F_SETFL, originalFlags_);
        ::tcsetattr(STDIN_FILENO, TCSANOW, &originalTermios_);
        active_ = false;
#endif
    }
};

} // namespace ball_stepper
