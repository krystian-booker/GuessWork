#include "posest/teensy/SerialTransport.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#if defined(__linux__)
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace posest::teensy {

namespace {

#if defined(__linux__)
speed_t baudToSpeed(std::uint32_t baud_rate) {
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#if defined(B460800)
        case 460800: return B460800;
#endif
#if defined(B921600)
        case 921600: return B921600;
#endif
        default:
            throw std::runtime_error("Unsupported serial baud rate: " +
                                     std::to_string(baud_rate));
    }
}

class PosixSerialTransport final : public ISerialTransport {
public:
    ~PosixSerialTransport() override {
        close();
    }

    void open(const std::string& path, std::uint32_t baud_rate) override {
        close();
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial port '" + path + "': " +
                                     std::strerror(errno));
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            const std::string error = std::strerror(errno);
            close();
            throw std::runtime_error("tcgetattr failed for '" + path + "': " + error);
        }

        cfmakeraw(&tty);
        const speed_t speed = baudToSpeed(baud_rate);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
        tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
        tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            const std::string error = std::strerror(errno);
            close();
            throw std::runtime_error("tcsetattr failed for '" + path + "': " + error);
        }
    }

    void close() override {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool isOpen() const override {
        return fd_ >= 0;
    }

    std::size_t read(
        std::uint8_t* buffer,
        std::size_t capacity,
        std::chrono::milliseconds timeout) override {
        if (fd_ < 0) {
            throw std::runtime_error("serial read requested while closed");
        }

        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int rc = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
        if (rc < 0) {
            if (errno == EINTR) {
                return 0;
            }
            throw std::runtime_error("serial poll failed: " + std::string(std::strerror(errno)));
        }
        if (rc == 0) {
            return 0;
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            throw std::runtime_error("serial port disconnected");
        }

        const ssize_t n = ::read(fd_, buffer, capacity);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return 0;
            }
            throw std::runtime_error("serial read failed: " + std::string(std::strerror(errno)));
        }
        return static_cast<std::size_t>(n);
    }

    void write(const std::vector<std::uint8_t>& bytes) override {
        if (fd_ < 0) {
            throw std::runtime_error("serial write requested while closed");
        }

        std::size_t written = 0;
        while (written < bytes.size()) {
            const ssize_t n = ::write(
                fd_,
                bytes.data() + static_cast<std::ptrdiff_t>(written),
                bytes.size() - written);
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                throw std::runtime_error("serial write failed: " +
                                         std::string(std::strerror(errno)));
            }
            written += static_cast<std::size_t>(n);
        }
    }

private:
    int fd_{-1};
};
#endif

}  // namespace

std::unique_ptr<ISerialTransport> makePosixSerialTransport() {
#if defined(__linux__)
    return std::make_unique<PosixSerialTransport>();
#else
    throw std::runtime_error("POSIX serial transport is only implemented on Linux");
#endif
}

}  // namespace posest::teensy
