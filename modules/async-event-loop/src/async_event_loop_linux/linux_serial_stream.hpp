#pragma once

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#include "async_event_loop/event_loop.hpp"

namespace async_event_loop {

class LinuxSerialStream final : public IByteStream {
public:
    LinuxSerialStream() = default;

    LinuxSerialStream(const LinuxSerialStream&) = delete;
    LinuxSerialStream& operator=(const LinuxSerialStream&) = delete;

    ~LinuxSerialStream() override { close(); }

    bool open(const char* path, unsigned long baud = 115200) {
        close();
        if (!path || !*path) {
            return false;
        }

        const speed_t speed = baud_to_speed(baud);
        if (speed == 0) {
            return false;
        }

        fd_ = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            return false;
        }

        if (!configure(speed)) {
            close();
            return false;
        }
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool valid() const override { return fd_ >= 0; }
    bool readable() const override { return fd_ >= 0; }
    bool writable() const override { return fd_ >= 0; }
    int native_fd() const override { return fd_; }

    int read(uint8_t* dst, size_t max_len) override {
        if (fd_ < 0 || !dst || max_len == 0) {
            return 0;
        }
        const ssize_t n = ::read(fd_, dst, max_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return 0;
            }
            return -1;
        }
        return static_cast<int>(n);
    }

    int write(const uint8_t* src, size_t len) override {
        if (fd_ < 0 || !src || len == 0) {
            return 0;
        }
        const ssize_t n = ::write(fd_, src, len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return 0;
            }
            return -1;
        }
        return static_cast<int>(n);
    }

private:
    static speed_t baud_to_speed(unsigned long baud) {
        switch (baud) {
            case 1200: return B1200;
            case 2400: return B2400;
            case 4800: return B4800;
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
#ifdef B230400
            case 230400: return B230400;
#endif
#ifdef B460800
            case 460800: return B460800;
#endif
#ifdef B921600
            case 921600: return B921600;
#endif
            default: return 0;
        }
    }

    bool configure(speed_t speed) {
        termios tio{};
        if (tcgetattr(fd_, &tio) != 0) {
            return false;
        }

        cfmakeraw(&tio);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
        tio.c_cflag &= ~CRTSCTS;
#endif
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;

        if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
            return false;
        }
        return tcsetattr(fd_, TCSANOW, &tio) == 0;
    }

    int fd_ = -1;
};

} // namespace async_event_loop
