#pragma once

#if defined(__linux__)
#include <fcntl.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "async_event_loop/pin_io.hpp"

namespace async_event_loop {

class LinuxFilePinAccess {
public:
    explicit LinuxFilePinAccess(const char* path = nullptr) : path_(path) {}
    bool valid() const { return path_ && *path_; }

protected:
    int read_int(int fallback = 0) const {
        if (!valid()) {
            return fallback;
        }
        char buffer[32]{};
        const int fd = ::open(path_, O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            return fallback;
        }
        const ssize_t n = ::read(fd, buffer, sizeof(buffer) - 1);
        ::close(fd);
        if (n <= 0) {
            return fallback;
        }
        return static_cast<int>(strtol(buffer, nullptr, 10));
    }

    bool write_int(int value) const {
        if (!valid()) {
            return false;
        }
        char buffer[32];
        int n = 0;
        if (value == 0) {
            buffer[n++] = '0';
        } else {
            char digits[16];
            int d = 0;
            int v = value < 0 ? -value : value;
            if (value < 0) {
                buffer[n++] = '-';
            }
            while (v > 0 && d < static_cast<int>(sizeof(digits))) {
                digits[d++] = static_cast<char>('0' + (v % 10));
                v /= 10;
            }
            while (d > 0) {
                buffer[n++] = digits[--d];
            }
        }
        buffer[n++] = '\n';
        const int fd = ::open(path_, O_WRONLY | O_NONBLOCK);
        if (fd < 0) {
            return false;
        }
        const ssize_t written = ::write(fd, buffer, static_cast<size_t>(n));
        ::close(fd);
        return written == n;
    }

private:
    const char* path_ = nullptr;
};

class LinuxFileDigitalInputPin final : public IDigitalInputPin, private LinuxFilePinAccess {
public:
    explicit LinuxFileDigitalInputPin(const char* value_path) : LinuxFilePinAccess(value_path) {}
    bool valid() const override { return LinuxFilePinAccess::valid(); }
    bool read() override { return read_int(0) != 0; }
};

class LinuxFileDigitalOutputPin final : public IDigitalOutputPin, private LinuxFilePinAccess {
public:
    explicit LinuxFileDigitalOutputPin(const char* value_path) : LinuxFilePinAccess(value_path) {}
    bool valid() const override { return LinuxFilePinAccess::valid(); }
    bool write(bool high) override { return write_int(high ? 1 : 0); }
};

class LinuxFileAnalogInputPin final : public IAnalogInputPin, private LinuxFilePinAccess {
public:
    LinuxFileAnalogInputPin(const char* raw_path, int max_value = 4095) : LinuxFilePinAccess(raw_path), max_value_(max_value) {}
    bool valid() const override { return LinuxFilePinAccess::valid(); }
    int read() override { return read_int(0); }
    int max_value() const override { return max_value_; }
private:
    int max_value_;
};

class LinuxFileAnalogOutputPin final : public IAnalogOutputPin, private LinuxFilePinAccess {
public:
    LinuxFileAnalogOutputPin(const char* raw_path, int max_value = 255) : LinuxFilePinAccess(raw_path), max_value_(max_value) {}
    bool valid() const override { return LinuxFilePinAccess::valid(); }
    bool write(int value) override {
        if (value < 0) {
            value = 0;
        }
        if (value > max_value_) {
            value = max_value_;
        }
        return write_int(value);
    }
    int max_value() const override { return max_value_; }
private:
    int max_value_;
};

} // namespace async_event_loop
#endif
