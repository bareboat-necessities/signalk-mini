#pragma once

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

#include "async_event_loop/event_loop.hpp"

namespace async_event_loop {

class LinuxFdStream final : public IByteStream {
public:
    explicit LinuxFdStream(int fd = -1) : fd_(fd) {
        set_nonblocking();
    }

    void reset(int fd) {
        fd_ = fd;
        set_nonblocking();
    }

    int fd() const { return fd_; }
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

    bool readable() const override { return fd_ >= 0; }
    bool writable() const override { return fd_ >= 0; }
    bool valid() const override { return fd_ >= 0; }

private:
    void set_nonblocking() {
        if (fd_ < 0) {
            return;
        }
        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        }
    }

    int fd_ = -1;
};

} // namespace async_event_loop
