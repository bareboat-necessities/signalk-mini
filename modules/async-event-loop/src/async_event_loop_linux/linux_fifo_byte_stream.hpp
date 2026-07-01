#pragma once

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/stat.h>
#include <unistd.h>

#include "async_event_loop/event_loop.hpp"

namespace async_event_loop {

struct LinuxFifoByteStreamOptions {
    bool create_if_missing = true;
    mode_t create_mode = 0660;
    bool keep_fd_stable = true;
};

struct LinuxFifoByteStreamStats {
    uint32_t open_attempts = 0;
    uint32_t open_failures = 0;
    uint32_t eof_count = 0;
    int last_errno = 0;
};

/**
 * Linux named FIFO byte stream.
 *
 * The default mode opens the FIFO O_RDWR | O_NONBLOCK. This keeps the fd stable
 * even when external writers disconnect, which is important because libevent
 * registrations are tied to a specific fd. Use a separate output FIFO if the
 * application needs bidirectional IPC without reading its own writes.
 */
class LinuxFifoByteStream final : public IByteStream {
public:
    explicit LinuxFifoByteStream(const char* path,
                                 LinuxFifoByteStreamOptions options = LinuxFifoByteStreamOptions{})
        : path_(path), options_(options) {
        open_fifo();
    }

    LinuxFifoByteStream(const LinuxFifoByteStream&) = delete;
    LinuxFifoByteStream& operator=(const LinuxFifoByteStream&) = delete;

    ~LinuxFifoByteStream() override {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    int read(uint8_t* dst, size_t max_len) override {
        if (fd_ < 0 || !dst || max_len == 0) {
            return 0;
        }
        const ssize_t n = ::read(fd_, dst, max_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return 0;
            }
            stats_.last_errno = errno;
            return -1;
        }
        if (n == 0) {
            ++stats_.eof_count;
            return 0;
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
            stats_.last_errno = errno;
            return -1;
        }
        return static_cast<int>(n);
    }

    bool readable() const override { return fd_ >= 0; }
    bool writable() const override { return fd_ >= 0; }
    bool valid() const override { return fd_ >= 0; }
    int native_fd() const override { return fd_; }

    const char* path() const { return path_; }
    int fd() const { return fd_; }
    const LinuxFifoByteStreamStats& stats() const { return stats_; }

private:
    void open_fifo() {
        ++stats_.open_attempts;
        if (options_.create_if_missing) {
            if (mkfifo(path_, options_.create_mode) != 0 && errno != EEXIST) {
                stats_.last_errno = errno;
                ++stats_.open_failures;
                return;
            }
        }

        const int access = options_.keep_fd_stable ? O_RDWR : O_RDONLY;
        fd_ = ::open(path_, access | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            stats_.last_errno = errno;
            ++stats_.open_failures;
            return;
        }
    }

    const char* path_ = nullptr;
    LinuxFifoByteStreamOptions options_;
    int fd_ = -1;
    LinuxFifoByteStreamStats stats_;
};

} // namespace async_event_loop
