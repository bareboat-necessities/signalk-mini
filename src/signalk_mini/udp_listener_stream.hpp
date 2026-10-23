#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if !defined(ARDUINO)
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "stream_io.hpp"
#include <async_event_loop.hpp>
#include "config.hpp"

namespace signalk_mini {

class UdpListenerStream final : public async_event_loop::IByteStream {
public:
    UdpListenerStream() = default;
    ~UdpListenerStream() override { close(); }

    UdpListenerStream(const UdpListenerStream&) = delete;
    UdpListenerStream& operator=(const UdpListenerStream&) = delete;

    bool open(const UdpTransportConfig& config) {
        close();
#if defined(ARDUINO)
        (void)config;
        return false;
#else
        if (config.listen_port == 0 && (!config.remote_host || config.remote_port == 0)) return false;
        const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) return false;

        int one = 1;
        (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        if (config.allow_broadcast) (void)::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
            ::close(fd);
            return false;
        }

        if (config.listen_port != 0) {
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(config.listen_port);
            if (!config.listen_host || strcmp(config.listen_host, "0.0.0.0") == 0) {
                addr.sin_addr.s_addr = htonl(INADDR_ANY);
            } else if (::inet_pton(AF_INET, config.listen_host, &addr.sin_addr) != 1) {
                ::close(fd);
                return false;
            }
            if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
                ::close(fd);
                return false;
            }
        }

        if (config.remote_host && config.remote_port != 0) {
            remote_addr_ = sockaddr_in{};
            remote_addr_.sin_family = AF_INET;
            remote_addr_.sin_port = htons(config.remote_port);
            if (strcmp(config.remote_host, "255.255.255.255") == 0) {
                remote_addr_.sin_addr.s_addr = htonl(INADDR_BROADCAST);
            } else if (::inet_pton(AF_INET, config.remote_host, &remote_addr_.sin_addr) != 1) {
                ::close(fd);
                return false;
            }
            remote_addr_ready_ = true;
        }

        fd_ = fd;
        return true;
#endif
    }

    void close() {
#if !defined(ARDUINO)
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        remote_addr_ = sockaddr_in{};
        remote_addr_ready_ = false;
#endif
    }

    int read(uint8_t* dst, size_t max_len) override {
#if defined(ARDUINO)
        (void)dst;
        (void)max_len;
        return 0;
#else
        if (fd_ < 0 || !dst || max_len == 0) return 0;
        const ssize_t n = ::recv(fd_, dst, max_len, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
            return -1;
        }
        return static_cast<int>(n);
#endif
    }

    int write(const uint8_t* src, size_t len) override {
#if defined(ARDUINO)
        (void)src;
        (void)len;
        return 0;
#else
        if (fd_ < 0 || !remote_addr_ready_ || !src || len == 0) return 0;
        const ssize_t n = ::sendto(fd_, src, len, 0, reinterpret_cast<const sockaddr*>(&remote_addr_), sizeof(remote_addr_));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
            return -1;
        }
        return static_cast<int>(n);
#endif
    }

    bool readable() const override {
#if defined(ARDUINO)
        return false;
#else
        if (fd_ < 0) return false;
        int bytes = 0;
        return ::ioctl(fd_, FIONREAD, &bytes) == 0 && bytes > 0;
#endif
    }

    bool writable() const override {
#if defined(ARDUINO)
        return false;
#else
        return fd_ >= 0 && remote_addr_ready_;
#endif
    }

    bool valid() const override {
#if defined(ARDUINO)
        return false;
#else
        return fd_ >= 0;
#endif
    }

    bool is_datagram() const override { return true; }

    int native_fd() const override {
#if defined(ARDUINO)
        return -1;
#else
        return fd_;
#endif
    }

private:
#if !defined(ARDUINO)
    int fd_ = -1;
    sockaddr_in remote_addr_{};
    bool remote_addr_ready_ = false;
#endif
};

} // namespace signalk_mini
