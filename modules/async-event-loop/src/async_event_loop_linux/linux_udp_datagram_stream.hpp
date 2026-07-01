#pragma once

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "async_event_loop/udp.hpp"

namespace async_event_loop {

class LinuxUdpDatagramStream final : public IUdpDatagramStream {
public:
    LinuxUdpDatagramStream() = default;
    ~LinuxUdpDatagramStream() override { close(); }

    bool bind(uint16_t local_port, bool reuse_address = true) override {
        close();
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            return false;
        }

        const int yes = 1;
        if (reuse_address) {
            setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        }
        setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

        sockaddr_in sin{};
        sin.sin_family = AF_INET;
        sin.sin_port = htons(local_port);
        sin.sin_addr.s_addr = htonl(INADDR_ANY);
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&sin), sizeof(sin)) != 0) {
            close();
            return false;
        }
        set_nonblocking();
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        has_remote_ = false;
    }

    bool set_remote(const char* host, uint16_t port) override {
        if (!host || port == 0) {
            return false;
        }
        sockaddr_in sin{};
        sin.sin_family = AF_INET;
        sin.sin_port = htons(port);
        if (inet_pton(AF_INET, host, &sin.sin_addr) != 1) {
            return false;
        }
        remote_ = sin;
        has_remote_ = true;
        return true;
    }

    int write(const uint8_t* src, size_t len) override {
        if (!has_remote_) {
            return 0;
        }
        return send_to_sockaddr(src, len, remote_);
    }

    int send_to(const uint8_t* src, size_t len, const UdpEndpoint& endpoint) override {
        sockaddr_in sin{};
        sin.sin_family = AF_INET;
        sin.sin_port = htons(endpoint.port);
        if (inet_pton(AF_INET, endpoint.host, &sin.sin_addr) != 1) {
            return -1;
        }
        return send_to_sockaddr(src, len, sin);
    }

    int read(uint8_t* dst, size_t max_len) override {
        return recv_from(dst, max_len, nullptr);
    }

    int recv_from(uint8_t* dst, size_t max_len, UdpEndpoint* source) override {
        if (fd_ < 0 || !dst || max_len == 0) {
            return 0;
        }
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        const ssize_t n = ::recvfrom(fd_, dst, max_len, 0,
                                     reinterpret_cast<sockaddr*>(&from), &from_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return 0;
            }
            return -1;
        }
        if (source) {
            inet_ntop(AF_INET, &from.sin_addr, source->host, sizeof(source->host));
            source->port = ntohs(from.sin_port);
        }
        return static_cast<int>(n);
    }

    bool readable() const override { return fd_ >= 0; }
    bool valid() const override { return fd_ >= 0; }
    int native_fd() const override { return fd_; }

private:
    int send_to_sockaddr(const uint8_t* src, size_t len, const sockaddr_in& sin) {
        if (fd_ < 0 || !src || len == 0) {
            return 0;
        }
        const ssize_t n = ::sendto(fd_, src, len, 0,
                                   reinterpret_cast<const sockaddr*>(&sin), sizeof(sin));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return 0;
            }
            return -1;
        }
        return static_cast<int>(n);
    }

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
    sockaddr_in remote_{};
    bool has_remote_ = false;
};

} // namespace async_event_loop
