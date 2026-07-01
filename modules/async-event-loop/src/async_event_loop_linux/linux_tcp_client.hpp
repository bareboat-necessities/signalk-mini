#pragma once

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "async_event_loop/scheduler.hpp"
#include "async_event_loop/tcp.hpp"
#include "async_event_loop_linux/libevent_loop.hpp"

namespace async_event_loop {

class LinuxTcpClientConnection final : public ITcpConnection {
public:
    ~LinuxTcpClientConnection() override { close(); }

    bool attach(int fd, const TcpPeerInfo& peer) {
        close();
        fd_ = fd;
        peer_ = peer;
        set_nonblocking_fd(fd_);
        return fd_ >= 0;
    }

    bool valid() const override { return fd_ >= 0; }
    void close() override {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        line_size_ = 0;
    }

    const TcpPeerInfo& peer() const override { return peer_; }

    size_t input_size() const override {
        if (fd_ < 0) {
            return 0;
        }
        int bytes = 0;
        if (ioctl(fd_, FIONREAD, &bytes) == 0 && bytes > 0) {
            return static_cast<size_t>(bytes);
        }
        return 0;
    }

    size_t output_size() const override { return 0; }

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
        int flags = 0;
#ifdef MSG_NOSIGNAL
        flags |= MSG_NOSIGNAL;
#endif
        const ssize_t n = ::send(fd_, src, len, flags);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return 0;
            }
            return -1;
        }
        return static_cast<int>(n);
    }

    bool peek(uint8_t* dst, size_t len) override {
        if (fd_ < 0 || !dst || len == 0) {
            return false;
        }
        const ssize_t n = ::recv(fd_, dst, len, MSG_PEEK);
        return n == static_cast<ssize_t>(len);
    }

    bool read_exact(uint8_t* dst, size_t len) override {
        if (!dst || input_size() < len) {
            return false;
        }
        return read(dst, len) == static_cast<int>(len);
    }

    bool read_line(char* dst, size_t max_len, bool strip_cr = true) override {
        if (!dst || max_len == 0) {
            return false;
        }
        uint8_t byte = 0;
        while (true) {
            const int n = read(&byte, 1);
            if (n <= 0) {
                return false;
            }
            if (byte == '\n') {
                size_t len = line_size_;
                if (strip_cr && len > 0 && line_[len - 1] == '\r') {
                    --len;
                }
                const size_t copy = len < max_len - 1 ? len : max_len - 1;
                memcpy(dst, line_, copy);
                dst[copy] = '\0';
                line_size_ = 0;
                return true;
            }
            if (line_size_ < sizeof(line_)) {
                line_[line_size_++] = static_cast<char>(byte);
            } else {
                line_size_ = 0;
            }
        }
    }

    int native_fd() const { return fd_; }

    static bool set_nonblocking_fd(int fd) {
        if (fd < 0) {
            return false;
        }
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags < 0) {
            return false;
        }
        return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
    }

private:
    int fd_ = -1;
    TcpPeerInfo peer_;
    char line_[256]{};
    size_t line_size_ = 0;
};

class LinuxTcpClient final : public IRuntimeTask {
public:
    explicit LinuxTcpClient(LinuxLibeventLoop& loop) : loop_(loop) {}
    ~LinuxTcpClient() override { close(); }

    bool connect(const TcpConnectOptions& options, ITcpClientHandler& handler) {
        close();
        if (!options.host || options.port == 0) {
            handler.on_error(EINVAL);
            return false;
        }

        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

        char port_text[8];
        snprintf(port_text, sizeof(port_text), "%u", static_cast<unsigned>(options.port));

        addrinfo* results = nullptr;
        const int gai = getaddrinfo(options.host, port_text, &hints, &results);
        if (gai != 0 || !results) {
            handler.on_error(EINVAL);
            return false;
        }

        int last_error = EINVAL;
        for (addrinfo* ai = results; ai; ai = ai->ai_next) {
            const int fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
            if (fd < 0) {
                last_error = errno;
                continue;
            }
            if (!LinuxTcpClientConnection::set_nonblocking_fd(fd)) {
                last_error = errno ? errno : EINVAL;
                ::close(fd);
                continue;
            }

            TcpPeerInfo peer{};
            fill_peer(peer, ai->ai_addr, ai->ai_addrlen, options.host, options.port);

            if (::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) {
                freeaddrinfo(results);
                handler_ = &handler;
                return activate_connected_fd(fd, peer);
            }

            if (errno == EINPROGRESS || errno == EALREADY || errno == EWOULDBLOCK) {
                freeaddrinfo(results);
                handler_ = &handler;
                pending_fd_ = fd;
                pending_peer_ = peer;
                state_ = State::Connecting;
                if (!loop_.add_writable_fd(pending_fd_, *this)) {
                    const int err = errno ? errno : EINVAL;
                    close();
                    handler.on_error(err);
                    return false;
                }
                registered_fd_ = pending_fd_;
                return true;
            }

            last_error = errno;
            ::close(fd);
        }

        freeaddrinfo(results);
        handler.on_error(last_error);
        return false;
    }

    void close() {
        unregister_fd();
        if (pending_fd_ >= 0) {
            ::close(pending_fd_);
            pending_fd_ = -1;
        }
        connection_.close();
        handler_ = nullptr;
        state_ = State::Idle;
    }

    bool valid() const { return state_ != State::Idle || connection_.valid(); }
    bool connected() const { return state_ == State::Connected && connection_.valid(); }
    bool connecting() const { return state_ == State::Connecting && pending_fd_ >= 0; }
    ITcpConnection& connection() { return connection_; }

    void poll(uint64_t) override {
        if (state_ == State::Connecting) {
            complete_connect();
            return;
        }
        if (state_ != State::Connected || !handler_ || !connection_.valid()) {
            return;
        }

        uint8_t probe = 0;
        const int fd = connection_.native_fd();
        const ssize_t n = ::recv(fd, &probe, 1, MSG_PEEK);
        if (n > 0) {
            handler_->on_data(connection_);
            return;
        }
        if (n == 0) {
            ITcpClientHandler* handler = handler_;
            handler->on_close(connection_);
            close();
            return;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return;
        }

        const int err = errno;
        ITcpClientHandler* handler = handler_;
        handler->on_error(err);
        close();
    }

private:
    enum class State {
        Idle,
        Connecting,
        Connected
    };

    static void fill_peer(TcpPeerInfo& peer,
                          const sockaddr* addr,
                          socklen_t addr_len,
                          const char* fallback_host,
                          uint16_t fallback_port) {
        peer.port = fallback_port;
        if (addr && addr_len >= sizeof(sockaddr_in) && addr->sa_family == AF_INET) {
            const auto* sin = reinterpret_cast<const sockaddr_in*>(addr);
            inet_ntop(AF_INET, &sin->sin_addr, peer.host, sizeof(peer.host));
            peer.port = ntohs(sin->sin_port);
            return;
        }
        snprintf(peer.host, sizeof(peer.host), "%s", fallback_host ? fallback_host : "");
    }

    bool activate_connected_fd(int fd, const TcpPeerInfo& peer) {
        connection_.attach(fd, peer);
        state_ = State::Connected;
        if (!loop_.add_readable_fd(connection_.native_fd(), *this)) {
            const int err = errno ? errno : EINVAL;
            ITcpClientHandler* handler = handler_;
            connection_.close();
            state_ = State::Idle;
            handler_ = nullptr;
            if (handler) {
                handler->on_error(err);
            }
            return false;
        }
        registered_fd_ = connection_.native_fd();
        if (handler_) {
            handler_->on_connect(connection_, connection_.peer());
        }
        return true;
    }

    void complete_connect() {
        if (!handler_ || pending_fd_ < 0) {
            close();
            return;
        }

        int err = 0;
        socklen_t err_len = sizeof(err);
        if (getsockopt(pending_fd_, SOL_SOCKET, SO_ERROR, &err, &err_len) != 0) {
            err = errno ? errno : EINVAL;
        }
        if (err != 0) {
            ITcpClientHandler* handler = handler_;
            handler->on_error(err);
            close();
            return;
        }

        const int fd = pending_fd_;
        const TcpPeerInfo peer = pending_peer_;
        unregister_fd();
        pending_fd_ = -1;
        activate_connected_fd(fd, peer);
    }

    void unregister_fd() {
        if (registered_fd_ >= 0) {
            loop_.remove_fd(registered_fd_, *this);
            registered_fd_ = -1;
        }
    }

    LinuxLibeventLoop& loop_;
    ITcpClientHandler* handler_ = nullptr;
    LinuxTcpClientConnection connection_;
    TcpPeerInfo pending_peer_{};
    int pending_fd_ = -1;
    int registered_fd_ = -1;
    State state_ = State::Idle;
};

} // namespace async_event_loop
