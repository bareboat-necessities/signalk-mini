#pragma once

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <memory>
#include <vector>

#include <event2/buffer.h>
#include <event2/bufferevent.h>
#include <event2/listener.h>
#include <event2/util.h>

#include "async_event_loop/tcp.hpp"
#include "async_event_loop_linux/libevent_loop.hpp"

namespace async_event_loop {

class LinuxTcpServer;

class LinuxTcpConnection final : public ITcpConnection {
public:
    LinuxTcpConnection(LinuxTcpServer* owner,
                       event_base* base,
                       evutil_socket_t fd,
                       const sockaddr* addr,
                       int addrlen,
                       ITcpServerHandler& handler);
    ~LinuxTcpConnection() override;

    bool valid() const override { return bev_ != nullptr; }
    void close() override;

    const TcpPeerInfo& peer() const override { return peer_; }
    size_t input_size() const override;
    size_t output_size() const override;
    int read(uint8_t* dst, size_t max_len) override;
    int write(const uint8_t* src, size_t len) override;
    bool peek(uint8_t* dst, size_t len) override;
    bool read_exact(uint8_t* dst, size_t len) override;
    bool read_line(char* dst, size_t max_len, bool strip_cr = true) override;
    bool set_timeouts(const TcpTimeoutOptions& options) override;
    bool set_watermarks(const TcpWatermarkOptions& options) override;

private:
    friend class LinuxTcpServer;

    static void read_callback(bufferevent* bev, void* ctx);
    static void write_callback(bufferevent* bev, void* ctx);
    static void event_callback(bufferevent* bev, short events, void* ctx);

    void notify_event(short events);
    void close_bev();
    void detach() { owner_ = nullptr; }
    bool remove_requested() const { return remove_requested_; }

    LinuxTcpServer* owner_ = nullptr;
    bufferevent* bev_ = nullptr;
    ITcpServerHandler& handler_;
    TcpPeerInfo peer_;
    bool remove_requested_ = false;
};

class LinuxTcpServer final {
    friend class LinuxTcpConnection;

public:
    explicit LinuxTcpServer(LinuxLibeventLoop& loop, size_t max_connections = 16)
        : loop_(loop), max_connections_(max_connections) {}

    LinuxTcpServer(const LinuxTcpServer&) = delete;
    LinuxTcpServer& operator=(const LinuxTcpServer&) = delete;

    ~LinuxTcpServer() { close(); }

    bool listen(const TcpListenOptions& options, ITcpServerHandler& handler) {
        close();

        sockaddr_in sin{};
        sin.sin_family = AF_INET;
        sin.sin_port = htons(options.port);
        if (!options.host || strcmp(options.host, "0.0.0.0") == 0) {
            sin.sin_addr.s_addr = htonl(INADDR_ANY);
        } else if (inet_pton(AF_INET, options.host, &sin.sin_addr) != 1) {
            return false;
        }

        unsigned flags = 0;
        if (options.close_on_free) {
            flags |= LEV_OPT_CLOSE_ON_FREE;
        }
        if (options.reuse_address) {
            flags |= LEV_OPT_REUSEABLE;
        }

        listener_ = evconnlistener_new_bind(
            loop_.base(),
            &LinuxTcpServer::accept_callback,
            this,
            flags,
            options.backlog,
            reinterpret_cast<sockaddr*>(&sin),
            sizeof(sin));
        if (!listener_) {
            return false;
        }
        handler_ = &handler;
        evconnlistener_set_error_cb(listener_, &LinuxTcpServer::listener_error_callback);
        update_bound_port();
        return true;
    }

    void close() {
        for (auto& connection : connections_) {
            if (connection) {
                connection->detach();
            }
        }
        connections_.clear();
        if (listener_) {
            evconnlistener_free(listener_);
            listener_ = nullptr;
        }
        handler_ = nullptr;
        bound_port_ = 0;
    }

    bool valid() const { return listener_ != nullptr; }
    uint16_t port() const { return bound_port_; }

    size_t connection_count() const {
        size_t count = 0;
        for (const auto& connection : connections_) {
            if (connection && connection->valid()) {
                ++count;
            }
        }
        return count;
    }

    void remove(LinuxTcpConnection* connection) {
        if (!connection) {
            return;
        }
        connection->close();
        sweep_closed();
    }

private:
    static void accept_callback(evconnlistener*, evutil_socket_t fd, sockaddr* addr, int addrlen, void* ctx) {
        auto* self = static_cast<LinuxTcpServer*>(ctx);
        if (!self || !self->handler_) {
            evutil_closesocket(fd);
            return;
        }
        self->sweep_closed();
        if (self->connection_count() >= self->max_connections_) {
            evutil_closesocket(fd);
            return;
        }

        std::unique_ptr<LinuxTcpConnection> connection(
            new LinuxTcpConnection(self, self->loop_.base(), fd, addr, addrlen, *self->handler_));
        if (!connection->valid()) {
            return;
        }
        LinuxTcpConnection* raw = connection.get();
        self->connections_.push_back(std::move(connection));
        self->handler_->on_accept(*raw, raw->peer());
        self->sweep_closed();
    }

    static void listener_error_callback(evconnlistener* listener, void* ctx) {
        auto* self = static_cast<LinuxTcpServer*>(ctx);
        if (!self || !self->handler_) {
            return;
        }
        const int err = EVUTIL_SOCKET_ERROR();
        self->handler_->on_listener_error(err);
        (void)listener;
    }

    void update_bound_port() {
        bound_port_ = 0;
        if (!listener_) {
            return;
        }
        const evutil_socket_t fd = evconnlistener_get_fd(listener_);
        sockaddr_in sin{};
        socklen_t len = sizeof(sin);
        if (getsockname(fd, reinterpret_cast<sockaddr*>(&sin), &len) == 0) {
            bound_port_ = ntohs(sin.sin_port);
        }
    }

    void sweep_closed() {
        for (auto it = connections_.begin(); it != connections_.end();) {
            LinuxTcpConnection* connection = it->get();
            if (!connection || connection->remove_requested()) {
                if (connection) {
                    connection->detach();
                }
                it = connections_.erase(it);
            } else {
                ++it;
            }
        }
    }

    LinuxLibeventLoop& loop_;
    size_t max_connections_ = 16;
    evconnlistener* listener_ = nullptr;
    ITcpServerHandler* handler_ = nullptr;
    uint16_t bound_port_ = 0;
    std::vector<std::unique_ptr<LinuxTcpConnection>> connections_;
};

inline LinuxTcpConnection::LinuxTcpConnection(LinuxTcpServer* owner,
                                              event_base* base,
                                              evutil_socket_t fd,
                                              const sockaddr* addr,
                                              int addrlen,
                                              ITcpServerHandler& handler)
    : owner_(owner), handler_(handler) {
    bev_ = bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE);
    if (!bev_) {
        evutil_closesocket(fd);
        remove_requested_ = true;
        return;
    }
    bufferevent_setcb(bev_, &LinuxTcpConnection::read_callback,
                      &LinuxTcpConnection::write_callback,
                      &LinuxTcpConnection::event_callback,
                      this);
    bufferevent_enable(bev_, EV_READ | EV_WRITE);

    if (addr && addrlen >= static_cast<int>(sizeof(sockaddr_in)) && addr->sa_family == AF_INET) {
        const auto* sin = reinterpret_cast<const sockaddr_in*>(addr);
        inet_ntop(AF_INET, &sin->sin_addr, peer_.host, sizeof(peer_.host));
        peer_.port = ntohs(sin->sin_port);
    }
}

inline LinuxTcpConnection::~LinuxTcpConnection() {
    close_bev();
}

inline void LinuxTcpConnection::close_bev() {
    if (bev_) {
        bufferevent* old = bev_;
        bev_ = nullptr;
        bufferevent_free(old);
    }
}

inline void LinuxTcpConnection::close() {
    close_bev();
    remove_requested_ = true;
}

inline size_t LinuxTcpConnection::input_size() const {
    return bev_ ? evbuffer_get_length(bufferevent_get_input(bev_)) : 0;
}

inline size_t LinuxTcpConnection::output_size() const {
    return bev_ ? evbuffer_get_length(bufferevent_get_output(bev_)) : 0;
}

inline int LinuxTcpConnection::read(uint8_t* dst, size_t max_len) {
    if (!bev_ || !dst || max_len == 0) {
        return 0;
    }
    return static_cast<int>(evbuffer_remove(bufferevent_get_input(bev_), dst, max_len));
}

inline int LinuxTcpConnection::write(const uint8_t* src, size_t len) {
    if (!bev_ || !src || len == 0) {
        return 0;
    }
    return bufferevent_write(bev_, src, len) == 0 ? static_cast<int>(len) : -1;
}

inline bool LinuxTcpConnection::peek(uint8_t* dst, size_t len) {
    if (!bev_ || !dst || input_size() < len) {
        return false;
    }
    return evbuffer_copyout(bufferevent_get_input(bev_), dst, len) == static_cast<ev_ssize_t>(len);
}

inline bool LinuxTcpConnection::read_exact(uint8_t* dst, size_t len) {
    if (!bev_ || !dst || input_size() < len) {
        return false;
    }
    return evbuffer_remove(bufferevent_get_input(bev_), dst, len) == static_cast<int>(len);
}

inline bool LinuxTcpConnection::read_line(char* dst, size_t max_len, bool strip_cr) {
    if (!bev_ || !dst || max_len == 0) {
        return false;
    }
    size_t n = 0;
    char* line = evbuffer_readln(bufferevent_get_input(bev_), &n, EVBUFFER_EOL_LF);
    if (!line) {
        return false;
    }
    if (strip_cr && n > 0 && line[n - 1] == '\r') {
        --n;
    }
    const size_t copy = n < (max_len - 1) ? n : (max_len - 1);
    memcpy(dst, line, copy);
    dst[copy] = '\0';
    free(line);
    return true;
}

inline bool LinuxTcpConnection::set_timeouts(const TcpTimeoutOptions& options) {
    if (!bev_) return false;
    timeval read_timeout{};
    timeval write_timeout{};
    timeval* read_ptr = nullptr;
    timeval* write_ptr = nullptr;
    if (options.read_timeout_ms != 0) {
        read_timeout.tv_sec = static_cast<long>(options.read_timeout_ms / 1000);
        read_timeout.tv_usec = static_cast<long>((options.read_timeout_ms % 1000) * 1000);
        read_ptr = &read_timeout;
    }
    if (options.write_timeout_ms != 0) {
        write_timeout.tv_sec = static_cast<long>(options.write_timeout_ms / 1000);
        write_timeout.tv_usec = static_cast<long>((options.write_timeout_ms % 1000) * 1000);
        write_ptr = &write_timeout;
    }
    bufferevent_set_timeouts(bev_, read_ptr, write_ptr);
    return true;
}

inline bool LinuxTcpConnection::set_watermarks(const TcpWatermarkOptions& options) {
    if (!bev_) return false;
    bufferevent_setwatermark(bev_, EV_READ, options.read_low, options.read_high);
    bufferevent_setwatermark(bev_, EV_WRITE, options.write_low, options.write_high);
    return true;
}

inline void LinuxTcpConnection::read_callback(bufferevent*, void* ctx) {
    auto* self = static_cast<LinuxTcpConnection*>(ctx);
    LinuxTcpServer* owner = self ? self->owner_ : nullptr;
    if (self && self->bev_) {
        self->handler_.on_data(*self);
    }
    if (owner) {
        owner->sweep_closed();
    }
}

inline void LinuxTcpConnection::write_callback(bufferevent*, void* ctx) {
    auto* self = static_cast<LinuxTcpConnection*>(ctx);
    LinuxTcpServer* owner = self ? self->owner_ : nullptr;
    if (self && self->bev_) {
        self->handler_.on_write_ready(*self);
    }
    if (owner) {
        owner->sweep_closed();
    }
}

inline void LinuxTcpConnection::event_callback(bufferevent*, short events, void* ctx) {
    auto* self = static_cast<LinuxTcpConnection*>(ctx);
    LinuxTcpServer* owner = self ? self->owner_ : nullptr;
    if (self) {
        self->notify_event(events);
    }
    if (owner) {
        owner->sweep_closed();
    }
}

inline void LinuxTcpConnection::notify_event(short events) {
    if (events & BEV_EVENT_TIMEOUT) {
        handler_.on_error(*this, ETIMEDOUT);
        close();
    } else if (events & BEV_EVENT_ERROR) {
        handler_.on_error(*this, EVUTIL_SOCKET_ERROR());
        close();
    } else if (events & BEV_EVENT_EOF) {
        handler_.on_close(*this);
        close();
    }
}

} // namespace async_event_loop
