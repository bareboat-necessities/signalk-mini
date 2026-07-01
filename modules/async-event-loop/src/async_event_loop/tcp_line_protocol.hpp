#pragma once

#include <string.h>

#include "protocol_reader.hpp"
#include "tcp.hpp"

namespace async_event_loop {

inline bool line_equals(LineView line, const char* text) {
    if (!text) {
        return line.size == 0;
    }
    const size_t len = strlen(text);
    return line.size == len && memcmp(line.data, text, len) == 0;
}

template<size_t Capacity>
inline const char* line_to_cstr(LineView line, char (&dst)[Capacity]) {
    if (Capacity == 0) {
        return "";
    }
    size_t n = line.size;
    if (n >= Capacity) {
        n = Capacity - 1;
    }
    if (n > 0 && line.data) {
        memcpy(dst, line.data, n);
    }
    dst[n] = '\0';
    return dst;
}

inline int tcp_write_text(ITcpConnection& connection, const char* text) {
    if (!text) {
        return 0;
    }
    return connection.write(reinterpret_cast<const uint8_t*>(text), strlen(text));
}

inline int tcp_write_newline(ITcpConnection& connection) {
    const uint8_t newline = '\n';
    return connection.write(&newline, 1);
}

inline bool tcp_write_line(ITcpConnection& connection, LineView line) {
    const int body = line.size == 0 ? 0 : connection.write(reinterpret_cast<const uint8_t*>(line.data), line.size);
    if (body < 0 || static_cast<size_t>(body) != line.size) {
        return false;
    }
    return tcp_write_newline(connection) == 1;
}

inline bool tcp_write_line(ITcpConnection& connection, const char* text) {
    if (tcp_write_text(connection, text) < 0) {
        return false;
    }
    return tcp_write_newline(connection) == 1;
}

struct TcpLineServerOptions {
    TcpBackpressureOptions backpressure;
};

class ITcpLineServerHandler {
public:
    virtual ~ITcpLineServerHandler() = default;

    virtual void on_accept(ITcpConnection& connection, const TcpPeerInfo& peer) {
        (void)connection;
        (void)peer;
    }

    virtual void on_line(ITcpConnection& connection, LineView line) = 0;

    virtual void on_close(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_error(ITcpConnection& connection, int error_code) {
        (void)connection;
        (void)error_code;
    }

    virtual void on_listener_error(int error_code) {
        (void)error_code;
    }

    virtual void on_too_many_connections(ITcpConnection& connection) {
        connection.close();
    }

    virtual void on_backpressure(ITcpConnection& connection, const TcpBackpressureInfo& info) {
        (void)connection;
        (void)info;
    }
};

template<size_t BufferSize = 256, size_t MaxConnections = 8, size_t CallbackStorageSize = 64>
class TcpLineServerHandler final : public ITcpServerHandler {
public:
    explicit TcpLineServerHandler(ITcpLineServerHandler& handler)
        : TcpLineServerHandler(handler, TcpLineServerOptions{}) {}

    TcpLineServerHandler(ITcpLineServerHandler& handler, const TcpLineServerOptions& options)
        : handler_(handler), options_(options) {
        for (size_t i = 0; i < MaxConnections; ++i) {
            slots_[i].owner = this;
            slots_[i].backpressure.configure(options_.backpressure);
        }
    }

    void on_accept(ITcpConnection& connection, const TcpPeerInfo& peer) override {
        Slot* slot = acquire(connection);
        if (!slot) {
            handler_.on_too_many_connections(connection);
            return;
        }
        slot->backpressure.reset();
        handler_.on_accept(connection, peer);
    }

    void on_data(ITcpConnection& connection) override {
        Slot* slot = acquire(connection);
        if (!slot) {
            handler_.on_too_many_connections(connection);
            return;
        }
        slot->reader.poll(0);
        check_all_backpressure();
    }

    void on_write_ready(ITcpConnection& connection) override {
        Slot* slot = find(connection);
        if (slot) {
            check_backpressure(*slot);
        }
    }

    void on_close(ITcpConnection& connection) override {
        handler_.on_close(connection);
        release(connection);
    }

    void on_error(ITcpConnection& connection, int error_code) override {
        handler_.on_error(connection, error_code);
        release(connection);
    }

    void on_listener_error(int error_code) override {
        handler_.on_listener_error(error_code);
    }

    size_t active_connections() const {
        size_t count = 0;
        for (size_t i = 0; i < MaxConnections; ++i) {
            if (slots_[i].stream.connection()) {
                ++count;
            }
        }
        return count;
    }

private:
    struct Slot {
        TcpConnectionByteStream stream;
        TcpLineServerHandler* owner = nullptr;
        TcpBackpressureMonitor backpressure;
        LineProtocolReader<BufferSize, CallbackStorageSize> reader;

        Slot() : reader(stream, LineProtocolOptions{}, [this](LineView line) {
            if (owner) {
                owner->handle_line(*this, line);
            }
        }) {}
    };

    Slot* find(ITcpConnection& connection) {
        for (size_t i = 0; i < MaxConnections; ++i) {
            if (slots_[i].stream.connection() == &connection) {
                return &slots_[i];
            }
        }
        return nullptr;
    }

    Slot* acquire(ITcpConnection& connection) {
        Slot* existing = find(connection);
        if (existing) {
            return existing;
        }
        for (size_t i = 0; i < MaxConnections; ++i) {
            if (!slots_[i].stream.connection()) {
                slots_[i].stream.bind(connection);
                slots_[i].backpressure.reset();
                return &slots_[i];
            }
        }
        return nullptr;
    }

    void release(ITcpConnection& connection) {
        Slot* slot = find(connection);
        if (slot) {
            slot->backpressure.reset();
            slot->stream.unbind();
        }
    }

    void handle_line(Slot& slot, LineView line) {
        ITcpConnection* connection = slot.stream.connection();
        if (connection && connection->valid()) {
            handler_.on_line(*connection, line);
        }
    }

    void check_all_backpressure() {
        for (size_t i = 0; i < MaxConnections; ++i) {
            check_backpressure(slots_[i]);
        }
    }

    void check_backpressure(Slot& slot) {
        ITcpConnection* connection = slot.stream.connection();
        if (!connection || !connection->valid()) {
            return;
        }

        TcpBackpressureInfo info;
        if (!slot.backpressure.check(*connection, &info)) {
            return;
        }

        handler_.on_backpressure(*connection, info);
        if (options_.backpressure.close_on_limit && connection->valid()) {
            connection->close();
        }
        if (!connection->valid()) {
            slot.backpressure.reset();
            slot.stream.unbind();
        }
    }

    ITcpLineServerHandler& handler_;
    TcpLineServerOptions options_;
    Slot slots_[MaxConnections];
};

class ITcpLineClientHandler {
public:
    virtual ~ITcpLineClientHandler() = default;

    virtual void on_connect(ITcpConnection& connection, const TcpPeerInfo& peer) {
        (void)connection;
        (void)peer;
    }

    virtual void on_line(ITcpConnection& connection, LineView line) = 0;

    virtual void on_close(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_error(int error_code) {
        (void)error_code;
    }
};

template<size_t BufferSize = 256, size_t CallbackStorageSize = 64>
class TcpLineClientHandler final : public ITcpClientHandler {
public:
    explicit TcpLineClientHandler(ITcpLineClientHandler& handler)
        : handler_(handler),
          reader_(stream_, LineProtocolOptions{}, [this](LineView line) { handle_line(line); }) {}

    ITcpConnection* connection() { return stream_.connection(); }
    const ITcpConnection* connection() const { return stream_.connection(); }

    void on_connect(ITcpConnection& connection, const TcpPeerInfo& peer) override {
        stream_.bind(connection);
        handler_.on_connect(connection, peer);
    }

    void on_data(ITcpConnection& connection) override {
        (void)connection;
        reader_.poll(0);
    }

    void on_close(ITcpConnection& connection) override {
        handler_.on_close(connection);
        stream_.unbind();
    }

    void on_error(int error_code) override {
        handler_.on_error(error_code);
        stream_.unbind();
    }

private:
    void handle_line(LineView line) {
        ITcpConnection* active = stream_.connection();
        if (active && active->valid()) {
            handler_.on_line(*active, line);
        }
    }

    ITcpLineClientHandler& handler_;
    TcpConnectionByteStream stream_;
    LineProtocolReader<BufferSize, CallbackStorageSize> reader_;
};

} // namespace async_event_loop
