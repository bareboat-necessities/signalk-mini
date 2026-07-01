#pragma once

#include <stddef.h>
#include <stdint.h>

#include "event_loop.hpp"

namespace async_event_loop {

struct TcpListenOptions {
    const char* host = "127.0.0.1";
    uint16_t port = 0;
    int backlog = -1;
    bool reuse_address = true;
    bool close_on_free = true;
};

struct TcpConnectOptions {
    const char* host = "127.0.0.1";
    uint16_t port = 0;
};

struct TcpTimeoutOptions {
    uint32_t read_timeout_ms = 0;
    uint32_t write_timeout_ms = 0;
};

struct TcpWatermarkOptions {
    size_t read_low = 0;
    size_t read_high = 0;
    size_t write_low = 0;
    size_t write_high = 0;
};

struct TcpPeerInfo {
    char host[64]{};
    uint16_t port = 0;
};

class ITcpConnection : public IByteStream {
public:
    ~ITcpConnection() override = default;

    virtual void close() = 0;

    virtual const TcpPeerInfo& peer() const = 0;
    virtual size_t input_size() const = 0;
    virtual size_t output_size() const = 0;

    int read(uint8_t* dst, size_t max_len) override = 0;
    int write(const uint8_t* src, size_t len) override = 0;

    bool readable() const override { return valid() && input_size() > 0; }
    bool writable() const override { return valid(); }

    virtual bool peek(uint8_t* dst, size_t len) = 0;
    virtual bool read_exact(uint8_t* dst, size_t len) = 0;
    virtual bool read_line(char* dst, size_t max_len, bool strip_cr = true) = 0;

    virtual bool set_timeouts(const TcpTimeoutOptions& options) {
        (void)options;
        return false;
    }

    virtual bool set_watermarks(const TcpWatermarkOptions& options) {
        (void)options;
        return false;
    }
};

struct TcpBackpressureOptions {
    bool enabled = false;
    size_t high_water_bytes = 32768;
    size_t low_water_bytes = 8192;
    uint32_t max_over_high_polls = 3;
    bool close_on_limit = true;

    static TcpBackpressureOptions disabled() {
        return TcpBackpressureOptions{};
    }

    static TcpBackpressureOptions server_default() {
        TcpBackpressureOptions options;
        options.enabled = true;
        return options;
    }

    static TcpBackpressureOptions embedded_default() {
        TcpBackpressureOptions options;
        options.enabled = true;
        options.high_water_bytes = 768;
        options.low_water_bytes = 256;
        options.max_over_high_polls = 2;
        return options;
    }
};

struct TcpBackpressureInfo {
    size_t pending_bytes = 0;
    size_t high_water_bytes = 0;
    size_t low_water_bytes = 0;
    uint32_t over_high_polls = 0;
    bool close_on_limit = false;
};

class TcpBackpressureMonitor final {
public:
    TcpBackpressureMonitor() = default;
    explicit TcpBackpressureMonitor(const TcpBackpressureOptions& options) : options_(options) {}

    void configure(const TcpBackpressureOptions& options) {
        options_ = options;
        reset();
    }

    const TcpBackpressureOptions& options() const { return options_; }
    bool enabled() const { return options_.enabled; }

    void reset() {
        over_high_polls_ = 0;
        reported_ = false;
    }

    bool check(ITcpConnection& connection, TcpBackpressureInfo* info = nullptr) {
        if (!options_.enabled || !connection.valid()) {
            fill_info(connection, info);
            return false;
        }

        const size_t pending = connection.output_size();
        if (pending > options_.high_water_bytes) {
            if (over_high_polls_ < UINT32_MAX) {
                ++over_high_polls_;
            }
        } else if (pending <= options_.low_water_bytes) {
            over_high_polls_ = 0;
            reported_ = false;
        }

        fill_info(connection, info);

        const uint32_t trigger_polls = options_.max_over_high_polls == 0 ? 1 : options_.max_over_high_polls;
        if (over_high_polls_ >= trigger_polls && !reported_) {
            reported_ = true;
            return true;
        }
        return false;
    }

private:
    void fill_info(ITcpConnection& connection, TcpBackpressureInfo* info) const {
        if (!info) {
            return;
        }
        info->pending_bytes = connection.valid() ? connection.output_size() : 0;
        info->high_water_bytes = options_.high_water_bytes;
        info->low_water_bytes = options_.low_water_bytes;
        info->over_high_polls = over_high_polls_;
        info->close_on_limit = options_.close_on_limit;
    }

    TcpBackpressureOptions options_;
    uint32_t over_high_polls_ = 0;
    bool reported_ = false;
};

template<size_t MaxConnections>
class TcpConnectionRegistry final {
public:
    static constexpr size_t capacity() { return MaxConnections; }

    bool add(ITcpConnection& connection) {
        const int existing = index_of(connection);
        if (existing >= 0) {
            return true;
        }
        for (size_t i = 0; i < MaxConnections; ++i) {
            if (!connections_[i]) {
                connections_[i] = &connection;
                ++count_;
                return true;
            }
        }
        return false;
    }

    bool remove(ITcpConnection& connection) {
        const int index = index_of(connection);
        if (index < 0) {
            return false;
        }
        connections_[static_cast<size_t>(index)] = nullptr;
        if (count_ > 0) {
            --count_;
        }
        return true;
    }

    void clear() {
        for (size_t i = 0; i < MaxConnections; ++i) {
            connections_[i] = nullptr;
        }
        count_ = 0;
    }

    void prune_invalid() {
        for (size_t i = 0; i < MaxConnections; ++i) {
            ITcpConnection* connection = connections_[i];
            if (connection && !connection->valid()) {
                connections_[i] = nullptr;
                if (count_ > 0) {
                    --count_;
                }
            }
        }
    }

    size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }
    bool full() const { return count_ >= MaxConnections; }

    bool contains(ITcpConnection& connection) const {
        return index_of(connection) >= 0;
    }

    int index_of(ITcpConnection& connection) const {
        for (size_t i = 0; i < MaxConnections; ++i) {
            if (connections_[i] == &connection) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    ITcpConnection* at(size_t index) {
        return index < MaxConnections ? connections_[index] : nullptr;
    }

    const ITcpConnection* at(size_t index) const {
        return index < MaxConnections ? connections_[index] : nullptr;
    }

    ITcpConnection* operator[](size_t index) { return at(index); }
    const ITcpConnection* operator[](size_t index) const { return at(index); }

    template<typename Callback>
    void for_each(Callback callback) {
        for (size_t i = 0; i < MaxConnections; ++i) {
            ITcpConnection* connection = connections_[i];
            if (connection && connection->valid()) {
                callback(*connection);
            }
        }
    }

private:
    ITcpConnection* connections_[MaxConnections]{};
    size_t count_ = 0;
};

class TcpConnectionByteStream final : public IByteStream {
public:
    TcpConnectionByteStream() = default;
    explicit TcpConnectionByteStream(ITcpConnection& connection) : connection_(&connection) {}

    void bind(ITcpConnection* connection) { connection_ = connection; }
    void bind(ITcpConnection& connection) { connection_ = &connection; }
    void unbind() { connection_ = nullptr; }

    ITcpConnection* connection() { return connection_; }
    const ITcpConnection* connection() const { return connection_; }

    int read(uint8_t* dst, size_t max_len) override {
        return connection_ ? connection_->read(dst, max_len) : 0;
    }

    int write(const uint8_t* src, size_t len) override {
        return connection_ ? connection_->write(src, len) : 0;
    }

    bool readable() const override {
        return connection_ && connection_->readable();
    }

    bool writable() const override {
        return connection_ && connection_->writable();
    }

    bool valid() const override {
        return connection_ && connection_->valid();
    }

private:
    ITcpConnection* connection_ = nullptr;
};

class ITcpServerHandler {
public:
    virtual ~ITcpServerHandler() = default;

    virtual void on_accept(ITcpConnection& connection, const TcpPeerInfo& peer) {
        (void)connection;
        (void)peer;
    }

    virtual void on_data(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_write_ready(ITcpConnection& connection) {
        (void)connection;
    }

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
};

class ITcpClientHandler {
public:
    virtual ~ITcpClientHandler() = default;

    virtual void on_connect(ITcpConnection& connection, const TcpPeerInfo& peer) {
        (void)connection;
        (void)peer;
    }

    virtual void on_data(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_write_ready(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_close(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_error(int error_code) {
        (void)error_code;
    }
};

} // namespace async_event_loop
