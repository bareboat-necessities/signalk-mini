#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <string.h>

#include "async_event_loop/scheduler.hpp"
#include "async_event_loop/tcp.hpp"
#include "async_event_loop_arduino/arduino_loop.hpp"

namespace async_event_loop {

class ArduinoWiFiTcpConnection final : public ITcpConnection {
public:
    ArduinoWiFiTcpConnection() = default;

    bool attach(WiFiClient client) {
        close();
        client_ = client;
        active_ = true;
        tx_head_ = 0;
        tx_size_ = 0;
        update_peer();
        return valid();
    }

    bool valid() const override { return active_ && client_; }

    bool closed_by_peer() const {
        return active_ && !client_.connected() && client_.available() == 0;
    }

    void close() override {
        if (active_) {
            client_.stop();
        }
        active_ = false;
        line_size_ = 0;
        tx_head_ = 0;
        tx_size_ = 0;
    }

    const TcpPeerInfo& peer() const override { return peer_; }
    size_t input_size() const override { return active_ ? static_cast<size_t>(client_.available()) : 0; }
    size_t output_size() const override { return tx_size_; }

    int read(uint8_t* dst, size_t max_len) override {
        if (!active_ || !dst || max_len == 0) {
            return 0;
        }
        size_t n = 0;
        while (n < max_len && client_.available() > 0) {
            const int c = client_.read();
            if (c < 0) {
                break;
            }
            dst[n++] = static_cast<uint8_t>(c);
        }
        return static_cast<int>(n);
    }

    int write(const uint8_t* src, size_t len) override {
        if (!active_ || !src || len == 0) {
            return 0;
        }
        flush_output();
        return static_cast<int>(push_output(src, len));
    }

    size_t flush_output() {
        if (!active_ || tx_size_ == 0) {
            return 0;
        }

        size_t flushed = 0;
        while (tx_size_ > 0 && client_.connected()) {
            size_t n = contiguous_output_size();
            if (n == 0) {
                break;
            }

#if defined(ESP32) || defined(ESP8266)
            const int writable = client_.availableForWrite();
            if (writable <= 0) {
                break;
            }
            if (n > static_cast<size_t>(writable)) {
                n = static_cast<size_t>(writable);
            }
#else
            if (n > tx_flush_chunk) {
                n = tx_flush_chunk;
            }
#endif

            const size_t written = client_.write(tx_ + tx_head_, n);
            if (written == 0) {
                break;
            }
            pop_output(written);
            flushed += written;
            if (written < n) {
                break;
            }
        }
        return flushed;
    }

    static constexpr size_t max_peek_size() { return 1; }

    /**
     * Arduino WiFiClient exposes only one-byte lookahead. Multi-byte protocol
     * headers must use read_exact() after input_size() reports enough bytes, or
     * wrap this connection in a buffering protocol reader.
     */
    bool peek(uint8_t* dst, size_t len) override {
        if (!active_ || !dst || len == 0 || client_.available() < static_cast<int>(len)) {
            return false;
        }
        if (len != max_peek_size()) {
            return false;
        }
        const int c = client_.peek();
        if (c < 0) {
            return false;
        }
        dst[0] = static_cast<uint8_t>(c);
        return true;
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
        while (client_.available() > 0) {
            const int c = client_.read();
            if (c < 0) {
                break;
            }
            if (c == '\n') {
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
                line_[line_size_++] = static_cast<char>(c);
            } else {
                line_size_ = 0;
            }
        }
        return false;
    }

private:
    static constexpr size_t tx_capacity = 1024;
    static constexpr size_t tx_flush_chunk = 64;

    size_t tx_tail() const {
        return (tx_head_ + tx_size_) % tx_capacity;
    }

    size_t tx_free() const {
        return tx_capacity - tx_size_;
    }

    size_t contiguous_output_size() const {
        if (tx_size_ == 0) {
            return 0;
        }
        const size_t until_end = tx_capacity - tx_head_;
        return tx_size_ < until_end ? tx_size_ : until_end;
    }

    size_t push_output(const uint8_t* src, size_t len) {
        size_t pushed = 0;
        while (pushed < len && tx_free() > 0) {
            const size_t tail = tx_tail();
            size_t contiguous_free = tx_capacity - tail;
            if (contiguous_free > tx_free()) {
                contiguous_free = tx_free();
            }
            size_t n = len - pushed;
            if (n > contiguous_free) {
                n = contiguous_free;
            }
            memcpy(tx_ + tail, src + pushed, n);
            tx_size_ += n;
            pushed += n;
        }
        return pushed;
    }

    void pop_output(size_t len) {
        if (len > tx_size_) {
            len = tx_size_;
        }
        tx_head_ = (tx_head_ + len) % tx_capacity;
        tx_size_ -= len;
        if (tx_size_ == 0) {
            tx_head_ = 0;
        }
    }

    void update_peer() {
        peer_ = TcpPeerInfo{};
        IPAddress ip = client_.remoteIP();
        snprintf(peer_.host, sizeof(peer_.host), "%u.%u.%u.%u",
                 static_cast<unsigned>(ip[0]), static_cast<unsigned>(ip[1]),
                 static_cast<unsigned>(ip[2]), static_cast<unsigned>(ip[3]));
    }

    mutable WiFiClient client_;
    bool active_ = false;
    TcpPeerInfo peer_;
    char line_[256]{};
    size_t line_size_ = 0;
    uint8_t tx_[tx_capacity]{};
    size_t tx_head_ = 0;
    size_t tx_size_ = 0;
};

class ArduinoWiFiTcpServer final : public IRuntimeTask {
public:
    explicit ArduinoWiFiTcpServer(IScheduler& loop, size_t max_connections = 4)
        : loop_(loop), max_connections_(max_connections > max_slots ? max_slots : max_connections) {
        registered_ = loop_.add_periodic(*this, 1000);
    }

    ~ArduinoWiFiTcpServer() override {
        if (registered_) {
            loop_.remove(*this);
            registered_ = false;
        }
        close();
    }

    bool listen(const TcpListenOptions& options, ITcpServerHandler& handler) {
        close();
        if (options.port == 0) {
            return false;
        }
        server_ = new WiFiServer(options.port);
        if (!server_) {
            return false;
        }
        server_->begin();
        handler_ = &handler;
        bound_port_ = options.port;
        active_ = true;
        return true;
    }

    void close() {
        for (size_t i = 0; i < max_slots; ++i) {
            if (connections_[i].valid()) {
                connections_[i].close();
            }
        }
        if (server_) {
            server_->stop();
            delete server_;
            server_ = nullptr;
        }
        active_ = false;
        handler_ = nullptr;
        bound_port_ = 0;
    }

    bool valid() const { return active_ && server_ != nullptr; }
    uint16_t port() const { return bound_port_; }

    size_t connection_count() const {
        size_t count = 0;
        for (size_t i = 0; i < max_slots; ++i) {
            if (connections_[i].valid()) {
                ++count;
            }
        }
        return count;
    }

    void poll(uint64_t) override {
        if (!valid() || !handler_) {
            return;
        }
        accept_ready_clients();
        poll_connections();
    }

private:
    void accept_ready_clients() {
        WiFiClient client = server_->available();
        if (!client) {
            return;
        }
        for (size_t i = 0; i < max_connections_; ++i) {
            if (!connections_[i].valid()) {
                connections_[i].attach(client);
                handler_->on_accept(connections_[i], connections_[i].peer());
                return;
            }
        }
        client.stop();
    }

    void poll_connections() {
        for (size_t i = 0; i < max_connections_; ++i) {
            ArduinoWiFiTcpConnection& connection = connections_[i];
            if (!connection.valid()) {
                continue;
            }
            if (connection.flush_output() > 0) {
                handler_->on_write_ready(connection);
            }
            if (connection.input_size() > 0) {
                handler_->on_data(connection);
            }
            if (connection.closed_by_peer()) {
                handler_->on_close(connection);
                connection.close();
            }
        }
    }

    static constexpr size_t max_slots = 4;
    IScheduler& loop_;
    size_t max_connections_ = max_slots;
    WiFiServer* server_ = nullptr;
    ITcpServerHandler* handler_ = nullptr;
    uint16_t bound_port_ = 0;
    bool active_ = false;
    bool registered_ = false;
    ArduinoWiFiTcpConnection connections_[max_slots];
};

class ArduinoWiFiTcpClient final : public IRuntimeTask {
public:
    explicit ArduinoWiFiTcpClient(IScheduler& loop) : loop_(loop) {
        registered_ = loop_.add_periodic(*this, 1000);
    }

    ~ArduinoWiFiTcpClient() override {
        if (registered_) {
            loop_.remove(*this);
            registered_ = false;
        }
        close();
    }

    bool connect(const TcpConnectOptions& options, ITcpClientHandler& handler) {
        close();
        if (!options.host || options.port == 0) {
            handler.on_error(-1);
            return false;
        }
        WiFiClient client;
        if (!client.connect(options.host, options.port)) {
            handler.on_error(-1);
            return false;
        }
        connection_.attach(client);
        handler_ = &handler;
        handler_->on_connect(connection_, connection_.peer());
        return true;
    }

    void close() {
        connection_.close();
        handler_ = nullptr;
    }

    bool valid() const { return connection_.valid(); }
    ITcpConnection& connection() { return connection_; }

    void poll(uint64_t) override {
        if (!handler_ || !connection_.valid()) {
            return;
        }
        if (connection_.flush_output() > 0) {
            handler_->on_write_ready(connection_);
        }
        if (connection_.input_size() > 0) {
            handler_->on_data(connection_);
        }
        if (connection_.closed_by_peer()) {
            handler_->on_close(connection_);
            connection_.close();
            handler_ = nullptr;
        }
    }

private:
    IScheduler& loop_;
    ITcpClientHandler* handler_ = nullptr;
    bool registered_ = false;
    ArduinoWiFiTcpConnection connection_;
};

} // namespace async_event_loop
#endif
