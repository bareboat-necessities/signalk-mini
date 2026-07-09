#pragma once

#include <stddef.h>
#include <stdint.h>
#include <async_event_loop.hpp>

namespace signalk_mini {

struct ConnectionFlags {
    bool allow_rx = true;
    bool allow_tx = true;
};

template<size_t MaxConnections>
class ConnectionRegistry {
public:
    bool add(async_event_loop::ITcpConnection& connection, ConnectionFlags flags) {
        const int existing = index_of(connection);
        if (existing >= 0) {
            entries_[existing].flags = flags;
            return true;
        }
        for (size_t i = 0; i < MaxConnections; ++i) {
            if (!entries_[i].connection) {
                entries_[i].connection = &connection;
                entries_[i].flags = flags;
                ++count_;
                return true;
            }
        }
        return false;
    }

    bool remove(async_event_loop::ITcpConnection& connection) {
        const int index = index_of(connection);
        if (index < 0) return false;
        entries_[index] = Entry{};
        if (count_ > 0) --count_;
        return true;
    }

    bool allow_rx(async_event_loop::ITcpConnection& connection) const {
        const int index = index_of(connection);
        return index >= 0 && entries_[index].flags.allow_rx;
    }

    bool allow_tx(async_event_loop::ITcpConnection& connection) const {
        const int index = index_of(connection);
        return index >= 0 && entries_[index].flags.allow_tx;
    }

    bool set_allow_tx(async_event_loop::ITcpConnection& connection, bool allow_tx) {
        const int index = index_of(connection);
        if (index < 0) return false;
        entries_[index].flags.allow_tx = allow_tx;
        return true;
    }

    size_t size() const { return count_; }

    template<typename Callback>
    void for_each_tx(Callback callback) {
        for (size_t i = 0; i < MaxConnections; ++i) {
            async_event_loop::ITcpConnection* c = entries_[i].connection;
            if (c && c->valid() && entries_[i].flags.allow_tx) {
                callback(*c);
            }
        }
    }

    void write_signal_k_delta(const char* json, size_t len) {
        if (!json || len == 0) return;
        for_each_tx([&](async_event_loop::ITcpConnection& connection) {
            connection.write(reinterpret_cast<const uint8_t*>(json), len);
        });
    }

private:
    struct Entry {
        async_event_loop::ITcpConnection* connection = nullptr;
        ConnectionFlags flags{};
    };

    int index_of(async_event_loop::ITcpConnection& connection) const {
        for (size_t i = 0; i < MaxConnections; ++i) {
            if (entries_[i].connection == &connection) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    Entry entries_[MaxConnections]{};
    size_t count_ = 0;
};

} // namespace signalk_mini
