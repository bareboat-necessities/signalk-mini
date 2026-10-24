#pragma once

#include <stddef.h>
#include <stdint.h>
#include <async_event_loop.hpp>

namespace signalk_mini {

struct ConnectionFlags {
    bool allow_rx = true;
    bool allow_tx = true;
};

// Delivery state for a Signal K stream client (unused by connector registries).
// A client that could not take a whole delta is caught up from the typed model
// with a resumable snapshot instead of queueing deltas for it.
struct StreamDeliveryState {
    uint16_t resync_position = 0;
    uint16_t resync_value = 0;
    bool resync = false;
    bool resync_again = false;
    bool include_other_contexts = true;
    uint32_t last_output_bytes = 0;
    // Bytes queued for this client since last_output_bytes was sampled.
    uint32_t queued_since_sample = 0;
    uint32_t last_drain_ms = 0;
};

// True when some queued output was sent between two samples: less is queued
// now than at the last sample plus what was added since. Comparing queue sizes
// alone would miss a queue that drained completely and was refilled to the
// same size before the next sample.
inline bool stream_output_progressed(uint32_t queued_now, uint32_t queued_at_sample, uint32_t queued_since_sample) {
    return queued_now == 0 ||
           static_cast<uint64_t>(queued_now) < static_cast<uint64_t>(queued_at_sample) + queued_since_sample;
}

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
                entries_[i] = Entry{};
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

    StreamDeliveryState* stream_state(async_event_loop::ITcpConnection& connection) {
        const int index = index_of(connection);
        return index >= 0 ? &entries_[index].stream : nullptr;
    }

    template<typename Callback>
    void for_each_tx_state(Callback callback) {
        for (size_t i = 0; i < MaxConnections; ++i) {
            async_event_loop::ITcpConnection* c = entries_[i].connection;
            if (c && c->valid() && entries_[i].flags.allow_tx) callback(*c, entries_[i].stream);
        }
    }

    template<typename Callback>
    void for_each_state(Callback callback) {
        for (size_t i = 0; i < MaxConnections; ++i) {
            async_event_loop::ITcpConnection* c = entries_[i].connection;
            if (c && c->valid()) callback(*c, entries_[i].stream);
        }
    }

private:
    struct Entry {
        async_event_loop::ITcpConnection* connection = nullptr;
        ConnectionFlags flags{};
        StreamDeliveryState stream{};
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
