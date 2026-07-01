#pragma once

#include <new>
#include <stddef.h>
#include <stdint.h>

#include "event_loop.hpp"

namespace async_event_loop {

/** View of one complete delimiter-based text line.
 *
 * The memory is owned by the reader and remains valid only until the reader
 * consumes more input. Copy the data inside the callback if it must outlive the
 * callback.
 */
struct LineView {
    constexpr LineView() = default;
    constexpr LineView(const char* data_value, size_t size_value)
        : data(data_value), size(size_value) {}

    const char* data = nullptr;
    size_t size = 0;
};

/** View of one complete JSON text message.
 *
 * The reader emits the original message bytes without a terminating nul. The
 * message is framed as JSON text but not parsed into a DOM. The memory is owned
 * by the reader and remains valid only until the reader consumes more input.
 */
struct JsonView {
    constexpr JsonView() = default;
    constexpr JsonView(const char* data_value, size_t size_value)
        : data(data_value), size(size_value) {}

    const char* data = nullptr;
    size_t size = 0;
};

/** View of one complete binary frame. */
struct FrameView {
    constexpr FrameView() = default;
    constexpr FrameView(const uint8_t* data_value, size_t size_value)
        : data(data_value), size(size_value) {}

    const uint8_t* data = nullptr;
    size_t size = 0;
};

/** Options for LineProtocolReader. */
struct LineProtocolOptions {
    constexpr LineProtocolOptions() = default;
    constexpr LineProtocolOptions(char delimiter_value, bool strip_cr_value = true, bool drop_overflow_value = true)
        : delimiter(delimiter_value), strip_cr(strip_cr_value), drop_overflow(drop_overflow_value) {}

    /** Byte that terminates a line. Usually '\n'. */
    char delimiter = '\n';

    /** Strip a trailing '\r' before emitting a line. Useful for CRLF streams. */
    bool strip_cr = true;

    /**
     * Overflow policy for a line longer than the fixed buffer.
     *
     * When true, an overlong line resets the current buffer and discards the
     * byte that caused the overflow. Bytes after that overflow byte begin a new
     * candidate line. When false, overflow bytes are discarded until a delimiter
     * is observed and the existing truncated buffer is emitted/reset normally.
     */
    bool drop_overflow = true;
};

/** Options for JsonProtocolReader. */
struct JsonProtocolOptions {
    constexpr JsonProtocolOptions() = default;

    /** Ignore spaces, tabs, carriage returns, and newlines before each JSON value. */
    bool skip_leading_whitespace = true;
};

/** Options for FixedFrameProtocolReader. */
struct FixedFrameProtocolOptions {
    constexpr FixedFrameProtocolOptions() = default;
    explicit constexpr FixedFrameProtocolOptions(size_t frame_size_value)
        : frame_size(frame_size_value) {}

    /** Number of bytes in each fixed-size frame. Must be nonzero and <= BufferSize. */
    size_t frame_size = 0;
};

/** Return payload length from a complete fixed-size header. */
using PayloadSizeFromHeaderFn = size_t (*)(const uint8_t* header, size_t header_size);

/** Options for HeaderPayloadProtocolReader. */
struct HeaderPayloadProtocolOptions {
    constexpr HeaderPayloadProtocolOptions() = default;
    constexpr HeaderPayloadProtocolOptions(size_t header_size_value,
                                           size_t max_payload_size_value,
                                           PayloadSizeFromHeaderFn payload_size_from_header_value)
        : header_size(header_size_value),
          max_payload_size(max_payload_size_value),
          payload_size_from_header(payload_size_from_header_value) {}

    /** Number of header bytes required before payload size can be computed. */
    size_t header_size = 0;

    /** Maximum accepted payload size. Larger payloads are protocol errors. */
    size_t max_payload_size = 0;

    /** User function that extracts payload length from the header bytes. */
    PayloadSizeFromHeaderFn payload_size_from_header = nullptr;
};

/** Counters shared by all protocol readers. */
struct ProtocolReaderStats {
    uint32_t messages = 0;
    uint32_t bytes = 0;
    uint32_t overflows = 0;
    uint32_t protocol_errors = 0;
};

/**
 * Small fixed-storage callback holder used by protocol readers.
 *
 * This avoids std::function and heap allocation on Arduino-sized targets. The
 * callable is constructed in-place and destroyed when replaced or when the
 * reader is destroyed.
 */
template<size_t StorageSize, typename ViewType>
class ProtocolCallbackStorage {
public:
    ProtocolCallbackStorage() = default;
    ProtocolCallbackStorage(const ProtocolCallbackStorage&) = delete;
    ProtocolCallbackStorage& operator=(const ProtocolCallbackStorage&) = delete;
    ~ProtocolCallbackStorage() { clear(); }

    template<typename Callable>
    bool set(Callable callable) {
        static_assert(sizeof(Callable) <= StorageSize, "protocol callback is too large for storage");
        static_assert(alignof(Callable) <= alignof(StorageAlignment), "protocol callback alignment is too large for storage");
        clear();
        new (storage_) Callable(static_cast<Callable&&>(callable));
        invoke_ = [](void* storage, const ViewType& view) {
            (*static_cast<Callable*>(storage))(view);
        };
        destroy_ = [](void* storage) {
            static_cast<Callable*>(storage)->~Callable();
        };
        active_ = true;
        return true;
    }

    void invoke(const ViewType& view) {
        if (active_ && invoke_) {
            invoke_(storage_, view);
        }
    }

    bool active() const { return active_; }

    void clear() {
        if (active_ && destroy_) {
            destroy_(storage_);
        }
        active_ = false;
        invoke_ = nullptr;
        destroy_ = nullptr;
    }

private:
    union StorageAlignment {
        void* pointer;
        long long integer;
        double floating;
        long double long_floating;
    };

    alignas(StorageAlignment) unsigned char storage_[StorageSize]{};
    void (*invoke_)(void*, const ViewType&) = nullptr;
    void (*destroy_)(void*) = nullptr;
    bool active_ = false;
};

/**
 * Incremental delimiter-based text protocol reader.
 *
 * Call poll() from an on_bytes_ready() callback or from a periodic task. The
 * reader drains currently available bytes from the stream and invokes the
 * callback once for each complete line.
 */
template<size_t BufferSize = 256, size_t CallbackStorageSize = 64>
class LineProtocolReader final : public IRuntimeTask {
public:
    explicit LineProtocolReader(IByteStream& stream,
                                LineProtocolOptions options = LineProtocolOptions{})
        : stream_(stream), options_(options) {}

    template<typename Callable>
    LineProtocolReader(IByteStream& stream,
                       LineProtocolOptions options,
                       Callable callable)
        : stream_(stream), options_(options) {
        on_data_ready(callable);
    }

    /** Install or replace the callback that receives LineView messages. */
    template<typename Callable>
    bool on_data_ready(Callable callable) {
        return callback_.set(callable);
    }

    IByteStream& stream() { return stream_; }
    const ProtocolReaderStats& stats() const { return stats_; }

    void poll(uint64_t) override {
        uint8_t byte = 0;
        while (stream_.readable()) {
            const int n = stream_.read(&byte, 1);
            if (n <= 0) {
                break;
            }
            ++stats_.bytes;
            consume(static_cast<char>(byte));
        }
    }

private:
    void consume(char c) {
        if (c == options_.delimiter) {
            emit_line();
            return;
        }

        if (size_ >= BufferSize) {
            ++stats_.overflows;
            if (options_.drop_overflow) {
                size_ = 0;
            }
            return;
        }

        buffer_[size_++] = c;
    }

    void emit_line() {
        size_t len = size_;
        if (options_.strip_cr && len > 0 && buffer_[len - 1] == '\r') {
            --len;
        }
        LineView view;
        view.data = buffer_;
        view.size = len;
        callback_.invoke(view);
        ++stats_.messages;
        size_ = 0;
    }

    IByteStream& stream_;
    LineProtocolOptions options_;
    char buffer_[BufferSize]{};
    size_t size_ = 0;
    ProtocolReaderStats stats_;
    ProtocolCallbackStorage<CallbackStorageSize, LineView> callback_;
};

/**
 * Reads adjacent JSON text messages from a byte stream.
 *
 * The reader frames complete JSON values and emits the original bytes through
 * JsonView. It deliberately does not build a JSON DOM or depend on a JSON
 * library. It is meant for transports where each message is a valid JSON value:
 * objects, arrays, strings, numbers, true, false, or null. Objects, arrays,
 * strings, and literals are self-delimiting. Top-level numbers are emitted when
 * a following non-number byte is observed, so numeric-only streams should use a
 * separator such as whitespace after each number.
 */
template<size_t BufferSize = 512, size_t CallbackStorageSize = 64>
class JsonProtocolReader final : public IRuntimeTask {
public:
    explicit JsonProtocolReader(IByteStream& stream,
                                JsonProtocolOptions options = JsonProtocolOptions{})
        : stream_(stream), options_(options) {}

    template<typename Callable>
    JsonProtocolReader(IByteStream& stream,
                       JsonProtocolOptions options,
                       Callable callable)
        : stream_(stream), options_(options) {
        on_data_ready(callable);
    }

    /** Install or replace the callback that receives JsonView messages. */
    template<typename Callable>
    bool on_data_ready(Callable callable) {
        return callback_.set(callable);
    }

    IByteStream& stream() { return stream_; }
    const ProtocolReaderStats& stats() const { return stats_; }

    void poll(uint64_t) override {
        uint8_t byte = 0;
        while (stream_.readable()) {
            const int n = stream_.read(&byte, 1);
            if (n <= 0) {
                break;
            }
            ++stats_.bytes;
            consume(static_cast<char>(byte));
        }
    }

private:
    enum class State {
        Idle,
        Container,
        StringValue,
        NumberValue,
        LiteralValue
    };

    static bool is_ws(char c) {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n';
    }

    static bool is_digit(char c) {
        return c >= '0' && c <= '9';
    }

    static bool is_nonzero_digit(char c) {
        return c >= '1' && c <= '9';
    }

    static bool is_number_byte(char c) {
        return is_digit(c) || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E';
    }

    static bool valid_number(const char* text, size_t len) {
        if (!text || len == 0) {
            return false;
        }
        size_t i = 0;
        if (text[i] == '-') {
            ++i;
            if (i == len) {
                return false;
            }
        }
        if (text[i] == '0') {
            ++i;
        } else if (is_nonzero_digit(text[i])) {
            while (i < len && is_digit(text[i])) {
                ++i;
            }
        } else {
            return false;
        }
        if (i < len && text[i] == '.') {
            ++i;
            if (i == len || !is_digit(text[i])) {
                return false;
            }
            while (i < len && is_digit(text[i])) {
                ++i;
            }
        }
        if (i < len && (text[i] == 'e' || text[i] == 'E')) {
            ++i;
            if (i < len && (text[i] == '+' || text[i] == '-')) {
                ++i;
            }
            if (i == len || !is_digit(text[i])) {
                return false;
            }
            while (i < len && is_digit(text[i])) {
                ++i;
            }
        }
        return i == len;
    }

    bool append(char c) {
        if (size_ >= BufferSize) {
            ++stats_.overflows;
            reset();
            return false;
        }
        buffer_[size_++] = c;
        return true;
    }

    bool push_container(char c) {
        if (depth_ >= BufferSize) {
            ++stats_.overflows;
            reset();
            return false;
        }
        stack_[depth_++] = c;
        return true;
    }

    bool pop_container(char c) {
        if (depth_ == 0) {
            ++stats_.protocol_errors;
            reset();
            return false;
        }
        const char open = stack_[depth_ - 1];
        if ((open == '{' && c != '}') || (open == '[' && c != ']')) {
            ++stats_.protocol_errors;
            reset();
            return false;
        }
        --depth_;
        return true;
    }

    void consume(char c) {
        switch (state_) {
        case State::Idle:
            consume_idle(c);
            break;
        case State::Container:
            consume_container(c);
            break;
        case State::StringValue:
            consume_string_value(c);
            break;
        case State::NumberValue:
            consume_number_value(c);
            break;
        case State::LiteralValue:
            consume_literal_value(c);
            break;
        }
    }

    void consume_idle(char c) {
        if (options_.skip_leading_whitespace && is_ws(c)) {
            return;
        }
        if (c == '{' || c == '[') {
            if (!append(c)) {
                return;
            }
            if (!push_container(c)) {
                return;
            }
            state_ = State::Container;
            in_string_ = false;
            escape_ = false;
            return;
        }
        if (c == '"') {
            if (!append(c)) {
                return;
            }
            state_ = State::StringValue;
            escape_ = false;
            return;
        }
        if (c == '-' || is_digit(c)) {
            if (!append(c)) {
                return;
            }
            state_ = State::NumberValue;
            return;
        }
        if (c == 't') {
            start_literal("true", c);
            return;
        }
        if (c == 'f') {
            start_literal("false", c);
            return;
        }
        if (c == 'n') {
            start_literal("null", c);
            return;
        }
        ++stats_.protocol_errors;
        reset();
    }

    void consume_container(char c) {
        if (!append(c)) {
            return;
        }
        if (in_string_) {
            if (escape_) {
                escape_ = false;
            } else if (c == '\\') {
                escape_ = true;
            } else if (c == '"') {
                in_string_ = false;
            }
            return;
        }
        if (c == '"') {
            in_string_ = true;
            return;
        }
        if (c == '{' || c == '[') {
            push_container(c);
            return;
        }
        if (c == '}' || c == ']') {
            if (pop_container(c) && depth_ == 0) {
                emit_json();
            }
        }
    }

    void consume_string_value(char c) {
        if (!append(c)) {
            return;
        }
        if (escape_) {
            escape_ = false;
            return;
        }
        if (c == '\\') {
            escape_ = true;
            return;
        }
        if (c == '"') {
            emit_json();
        }
    }

    void consume_number_value(char c) {
        if (is_number_byte(c)) {
            append(c);
            return;
        }
        if (valid_number(buffer_, size_)) {
            emit_json();
        } else {
            ++stats_.protocol_errors;
            reset();
        }
        consume(c);
    }

    void start_literal(const char* literal, char first) {
        literal_ = literal;
        literal_pos_ = 0;
        state_ = State::LiteralValue;
        consume_literal_value(first);
    }

    void consume_literal_value(char c) {
        if (!literal_ || literal_[literal_pos_] == '\0' || literal_[literal_pos_] != c) {
            ++stats_.protocol_errors;
            reset();
            return;
        }
        if (!append(c)) {
            return;
        }
        ++literal_pos_;
        if (literal_[literal_pos_] == '\0') {
            emit_json();
        }
    }

    void emit_json() {
        JsonView view;
        view.data = buffer_;
        view.size = size_;
        callback_.invoke(view);
        ++stats_.messages;
        reset();
    }

    void reset() {
        state_ = State::Idle;
        size_ = 0;
        depth_ = 0;
        in_string_ = false;
        escape_ = false;
        literal_ = nullptr;
        literal_pos_ = 0;
    }

    IByteStream& stream_;
    JsonProtocolOptions options_;
    char buffer_[BufferSize]{};
    char stack_[BufferSize]{};
    size_t size_ = 0;
    size_t depth_ = 0;
    bool in_string_ = false;
    bool escape_ = false;
    const char* literal_ = nullptr;
    size_t literal_pos_ = 0;
    State state_ = State::Idle;
    ProtocolReaderStats stats_;
    ProtocolCallbackStorage<CallbackStorageSize, JsonView> callback_;
};

/**
 * Incremental fixed-size binary frame reader.
 *
 * Every frame has exactly options.frame_size bytes. This is useful for simple
 * binary transports with fixed-width records and no delimiter or header.
 */
template<size_t BufferSize = 256, size_t CallbackStorageSize = 64>
class FixedFrameProtocolReader final : public IRuntimeTask {
public:
    explicit FixedFrameProtocolReader(IByteStream& stream,
                                      FixedFrameProtocolOptions options)
        : stream_(stream), options_(options) {}

    template<typename Callable>
    FixedFrameProtocolReader(IByteStream& stream,
                             FixedFrameProtocolOptions options,
                             Callable callable)
        : stream_(stream), options_(options) {
        on_data_ready(callable);
    }

    /** Install or replace the callback that receives FrameView messages. */
    template<typename Callable>
    bool on_data_ready(Callable callable) {
        return callback_.set(callable);
    }

    IByteStream& stream() { return stream_; }
    const ProtocolReaderStats& stats() const { return stats_; }

    void poll(uint64_t) override {
        if (options_.frame_size == 0 || options_.frame_size > BufferSize) {
            ++stats_.protocol_errors;
            return;
        }

        uint8_t byte = 0;
        while (stream_.readable()) {
            const int n = stream_.read(&byte, 1);
            if (n <= 0) {
                break;
            }
            ++stats_.bytes;
            buffer_[size_++] = byte;
            if (size_ == options_.frame_size) {
                FrameView view;
                view.data = buffer_;
                view.size = size_;
                callback_.invoke(view);
                ++stats_.messages;
                size_ = 0;
            }
        }
    }

private:
    IByteStream& stream_;
    FixedFrameProtocolOptions options_;
    uint8_t buffer_[BufferSize]{};
    size_t size_ = 0;
    ProtocolReaderStats stats_;
    ProtocolCallbackStorage<CallbackStorageSize, FrameView> callback_;
};

/**
 * Incremental header+payload binary frame reader.
 *
 * The reader first collects a fixed-size header, asks payload_size_from_header()
 * for the payload length, then emits the complete header+payload frame. Any
 * extra bytes already buffered after a frame are retained for the next frame.
 */
template<size_t BufferSize = 512, size_t CallbackStorageSize = 64>
class HeaderPayloadProtocolReader final : public IRuntimeTask {
public:
    explicit HeaderPayloadProtocolReader(IByteStream& stream,
                                         HeaderPayloadProtocolOptions options)
        : stream_(stream), options_(options) {}

    template<typename Callable>
    HeaderPayloadProtocolReader(IByteStream& stream,
                                HeaderPayloadProtocolOptions options,
                                Callable callable)
        : stream_(stream), options_(options) {
        on_data_ready(callable);
    }

    /** Install or replace the callback that receives FrameView messages. */
    template<typename Callable>
    bool on_data_ready(Callable callable) {
        return callback_.set(callable);
    }

    IByteStream& stream() { return stream_; }
    const ProtocolReaderStats& stats() const { return stats_; }

    void poll(uint64_t) override {
        if (!valid_options()) {
            ++stats_.protocol_errors;
            return;
        }

        uint8_t byte = 0;
        while (stream_.readable()) {
            const int n = stream_.read(&byte, 1);
            if (n <= 0) {
                break;
            }
            ++stats_.bytes;
            if (!append(byte)) {
                continue;
            }
            try_emit();
        }
    }

private:
    bool valid_options() const {
        return options_.header_size != 0 &&
               options_.payload_size_from_header != nullptr &&
               options_.header_size <= BufferSize &&
               options_.max_payload_size <= BufferSize &&
               options_.header_size + options_.max_payload_size <= BufferSize;
    }

    bool append(uint8_t byte) {
        if (size_ >= BufferSize) {
            ++stats_.overflows;
            reset();
            return false;
        }
        buffer_[size_++] = byte;
        return true;
    }

    void try_emit() {
        if (size_ < options_.header_size) {
            return;
        }

        if (!have_expected_size_) {
            const size_t payload = options_.payload_size_from_header(buffer_, options_.header_size);
            if (payload > options_.max_payload_size || options_.header_size + payload > BufferSize) {
                ++stats_.protocol_errors;
                reset();
                return;
            }
            expected_size_ = options_.header_size + payload;
            have_expected_size_ = true;
        }

        if (size_ >= expected_size_) {
            FrameView view;
            view.data = buffer_;
            view.size = expected_size_;
            callback_.invoke(view);
            ++stats_.messages;

            const size_t remaining = size_ - expected_size_;
            for (size_t i = 0; i < remaining; ++i) {
                buffer_[i] = buffer_[expected_size_ + i];
            }
            size_ = remaining;
            expected_size_ = 0;
            have_expected_size_ = false;
            if (size_ >= options_.header_size) {
                try_emit();
            }
        }
    }

    void reset() {
        size_ = 0;
        expected_size_ = 0;
        have_expected_size_ = false;
    }

    IByteStream& stream_;
    HeaderPayloadProtocolOptions options_;
    uint8_t buffer_[BufferSize]{};
    size_t size_ = 0;
    size_t expected_size_ = 0;
    bool have_expected_size_ = false;
    ProtocolReaderStats stats_;
    ProtocolCallbackStorage<CallbackStorageSize, FrameView> callback_;
};

} // namespace async_event_loop
