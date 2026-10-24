#pragma once

#include <stddef.h>
#include <string.h>

namespace signalk_mini {

// signalk-mini streams everything or nothing; it does not implement per-path
// Signal K subscriptions. A client message either starts the full stream,
// stops it, or is ignored.
enum class SignalKStreamCommand : unsigned char { None, SubscribeAll, Unsubscribe };

inline SignalKStreamCommand signalk_stream_command(const char* source, size_t len) {
    if (!source) return SignalKStreamCommand::None;
    const char* begin = source;
    const char* end = source + strnlen(source, len);
    auto is_space = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
    while (begin < end && is_space(*begin)) ++begin;
    while (end > begin && is_space(end[-1])) --end;
    const size_t n = static_cast<size_t>(end - begin);
    auto equals = [&](const char* word) { return strlen(word) == n && strncmp(begin, word, n) == 0; };
    auto starts_with = [&](const char* word) { const size_t w = strlen(word); return w <= n && strncmp(begin, word, w) == 0; };
    auto contains = [&](const char* needle) {
        const size_t w = strlen(needle);
        for (const char* p = begin; p + w <= end; ++p) {
            if (strncmp(p, needle, w) == 0) return true;
        }
        return false;
    };

    if (n == 0) return SignalKStreamCommand::None;
    if (begin[0] == '{') {
        // Signal K JSON: {"context":"*","unsubscribe":[...]} or {"context":...,"subscribe":[...]}.
        if (contains("\"unsubscribe\"")) return SignalKStreamCommand::Unsubscribe;
        if (contains("\"subscribe\"")) return SignalKStreamCommand::SubscribeAll;
        return SignalKStreamCommand::None;
    }
    if (starts_with("unsubscribe") || starts_with("UNSUBSCRIBE") || starts_with("Unsubscribe")) {
        return SignalKStreamCommand::Unsubscribe;
    }
    if (equals("*") || starts_with("subscribe") || starts_with("SUBSCRIBE") || starts_with("Subscribe")) {
        return SignalKStreamCommand::SubscribeAll;
    }
    return SignalKStreamCommand::None;
}

} // namespace signalk_mini
