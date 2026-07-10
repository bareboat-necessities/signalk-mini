#pragma once

#include <ArduinoJson.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace signalk_mini {

enum class SubscriptionPolicy : uint8_t { Instant, Ideal, Fixed };
enum class SubscriptionFormat : uint8_t { Delta, Full };

enum class SubscriptionApplyResult : uint8_t {
    Applied,
    FallbackToAll,
};

struct SubscriptionTiming {
    uint32_t period_ms = 1000;
    uint32_t min_period_ms = 0;
    SubscriptionPolicy policy = SubscriptionPolicy::Ideal;
    SubscriptionFormat format = SubscriptionFormat::Delta;
};

template<size_t MaxPathLength>
struct SignalKSubscription {
    char path[MaxPathLength]{};
    SubscriptionTiming timing{};
};

inline bool signalk_path_matches(const char* pattern, const char* path) {
    if (!pattern || !path) return false;
    if (strcmp(pattern, "*") == 0) return true;
    const char* p = pattern;
    const char* v = path;
    while (*p || *v) {
        const char* p_end = strchr(p, '.');
        const char* v_end = strchr(v, '.');
        const size_t p_len = p_end ? static_cast<size_t>(p_end - p) : strlen(p);
        const size_t v_len = v_end ? static_cast<size_t>(v_end - v) : strlen(v);
        if (p_len == 1 && p[0] == '*') {
            if (!p_end) return true;
        } else if (p_len != v_len || strncmp(p, v, p_len) != 0) {
            return false;
        }
        if (!p_end || !v_end) return !p_end && !v_end;
        p = p_end + 1;
        v = v_end + 1;
    }
    return true;
}

template<size_t MaxSubscriptions, size_t MaxPathLength = 96>
class SignalKSubscriptionSet {
public:
    static_assert(MaxSubscriptions > 0, "MaxSubscriptions must be greater than zero");
    static_assert(MaxPathLength > 2, "MaxPathLength is too small");

    SignalKSubscriptionSet() { subscribe_all(); }

    void subscribe_all() {
        count_ = 1;
        memset(entries_, 0, sizeof(entries_));
        entries_[0].path[0] = '*';
        entries_[0].path[1] = '\0';
        entries_[0].timing = SubscriptionTiming{};
    }

    void unsubscribe_all() {
        count_ = 0;
        memset(entries_, 0, sizeof(entries_));
    }

    size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }
    bool all() const { return count_ == 1 && strcmp(entries_[0].path, "*") == 0; }
    const SignalKSubscription<MaxPathLength>& operator[](size_t index) const { return entries_[index]; }

    bool matches(const char* path) const {
        for (size_t i = 0; i < count_; ++i) {
            if (signalk_path_matches(entries_[i].path, path)) return true;
        }
        return false;
    }

    SubscriptionApplyResult apply_message(const char* json, size_t len) {
        if (!json || len == 0) return fallback_all();
        JsonDocument document;
        const DeserializationError error = deserializeJson(document, json, len);
        if (error) return fallback_all();

        JsonObject root = document.as<JsonObject>();
        if (root.isNull()) return fallback_all();
        const char* context = root["context"].as<const char*>();
        if (!context) context = "vessels.self";
        if (!supported_context(context)) return fallback_all();

        JsonArray subscribe = root["subscribe"].as<JsonArray>();
        JsonArray unsubscribe = root["unsubscribe"].as<JsonArray>();
        const bool has_subscribe = !subscribe.isNull();
        const bool has_unsubscribe = !unsubscribe.isNull();
        if (has_subscribe == has_unsubscribe) return fallback_all();

        SignalKSubscriptionSet candidate = *this;
        const bool ok = has_subscribe
            ? candidate.apply_subscribe_array(subscribe)
            : candidate.apply_unsubscribe_array(unsubscribe);
        if (!ok) return fallback_all();
        *this = candidate;
        return SubscriptionApplyResult::Applied;
    }

private:
    static bool supported_context(const char* context) {
        return context && (strcmp(context, "vessels.self") == 0 || strcmp(context, "*") == 0);
    }

    SubscriptionApplyResult fallback_all() {
        subscribe_all();
        return SubscriptionApplyResult::FallbackToAll;
    }

    static bool parse_policy(const char* value, SubscriptionPolicy& out) {
        if (!value || strcmp(value, "ideal") == 0) { out = SubscriptionPolicy::Ideal; return true; }
        if (strcmp(value, "instant") == 0) { out = SubscriptionPolicy::Instant; return true; }
        if (strcmp(value, "fixed") == 0) { out = SubscriptionPolicy::Fixed; return true; }
        return false;
    }

    static bool parse_format(const char* value, SubscriptionFormat& out) {
        if (!value || strcmp(value, "delta") == 0) { out = SubscriptionFormat::Delta; return true; }
        if (strcmp(value, "full") == 0) { out = SubscriptionFormat::Full; return true; }
        return false;
    }

    static bool copy_path(const char* source, char* destination) {
        if (!source || !source[0]) return false;
        const size_t len = strlen(source);
        if (len >= MaxPathLength) return false;
        memcpy(destination, source, len + 1);
        return true;
    }

    static bool parse_entry(JsonObject object, SignalKSubscription<MaxPathLength>& entry) {
        if (object.isNull()) return false;
        const char* path = object["path"].as<const char*>();
        if (!copy_path(path, entry.path)) return false;
        const uint64_t period = object["period"] | 1000ULL;
        const uint64_t min_period = object["minPeriod"] | 0ULL;
        if (period > UINT32_MAX || min_period > UINT32_MAX) return false;
        entry.timing.period_ms = static_cast<uint32_t>(period);
        entry.timing.min_period_ms = static_cast<uint32_t>(min_period);
        if (!parse_policy(object["policy"].as<const char*>(), entry.timing.policy)) return false;
        if (!parse_format(object["format"].as<const char*>(), entry.timing.format)) return false;
        return true;
    }

    bool apply_subscribe_array(JsonArray array) {
        if (array.isNull() || array.size() == 0) return false;
        for (JsonVariant item : array) {
            JsonObject object = item.as<JsonObject>();
            if (object.isNull()) return false;
            SignalKSubscription<MaxPathLength> parsed{};
            if (!parse_entry(object, parsed)) return false;
            const int existing = find_exact(parsed.path);
            if (existing >= 0) {
                entries_[existing] = parsed;
                continue;
            }
            if (count_ >= MaxSubscriptions) return false;
            entries_[count_++] = parsed;
        }
        return true;
    }

    bool apply_unsubscribe_array(JsonArray array) {
        if (array.isNull() || array.size() == 0) return false;
        for (JsonVariant item : array) {
            JsonObject object = item.as<JsonObject>();
            if (object.isNull()) return false;
            const char* path = object["path"].as<const char*>();
            if (!path || !path[0]) return false;
            if (strcmp(path, "*") == 0) {
                unsubscribe_all();
                continue;
            }
            remove_exact(path);
        }
        return true;
    }

    int find_exact(const char* path) const {
        for (size_t i = 0; i < count_; ++i) {
            if (strcmp(entries_[i].path, path) == 0) return static_cast<int>(i);
        }
        return -1;
    }

    void remove_exact(const char* path) {
        const int index = find_exact(path);
        if (index < 0) return;
        for (size_t i = static_cast<size_t>(index) + 1; i < count_; ++i) entries_[i - 1] = entries_[i];
        --count_;
        entries_[count_] = SignalKSubscription<MaxPathLength>{};
    }

    SignalKSubscription<MaxPathLength> entries_[MaxSubscriptions]{};
    size_t count_ = 0;
};

} // namespace signalk_mini
