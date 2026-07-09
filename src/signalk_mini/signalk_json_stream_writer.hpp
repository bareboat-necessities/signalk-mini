#pragma once

#include <stddef.h>
#include <stdio.h>
#include <string.h>

namespace signalk_mini {

class SignalKJsonStreamWriter {
public:
    SignalKJsonStreamWriter(char* dst, size_t capacity) : dst_(dst), capacity_(capacity) {
        if (dst_ && capacity_ > 0) dst_[0] = '\0';
        else ok_ = false;
    }

    bool ok() const { return ok_; }
    size_t size() const { return pos_; }

    bool append_raw(const char* text) {
        if (!text) return append_raw("");
        return append_bytes(text, strlen(text));
    }

    bool append_char(char c) {
        return append_bytes(&c, 1);
    }

    bool append_quoted(const char* text) {
        if (!append_char('"')) return false;
        if (text) {
            for (const char* p = text; *p; ++p) {
                const char c = *p;
                switch (c) {
                case '"': if (!append_raw("\\\"")) return false; break;
                case '\\': if (!append_raw("\\\\")) return false; break;
                case '\b': if (!append_raw("\\b")) return false; break;
                case '\f': if (!append_raw("\\f")) return false; break;
                case '\n': if (!append_raw("\\n")) return false; break;
                case '\r': if (!append_raw("\\r")) return false; break;
                case '\t': if (!append_raw("\\t")) return false; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        char tmp[7];
                        const int n = snprintf(tmp, sizeof(tmp), "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                        if (n != 6 || !append_bytes(tmp, 6)) return false;
                    } else if (!append_char(c)) return false;
                    break;
                }
            }
        }
        return append_char('"');
    }

    bool append_bool(bool value) {
        return append_raw(value ? "true" : "false");
    }

    template<typename Real>
    bool append_real(Real value) {
        char tmp[32];
        const int precision = sizeof(Real) <= sizeof(float) ? 9 : 17;
        const int n = snprintf(tmp, sizeof(tmp), "%.*g", precision, static_cast<double>(value));
        if (n <= 0 || static_cast<size_t>(n) >= sizeof(tmp)) return fail();
        return append_bytes(tmp, static_cast<size_t>(n));
    }

    bool finish_crlf() {
        return append_raw("\r\n");
    }

private:
    bool append_bytes(const char* data, size_t len) {
        if (!ok_ || !dst_ || capacity_ == 0 || !data) return false;
        if (pos_ + len >= capacity_) return fail();
        memcpy(dst_ + pos_, data, len);
        pos_ += len;
        dst_[pos_] = '\0';
        return true;
    }

    bool fail() {
        ok_ = false;
        if (dst_ && capacity_ > 0) dst_[0] = '\0';
        pos_ = 0;
        return false;
    }

    char* dst_ = nullptr;
    size_t capacity_ = 0;
    size_t pos_ = 0;
    bool ok_ = true;
};

template<typename Real>
inline bool signalk_json_write_scalar_value(SignalKJsonStreamWriter& out, SignalKMappedValueKind kind, Real number, bool boolean, const char* text) {
    if (kind == SignalKMappedValueKind::Number) return out.append_real(number);
    if (kind == SignalKMappedValueKind::Bool) return out.append_bool(boolean);
    if (kind == SignalKMappedValueKind::Text) return out.append_quoted(text ? text : "");
    return false;
}

template<typename Real>
inline int signalk_write_scalar_delta(char* dst,
                                      size_t dst_size,
                                      const char* source_label,
                                      const char* path,
                                      SignalKMappedValueKind kind,
                                      Real number,
                                      bool boolean,
                                      const char* text) {
    if (!dst || dst_size == 0 || !path) return 0;
    SignalKJsonStreamWriter out(dst, dst_size);
    if (!out.append_raw("{\"updates\":[{\"source\":{\"label\":")) return 0;
    if (!out.append_quoted(source_label ? source_label : "signalk-mini")) return 0;
    if (!out.append_raw("},\"values\":[{\"path\":")) return 0;
    if (!out.append_quoted(path)) return 0;
    if (!out.append_raw(",\"value\":")) return 0;
    if (!signalk_json_write_scalar_value(out, kind, number, boolean, text)) return 0;
    if (!out.append_raw("}]}]}")) return 0;
    if (!out.finish_crlf()) return 0;
    return out.ok() ? static_cast<int>(out.size()) : 0;
}

} // namespace signalk_mini
