#pragma once

// Included inside Nmea0183RxConnector private section.

static bool seatalk_hex_separator(char c) {
    return c == ' ' || c == ':' || c == '-' || c == '_' || c == '.' || c == ';' || c == '|';
}

static bool append_seatalk_hex_span(NmeaSpan span, uint8_t* bytes, size_t& count, size_t capacity) {
    int high_nibble = -1;

    for (uint8_t i = 0; i < span.length; ++i) {
        const char c = span[i];
        if (seatalk_hex_separator(c)) continue;

        if (c == 'x' || c == 'X') {
            if (high_nibble == 0) {
                high_nibble = -1;
                continue;
            }
            return false;
        }

        const uint8_t value = from_hex(c);
        if (value > 0x0f) return false;

        if (high_nibble < 0) {
            high_nibble = value;
            continue;
        }

        if (count >= capacity) return false;
        bytes[count++] = static_cast<uint8_t>((high_nibble << 4) | value);
        high_nibble = -1;
    }

    return high_nibble < 0;
}

static bool parse_seatalk_payload_text(const char* text, uint8_t* bytes, size_t& count, size_t capacity) {
    count = 0;
    if (!text) return false;

    const size_t length = strlen(text);
    if (length == 0 || length > 255u) return false;

    return append_seatalk_hex_span(NmeaSpan(text, length), bytes, count, capacity) && count > 0;
}

static bool parse_seatalk_payload_fields(const NmeaSentence& sentence, uint8_t* bytes, size_t& count, size_t capacity) {
    count = 0;
    if (sentence.field_count == 0) return false;

    for (uint8_t i = 0; i < sentence.field_count; ++i) {
        if (!append_seatalk_hex_span(sentence.field(i), bytes, count, capacity)) return false;
    }

    return count > 0;
}

template<typename Model>
bool apply_seatalk_carried(const NmeaSentence& sentence,
                           Model& model,
                           uint64_t now_us,
                           ship_data_model::SensorSource source) {
    uint8_t bytes[seatalk::SEATALK_FRAME_MAX_BYTES] = {0};
    size_t count = 0;

    if (sentence.fragment.is_fragmented) {
        if (!state_.seatalk_message.complete) return true;

        if (state_.seatalk_message.overflow) {
            last_error_ = "SeaTalk carried payload overflow";
            return false;
        }

        if (!parse_seatalk_payload_text(state_.seatalk_message.text, bytes, count, sizeof(bytes))) {
            last_error_ = "bad SeaTalk carried payload";
            return false;
        }
    } else if (!parse_seatalk_payload_fields(sentence, bytes, count, sizeof(bytes))) {
        last_error_ = "bad SeaTalk carried payload";
        return false;
    }

    if (!seatalk_receiver_.accept_datagram(bytes, count, model, now_us, source)) {
        last_error_ = "bad SeaTalk carried frame";
        return false;
    }

    return true;
}
