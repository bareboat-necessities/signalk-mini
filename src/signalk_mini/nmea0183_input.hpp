#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nmea0183_connector.hpp>
#include <seatalk.hpp>
#include <ship_data_model.hpp>
#include "model_store.hpp"
#include "types.hpp"

namespace signalk_mini {

template<typename Real>
class Nmea0183Input {
public:
    explicit Nmea0183Input(ModelStore<Real>& store) : store_(store) {}

    bool feed_line(const char* line, SourceId source_id, uint64_t now_us, bool validate_checksum = true) {
        last_error_ = "";
        const nmea0183_connector::NmeaTokenizeResult tokens = nmea0183_connector::tokenize_nmea_line(line);
        const nmea0183_connector::NmeaToken* token = tokens.first_sentence();
        if (!token) { last_error_ = "no NMEA sentence"; return false; }

        char sentence_text[nmea0183_connector::NMEA_MAX_SENTENCE_LEN];
        nmea0183_connector::nmea_copy_span(sentence_text, sizeof(sentence_text), token->text);

        nmea0183_connector::NmeaSentence sentence;
        if (!parser_.parse_line(sentence_text, sentence, validate_checksum)) { last_error_ = parser_.last_error(); return false; }
        if (sentence.family == nmea0183_connector::NmeaSentenceFamily::SeaTalk) return apply_seatalk_tunnel(sentence, source_id, now_us);

        const bool applied = rx_.apply_sentence(sentence, store_.model(), now_us, ship_data_model::SensorSource::serial);
        if (!applied) { last_error_ = rx_.last_error(); return false; }
        mark_changed_from_sentence(sentence, source_id, now_us);
        return true;
    }

    const char* last_error() const { return last_error_ && last_error_[0] ? last_error_ : rx_.last_error(); }
    const nmea0183_connector::NmeaMessageState& message_state() const { return rx_.message_state(); }
    const nmea0183_connector::NmeaDscMessageState& dsc_state() const { return rx_.dsc_state(); }
    const seatalk::SeaTalkReceiver<Real>& seatalk_receiver() const { return seatalk_receiver_; }

private:
    struct SeaTalkTunnelAssembly {
        char message_id[16] = {0};
        uint8_t total = 0;
        uint8_t next_number = 1;
        bool in_progress = false;
        bool complete = false;
        bool overflow = false;
        char text[96] = {0};
    };

    static bool hex_separator(char c) { return c == ' ' || c == ':' || c == '-' || c == '_' || c == '.' || c == ';' || c == '|'; }

    static bool append_hex_span(nmea0183_connector::NmeaSpan span, uint8_t* bytes, size_t& count, size_t capacity) {
        int high_nibble = -1;
        for (uint8_t i = 0; i < span.length; ++i) {
            const char c = span[i];
            if (hex_separator(c)) continue;
            if (c == 'x' || c == 'X') {
                if (high_nibble == 0) { high_nibble = -1; continue; }
                return false;
            }
            const uint8_t value = nmea0183_connector::from_hex(c);
            if (value > 0x0f) return false;
            if (high_nibble < 0) high_nibble = value;
            else {
                if (count >= capacity) return false;
                bytes[count++] = static_cast<uint8_t>((high_nibble << 4) | value);
                high_nibble = -1;
            }
        }
        return high_nibble < 0;
    }

    static bool parse_payload_text(const char* text, uint8_t* bytes, size_t& count, size_t capacity) {
        count = 0;
        if (!text) return false;
        const size_t n = strlen(text);
        if (n == 0 || n > 255u) return false;
        return append_hex_span(nmea0183_connector::NmeaSpan(text, n), bytes, count, capacity) && count > 0;
    }

    static bool parse_payload_fields(const nmea0183_connector::NmeaSentence& sentence, uint8_t* bytes, size_t& count, size_t capacity) {
        count = 0;
        if (sentence.field_count == 0) return false;
        for (uint8_t i = 0; i < sentence.field_count; ++i) if (!append_hex_span(sentence.field(i), bytes, count, capacity)) return false;
        return count > 0;
    }

    bool seatalk_fragment_matches(const nmea0183_connector::NmeaSentence& sentence) const {
        if (!seatalk_tunnel_.in_progress && !seatalk_tunnel_.complete) return false;
        char id[sizeof(seatalk_tunnel_.message_id)] = {0};
        nmea0183_connector::nmea_copy_span(id, sizeof(id), sentence.fragment.message_id);
        return strcmp(id, seatalk_tunnel_.message_id) == 0;
    }

    bool update_seatalk_tunnel_fragment(const nmea0183_connector::NmeaSentence& sentence) {
        if (!sentence.fragment.is_fragmented) return false;
        if (sentence.fragment.is_first()) {
            seatalk_tunnel_ = SeaTalkTunnelAssembly{};
            seatalk_tunnel_.in_progress = true;
            seatalk_tunnel_.total = sentence.fragment.total;
            seatalk_tunnel_.next_number = 1;
            nmea0183_connector::nmea_copy_span(seatalk_tunnel_.message_id, sizeof(seatalk_tunnel_.message_id), sentence.fragment.message_id);
        } else if (!seatalk_fragment_matches(sentence)) {
            last_error_ = "SeaTalk tunnel fragment without start";
            return false;
        }
        if (sentence.fragment.number != seatalk_tunnel_.next_number || sentence.fragment.total != seatalk_tunnel_.total) {
            last_error_ = "SeaTalk tunnel fragment order";
            seatalk_tunnel_ = SeaTalkTunnelAssembly{};
            return false;
        }
        const size_t current = strlen(seatalk_tunnel_.text);
        size_t append = sentence.fragment.payload.length;
        if (current + append + 1u > sizeof(seatalk_tunnel_.text)) {
            append = current < sizeof(seatalk_tunnel_.text) ? sizeof(seatalk_tunnel_.text) - current - 1u : 0u;
            seatalk_tunnel_.overflow = true;
        }
        if (append > 0u && sentence.fragment.payload.data) {
            memcpy(seatalk_tunnel_.text + current, sentence.fragment.payload.data, append);
            seatalk_tunnel_.text[current + append] = '\0';
        }
        if (sentence.fragment.number == seatalk_tunnel_.total) {
            seatalk_tunnel_.complete = true;
            seatalk_tunnel_.in_progress = false;
        } else ++seatalk_tunnel_.next_number;
        return true;
    }

    bool apply_seatalk_tunnel(const nmea0183_connector::NmeaSentence& sentence, SourceId source_id, uint64_t now_us) {
        uint8_t bytes[seatalk::SEATALK_FRAME_MAX_BYTES] = {0};
        size_t count = 0;
        if (sentence.fragment.is_fragmented) {
            if (!update_seatalk_tunnel_fragment(sentence)) return false;
            if (!seatalk_tunnel_.complete) return true;
            if (seatalk_tunnel_.overflow) { last_error_ = "SeaTalk tunnel overflow"; return false; }
            if (!parse_payload_text(seatalk_tunnel_.text, bytes, count, sizeof(bytes))) { last_error_ = "bad SeaTalk tunnel payload"; return false; }
        } else if (!parse_payload_fields(sentence, bytes, count, sizeof(bytes))) {
            last_error_ = "bad SeaTalk tunnel payload";
            return false;
        }
        const uint32_t before = seatalk_receiver_.decoded_count();
        if (!seatalk_receiver_.accept_datagram(bytes, count, store_.model(), now_us, ship_data_model::SensorSource::serial)) { last_error_ = "bad SeaTalk tunnel frame"; return false; }
        if (seatalk_receiver_.decoded_count() != before) mark_changed_from_seatalk(seatalk_receiver_.last_decoded(), source_id, now_us);
        return true;
    }

    void mark_changed_from_seatalk(const seatalk::SeaTalkDecoded<Real>& decoded, SourceId source_id, uint64_t now_us) {
        using Kind = seatalk::SeaTalkDecodedKind;
        switch (decoded.kind) {
        case Kind::depth: store_.mark_changed(ModelField::SeaDepthM, source_id, now_us); break;
        case Kind::apparent_wind_angle: store_.mark_changed(ModelField::WindApparentDirectionDeg, source_id, now_us); break;
        case Kind::apparent_wind_speed: store_.mark_changed(ModelField::WindApparentSpeedKn, source_id, now_us); break;
        case Kind::heading_magnetic:
        case Kind::rudder_angle:
        case Kind::autopilot_state: store_.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us); break;
        case Kind::position_latitude: store_.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us); break;
        case Kind::position_longitude: store_.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us); break;
        case Kind::position_lat_lon: store_.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us); store_.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us); break;
        case Kind::speed_over_ground: store_.mark_changed(ModelField::GnssSpeedKn, source_id, now_us); break;
        case Kind::course_over_ground: store_.mark_changed(ModelField::GnssTrackDeg, source_id, now_us); break;
        default: break;
        }
    }

    void mark_changed_from_sentence(const nmea0183_connector::NmeaSentence& sentence, SourceId source_id, uint64_t now_us) {
        if (nmea0183_connector::sentence_is(sentence, "RMC") || nmea0183_connector::sentence_is(sentence, "GGA") || nmea0183_connector::sentence_is(sentence, "GLL")) {
            store_.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us);
            store_.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "RMC") || nmea0183_connector::sentence_is(sentence, "VTG")) {
            store_.mark_changed(ModelField::GnssSpeedKn, source_id, now_us);
            store_.mark_changed(ModelField::GnssTrackDeg, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "HDT") || nmea0183_connector::sentence_is(sentence, "HDM") || nmea0183_connector::sentence_is(sentence, "HDG")) store_.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
        if (nmea0183_connector::sentence_is(sentence, "MWV") || nmea0183_connector::sentence_is(sentence, "MWD") || nmea0183_connector::sentence_is(sentence, "VWR")) {
            store_.mark_changed(ModelField::WindApparentDirectionDeg, source_id, now_us);
            store_.mark_changed(ModelField::WindApparentSpeedKn, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "DBT") || nmea0183_connector::sentence_is(sentence, "DPT")) store_.mark_changed(ModelField::SeaDepthM, source_id, now_us);
        if (nmea0183_connector::sentence_is(sentence, "DBK")) store_.mark_changed(ModelField::SeaDepthBelowKeelM, source_id, now_us);
        if (nmea0183_connector::sentence_is(sentence, "DBS")) store_.mark_changed(ModelField::SeaDepthBelowSurfaceM, source_id, now_us);
    }

    ModelStore<Real>& store_;
    nmea0183_connector::Nmea0183StreamParser parser_;
    nmea0183_connector::Nmea0183RxConnector<Real> rx_;
    seatalk::SeaTalkReceiver<Real> seatalk_receiver_;
    SeaTalkTunnelAssembly seatalk_tunnel_;
    const char* last_error_ = "";
};

} // namespace signalk_mini
