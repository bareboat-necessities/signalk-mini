#pragma once

#include <stdint.h>

#include <nmea0183_connector.hpp>
#include <seatalk.hpp>
#include <ship_data_model.hpp>

#include "model_store.hpp"
#include "seatalk_change_mapper.hpp"
#include "types.hpp"

namespace signalk_mini {

template<typename Real>
class Nmea0183Input {
public:
    explicit Nmea0183Input(ModelStore<Real>& store) : store_(store) {}

    bool feed_line(const char* line, SourceId source_id, uint64_t now_us, bool validate_checksum = true) {
        const nmea0183_connector::NmeaTokenizeResult tokens = nmea0183_connector::tokenize_nmea_line(line);
        const nmea0183_connector::NmeaToken* token = tokens.first_sentence();
        if (!token) return false;

        char sentence_text[nmea0183_connector::NMEA_MAX_SENTENCE_LEN];
        nmea0183_connector::nmea_copy_span(sentence_text, sizeof(sentence_text), token->text);

        nmea0183_connector::NmeaSentence sentence;
        if (!parser_.parse_line(sentence_text, sentence, validate_checksum)) return false;

        const uint32_t decoded_before = rx_.seatalk_receiver().decoded_count();
        const bool applied = rx_.apply_sentence(sentence, store_.model(), now_us, ship_data_model::SensorSource::serial);
        if (!applied) return false;

        if (rx_.seatalk_receiver().decoded_count() != decoded_before) {
            mark_seatalk_changes(store_, rx_.seatalk_receiver().last_decoded(), source_id, now_us);
        } else {
            mark_changed_from_sentence(sentence, source_id, now_us);
        }
        return true;
    }

    const char* last_error() const { return rx_.last_error(); }
    const nmea0183_connector::NmeaMessageState& message_state() const { return rx_.message_state(); }
    const nmea0183_connector::NmeaDscMessageState& dsc_state() const { return rx_.dsc_state(); }
    const seatalk::SeaTalkReceiver<Real>& seatalk_receiver() const { return rx_.seatalk_receiver(); }

private:
    void mark_changed_from_sentence(const nmea0183_connector::NmeaSentence& sentence, SourceId source_id, uint64_t now_us) {
        if (nmea0183_connector::sentence_is(sentence, "RMC") ||
            nmea0183_connector::sentence_is(sentence, "GGA") ||
            nmea0183_connector::sentence_is(sentence, "GLL")) {
            store_.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us);
            store_.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us);
        }

        if (nmea0183_connector::sentence_is(sentence, "RMC") ||
            nmea0183_connector::sentence_is(sentence, "VTG")) {
            store_.mark_changed(ModelField::GnssSpeedKn, source_id, now_us);
            store_.mark_changed(ModelField::GnssTrackDeg, source_id, now_us);
        }

        if (nmea0183_connector::sentence_is(sentence, "HDT") ||
            nmea0183_connector::sentence_is(sentence, "HDM") ||
            nmea0183_connector::sentence_is(sentence, "HDG")) {
            store_.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
        }

        if (nmea0183_connector::sentence_is(sentence, "MWV") ||
            nmea0183_connector::sentence_is(sentence, "MWD") ||
            nmea0183_connector::sentence_is(sentence, "VWR")) {
            store_.mark_changed(ModelField::WindApparentDirectionDeg, source_id, now_us);
            store_.mark_changed(ModelField::WindApparentSpeedKn, source_id, now_us);
        }

        if (nmea0183_connector::sentence_is(sentence, "DBT") ||
            nmea0183_connector::sentence_is(sentence, "DPT")) {
            store_.mark_changed(ModelField::SeaDepthM, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "DBK")) {
            store_.mark_changed(ModelField::SeaDepthBelowKeelM, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "DBS")) {
            store_.mark_changed(ModelField::SeaDepthBelowSurfaceM, source_id, now_us);
        }
    }

    ModelStore<Real>& store_;
    nmea0183_connector::Nmea0183StreamParser parser_;
    nmea0183_connector::Nmea0183RxConnector<Real> rx_;
};

} // namespace signalk_mini
