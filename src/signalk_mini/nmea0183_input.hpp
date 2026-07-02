#pragma once

#include <stdint.h>
#include <nmea0183_connector.hpp>
#include <ship_data_model.hpp>
#include "model_store.hpp"
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
        const bool applied = rx_.apply_sentence(sentence, store_.model(), now_us, ship_data_model::SensorSource::serial);
        if (!applied) return false;
        mark_changed_from_sentence(sentence, source_id, now_us);
        return true;
    }

    const char* last_error() const { return rx_.last_error(); }

private:
    void mark_changed_from_sentence(const nmea0183_connector::NmeaSentence& sentence, SourceId source_id, uint64_t now_us) {
        if (nmea0183_connector::sentence_is(sentence, "RMC") || nmea0183_connector::sentence_is(sentence, "GGA") || nmea0183_connector::sentence_is(sentence, "GLL")) {
            store_.mark_changed(ModelField::NavigationGpsFixLatDeg, source_id, now_us);
            store_.mark_changed(ModelField::NavigationGpsFixLonDeg, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "RMC") || nmea0183_connector::sentence_is(sentence, "VTG")) {
            store_.mark_changed(ModelField::NavigationGpsSpeedKn, source_id, now_us);
            store_.mark_changed(ModelField::NavigationGpsTrackDeg, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "HDT") || nmea0183_connector::sentence_is(sentence, "HDM") || nmea0183_connector::sentence_is(sentence, "HDG")) {
            store_.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "MWV") || nmea0183_connector::sentence_is(sentence, "MWD") || nmea0183_connector::sentence_is(sentence, "VWR")) {
            store_.mark_changed(ModelField::WindApparentDirectionDeg, source_id, now_us);
            store_.mark_changed(ModelField::WindApparentSpeedKn, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "DBT") || nmea0183_connector::sentence_is(sentence, "DPT")) {
            store_.mark_changed(ModelField::WaterDepthM, source_id, now_us);
        }
    }

    ModelStore<Real>& store_;
    nmea0183_connector::Nmea0183StreamParser parser_;
    nmea0183_connector::Nmea0183RxConnector<Real> rx_;
};

} // namespace signalk_mini
