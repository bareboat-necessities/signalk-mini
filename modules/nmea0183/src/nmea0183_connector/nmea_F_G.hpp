#pragma once

// Included inside Nmea0183RxConnector.

#include "nmea_alerts.hpp"
#include "nmea_dsc.hpp"
#include "nmea_gnss.hpp"

template<typename Model>
bool apply_fir(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    return apply_nmea_text_record(sentence, model.notifications.alerts.fire, now_us, source);
}

template<typename Model>
bool apply_fsi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short FSI"; return false; }
    float value = 0.0f; int32_t integer_value = 0;
    if (parse_real(sentence.field(0), value)) model.comm.radio_frequency_set.transmitting_frequency_hz.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.comm.radio_frequency_set.receiving_frequency_hz.set(static_cast<Real>(value), now_us);
    model.comm.radio_frequency_set.communication_mode = sentence.field(2)[0];
    if (parse_int32(sentence.field(3), integer_value)) model.comm.radio_frequency_set.power_level.set(integer_value, now_us);
    set_source(model.comm.radio_frequency_set.source, source); model.comm.radio_frequency_set.last_update_us = now_us;
    return true;
}
