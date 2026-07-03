#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_fir(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    return apply_notification_text_record(sentence, model.events.fire, now_us, source);
}

#include "nmea_F_G_rest.hpp"
