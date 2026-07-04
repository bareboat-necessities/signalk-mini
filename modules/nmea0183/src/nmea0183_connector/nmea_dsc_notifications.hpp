#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_dsi(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) {
        last_error_ = "short DSI";
        return false;
    }

    auto& interrogation = model.notifications.dsc.interrogation;
    nmea_copy_span(interrogation.request_id,
                   sizeof(interrogation.request_id),
                   sentence.field(0));
    if (sentence.field_count > 1) {
        nmea_copy_span(interrogation.remote_mmsi,
                       sizeof(interrogation.remote_mmsi),
                       sentence.field(1));
    }
    interrogation.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(interrogation.source, source);
    interrogation.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dsr(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) {
        last_error_ = "short DSR";
        return false;
    }

    auto& response = model.notifications.dsc.response;
    nmea_copy_span(response.response_id,
                   sizeof(response.response_id),
                   sentence.field(0));
    if (sentence.field_count > 1) {
        nmea_copy_span(response.remote_mmsi,
                       sizeof(response.remote_mmsi),
                       sentence.field(1));
    }
    response.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(response.source, source);
    response.last_update_us = now_us;
    return true;
}
