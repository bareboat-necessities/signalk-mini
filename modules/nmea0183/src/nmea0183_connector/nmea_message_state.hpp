#pragma once

#include <stdint.h>
#include <ship_data_model.hpp>

namespace nmea0183_connector {

static const uint8_t NMEA_MESSAGE_MULTIPART_TEXT_BYTES = 96;
static const uint8_t NMEA_MESSAGE_MULTIPART_ID_BYTES = 16;

struct NmeaMessageSource {
    ship_data_model::SensorSource value = ship_data_model::SensorSource::none;
};

struct NmeaMessageStampedInt {
    int32_t value = 0;
    uint64_t last_update_us = 0;

    void set(int32_t next, uint64_t now_us) {
        value = next;
        last_update_us = now_us;
    }
};

struct NmeaMultipartMessageRecord {
    NmeaMessageSource source;
    char sentence_id[4] = {0};
    char message_id[NMEA_MESSAGE_MULTIPART_ID_BYTES] = {0};
    NmeaMessageStampedInt total_fragments;
    NmeaMessageStampedInt last_fragment_number;
    uint16_t received_mask = 0;
    bool in_progress = false;
    bool complete = false;
    bool overflow = false;
    char text[NMEA_MESSAGE_MULTIPART_TEXT_BYTES] = {0};
    NmeaMessageStampedInt text_length;
    uint64_t last_update_us = 0;
};

struct NmeaMessageState {
    NmeaMultipartMessageRecord text_message;
    NmeaMultipartMessageRecord ais_message;
    NmeaMultipartMessageRecord navtex_message;
    NmeaMultipartMessageRecord seatalk_message;
    NmeaMultipartMessageRecord inmarsat_message;
    NmeaMultipartMessageRecord generic_multipart_message;
};

} // namespace nmea0183_connector
