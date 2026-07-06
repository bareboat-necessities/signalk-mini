#pragma once

// Included inside Nmea0183RxConnector.

static const char* nmea_phase3_abk_ack_label(int32_t value) {
    switch (value) {
    case 0: return "acknowledged";
    case 1: return "requested_message_not_available";
    case 2: return "requested_message_not_supported";
    case 3: return "addressed_binary_acknowledged";
    case 4: return "broadcast_binary_acknowledged";
    default: return "unknown";
    }
}

template<typename StampedInt>
static void nmea_phase3_parse_int_to(NmeaSpan field, StampedInt& dst, uint64_t now_us) {
    int32_t parsed = 0;
    if (parse_int32(field, parsed)) dst.set(parsed, now_us);
}

template<typename Model>
bool apply_abk(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) { last_error_ = "short ABK"; return false; }

    auto& rec = model.ais.acknowledgement;
    int32_t parsed = 0;
    if (parse_int32(sentence.field(0), parsed)) {
        rec.mmsi.set(parsed, now_us);
        rec.destination_mmsi[0].set(parsed, now_us);
    }
    if (parse_int32(sentence.field(2), parsed)) rec.message_type.set(parsed, now_us);
    if (parse_int32(sentence.field(3), parsed)) rec.sequence_number[0].set(parsed, now_us);
    rec.acknowledgement_count.set(1, now_us);

    auto& status = model.ais.data_link_status;
    nmea_copy_span(status.station_id, sizeof(status.station_id), sentence.field(0));
    nmea_copy_span(status.slot_status, sizeof(status.slot_status), sentence.field(1));
    if (parse_int32(sentence.field(4), parsed)) {
        nmea_copy_cstr(status.status_text, sizeof(status.status_text), nmea_phase3_abk_ack_label(parsed));
    } else {
        nmea_copy_span(status.status_text, sizeof(status.status_text), sentence.field(4));
    }
    status.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(status.source, source);
    status.last_update_us = now_us;

    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_aga(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 15) { last_error_ = "short AGA"; return false; }

    auto& rec = model.ais.group_assignment;
    nmea_phase3_parse_int_to(sentence.field(0), rec.mmsi, now_us);
    nmea_phase3_parse_int_to(sentence.field(1), rec.station_type, now_us);
    nmea_phase3_parse_int_to(sentence.field(2), rec.ship_type, now_us);

    float lat1 = 0.0f;
    float lon1 = 0.0f;
    float lat2 = 0.0f;
    float lon2 = 0.0f;
    const bool have_lat1 = parse_lat_lon(sentence.field(3), sentence.field(4), lat1);
    const bool have_lon1 = parse_lat_lon(sentence.field(5), sentence.field(6), lon1);
    const bool have_lat2 = parse_lat_lon(sentence.field(7), sentence.field(8), lat2);
    const bool have_lon2 = parse_lat_lon(sentence.field(9), sentence.field(10), lon2);
    if (have_lat1 && have_lat2) {
        const float north = lat1 > lat2 ? lat1 : lat2;
        const float south = lat1 > lat2 ? lat2 : lat1;
        rec.northeast_lat_deg.set(static_cast<Real>(north), now_us);
        rec.southwest_lat_deg.set(static_cast<Real>(south), now_us);
    }
    if (have_lon1 && have_lon2) {
        const float east = lon1 > lon2 ? lon1 : lon2;
        const float west = lon1 > lon2 ? lon2 : lon1;
        rec.northeast_lon_deg.set(static_cast<Real>(east), now_us);
        rec.southwest_lon_deg.set(static_cast<Real>(west), now_us);
    }

    nmea_phase3_parse_int_to(sentence.field(11), rec.report_interval, now_us);
    nmea_phase3_parse_int_to(sentence.field(12), rec.txrx_mode, now_us);
    nmea_phase3_parse_int_to(sentence.field(13), rec.quiet_time_min, now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_bcl(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) { last_error_ = "short BCL"; return false; }

    auto& rec = model.ais.base_station;
    nmea_phase3_parse_int_to(sentence.field(0), rec.mmsi, now_us);
    nmea_phase3_parse_int_to(sentence.field(1), rec.epfd_type, now_us);

    const uint8_t lat_index = sentence.field_count >= 10 ? 3 : 2;
    float value = 0.0f;
    if (parse_lat_lon(sentence.field(lat_index), sentence.field(lat_index + 1), value)) {
        rec.latitude_deg.set(static_cast<Real>(value), now_us);
    }
    if (parse_lat_lon(sentence.field(lat_index + 2), sentence.field(lat_index + 3), value)) {
        rec.longitude_deg.set(static_cast<Real>(value), now_us);
    }
    const NmeaSpan accuracy = sentence.field_count > lat_index + 4 ? sentence.field(lat_index + 4) : NmeaSpan();
    if (!accuracy.empty()) rec.position_accuracy = accuracy[0] == '1' || accuracy[0] == 'A' || accuracy[0] == 'H';

    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mob(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 2) { last_error_ = "short MOB"; return false; }

    auto& rec = model.notifications.special.smv;
    nmea_copy_span(rec.message_id, sizeof(rec.message_id), sentence.field(0));
    nmea_copy_cstr(rec.event_type, sizeof(rec.event_type), "MOB");
    rec.status = sentence.field(1)[0];

    float value = 0.0f;
    if (sentence.field_count > 2 && parse_utc_time_of_day_s(sentence.field(2), value)) {
        rec.utc_time_s.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field_count > 4 && parse_lat_lon(sentence.field(3), sentence.field(4), value)) {
        rec.latitude_deg.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field_count > 6 && parse_lat_lon(sentence.field(5), sentence.field(6), value)) {
        rec.longitude_deg.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field_count > 7) nmea_copy_span(rec.description, sizeof(rec.description), sentence.field(7));
    else nmea_copy_cstr(rec.description, sizeof(rec.description), "MOB event");
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}
