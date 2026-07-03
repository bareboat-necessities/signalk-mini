#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_aam(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) { last_error_ = "short AAM"; return false; }
    float radius_nmi = 0.0f;
    model.route.waypoint_arrival.arrival_circle_entered.value = sentence.field(0)[0] == 'A';
    model.route.waypoint_arrival.perpendicular_passed.value = sentence.field(1)[0] == 'A';
    if (parse_distance_nmi(sentence.field(2), sentence.field(3), radius_nmi)) model.route.waypoint_arrival.arrival_radius_nmi.set(static_cast<Real>(radius_nmi), now_us);
    nmea_copy_span(model.route.waypoint_arrival.waypoint_id, sizeof(model.route.waypoint_arrival.waypoint_id), sentence.field(4));
    set_source(model.route.waypoint_arrival.source, source); model.route.waypoint_arrival.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ack(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short ACK"; return false; }
    auto& ack = model.notifications.alerts.acknowledgement;
    int32_t alarm_number = 0;
    nmea_copy_span(ack.alarm_identifier, sizeof(ack.alarm_identifier), sentence.field(0));
    if (parse_int32(sentence.field(0), alarm_number)) ack.local_alarm_number.set(alarm_number, now_us);
    ack.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(ack.source, source);
    ack.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ads(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    auto& ads = model.ais.data_link_status;
    if (sentence.field_count > 0) nmea_copy_span(ads.station_id, sizeof(ads.station_id), sentence.field(0));
    if (sentence.field_count > 1) nmea_copy_span(ads.slot_status, sizeof(ads.slot_status), sentence.field(1));
    if (sentence.field_count > 2) nmea_copy_span(ads.status_text, sizeof(ads.status_text), sentence.field(2));
    ads.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(ads.source, source);
    ads.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_akd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short AKD"; return false; }
    auto& akd = model.notifications.alerts.acknowledgement_detail;
    int32_t parsed = 0;
    nmea_copy_span(akd.alarm_identifier, sizeof(akd.alarm_identifier), sentence.field(0));
    if (parse_int32(sentence.field(0), parsed)) akd.local_alarm_number.set(parsed, now_us);
    if (sentence.field_count > 1 && parse_int32(sentence.field(1), parsed)) akd.alert_instance.set(parsed, now_us);
    if (sentence.field_count > 2) akd.acknowledgement_state = sentence.field(2)[0];
    if (sentence.field_count > 3) nmea_copy_span(akd.operator_id, sizeof(akd.operator_id), sentence.field(3));
    if (sentence.field_count > 4) nmea_copy_span(akd.detail, sizeof(akd.detail), sentence.field(4));
    akd.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(akd.source, source);
    akd.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ala(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short ALA"; return false; }
    auto& alarm = model.notifications.alerts.condition;
    int32_t parsed = 0;
    nmea_copy_span(alarm.alarm_identifier, sizeof(alarm.alarm_identifier), sentence.field(0));
    if (parse_int32(sentence.field(0), parsed)) alarm.local_alarm_number.set(parsed, now_us);
    if (sentence.field_count > 1 && parse_int32(sentence.field(1), parsed)) alarm.alert_instance.set(parsed, now_us);
    if (sentence.field_count > 2) alarm.condition_state = sentence.field(2)[0];
    if (sentence.field_count > 3) alarm.priority = sentence.field(3)[0];
    if (sentence.field_count > 4) nmea_copy_span(alarm.description, sizeof(alarm.description), sentence.field(4));
    else if (sentence.field_count > 3) nmea_copy_span(alarm.description, sizeof(alarm.description), sentence.field(3));
    alarm.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(alarm.source, source);
    alarm.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_alm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 15) { last_error_ = "short ALM"; return false; }
    int32_t integer_value = 0; float value = 0.0f;
    if (parse_int32(sentence.field(0), integer_value)) model.gnss.almanac.total_messages.set(integer_value, now_us);
    if (parse_int32(sentence.field(1), integer_value)) model.gnss.almanac.message_number.set(integer_value, now_us);
    if (parse_int32(sentence.field(2), integer_value)) model.gnss.almanac.satellite_prn.set(integer_value, now_us);
    if (parse_int32(sentence.field(3), integer_value)) model.gnss.almanac.gnss_week.set(integer_value, now_us);
    if (parse_int32(sentence.field(4), integer_value)) model.gnss.almanac.sv_health.set(integer_value, now_us);
    if (parse_real(sentence.field(5), value)) model.gnss.almanac.eccentricity.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.gnss.almanac.reference_time_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(7), value)) model.gnss.almanac.inclination_rad.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.gnss.almanac.right_ascension_rate_rad_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.gnss.almanac.sqrt_semi_major_axis.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(10), value)) model.gnss.almanac.argument_of_perigee_rad.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(11), value)) model.gnss.almanac.longitude_ascension_node_rad.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(12), value)) model.gnss.almanac.mean_anomaly_rad.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(13), value)) model.gnss.almanac.clock_f0_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(14), value)) model.gnss.almanac.clock_f1_s_s.set(static_cast<Real>(value), now_us);
    set_source(model.gnss.almanac.source, source); model.gnss.almanac.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_apa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 10) { last_error_ = "short APA"; return false; }
    float xte_nmi = 0.0f; float bearing_deg = 0.0f;
    if (parse_distance_nmi(sentence.field(2), sentence.field(4), xte_nmi)) {
        if (sentence.field(3)[0] == 'L') xte_nmi = -xte_nmi;
        else if (sentence.field(3)[0] != 'R') { last_error_ = "bad APA steer direction"; return false; }
        model.route.apb.xte_nmi.set(static_cast<Real>(xte_nmi), now_us);
    }
    model.route.apb.arrival_circle_entered.value = sentence.field(5)[0] == 'A';
    model.route.apb.perpendicular_passed.value = sentence.field(6)[0] == 'A';
    if (parse_real(sentence.field(7), bearing_deg)) model.route.apb.origin_to_destination_bearing_deg.set(static_cast<Real>(wrap_360_deg(bearing_deg)), now_us);
    nmea_copy_span(model.route.apb.destination_id, sizeof(model.route.apb.destination_id), sentence.field(9));
    set_source(model.route.apb.source, source); model.route.apb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_apb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 14) { last_error_ = "short APB"; return false; }
    bool any = false; float value = 0.0f;
    if (parse_left_right_signed(sentence.field(2), sentence.field(3), value)) { if (value > 0.15f) value = 0.15f; if (value < -0.15f) value = -0.15f; model.route.apb.xte_nmi.set(static_cast<Real>(value), now_us); any = true; }
    model.route.apb.arrival_circle_entered.value = sentence.field(5)[0] == 'A';
    model.route.apb.perpendicular_passed.value = sentence.field(6)[0] == 'A';
    if (parse_real(sentence.field(7), value)) { model.route.apb.origin_to_destination_bearing_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us); any = true; }
    nmea_copy_span(model.route.apb.destination_id, sizeof(model.route.apb.destination_id), sentence.field(9));
    if (parse_real(sentence.field(10), value)) { model.route.apb.present_to_destination_bearing_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us); any = true; }
    if (parse_real(sentence.field(12), value)) { model.route.apb.track_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us); model.route.apb.heading_to_steer_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us); any = true; }
    last_apb_mode_ = sentence.field(13)[0] == 'M' ? ship_data_model::AutopilotMode::compass : ship_data_model::AutopilotMode::gps;
    model.route.apb.mode_hint.value = last_apb_mode_;
    last_apb_sender_id_[0] = sentence.talker[0]; last_apb_sender_id_[1] = sentence.talker[1]; last_apb_sender_id_[2] = '\0';
    model.route.apb.sender_id[0] = last_apb_sender_id_[0]; model.route.apb.sender_id[1] = last_apb_sender_id_[1]; model.route.apb.sender_id[2] = '\0';
    set_source(model.route.apb.source, source); model.route.apb.last_update_us = now_us;
    if (!any) last_error_ = "bad APB";
    return any;
}

template<typename Model>
bool apply_asd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short ASD"; return false; }
    auto& asd = model.ais.addressed_safety;
    int32_t parsed = 0;
    if (parse_int32(sentence.field(0), parsed)) asd.total_messages.set(parsed, now_us);
    if (sentence.field_count > 1 && parse_int32(sentence.field(1), parsed)) asd.message_number.set(parsed, now_us);
    if (sentence.field_count > 2) nmea_copy_span(asd.sequential_message_id, sizeof(asd.sequential_message_id), sentence.field(2));
    if (sentence.field_count > 3) nmea_copy_span(asd.destination_mmsi, sizeof(asd.destination_mmsi), sentence.field(3));
    if (sentence.field_count > 4) asd.retransmit_flag = sentence.field(4)[0];
    if (sentence.field_count > 5) nmea_copy_span(asd.safety_text, sizeof(asd.safety_text), sentence.field(5));
    else if (sentence.field_count > 3) nmea_copy_span(asd.safety_text, sizeof(asd.safety_text), sentence.field(sentence.field_count - 1));
    asd.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(asd.source, source);
    asd.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_bec(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    return apply_bwc_bwr(sentence, model, now_us, source);
}

template<typename Model>
bool apply_bod_bww(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = sentence_is(sentence, "BOD") ? "short BOD" : "short BWW"; return false; }
    float bearing_deg = 0.0f;
    if (parse_real(sentence.field(0), bearing_deg)) model.route.waypoint.bearing_true_deg.set(static_cast<Real>(wrap_360_deg(bearing_deg)), now_us);
    if (parse_real(sentence.field(2), bearing_deg)) model.route.waypoint.bearing_magnetic_deg.set(static_cast<Real>(wrap_360_deg(bearing_deg)), now_us);
    nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(4));
    nmea_copy_span(model.route.waypoint.from_waypoint_id, sizeof(model.route.waypoint.from_waypoint_id), sentence.field(5));
    set_source(model.route.waypoint.source, source); model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_bwc_bwr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) { last_error_ = sentence_is(sentence, "BWC") ? "short BWC" : "short BWR"; return false; }
    float value = 0.0f;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.route.waypoint.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.route.waypoint.latitude_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.route.waypoint.longitude_deg.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(5), value)) model.route.waypoint.bearing_true_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (parse_real(sentence.field(7), value)) model.route.waypoint.bearing_magnetic_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (parse_distance_nmi(sentence.field(9), sentence.field(10), value)) model.route.waypoint.distance_nmi.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(11));
    set_source(model.route.waypoint.source, source); model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_cek(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short CEK"; return false; }
    auto& rec = model.comm.equipment.control_command;
    nmea_copy_span(rec.equipment_id, sizeof(rec.equipment_id), sentence.field(0));
    if (sentence.field_count > 1) nmea_copy_span(rec.command_id, sizeof(rec.command_id), sentence.field(1));
    if (sentence.field_count > 2) rec.command_state = sentence.field(2)[0];
    if (sentence.field_count > 3) nmea_copy_span(rec.parameter, sizeof(rec.parameter), sentence.field(3));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_cop(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short COP"; return false; }
    auto& rec = model.comm.equipment.control_operation;
    nmea_copy_span(rec.equipment_id, sizeof(rec.equipment_id), sentence.field(0));
    if (sentence.field_count > 1) nmea_copy_span(rec.operation_id, sizeof(rec.operation_id), sentence.field(1));
    if (sentence.field_count > 2) rec.operation_state = sentence.field(2)[0];
    if (sentence.field_count > 3) nmea_copy_span(rec.value, sizeof(rec.value), sentence.field(3));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_cur(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short CUR"; return false; }
    float value = 0.0f;
    if (parse_real(sentence.field(1), value)) { if (sentence.field(2)[0] == 'M') model.sea.current_direction_magnetic_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us); else model.sea.current_direction_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us); }
    if (parse_knots(sentence.field(3), sentence.field_count > 4 ? sentence.field(4) : NmeaSpan(), value)) model.sea.current_speed_kn.set(static_cast<Real>(value), now_us);
    set_source(model.sea.source, source); model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dbt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) { float depth_m = 0.0f; if (!parse_depth_m_from_triplet(sentence, depth_m)) { last_error_ = "bad DBT"; return false; } model.sea.depth_m.set(static_cast<Real>(depth_m), now_us); set_source(model.sea.depth_source, source); model.sea.last_update_us = now_us; return true; }

template<typename Model>
bool apply_depth_below_keel(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) { float depth_m = 0.0f; if (!parse_depth_m_from_triplet(sentence, depth_m)) { last_error_ = "bad DBK"; return false; } model.sea.depth_below_keel_m.set(static_cast<Real>(depth_m), now_us); set_source(model.sea.depth_source, source); model.sea.last_update_us = now_us; return true; }

template<typename Model>
bool apply_depth_below_surface(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) { float depth_m = 0.0f; if (!parse_depth_m_from_triplet(sentence, depth_m)) { last_error_ = "bad DBS"; return false; } model.sea.depth_below_surface_m.set(static_cast<Real>(depth_m), now_us); set_source(model.sea.depth_source, source); model.sea.last_update_us = now_us; return true; }

template<typename Model>
bool apply_dcn(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 16) { last_error_ = "short DCN"; return false; }
    float value = 0.0f; int32_t integer_value = 0;
    nmea_copy_span(model.nav.decca.chain_id, sizeof(model.nav.decca.chain_id), sentence.field(0));
    nmea_copy_span(model.nav.decca.red_zone, sizeof(model.nav.decca.red_zone), sentence.field(1));
    if (parse_real(sentence.field(2), value)) model.nav.decca.red_line_of_position.set(static_cast<Real>(value), now_us); model.nav.decca.red_master_status = sentence.field(3)[0];
    nmea_copy_span(model.nav.decca.green_zone, sizeof(model.nav.decca.green_zone), sentence.field(4));
    if (parse_real(sentence.field(5), value)) model.nav.decca.green_line_of_position.set(static_cast<Real>(value), now_us); model.nav.decca.green_master_status = sentence.field(6)[0];
    nmea_copy_span(model.nav.decca.purple_zone, sizeof(model.nav.decca.purple_zone), sentence.field(7));
    if (parse_real(sentence.field(8), value)) model.nav.decca.purple_line_of_position.set(static_cast<Real>(value), now_us); model.nav.decca.purple_master_status = sentence.field(9)[0];
    model.nav.decca.red_line_navigation_use = sentence.field(10)[0]; model.nav.decca.green_line_navigation_use = sentence.field(11)[0]; model.nav.decca.purple_line_navigation_use = sentence.field(12)[0];
    if (parse_distance_nmi(sentence.field(13), sentence.field(14), value)) model.nav.decca.position_uncertainty_nmi.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(15), integer_value)) model.nav.decca.fix_data_basis.set(integer_value, now_us);
    set_source(model.nav.decca.source, source); model.nav.decca.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dcr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short DCR"; return false; }
    auto& rec = model.comm.equipment.control_response;
    nmea_copy_span(rec.equipment_id, sizeof(rec.equipment_id), sentence.field(0));
    if (sentence.field_count > 1) nmea_copy_span(rec.command_id, sizeof(rec.command_id), sentence.field(1));
    if (sentence.field_count > 2) rec.response_state = sentence.field(2)[0];
    if (sentence.field_count > 3) nmea_copy_span(rec.result, sizeof(rec.result), sentence.field(3));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ddc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short DDC"; return false; }
    auto& rec = model.comm.equipment.display_control;
    nmea_copy_span(rec.display_id, sizeof(rec.display_id), sentence.field(0));
    if (sentence.field_count > 1) nmea_copy_span(rec.page_id, sizeof(rec.page_id), sentence.field(1));
    if (sentence.field_count > 2) rec.control_state = sentence.field(2)[0];
    if (sentence.field_count > 3) nmea_copy_span(rec.value, sizeof(rec.value), sentence.field(3));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dor(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short DOR"; return false; }
    auto& rec = model.comm.equipment.door_status;
    nmea_copy_span(rec.door_id, sizeof(rec.door_id), sentence.field(0));
    if (sentence.field_count > 1) rec.door_state = sentence.field(1)[0];
    if (sentence.field_count > 2) rec.lock_state = sentence.field(2)[0];
    if (sentence.field_count > 3) nmea_copy_span(rec.description, sizeof(rec.description), sentence.field(3));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dpt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) { float depth_m = 0.0f; float offset_m = 0.0f; if (!parse_real(sentence.field(0), depth_m)) { last_error_ = "bad DPT"; return false; } model.sea.depth_m.set(static_cast<Real>(depth_m), now_us); if (parse_real(sentence.field(1), offset_m)) model.sea.depth_offset_m.set(static_cast<Real>(offset_m), now_us); set_source(model.sea.depth_source, source); model.sea.last_update_us = now_us; return true; }

template<typename Model>
bool apply_dsc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model; if (sentence.field_count < 10) { last_error_ = "short DSC"; return false; }
    auto& dsc = dsc_state_.message; int32_t integer_value = 0; float value = 0.0f;
    if (parse_int32(sentence.field(0), integer_value)) dsc.format_specifier.set(integer_value, now_us);
    nmea_copy_span(dsc.sender_mmsi, sizeof(dsc.sender_mmsi), sentence.field(1));
    if (parse_int32(sentence.field(2), integer_value)) dsc.category.set(integer_value, now_us);
    if (parse_int32(sentence.field(3), integer_value)) dsc.nature_or_first_telecommand.set(integer_value, now_us);
    if (parse_int32(sentence.field(4), integer_value)) dsc.communication_or_second_telecommand.set(integer_value, now_us);
    nmea_copy_span(dsc.position_code, sizeof(dsc.position_code), sentence.field(5));
    float lat = 0.0f; float lon = 0.0f; if (parse_dsc_compact_position(sentence.field(5), lat, lon)) { dsc.latitude_deg = lat; dsc.longitude_deg = lon; dsc.has_position = true; }
    if (parse_nmea_hhmm_time_s(sentence.field(6), value)) { dsc.utc_time_s = value; dsc.has_utc_time = true; }
    nmea_copy_span(dsc.address_or_distress_mmsi, sizeof(dsc.address_or_distress_mmsi), sentence.field(7));
    nmea_copy_span(dsc.field10, sizeof(dsc.field10), sentence.field(8));
    dsc.end_of_sequence = sentence.field(9)[0]; if (sentence.field_count > 10) dsc.expansion_flag = sentence.field(10)[0];
    set_source(dsc.source, source); dsc.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dse(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model; if (sentence.field_count < 6) { last_error_ = "short DSE"; return false; }
    auto& dse = dsc_state_.expansion; int32_t integer_value = 0;
    if (parse_int32(sentence.field(0), integer_value)) dse.total_messages.set(integer_value, now_us);
    if (parse_int32(sentence.field(1), integer_value)) dse.message_number.set(integer_value, now_us);
    dse.query_flag = sentence.field(2)[0]; nmea_copy_span(dse.sender_mmsi, sizeof(dse.sender_mmsi), sentence.field(3));
    if (parse_int32(sentence.field(4), integer_value)) dse.expansion_specifier.set(integer_value, now_us);
    nmea_copy_span(dse.payload, sizeof(dse.payload), sentence.field(5)); set_source(dse.source, source); dse.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dsi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) { return apply_nmea_text_record(sentence, model.notifications.dsc.interrogation, now_us, source); }

template<typename Model>
bool apply_dsr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) { return apply_nmea_text_record(sentence, model.notifications.dsc.response, now_us, source); }

template<typename Model>
bool apply_dtm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) { last_error_ = "short DTM"; return false; }
    float value = 0.0f;
    nmea_copy_span(model.nav.datum.local_datum_code, sizeof(model.nav.datum.local_datum_code), sentence.field(0));
    nmea_copy_span(model.nav.datum.local_datum_subcode, sizeof(model.nav.datum.local_datum_subcode), sentence.field(1));
    if (parse_north_south_signed(sentence.field(2), sentence.field(3), value)) model.nav.datum.latitude_offset_min.set(static_cast<Real>(value), now_us);
    if (parse_east_west_signed(sentence.field(4), sentence.field(5), value)) model.nav.datum.longitude_offset_min.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.nav.datum.altitude_offset_m.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.nav.datum.reference_datum_code, sizeof(model.nav.datum.reference_datum_code), sentence.field(7));
    set_source(model.nav.datum.source, source); model.nav.datum.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_etl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) { return apply_nmea_text_record(sentence, model.notifications.messages.event_log, now_us, source); }

template<typename Model>
bool apply_eve(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) { return apply_nmea_text_record(sentence, model.notifications.messages.event, now_us, source); }
