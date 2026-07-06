#pragma once

#include <string.h>

#include "../core_data_types.hpp"

namespace ship_data_model {

static const uint8_t NAVTEX_MESSAGE_HISTORY_CAPACITY = 8;
static const uint8_t ALERT_LIST_ENTRY_CAPACITY = 8;
static const uint8_t INMARSAT_SAFETYNET_MESSAGE_HISTORY_CAPACITY = 4;
static const uint16_t INMARSAT_SAFETYNET_MESSAGE_TEXT_CAPACITY = 512;

template<typename Real = float>
struct AlertAcknowledgementData {
    Setting<SensorSource> source;
    char alarm_identifier[24] = {0};
    Stamped<int32_t> local_alarm_number;
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertAcknowledgementDetailData {
    Setting<SensorSource> source;
    char alarm_identifier[24] = {0};
    Stamped<int32_t> local_alarm_number;
    Stamped<int32_t> alert_instance;
    char acknowledgement_state = 0;
    char operator_id[24] = {0};
    char detail[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertConditionData {
    Setting<SensorSource> source;
    char alarm_identifier[24] = {0};
    Stamped<int32_t> local_alarm_number;
    Stamped<int32_t> alert_instance;
    char condition_state = 0;
    char priority = 0;
    char description[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertCyclicListData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    char sequential_message_id[16] = {0};
    char alert_identifier[ALERT_LIST_ENTRY_CAPACITY][24] = {{0}};
    Stamped<int32_t> alert_count;
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertReportData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_fragments;
    Stamped<int32_t> fragment_number;
    char sequential_message_id[16] = {0};
    char alert_identifier[24] = {0};
    Stamped<int32_t> alert_instance;
    Stamped<int32_t> revision_counter;
    Stamped<int32_t> escalation_counter;
    char category = 0;
    char priority = 0;
    char alert_state = 0;
    char alert_text[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertAlarmStateData {
    Setting<SensorSource> source;
    Stamped<Real> utc_time_s;
    char alarm_identifier[24] = {0};
    Stamped<int32_t> local_alarm_number;
    char condition_state = 0;
    char acknowledgement_state = 0;
    char description[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertCommandRefusedData {
    Setting<SensorSource> source;
    char alert_identifier[24] = {0};
    Stamped<int32_t> alert_instance;
    char refused_command[24] = {0};
    char reason_code[24] = {0};
    char reason_text[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertHeartbeatData {
    Setting<SensorSource> source;
    char status = 0;
    Stamped<Real> interval_s;
    char sequential_message_id[16] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NotificationAlertsData {
    AlertAcknowledgementData<Real> acknowledgement;
    AlertAcknowledgementDetailData<Real> acknowledgement_detail;
    AlertConditionData<Real> condition;
    AlertCyclicListData<Real> cyclic_list;
    AlertReportData<Real> alert_report;
    AlertAlarmStateData<Real> alarm_state;
    AlertCommandRefusedData<Real> command_refused;
    AlertHeartbeatData<Real> heartbeat;
    NmeaTextRecordData<Real> fire;
};

template<typename Real = float>
struct EventLogData {
    Setting<SensorSource> source;
    char event_id[24] = {0};
    char event_type[16] = {0};
    char event_state = 0;
    char event_text[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct EventData {
    Setting<SensorSource> source;
    char event_id[24] = {0};
    char event_source[24] = {0};
    char event_state = 0;
    char event_text[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NotificationMessagesData {
    NmeaTextRecordData<Real> text;
    EventLogData<Real> event_log;
    EventData<Real> event;
};

template<typename Real = float>
struct DscInterrogationData {
    Setting<SensorSource> source;
    char request_id[24] = {0};
    char remote_mmsi[16] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct DscResponseData {
    Setting<SensorSource> source;
    char response_id[24] = {0};
    char remote_mmsi[16] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct DscAlertData {
    Setting<SensorSource> source;
    char sender_mmsi[16] = {0};
    char address_or_distress_mmsi[16] = {0};
    Stamped<int32_t> category;
    Stamped<int32_t> nature_or_first_telecommand;
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    Stamped<Real> utc_time_s;
    char alert_text[96] = {0};
    bool active = false;
    bool acknowledged = false;
    bool resolved = false;
    bool duplicate = false;
    Stamped<int32_t> repeat_count;
    Stamped<int32_t> field_count;
    uint64_t first_seen_us = 0;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NotificationDscData {
    DscInterrogationData<Real> interrogation;
    DscResponseData<Real> response;
    DscAlertData<Real> distress;
    DscAlertData<Real> urgency;
    DscAlertData<Real> safety;
};

template<typename Real = float>
struct InmarsatSafetyNetMessageData {
    Setting<SensorSource> source;
    char message_id[16] = {0};
    char terminal_id[24] = {0};
    char msi_status = 0;
    char les_sequence_number[8] = {0};
    char les_id[4] = {0};
    char ocean_region_code = 0;
    char ocean_region_label[32] = {0};
    char priority_code = 0;
    char priority_label[16] = {0};
    char service_code[4] = {0};
    char service_label[64] = {0};
    char presentation_code[4] = {0};
    char presentation_label[32] = {0};
    Stamped<int32_t> received_year;
    Stamped<int32_t> received_month;
    Stamped<int32_t> received_day;
    Stamped<int32_t> received_hour;
    Stamped<int32_t> received_minute;
    char address_kind[24] = {0};
    Stamped<int32_t> navarea_metarea_code;
    Stamped<int32_t> coastal_warning_navarea_metarea;
    char coastal_warning_area = 0;
    char coastal_warning_subject = 0;
    char coastal_warning_subject_label[56] = {0};
    bool coastal_warning_subject_mandatory = false;
    Stamped<Real> circular_center_lat_deg;
    Stamped<Real> circular_center_lon_deg;
    Stamped<int32_t> circular_radius_nmi;
    Stamped<Real> rectangle_sw_lat_deg;
    Stamped<Real> rectangle_sw_lon_deg;
    Stamped<int32_t> rectangle_extent_lat_deg;
    Stamped<int32_t> rectangle_extent_lon_deg;
    char sequential_message_id[8] = {0};
    char message_text[INMARSAT_SAFETYNET_MESSAGE_TEXT_CAPACITY] = {0};
    Stamped<int32_t> total_fragments;
    Stamped<int32_t> fragment_number;
    Stamped<int32_t> text_length;
    Stamped<int32_t> field_count;
    bool body_complete = false;
    bool complete = false;
    bool overflow = false;
    bool duplicate = false;
    bool acknowledged = false;
    bool body_has_unknown_char = false;
    bool requires_alarm = false;
    bool mandatory_reception = false;
    bool suppressible_by_receiver = false;
    Stamped<int32_t> repeat_count;
    uint64_t first_seen_us = 0;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct InmarsatSafetyNetData {
    InmarsatSafetyNetMessageData<Real> latest_message;
    InmarsatSafetyNetMessageData<Real> recent_messages[INMARSAT_SAFETYNET_MESSAGE_HISTORY_CAPACITY];
    Stamped<int32_t> recent_message_count;
    Stamped<int32_t> recent_message_next_index;
    Stamped<int32_t> message_count;
    Stamped<int32_t> duplicate_count;
    Stamped<int32_t> overwrite_count;
    Stamped<int32_t> unsupported_count;
    Stamped<int32_t> bad_fragment_count;
};

template<typename Real = float>
struct NotificationInmarsatData {
    InmarsatSafetyNetData<Real> safetynet;
};

template<typename Real = float>
struct SmvData {
    Setting<SensorSource> source;
    char message_id[24] = {0};
    Stamped<int32_t> mmsi;
    char mmsi_text[16] = {0};
    Stamped<Real> utc_time_s;
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    char event_type[24] = {0};
    char sar_capability[32] = {0};
    char route_id[24] = {0};
    char status = 0;
    char description[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NotificationSpecialData {
    SmvData<Real> smv;
};

template<typename Real = float>
struct NavtexReceivedMessageData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_fragments;
    Stamped<int32_t> fragment_number;
    char sentence_message_id[16] = {0};
    char navtex_message_id[8] = {0};
    char transmitter_id = 0;
    char subject_indicator = 0;
    Stamped<int32_t> subject_category;
    char subject_label[32] = {0};
    bool subject_is_service = false;
    Stamped<int32_t> serial_number;
    char message_text[192] = {0};
    char body_text[192] = {0};
    Stamped<int32_t> text_length;
    Stamped<int32_t> body_length;
    Stamped<int32_t> repeat_count;
    bool complete = false;
    bool overflow = false;
    bool end_of_message = false;
    bool framing_valid = false;
    bool duplicate = false;
    bool acknowledged = false;
    uint64_t first_seen_us = 0;
    uint64_t last_update_us = 0;
};

template<typename Real = float, uint8_t Capacity = NAVTEX_MESSAGE_HISTORY_CAPACITY>
struct NavtexMessageHistoryData {
    NavtexReceivedMessageData<Real> messages[Capacity];
    Stamped<int32_t> count;
    Stamped<int32_t> next_index;
    Stamped<int32_t> overwrite_count;
    Stamped<int32_t> duplicate_count;
};

template<typename Real = float>
struct NavtexReceiverMaskData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_fragments;
    Stamped<int32_t> fragment_number;
    char sentence_message_id[16] = {0};
    char receiver_id[24] = {0};
    char station_mask[32] = {0};
    char subject_mask[32] = {0};
    Stamped<int32_t> enabled_station_count;
    Stamped<int32_t> enabled_subject_count;
    uint32_t station_mask_bits = 0;
    uint32_t subject_mask_bits = 0;
    char status_text[72] = {0};
    bool complete = false;
    bool overflow = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NotificationNavtexData {
    NavtexReceivedMessageData<Real> received;
    NavtexMessageHistoryData<Real> history;
    NavtexReceiverMaskData<Real> receiver_mask;
};

template<typename Real = float>
struct NotificationsData {
    NotificationAlertsData<Real> alerts;
    NotificationMessagesData<Real> messages;
    NotificationDscData<Real> dsc;
    NotificationInmarsatData<Real> inmarsat;
    NotificationSpecialData<Real> special;
    NotificationNavtexData<Real> navtex;
};

template<typename Real, uint8_t Capacity>
int32_t navtex_find_message_index(
    const NavtexMessageHistoryData<Real, Capacity>& history,
    const char* navtex_message_id) {
    if (!navtex_message_id || navtex_message_id[0] == '\0') return -1;

    for (uint8_t i = 0; i < Capacity; ++i) {
        const auto& slot = history.messages[i];
        if (slot.first_seen_us != 0 && strcmp(slot.navtex_message_id, navtex_message_id) == 0) {
            return static_cast<int32_t>(i);
        }
    }

    return -1;
}

template<typename Real, uint8_t Capacity>
bool navtex_message_id_matches_latest(
    const NotificationNavtexData<Real>& navtex,
    const char* navtex_message_id) {
    return navtex_message_id &&
           navtex_message_id[0] != '\0' &&
           navtex.received.first_seen_us != 0 &&
           strcmp(navtex.received.navtex_message_id, navtex_message_id) == 0;
}

template<typename Real, uint8_t Capacity>
bool navtex_acknowledge_message(
    NotificationNavtexData<Real>& navtex,
    const char* navtex_message_id,
    uint64_t now_us) {
    const int32_t index = navtex_find_message_index<Real, Capacity>(navtex.history, navtex_message_id);
    if (index < 0) return false;

    auto& slot = navtex.history.messages[static_cast<uint8_t>(index)];
    slot.acknowledged = true;
    slot.last_update_us = now_us;

    if (navtex_message_id_matches_latest<Real, Capacity>(navtex, navtex_message_id)) {
        navtex.received.acknowledged = true;
        navtex.received.last_update_us = now_us;
    }

    return true;
}

template<typename Real, uint8_t Capacity>
bool navtex_clear_message(
    NotificationNavtexData<Real>& navtex,
    const char* navtex_message_id,
    uint64_t now_us) {
    const int32_t index = navtex_find_message_index<Real, Capacity>(navtex.history, navtex_message_id);
    if (index < 0) return false;

    navtex.history.messages[static_cast<uint8_t>(index)] = NavtexReceivedMessageData<Real>{};

    const int32_t count = navtex.history.count.valid ? navtex.history.count.value : 0;
    if (count > 0) navtex.history.count.set(count - 1, now_us);

    if (navtex_message_id_matches_latest<Real, Capacity>(navtex, navtex_message_id)) {
        navtex.received = NavtexReceivedMessageData<Real>{};
    }

    return true;
}

template<typename Real, uint8_t Capacity>
int32_t navtex_expire_history_older_than(
    NotificationNavtexData<Real>& navtex,
    uint64_t cutoff_us,
    uint64_t now_us) {
    int32_t expired = 0;
    int32_t remaining = 0;
    bool latest_expired = false;

    for (uint8_t i = 0; i < Capacity; ++i) {
        auto& slot = navtex.history.messages[i];
        if (slot.first_seen_us == 0) continue;

        const uint64_t age_reference_us = slot.last_update_us != 0 ? slot.last_update_us : slot.first_seen_us;
        if (age_reference_us < cutoff_us) {
            if (navtex_message_id_matches_latest<Real, Capacity>(navtex, slot.navtex_message_id)) {
                latest_expired = true;
            }
            slot = NavtexReceivedMessageData<Real>{};
            ++expired;
        } else {
            ++remaining;
        }
    }

    navtex.history.count.set(remaining, now_us);
    if (latest_expired) navtex.received = NavtexReceivedMessageData<Real>{};
    return expired;
}

template<typename Real>
bool navtex_acknowledge_message(
    NotificationNavtexData<Real>& navtex,
    const char* navtex_message_id,
    uint64_t now_us) {
    return navtex_acknowledge_message<Real, NAVTEX_MESSAGE_HISTORY_CAPACITY>(
        navtex,
        navtex_message_id,
        now_us);
}

template<typename Real>
bool navtex_clear_message(
    NotificationNavtexData<Real>& navtex,
    const char* navtex_message_id,
    uint64_t now_us) {
    return navtex_clear_message<Real, NAVTEX_MESSAGE_HISTORY_CAPACITY>(
        navtex,
        navtex_message_id,
        now_us);
}

template<typename Real>
int32_t navtex_expire_history_older_than(
    NotificationNavtexData<Real>& navtex,
    uint64_t cutoff_us,
    uint64_t now_us) {
    return navtex_expire_history_older_than<Real, NAVTEX_MESSAGE_HISTORY_CAPACITY>(
        navtex,
        cutoff_us,
        now_us);
}

template<typename Real>
void navtex_clear_history(NotificationNavtexData<Real>& navtex) {
    navtex.received = NavtexReceivedMessageData<Real>{};
    navtex.history = NavtexMessageHistoryData<Real>{};
}

} // namespace ship_data_model
