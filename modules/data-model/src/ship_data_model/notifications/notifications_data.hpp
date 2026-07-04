#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

static const uint8_t NAVTEX_MESSAGE_HISTORY_CAPACITY = 8;

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
struct NotificationAlertsData {
    AlertAcknowledgementData<Real> acknowledgement;
    AlertAcknowledgementDetailData<Real> acknowledgement_detail;
    AlertConditionData<Real> condition;
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
struct NotificationDscData {
    DscInterrogationData<Real> interrogation;
    DscResponseData<Real> response;
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
    NotificationNavtexData<Real> navtex;
};

} // namespace ship_data_model
