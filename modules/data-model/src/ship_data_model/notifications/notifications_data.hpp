#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

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
struct NotificationsData {
    NotificationAlertsData<Real> alerts;
    NotificationMessagesData<Real> messages;
    NotificationDscData<Real> dsc;
};

} // namespace ship_data_model
