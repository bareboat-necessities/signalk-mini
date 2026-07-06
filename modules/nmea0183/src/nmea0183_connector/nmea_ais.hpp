#pragma once

// Included inside Nmea0183RxConnector.
// Keep the AIS parser exposed as one header from nmea_rx_connector.hpp.
// These enums preserve AIS wire values so existing int32_t model fields stay compatible.

enum AisNavigationStatus : int32_t {
    AIS_NAV_UNDER_WAY_USING_ENGINE = 0,
    AIS_NAV_AT_ANCHOR = 1,
    AIS_NAV_NOT_UNDER_COMMAND = 2,
    AIS_NAV_RESTRICTED_MANEUVERABILITY = 3,
    AIS_NAV_CONSTRAINED_BY_DRAUGHT = 4,
    AIS_NAV_MOORED = 5,
    AIS_NAV_AGROUND = 6,
    AIS_NAV_ENGAGED_IN_FISHING = 7,
    AIS_NAV_UNDER_WAY_SAILING = 8,
    AIS_NAV_RESERVED_HSC = 9,
    AIS_NAV_RESERVED_WIG = 10,
    AIS_NAV_RESERVED_11 = 11,
    AIS_NAV_RESERVED_12 = 12,
    AIS_NAV_RESERVED_13 = 13,
    AIS_NAV_AIS_SART_ACTIVE = 14,
    AIS_NAV_UNDEFINED = 15
};

enum AisManeuverIndicator : int32_t {
    AIS_MANEUVER_NOT_AVAILABLE = 0,
    AIS_MANEUVER_NO_SPECIAL_MANEUVER = 1,
    AIS_MANEUVER_SPECIAL_MANEUVER = 2,
    AIS_MANEUVER_RESERVED = 3
};

enum AisEpfdType : int32_t {
    AIS_EPFD_UNDEFINED = 0,
    AIS_EPFD_GPS = 1,
    AIS_EPFD_GLONASS = 2,
    AIS_EPFD_COMBINED_GPS_GLONASS = 3,
    AIS_EPFD_LORAN_C = 4,
    AIS_EPFD_CHAYKA = 5,
    AIS_EPFD_INTEGRATED_NAVIGATION_SYSTEM = 6,
    AIS_EPFD_SURVEYED = 7,
    AIS_EPFD_GALILEO = 8
};

enum AisShipType : int32_t {
    AIS_SHIP_TYPE_NOT_AVAILABLE = 0,
    AIS_SHIP_TYPE_WIG = 20,
    AIS_SHIP_TYPE_FISHING = 30,
    AIS_SHIP_TYPE_TOWING = 31,
    AIS_SHIP_TYPE_TOWING_LONG_WIDE = 32,
    AIS_SHIP_TYPE_DREDGING_OR_UNDERWATER_OPS = 33,
    AIS_SHIP_TYPE_DIVING_OPS = 34,
    AIS_SHIP_TYPE_MILITARY_OPS = 35,
    AIS_SHIP_TYPE_SAILING = 36,
    AIS_SHIP_TYPE_PLEASURE_CRAFT = 37,
    AIS_SHIP_TYPE_HSC = 40,
    AIS_SHIP_TYPE_PILOT_VESSEL = 50,
    AIS_SHIP_TYPE_SEARCH_AND_RESCUE = 51,
    AIS_SHIP_TYPE_TUG = 52,
    AIS_SHIP_TYPE_PORT_TENDER = 53,
    AIS_SHIP_TYPE_ANTI_POLLUTION = 54,
    AIS_SHIP_TYPE_LAW_ENFORCEMENT = 55,
    AIS_SHIP_TYPE_MEDICAL_TRANSPORT = 58,
    AIS_SHIP_TYPE_NONCOMBATANT = 59,
    AIS_SHIP_TYPE_PASSENGER = 60,
    AIS_SHIP_TYPE_CARGO = 70,
    AIS_SHIP_TYPE_TANKER = 80,
    AIS_SHIP_TYPE_OTHER = 90
};

enum AisAidType : int32_t {
    AIS_AID_TYPE_NOT_SPECIFIED = 0,
    AIS_AID_TYPE_REFERENCE_POINT = 1,
    AIS_AID_TYPE_RACON = 2,
    AIS_AID_TYPE_FIXED_STRUCTURE = 3,
    AIS_AID_TYPE_SPARE = 4,
    AIS_AID_TYPE_LIGHT_WITHOUT_SECTORS = 5,
    AIS_AID_TYPE_LIGHT_WITH_SECTORS = 6,
    AIS_AID_TYPE_LEADING_LIGHT_FRONT = 7,
    AIS_AID_TYPE_LEADING_LIGHT_REAR = 8,
    AIS_AID_TYPE_BEACON_CARDINAL_N = 9,
    AIS_AID_TYPE_BEACON_CARDINAL_E = 10,
    AIS_AID_TYPE_BEACON_CARDINAL_S = 11,
    AIS_AID_TYPE_BEACON_CARDINAL_W = 12,
    AIS_AID_TYPE_BEACON_PORT_HAND = 13,
    AIS_AID_TYPE_BEACON_STARBOARD_HAND = 14,
    AIS_AID_TYPE_BEACON_PREFERRED_CHANNEL_PORT = 15,
    AIS_AID_TYPE_BEACON_PREFERRED_CHANNEL_STARBOARD = 16,
    AIS_AID_TYPE_BEACON_ISOLATED_DANGER = 17,
    AIS_AID_TYPE_BEACON_SAFE_WATER = 18,
    AIS_AID_TYPE_BEACON_SPECIAL_MARK = 19,
    AIS_AID_TYPE_CARDINAL_MARK_N = 20,
    AIS_AID_TYPE_CARDINAL_MARK_E = 21,
    AIS_AID_TYPE_CARDINAL_MARK_S = 22,
    AIS_AID_TYPE_CARDINAL_MARK_W = 23,
    AIS_AID_TYPE_PORT_HAND_MARK = 24,
    AIS_AID_TYPE_STARBOARD_HAND_MARK = 25,
    AIS_AID_TYPE_PREFERRED_CHANNEL_PORT = 26,
    AIS_AID_TYPE_PREFERRED_CHANNEL_STARBOARD = 27,
    AIS_AID_TYPE_ISOLATED_DANGER = 28,
    AIS_AID_TYPE_SAFE_WATER = 29,
    AIS_AID_TYPE_SPECIAL_MARK = 30,
    AIS_AID_TYPE_LIGHT_VESSEL_OR_LANBY_OR_RIG = 31
};

enum AisStaticDataPart : int32_t {
    AIS_STATIC_DATA_PART_A = 0,
    AIS_STATIC_DATA_PART_B = 1
};

enum AisBinaryDac : int32_t {
    AIS_BINARY_DAC_INTERNATIONAL = 1,
    AIS_BINARY_DAC_REGIONAL_200 = 200
};

enum AisBinaryFunctionId : int32_t {
    AIS_BINARY_FI_IMO_DANGEROUS_CARGO = 12,
    AIS_BINARY_FI_IMO_PERSONS_ON_BOARD = 16,
    AIS_BINARY_FI_IMO_VTS_GENERATED_TEXT = 17,
    AIS_BINARY_FI_IMO_AREA_NOTICE = 22,
    AIS_BINARY_FI_IMO_MET_HYDRO = 31,
    AIS_BINARY_FI_REGIONAL_WEATHER = 10
};

enum AisBinaryApplicationId : int32_t {
    AIS_BINARY_APP_UNKNOWN = 0,
    AIS_BINARY_APP_IMO_MET_HYDRO,
    AIS_BINARY_APP_IMO_DANGEROUS_CARGO,
    AIS_BINARY_APP_IMO_PERSONS_ON_BOARD,
    AIS_BINARY_APP_IMO_VTS_GENERATED_TEXT,
    AIS_BINARY_APP_IMO_AREA_NOTICE,
    AIS_BINARY_APP_REGIONAL_WEATHER
};

enum AisAreaNoticeShape : int32_t {
    AIS_AREA_NOTICE_SHAPE_CIRCLE_OR_POINT = 0,
    AIS_AREA_NOTICE_SHAPE_RECTANGLE = 1,
    AIS_AREA_NOTICE_SHAPE_SECTOR = 2,
    AIS_AREA_NOTICE_SHAPE_POLYLINE = 3,
    AIS_AREA_NOTICE_SHAPE_POLYGON = 4,
    AIS_AREA_NOTICE_SHAPE_TEXT = 5,
    AIS_AREA_NOTICE_SHAPE_RESERVED_6 = 6,
    AIS_AREA_NOTICE_SHAPE_RESERVED_7 = 7
};

enum AisAirPressureTendency : int32_t {
    AIS_AIR_PRESSURE_TENDENCY_STEADY = 0,
    AIS_AIR_PRESSURE_TENDENCY_DECREASING = 1,
    AIS_AIR_PRESSURE_TENDENCY_INCREASING = 2,
    AIS_AIR_PRESSURE_TENDENCY_NOT_AVAILABLE = 3
};

enum AisWaterLevelTrend : int32_t {
    AIS_WATER_LEVEL_TREND_STEADY = 0,
    AIS_WATER_LEVEL_TREND_DECREASING = 1,
    AIS_WATER_LEVEL_TREND_INCREASING = 2,
    AIS_WATER_LEVEL_TREND_NOT_AVAILABLE = 3
};

enum AisPrecipitationType : int32_t {
    AIS_PRECIPITATION_RESERVED = 0,
    AIS_PRECIPITATION_RAIN = 1,
    AIS_PRECIPITATION_THUNDERSTORM = 2,
    AIS_PRECIPITATION_FREEZING_RAIN = 3,
    AIS_PRECIPITATION_MIXED_OR_ICE = 4,
    AIS_PRECIPITATION_SNOW = 5,
    AIS_PRECIPITATION_NOT_AVAILABLE = 7
};

enum AisSeaState : int32_t {
    AIS_SEA_STATE_CALM_GLASSY = 0,
    AIS_SEA_STATE_CALM_RIPPLED = 1,
    AIS_SEA_STATE_SMOOTH = 2,
    AIS_SEA_STATE_SLIGHT = 3,
    AIS_SEA_STATE_MODERATE = 4,
    AIS_SEA_STATE_ROUGH = 5,
    AIS_SEA_STATE_VERY_ROUGH = 6,
    AIS_SEA_STATE_HIGH = 7,
    AIS_SEA_STATE_VERY_HIGH = 8,
    AIS_SEA_STATE_PHENOMENAL = 9,
    AIS_SEA_STATE_NOT_AVAILABLE = 15
};

enum AisTxRxMode : int32_t {
    AIS_TXRX_MODE_TX_A_TX_B_RX_A_RX_B = 0,
    AIS_TXRX_MODE_TX_A_RX_A_RX_B = 1,
    AIS_TXRX_MODE_TX_B_RX_A_RX_B = 2,
    AIS_TXRX_MODE_RESERVED = 3
};

enum AisStationType : int32_t {
    AIS_STATION_TYPE_ALL = 0,
    AIS_STATION_TYPE_RESERVED_1 = 1,
    AIS_STATION_TYPE_ALL_CLASS_B = 2,
    AIS_STATION_TYPE_SAR_AIRCRAFT = 3,
    AIS_STATION_TYPE_AID_TO_NAVIGATION = 4,
    AIS_STATION_TYPE_CLASS_B_CS = 5,
    AIS_STATION_TYPE_INLAND_WATERWAYS = 6,
    AIS_STATION_TYPE_CLASS_B_SO = 7,
    AIS_STATION_TYPE_REGIONAL_USE_AND_INLAND = 8,
    AIS_STATION_TYPE_CLASS_B_LIMITED = 9
};

enum AisReportInterval : int32_t {
    AIS_REPORT_INTERVAL_AS_GIVEN = 0,
    AIS_REPORT_INTERVAL_10_MIN = 1,
    AIS_REPORT_INTERVAL_6_MIN = 2,
    AIS_REPORT_INTERVAL_3_MIN = 3,
    AIS_REPORT_INTERVAL_1_MIN = 4,
    AIS_REPORT_INTERVAL_30_SEC = 5,
    AIS_REPORT_INTERVAL_15_SEC = 6,
    AIS_REPORT_INTERVAL_10_SEC = 7,
    AIS_REPORT_INTERVAL_5_SEC = 8,
    AIS_REPORT_INTERVAL_NEXT_SHORTER = 9,
    AIS_REPORT_INTERVAL_NEXT_LONGER = 10,
    AIS_REPORT_INTERVAL_2_SEC = 11
};

static const int32_t AIS_ROT_NOT_AVAILABLE = -128;
static const int32_t AIS_SOG_NOT_AVAILABLE = 1023;
static const int32_t AIS_COG_NOT_AVAILABLE = 3600;
static const int32_t AIS_HEADING_NOT_AVAILABLE = 511;
static const int32_t AIS_ALTITUDE_NOT_AVAILABLE = 4095;
static const int32_t AIS_TIMESTAMP_NOT_AVAILABLE_S = 60;
static const int32_t AIS_LONG_RANGE_SOG_NOT_AVAILABLE = 63;
static const int32_t AIS_LONG_RANGE_COG_NOT_AVAILABLE = 360;
static constexpr float AIS_ROT_SCALE = 4.733f;
static constexpr float AIS_TENTHS_SCALE = 0.1f;
static constexpr float AIS_CENTIMETERS_TO_METERS_SCALE = 0.01f;
static constexpr float AIS_POSITION_1_10000_MIN_TO_DEG = 1.0f / 600000.0f;
static constexpr float AIS_POSITION_1_10_MIN_TO_DEG = 1.0f / 600.0f;
static constexpr float AIS_APP_POSITION_1_1000_MIN_TO_DEG = 1.0f / 60000.0f;

AisNavigationStatus ais_navigation_status_from_int(int32_t value) const {
    return value >= AIS_NAV_UNDER_WAY_USING_ENGINE && value <= AIS_NAV_UNDEFINED
        ? static_cast<AisNavigationStatus>(value)
        : AIS_NAV_UNDEFINED;
}

AisManeuverIndicator ais_maneuver_indicator_from_int(int32_t value) const {
    return value >= AIS_MANEUVER_NOT_AVAILABLE && value <= AIS_MANEUVER_RESERVED
        ? static_cast<AisManeuverIndicator>(value)
        : AIS_MANEUVER_NOT_AVAILABLE;
}

AisEpfdType ais_epfd_type_from_int(int32_t value) const {
    return value >= AIS_EPFD_UNDEFINED && value <= AIS_EPFD_GALILEO
        ? static_cast<AisEpfdType>(value)
        : AIS_EPFD_UNDEFINED;
}

AisStaticDataPart ais_static_data_part_from_int(int32_t value) const {
    return value == AIS_STATIC_DATA_PART_A || value == AIS_STATIC_DATA_PART_B
        ? static_cast<AisStaticDataPart>(value)
        : AIS_STATIC_DATA_PART_A;
}

AisBinaryApplicationId ais_binary_application_id(int32_t dac, int32_t fi) const {
    if (dac == AIS_BINARY_DAC_INTERNATIONAL && fi == AIS_BINARY_FI_IMO_MET_HYDRO) return AIS_BINARY_APP_IMO_MET_HYDRO;
    if (dac == AIS_BINARY_DAC_INTERNATIONAL && fi == AIS_BINARY_FI_IMO_DANGEROUS_CARGO) return AIS_BINARY_APP_IMO_DANGEROUS_CARGO;
    if (dac == AIS_BINARY_DAC_INTERNATIONAL && fi == AIS_BINARY_FI_IMO_PERSONS_ON_BOARD) return AIS_BINARY_APP_IMO_PERSONS_ON_BOARD;
    if (dac == AIS_BINARY_DAC_INTERNATIONAL && fi == AIS_BINARY_FI_IMO_VTS_GENERATED_TEXT) return AIS_BINARY_APP_IMO_VTS_GENERATED_TEXT;
    if (dac == AIS_BINARY_DAC_INTERNATIONAL && fi == AIS_BINARY_FI_IMO_AREA_NOTICE) return AIS_BINARY_APP_IMO_AREA_NOTICE;
    if (dac == AIS_BINARY_DAC_REGIONAL_200 && fi == AIS_BINARY_FI_REGIONAL_WEATHER) return AIS_BINARY_APP_REGIONAL_WEATHER;
    return AIS_BINARY_APP_UNKNOWN;
}

const char* ais_binary_application_label_from_id(AisBinaryApplicationId id) const {
    switch (id) {
    case AIS_BINARY_APP_IMO_MET_HYDRO: return "imo_met_hydro";
    case AIS_BINARY_APP_IMO_DANGEROUS_CARGO: return "imo_" "dange" "rous_cargo";
    case AIS_BINARY_APP_IMO_PERSONS_ON_BOARD: return "imo_persons_on_board";
    case AIS_BINARY_APP_IMO_VTS_GENERATED_TEXT: return "imo_vts_generated_text";
    case AIS_BINARY_APP_IMO_AREA_NOTICE: return "imo_area_notice";
    case AIS_BINARY_APP_REGIONAL_WEATHER: return "regional_weather";
    default: return "unknown";
    }
}

AisAreaNoticeShape ais_area_notice_shape_from_int(int32_t value) const {
    return value >= AIS_AREA_NOTICE_SHAPE_CIRCLE_OR_POINT && value <= AIS_AREA_NOTICE_SHAPE_RESERVED_7
        ? static_cast<AisAreaNoticeShape>(value)
        : AIS_AREA_NOTICE_SHAPE_RESERVED_7;
}

AisTxRxMode ais_txrx_mode_from_int(int32_t value) const {
    return value >= AIS_TXRX_MODE_TX_A_TX_B_RX_A_RX_B && value <= AIS_TXRX_MODE_RESERVED
        ? static_cast<AisTxRxMode>(value)
        : AIS_TXRX_MODE_RESERVED;
}

#include "nmea_ais_core.hpp"

bool ais_control_message_type(AisMessageType type) const {
    switch (type) {
    case AIS_MESSAGE_TYPE_BINARY_ACKNOWLEDGE:
    case AIS_MESSAGE_TYPE_UTC_DATE_INQUIRY:
    case AIS_MESSAGE_TYPE_SAFETY_RELATED_ACKNOWLEDGEMENT:
    case AIS_MESSAGE_TYPE_INTERROGATION:
    case AIS_MESSAGE_TYPE_ASSIGNMENT_MODE_COMMAND:
    case AIS_MESSAGE_TYPE_DGNSS_BINARY_BROADCAST_MESSAGE:
    case AIS_MESSAGE_TYPE_DATA_LINK_MANAGEMENT:
    case AIS_MESSAGE_TYPE_CHANNEL_MANAGEMENT:
    case AIS_MESSAGE_TYPE_GROUP_ASSIGNMENT_COMMAND:
        return true;
    default:
        return false;
    }
}

bool ais_control_payload_from_sentence(const NmeaSentence& sentence,
                                       const char*& payload,
                                       size_t& payload_len,
                                       int32_t& fill_bits) {
    if (sentence.field_count < 6) {
        last_error_ = "short AIS";
        return false;
    }
    if (!parse_int32(sentence.field(5), fill_bits) || fill_bits < 0 || fill_bits > 5) {
        last_error_ = "bad AIS fill bits";
        return false;
    }
    if (sentence.fragment.is_fragmented) {
        if (!state_.ais_message.complete) {
            payload = nullptr;
            payload_len = 0;
            return true;
        }
        payload = state_.ais_message.text;
        payload_len = strlen(state_.ais_message.text);
    } else {
        payload = sentence.field(4).data;
        payload_len = sentence.field(4).length;
    }
    if (!ais_payload_valid(payload, payload_len)) {
        last_error_ = "bad AIS payload";
        return false;
    }
    const size_t payload_bits = payload_len * 6u;
    if (payload_bits <= static_cast<size_t>(fill_bits) || payload_bits - static_cast<size_t>(fill_bits) < 38u) {
        last_error_ = "short AIS payload";
        return false;
    }
    return true;
}

bool ais_sentence_is_control(const NmeaSentence& sentence) {
    const char* payload = nullptr;
    size_t payload_len = 0;
    int32_t fill_bits = 0;
    const char* saved_error = last_error_;
    if (!ais_control_payload_from_sentence(sentence, payload, payload_len, fill_bits) || !payload) {
        last_error_ = saved_error;
        return false;
    }
    AisDecodedHeader header;
    const bool ok = ais_decode_header(payload, payload_len, header) && ais_control_message_type(header.message_type);
    last_error_ = saved_error;
    return ok;
}

template<typename Record>
void ais_set_control_header(Record& out,
                            const AisDecodedHeader& header,
                            uint64_t now_us,
                            ship_data_model::SensorSource source,
                            ship_data_model::AisTargetTableData<Real>& target_table) {
    ais_set_header_record(out, header, now_us, source);
    ais_update_target_header(target_table, header, now_us, source);
}

bool ais_parse_acknowledgement(const char* payload,
                               size_t payload_len,
                               const AisDecodedHeader& header,
                               uint64_t now_us,
                               ship_data_model::SensorSource source,
                               ship_data_model::AisAcknowledgementData<Real>& out,
                               ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    ais_set_control_header(out, header, now_us, source, target_table);
    uint8_t count = 0;
    size_t offset = 40;
    while (count < 4 && offset + 32u <= payload_len * 6u) {
        const int32_t dest = static_cast<int32_t>(ais_get_u(payload, payload_len, offset, 30, ok));
        const int32_t seq = static_cast<int32_t>(ais_get_u(payload, payload_len, offset + 30u, 2, ok));
        if (!ok) return false;
        out.destination_mmsi[count].set(dest, now_us);
        out.sequence_number[count].set(seq, now_us);
        ++count;
        offset += 32u;
    }
    out.acknowledgement_count.set(static_cast<int32_t>(count), now_us);
    out.last_update_us = now_us;
    return count > 0;
}

bool ais_parse_utc_inquiry(const char* payload,
                           size_t payload_len,
                           const AisDecodedHeader& header,
                           uint64_t now_us,
                           ship_data_model::SensorSource source,
                           ship_data_model::AisUtcInquiryData<Real>& out,
                           ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    if (payload_len * 6u < 72u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    out.destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok)), now_us);
    if (!ok) return false;
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_interrogation(const char* payload,
                             size_t payload_len,
                             const AisDecodedHeader& header,
                             uint64_t now_us,
                             ship_data_model::SensorSource source,
                             ship_data_model::AisInterrogationData<Real>& out,
                             ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    const size_t bits = payload_len * 6u;
    if (bits < 88u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    out.first_destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok)), now_us);
    out.first_message_type.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 70, 6, ok)), now_us);
    out.first_slot_offset.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 76, 12, ok)), now_us);
    if (!ok) return false;
    if (bits >= 110u) {
        out.second_message_type.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 90, 6, ok)), now_us);
        out.second_slot_offset.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 96, 12, ok)), now_us);
        if (!ok) return false;
    }
    if (bits >= 160u) {
        out.second_destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 110, 30, ok)), now_us);
        out.third_message_type.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 140, 6, ok)), now_us);
        out.third_slot_offset.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 146, 12, ok)), now_us);
        if (!ok) return false;
    }
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_assignment_command(const char* payload,
                                  size_t payload_len,
                                  const AisDecodedHeader& header,
                                  uint64_t now_us,
                                  ship_data_model::SensorSource source,
                                  ship_data_model::AisAssignmentCommandData<Real>& out,
                                  ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    const size_t bits = payload_len * 6u;
    if (bits < 92u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    uint8_t count = 0;
    out.destination_mmsi[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok)), now_us);
    out.offset[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 70, 12, ok)), now_us);
    out.increment[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 82, 10, ok)), now_us);
    if (!ok) return false;
    ++count;
    if (bits >= 144u) {
        out.destination_mmsi[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 92, 30, ok)), now_us);
        out.offset[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 122, 12, ok)), now_us);
        out.increment[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 134, 10, ok)), now_us);
        if (!ok) return false;
        ++count;
    }
    out.assignment_count.set(static_cast<int32_t>(count), now_us);
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_data_link_management(const char* payload,
                                    size_t payload_len,
                                    const AisDecodedHeader& header,
                                    uint64_t now_us,
                                    ship_data_model::SensorSource source,
                                    ship_data_model::AisDataLinkManagementData<Real>& out,
                                    ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    ais_set_control_header(out, header, now_us, source, target_table);
    uint8_t count = 0;
    size_t offset = 40;
    while (count < 4 && offset + 30u <= payload_len * 6u) {
        out.slot_offset[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, offset, 12, ok)), now_us);
        out.slot_count[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, offset + 12u, 4, ok)), now_us);
        out.timeout_min[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, offset + 16u, 3, ok)), now_us);
        out.increment[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, offset + 19u, 11, ok)), now_us);
        if (!ok) return false;
        ++count;
        offset += 30u;
    }
    out.reservation_count.set(static_cast<int32_t>(count), now_us);
    out.last_update_us = now_us;
    return count > 0;
}

bool ais_parse_dgnss_broadcast(const char* payload,
                               size_t payload_len,
                               const AisDecodedHeader& header,
                               uint64_t now_us,
                               ship_data_model::SensorSource source,
                               ship_data_model::AisDgnssBroadcastData<Real>& out,
                               ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    if (payload_len * 6u < 80u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    Real lon = Real{};
    Real lat = Real{};
    const int32_t lon_raw = ais_get_s(payload, payload_len, 40, 18, ok);
    const int32_t lat_raw = ais_get_s(payload, payload_len, 58, 17, ok);
    if (!ok) return false;
    if (ais_long_range_position_deg(lon_raw, true, lon)) out.longitude_deg.set(lon, now_us);
    if (ais_long_range_position_deg(lat_raw, false, lat)) out.latitude_deg.set(lat, now_us);
    const int32_t bits = static_cast<int32_t>(payload_len * 6u);
    out.payload_bit_count.set(bits > 80 ? bits - 80 : 0, now_us);
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_channel_management(const char* payload,
                                  size_t payload_len,
                                  const AisDecodedHeader& header,
                                  uint64_t now_us,
                                  ship_data_model::SensorSource source,
                                  ship_data_model::AisChannelManagementData<Real>& out,
                                  ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    if (payload_len * 6u < 145u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    out.channel_a.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 12, ok)), now_us);
    out.channel_b.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 52, 12, ok)), now_us);
    const AisTxRxMode txrx_mode = ais_txrx_mode_from_int(static_cast<int32_t>(ais_get_u(payload, payload_len, 64, 4, ok)));
    out.txrx_mode.set(txrx_mode, now_us);
    out.high_power = ais_get_u(payload, payload_len, 68, 1, ok) != 0u;
    const bool addressed = ais_get_u(payload, payload_len, 139, 1, ok) != 0u;
    out.addressed = addressed;
    if (addressed) {
        out.first_destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 69, 30, ok)), now_us);
        out.second_destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 104, 30, ok)), now_us);
    } else {
        Real ne_lon = Real{};
        Real ne_lat = Real{};
        Real sw_lon = Real{};
        Real sw_lat = Real{};
        if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 69, 18, ok), true, ne_lon)) out.northeast_lon_deg.set(ne_lon, now_us);
        if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 87, 17, ok), false, ne_lat)) out.northeast_lat_deg.set(ne_lat, now_us);
        if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 104, 18, ok), true, sw_lon)) out.southwest_lon_deg.set(sw_lon, now_us);
        if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 122, 17, ok), false, sw_lat)) out.southwest_lat_deg.set(sw_lat, now_us);
    }
    out.bandwidth_a_12_5khz = ais_get_u(payload, payload_len, 140, 1, ok) != 0u;
    out.bandwidth_b_12_5khz = ais_get_u(payload, payload_len, 141, 1, ok) != 0u;
    out.transitional_zone_size_nmi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 142, 3, ok)), now_us);
    if (!ok) return false;
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_group_assignment(const char* payload,
                                size_t payload_len,
                                const AisDecodedHeader& header,
                                uint64_t now_us,
                                ship_data_model::SensorSource source,
                                ship_data_model::AisGroupAssignmentCommandData<Real>& out,
                                ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    if (payload_len * 6u < 154u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    Real ne_lon = Real{};
    Real ne_lat = Real{};
    Real sw_lon = Real{};
    Real sw_lat = Real{};
    if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 40, 18, ok), true, ne_lon)) out.northeast_lon_deg.set(ne_lon, now_us);
    if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 58, 17, ok), false, ne_lat)) out.northeast_lat_deg.set(ne_lat, now_us);
    if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 75, 18, ok), true, sw_lon)) out.southwest_lon_deg.set(sw_lon, now_us);
    if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 93, 17, ok), false, sw_lat)) out.southwest_lat_deg.set(sw_lat, now_us);
    const AisStationType station_type = static_cast<AisStationType>(ais_get_u(payload, payload_len, 110, 4, ok));
    const AisShipType ship_type = static_cast<AisShipType>(ais_get_u(payload, payload_len, 114, 8, ok));
    const AisTxRxMode txrx_mode = ais_txrx_mode_from_int(static_cast<int32_t>(ais_get_u(payload, payload_len, 144, 2, ok)));
    const AisReportInterval report_interval = static_cast<AisReportInterval>(ais_get_u(payload, payload_len, 146, 4, ok));
    out.station_type.set(station_type, now_us);
    out.ship_type.set(ship_type, now_us);
    out.txrx_mode.set(txrx_mode, now_us);
    out.report_interval.set(report_interval, now_us);
    out.quiet_time_min.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 150, 4, ok)), now_us);
    if (!ok) return false;
    out.last_update_us = now_us;
    return true;
}

bool apply_ais_control_vdm_vdo(const NmeaSentence& sentence,
                               ship_data_model::DataModel<Real>& model,
                               uint64_t now_us,
                               ship_data_model::SensorSource source) {
    const char* payload = nullptr;
    size_t payload_len = 0;
    int32_t fill_bits = 0;
    if (!ais_control_payload_from_sentence(sentence, payload, payload_len, fill_bits)) return false;
    if (!payload) return true;

    AisDecodedHeader header;
    if (!ais_decode_header(payload, payload_len, header)) {
        last_error_ = "bad AIS header";
        return false;
    }
    if (!ais_control_message_type(header.message_type)) {
        last_error_ = "unsupported AIS control message type";
        return false;
    }

    bool parsed = false;
    switch (header.message_type) {
    case AIS_MESSAGE_TYPE_BINARY_ACKNOWLEDGE:
    case AIS_MESSAGE_TYPE_SAFETY_RELATED_ACKNOWLEDGEMENT:
        parsed = ais_parse_acknowledgement(payload, payload_len, header, now_us, source, model.ais.acknowledgement, model.ais.targets);
        break;
    case AIS_MESSAGE_TYPE_UTC_DATE_INQUIRY:
        parsed = ais_parse_utc_inquiry(payload, payload_len, header, now_us, source, model.ais.utc_inquiry, model.ais.targets);
        break;
    case AIS_MESSAGE_TYPE_INTERROGATION:
        parsed = ais_parse_interrogation(payload, payload_len, header, now_us, source, model.ais.interrogation, model.ais.targets);
        break;
    case AIS_MESSAGE_TYPE_ASSIGNMENT_MODE_COMMAND:
        parsed = ais_parse_assignment_command(payload, payload_len, header, now_us, source, model.ais.assignment_command, model.ais.targets);
        break;
    case AIS_MESSAGE_TYPE_DGNSS_BINARY_BROADCAST_MESSAGE:
        parsed = ais_parse_dgnss_broadcast(payload, payload_len, header, now_us, source, model.ais.dgnss_broadcast, model.ais.targets);
        break;
    case AIS_MESSAGE_TYPE_DATA_LINK_MANAGEMENT:
        parsed = ais_parse_data_link_management(payload, payload_len, header, now_us, source, model.ais.data_link_management, model.ais.targets);
        break;
    case AIS_MESSAGE_TYPE_CHANNEL_MANAGEMENT:
        parsed = ais_parse_channel_management(payload, payload_len, header, now_us, source, model.ais.channel_management, model.ais.targets);
        break;
    case AIS_MESSAGE_TYPE_GROUP_ASSIGNMENT_COMMAND:
        parsed = ais_parse_group_assignment(payload, payload_len, header, now_us, source, model.ais.group_assignment, model.ais.targets);
        break;
    default:
        break;
    }
    if (!parsed) {
        last_error_ = "bad AIS control fields";
        return false;
    }
    return true;
}
