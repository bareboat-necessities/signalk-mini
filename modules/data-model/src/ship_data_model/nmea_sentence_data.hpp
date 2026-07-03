#pragma once

#include "core_data_types.hpp"

namespace ship_data_model {

static const uint8_t NMEA_MODEL_RAW_BYTES = 96;

template<typename Real = float>
struct NmeaSentenceRecordData {
    Setting<SensorSource> source;
    char talker[3] = {0, 0, 0};
    char sentence_id[4] = {0, 0, 0, 0};
    char raw[NMEA_MODEL_RAW_BYTES] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NmeaModeledSentenceData {
    NmeaSentenceRecordData<Real> ack;
    NmeaSentenceRecordData<Real> ads;
    NmeaSentenceRecordData<Real> akd;
    NmeaSentenceRecordData<Real> ala;
    NmeaSentenceRecordData<Real> asd;
    NmeaSentenceRecordData<Real> bec;
    NmeaSentenceRecordData<Real> cek;
    NmeaSentenceRecordData<Real> cop;
    NmeaSentenceRecordData<Real> dcr;
    NmeaSentenceRecordData<Real> ddc;
    NmeaSentenceRecordData<Real> dor;
    NmeaSentenceRecordData<Real> dsi;
    NmeaSentenceRecordData<Real> dsr;
    NmeaSentenceRecordData<Real> etl;
    NmeaSentenceRecordData<Real> eve;
    NmeaSentenceRecordData<Real> fir;
    NmeaSentenceRecordData<Real> wdc;
    NmeaSentenceRecordData<Real> wdr;
    NmeaSentenceRecordData<Real> zdl;
};

} // namespace ship_data_model
