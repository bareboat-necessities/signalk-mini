#pragma once

#include <stdint.h>

#include <nmea0183_connector.hpp>
#include <seatalk.hpp>
#include <ship_data_model.hpp>

#include "model_store.hpp"
#include "seatalk_change_mapper.hpp"
#include "types.hpp"

namespace signalk_mini {

enum class NmeaSentenceId : uint16_t {
    Unknown = 0,
    R00, RMC, GGA, GLL, GNS, ZDA, VTG, GSA, GFA, GSV,
    HDT, HDM, HDG, XDR,
    MWV, MWD, VWR, VWT, MDA,
    DBT, DPT, DBK, DBS, MTW, VBW, VHW, VPW, LWY, VDR, CUR,
    RPM, RSA, VLW,
    AAM, APA, APB, RMB, BOD, BWW, BEC, BWC, BWR, WPL, WCV, WDC, WDR, WNC, XTE, XTR, RTE, HSC, ZDL, ZFO, ZTG,
    HFB, TDS, TPC, TPR, TPT,
    VDM, VDO, ABK, ADS, AGA, BCL, ASD,
    DSC, DSE, NRX, NRM,
    SM1, SM2, SM3, SM4, SMB, CAN, CRQ, DSM, TMD,
    ACK, AKD, ALA, ALC, ALF, ALR, ARC, HBT, FIR,
    MOB, SMV,
    FSI, MSK, MSS, RLM, CEK, COP, DCR, DDC, DOR,
    TXT, EVE, ETL,
};

inline constexpr uint32_t nmea_sentence_code(char a, char b, char c) {
    return (static_cast<uint32_t>(static_cast<uint8_t>(a)) << 16) |
           (static_cast<uint32_t>(static_cast<uint8_t>(b)) << 8) |
           static_cast<uint32_t>(static_cast<uint8_t>(c));
}

inline uint32_t nmea_sentence_code(const nmea0183_connector::NmeaSentence& sentence) {
    if (!sentence.sentence.data || sentence.sentence.length != 3) return 0;
    return nmea_sentence_code(sentence.sentence[0], sentence.sentence[1], sentence.sentence[2]);
}

inline NmeaSentenceId classify_nmea_sentence_id(const nmea0183_connector::NmeaSentence& sentence) {
    switch (nmea_sentence_code(sentence)) {
    case nmea_sentence_code('R','0','0'): return NmeaSentenceId::R00;
    case nmea_sentence_code('R','M','C'): return NmeaSentenceId::RMC;
    case nmea_sentence_code('G','G','A'): return NmeaSentenceId::GGA;
    case nmea_sentence_code('G','L','L'): return NmeaSentenceId::GLL;
    case nmea_sentence_code('G','N','S'): return NmeaSentenceId::GNS;
    case nmea_sentence_code('Z','D','A'): return NmeaSentenceId::ZDA;
    case nmea_sentence_code('V','T','G'): return NmeaSentenceId::VTG;
    case nmea_sentence_code('G','S','A'): return NmeaSentenceId::GSA;
    case nmea_sentence_code('G','F','A'): return NmeaSentenceId::GFA;
    case nmea_sentence_code('G','S','V'): return NmeaSentenceId::GSV;
    case nmea_sentence_code('H','D','T'): return NmeaSentenceId::HDT;
    case nmea_sentence_code('H','D','M'): return NmeaSentenceId::HDM;
    case nmea_sentence_code('H','D','G'): return NmeaSentenceId::HDG;
    case nmea_sentence_code('X','D','R'): return NmeaSentenceId::XDR;
    case nmea_sentence_code('M','W','V'): return NmeaSentenceId::MWV;
    case nmea_sentence_code('M','W','D'): return NmeaSentenceId::MWD;
    case nmea_sentence_code('V','W','R'): return NmeaSentenceId::VWR;
    case nmea_sentence_code('V','W','T'): return NmeaSentenceId::VWT;
    case nmea_sentence_code('M','D','A'): return NmeaSentenceId::MDA;
    case nmea_sentence_code('D','B','T'): return NmeaSentenceId::DBT;
    case nmea_sentence_code('D','P','T'): return NmeaSentenceId::DPT;
    case nmea_sentence_code('D','B','K'): return NmeaSentenceId::DBK;
    case nmea_sentence_code('D','B','S'): return NmeaSentenceId::DBS;
    case nmea_sentence_code('M','T','W'): return NmeaSentenceId::MTW;
    case nmea_sentence_code('V','B','W'): return NmeaSentenceId::VBW;
    case nmea_sentence_code('V','H','W'): return NmeaSentenceId::VHW;
    case nmea_sentence_code('V','P','W'): return NmeaSentenceId::VPW;
    case nmea_sentence_code('L','W','Y'): return NmeaSentenceId::LWY;
    case nmea_sentence_code('V','D','R'): return NmeaSentenceId::VDR;
    case nmea_sentence_code('C','U','R'): return NmeaSentenceId::CUR;
    case nmea_sentence_code('R','P','M'): return NmeaSentenceId::RPM;
    case nmea_sentence_code('R','S','A'): return NmeaSentenceId::RSA;
    case nmea_sentence_code('V','L','W'): return NmeaSentenceId::VLW;
    case nmea_sentence_code('A','A','M'): return NmeaSentenceId::AAM;
    case nmea_sentence_code('A','P','A'): return NmeaSentenceId::APA;
    case nmea_sentence_code('A','P','B'): return NmeaSentenceId::APB;
    case nmea_sentence_code('R','M','B'): return NmeaSentenceId::RMB;
    case nmea_sentence_code('B','O','D'): return NmeaSentenceId::BOD;
    case nmea_sentence_code('B','W','W'): return NmeaSentenceId::BWW;
    case nmea_sentence_code('B','E','C'): return NmeaSentenceId::BEC;
    case nmea_sentence_code('B','W','C'): return NmeaSentenceId::BWC;
    case nmea_sentence_code('B','W','R'): return NmeaSentenceId::BWR;
    case nmea_sentence_code('W','P','L'): return NmeaSentenceId::WPL;
    case nmea_sentence_code('W','C','V'): return NmeaSentenceId::WCV;
    case nmea_sentence_code('W','D','C'): return NmeaSentenceId::WDC;
    case nmea_sentence_code('W','D','R'): return NmeaSentenceId::WDR;
    case nmea_sentence_code('W','N','C'): return NmeaSentenceId::WNC;
    case nmea_sentence_code('X','T','E'): return NmeaSentenceId::XTE;
    case nmea_sentence_code('X','T','R'): return NmeaSentenceId::XTR;
    case nmea_sentence_code('R','T','E'): return NmeaSentenceId::RTE;
    case nmea_sentence_code('H','S','C'): return NmeaSentenceId::HSC;
    case nmea_sentence_code('Z','D','L'): return NmeaSentenceId::ZDL;
    case nmea_sentence_code('Z','F','O'): return NmeaSentenceId::ZFO;
    case nmea_sentence_code('Z','T','G'): return NmeaSentenceId::ZTG;
    case nmea_sentence_code('H','F','B'): return NmeaSentenceId::HFB;
    case nmea_sentence_code('T','D','S'): return NmeaSentenceId::TDS;
    case nmea_sentence_code('T','P','C'): return NmeaSentenceId::TPC;
    case nmea_sentence_code('T','P','R'): return NmeaSentenceId::TPR;
    case nmea_sentence_code('T','P','T'): return NmeaSentenceId::TPT;
    case nmea_sentence_code('V','D','M'): return NmeaSentenceId::VDM;
    case nmea_sentence_code('V','D','O'): return NmeaSentenceId::VDO;
    case nmea_sentence_code('A','B','K'): return NmeaSentenceId::ABK;
    case nmea_sentence_code('A','D','S'): return NmeaSentenceId::ADS;
    case nmea_sentence_code('A','G','A'): return NmeaSentenceId::AGA;
    case nmea_sentence_code('B','C','L'): return NmeaSentenceId::BCL;
    case nmea_sentence_code('A','S','D'): return NmeaSentenceId::ASD;
    case nmea_sentence_code('D','S','C'): return NmeaSentenceId::DSC;
    case nmea_sentence_code('D','S','E'): return NmeaSentenceId::DSE;
    case nmea_sentence_code('N','R','X'): return NmeaSentenceId::NRX;
    case nmea_sentence_code('N','R','M'): return NmeaSentenceId::NRM;
    case nmea_sentence_code('S','M','1'): return NmeaSentenceId::SM1;
    case nmea_sentence_code('S','M','2'): return NmeaSentenceId::SM2;
    case nmea_sentence_code('S','M','3'): return NmeaSentenceId::SM3;
    case nmea_sentence_code('S','M','4'): return NmeaSentenceId::SM4;
    case nmea_sentence_code('S','M','B'): return NmeaSentenceId::SMB;
    case nmea_sentence_code('C','A','N'): return NmeaSentenceId::CAN;
    case nmea_sentence_code('C','R','Q'): return NmeaSentenceId::CRQ;
    case nmea_sentence_code('D','S','M'): return NmeaSentenceId::DSM;
    case nmea_sentence_code('T','M','D'): return NmeaSentenceId::TMD;
    case nmea_sentence_code('A','C','K'): return NmeaSentenceId::ACK;
    case nmea_sentence_code('A','K','D'): return NmeaSentenceId::AKD;
    case nmea_sentence_code('A','L','A'): return NmeaSentenceId::ALA;
    case nmea_sentence_code('A','L','C'): return NmeaSentenceId::ALC;
    case nmea_sentence_code('A','L','F'): return NmeaSentenceId::ALF;
    case nmea_sentence_code('A','L','R'): return NmeaSentenceId::ALR;
    case nmea_sentence_code('A','R','C'): return NmeaSentenceId::ARC;
    case nmea_sentence_code('H','B','T'): return NmeaSentenceId::HBT;
    case nmea_sentence_code('F','I','R'): return NmeaSentenceId::FIR;
    case nmea_sentence_code('M','O','B'): return NmeaSentenceId::MOB;
    case nmea_sentence_code('S','M','V'): return NmeaSentenceId::SMV;
    case nmea_sentence_code('F','S','I'): return NmeaSentenceId::FSI;
    case nmea_sentence_code('M','S','K'): return NmeaSentenceId::MSK;
    case nmea_sentence_code('M','S','S'): return NmeaSentenceId::MSS;
    case nmea_sentence_code('R','L','M'): return NmeaSentenceId::RLM;
    case nmea_sentence_code('C','E','K'): return NmeaSentenceId::CEK;
    case nmea_sentence_code('C','O','P'): return NmeaSentenceId::COP;
    case nmea_sentence_code('D','C','R'): return NmeaSentenceId::DCR;
    case nmea_sentence_code('D','D','C'): return NmeaSentenceId::DDC;
    case nmea_sentence_code('D','O','R'): return NmeaSentenceId::DOR;
    case nmea_sentence_code('T','X','T'): return NmeaSentenceId::TXT;
    case nmea_sentence_code('E','V','E'): return NmeaSentenceId::EVE;
    case nmea_sentence_code('E','T','L'): return NmeaSentenceId::ETL;
    default: return NmeaSentenceId::Unknown;
    }
}

template<typename Real>
class Nmea0183Input {
public:
    explicit Nmea0183Input(ModelStore<Real>& store) : store_(store) {}

    bool feed_line(const char* line,
                   SourceId source_id,
                   uint64_t now_us,
                   bool validate_checksum = true,
                   ship_data_model::SensorSource source = ship_data_model::SensorSource::serial) {
        const nmea0183_connector::NmeaTokenizeResult tokens = nmea0183_connector::tokenize_nmea_line(line);
        const nmea0183_connector::NmeaToken* token = tokens.first_sentence();
        if (!token) return false;

        char sentence_text[nmea0183_connector::NMEA_MAX_SENTENCE_LEN];
        nmea0183_connector::nmea_copy_span(sentence_text, sizeof(sentence_text), token->text);

        nmea0183_connector::NmeaSentence sentence;
        if (!parser_.parse_line(sentence_text, sentence, validate_checksum)) return false;

        const NmeaSentenceId sentence_id = classify_nmea_sentence_id(sentence);
        const bool magnetic_heading_only = sentence_id == NmeaSentenceId::HDM || sentence_id == NmeaSentenceId::HDG;
        const auto previous_heading_deg = store_.model().ins.imu.heading_deg;

        const uint32_t decoded_before = rx_.seatalk_receiver().decoded_count();
        const bool applied = rx_.apply_sentence(sentence, store_.model(), now_us, source);
        if (!applied) return false;

        if (magnetic_heading_only) {
            store_.model().ins.imu.heading_deg = previous_heading_deg;
        }

        if (rx_.seatalk_receiver().decoded_count() != decoded_before) {
            mark_seatalk_changes(store_, rx_.seatalk_receiver().last_decoded(), source_id, now_us);
        } else {
            mark_changed_from_sentence(sentence_id, sentence, source_id, now_us);
        }
        return true;
    }

    const char* last_error() const { return rx_.last_error(); }
    const nmea0183_connector::NmeaMessageState& message_state() const { return rx_.message_state(); }
    const nmea0183_connector::NmeaDscMessageState& dsc_state() const { return rx_.dsc_state(); }
    const seatalk::SeaTalkReceiver<Real>& seatalk_receiver() const { return rx_.seatalk_receiver(); }

private:
    void mark(ModelField field, SourceId source_id, uint64_t now_us) { store_.mark_changed(field, source_id, now_us); }

    void mark_gnss_position(SourceId source_id, uint64_t now_us) { mark(ModelField::GnssFixLatDeg, source_id, now_us); mark(ModelField::GnssFixLonDeg, source_id, now_us); }
    void mark_gnss_time(SourceId source_id, uint64_t now_us) { mark(ModelField::GnssTimestampS, source_id, now_us); }
    void mark_gnss_date(SourceId source_id, uint64_t now_us) { mark(ModelField::GnssDateDay, source_id, now_us); mark(ModelField::GnssDateMonth, source_id, now_us); mark(ModelField::GnssDateYear, source_id, now_us); }
    void mark_gnss_fix_quality(SourceId source_id, uint64_t now_us) {
        mark(ModelField::GnssFixQuality, source_id, now_us); mark(ModelField::GnssSatellitesUsed, source_id, now_us); mark(ModelField::GnssHdop, source_id, now_us);
        mark(ModelField::GnssFixAltitudeM, source_id, now_us); mark(ModelField::GnssGeoidalSeparationM, source_id, now_us); mark(ModelField::GnssDgpsAgeS, source_id, now_us); mark(ModelField::GnssDgpsStationId, source_id, now_us);
    }
    void mark_route_course(SourceId source_id, uint64_t now_us) {
        mark(ModelField::RouteApbXteNmi, source_id, now_us); mark(ModelField::RouteApbOriginToDestinationBearingDeg, source_id, now_us); mark(ModelField::RouteApbPresentToDestinationBearingDeg, source_id, now_us);
        mark(ModelField::RouteApbHeadingToSteerDeg, source_id, now_us); mark(ModelField::RouteApbArrivalCircleEntered, source_id, now_us); mark(ModelField::RouteApbPerpendicularPassed, source_id, now_us); mark(ModelField::RouteApbDestinationId, source_id, now_us);
    }
    void mark_rmb(SourceId source_id, uint64_t now_us) {
        mark(ModelField::RouteRmbXteNmi, source_id, now_us); mark(ModelField::RouteRmbDestinationLatDeg, source_id, now_us); mark(ModelField::RouteRmbDestinationLonDeg, source_id, now_us);
        mark(ModelField::RouteRmbRangeNmi, source_id, now_us); mark(ModelField::RouteRmbBearingDeg, source_id, now_us); mark(ModelField::RouteRmbClosingVelocityKn, source_id, now_us); mark(ModelField::RouteRmbArrived, source_id, now_us); mark(ModelField::RouteRmbDestinationId, source_id, now_us);
    }
    void mark_waypoint(SourceId source_id, uint64_t now_us) {
        mark(ModelField::RouteWaypointUtcTimeS, source_id, now_us); mark(ModelField::RouteWaypointLatDeg, source_id, now_us); mark(ModelField::RouteWaypointLonDeg, source_id, now_us);
        mark(ModelField::RouteWaypointBearingTrueDeg, source_id, now_us); mark(ModelField::RouteWaypointBearingMagneticDeg, source_id, now_us); mark(ModelField::RouteWaypointDistanceNmi, source_id, now_us); mark(ModelField::RouteWaypointToId, source_id, now_us); mark(ModelField::RouteWaypointFromId, source_id, now_us);
    }
    void mark_waypoint_arrival(SourceId source_id, uint64_t now_us) {
        mark(ModelField::RouteWaypointArrivalCircleEntered, source_id, now_us); mark(ModelField::RouteWaypointPerpendicularPassed, source_id, now_us); mark(ModelField::RouteWaypointArrivalRadiusNmi, source_id, now_us); mark(ModelField::RouteWaypointArrivalId, source_id, now_us);
    }
    void mark_current(SourceId source_id, uint64_t now_us) { mark(ModelField::SeaCurrentDirectionDeg, source_id, now_us); mark(ModelField::SeaCurrentDirectionMagneticDeg, source_id, now_us); mark(ModelField::SeaCurrentSpeedKn, source_id, now_us); }
    void mark_trawl(SourceId source_id, uint64_t now_us) {
        mark(ModelField::TrawlHeadropeToFootropeM, source_id, now_us); mark(ModelField::TrawlHeadropeToBottomM, source_id, now_us); mark(ModelField::TrawlDoorCenterlineOffsetM, source_id, now_us); mark(ModelField::TrawlDoorAlongCenterlineM, source_id, now_us);
        mark(ModelField::TrawlCartesianCenterlineOffsetM, source_id, now_us); mark(ModelField::TrawlCartesianAlongCenterlineM, source_id, now_us); mark(ModelField::TrawlDepthBelowSurfaceM, source_id, now_us); mark(ModelField::TrawlRelativeRangeM, source_id, now_us); mark(ModelField::TrawlRelativeBearingDeg, source_id, now_us); mark(ModelField::TrawlTrueRangeM, source_id, now_us); mark(ModelField::TrawlTrueBearingDeg, source_id, now_us);
    }
    void mark_notification_message(SourceId source_id, uint64_t now_us) { mark(ModelField::NotificationText, source_id, now_us); mark(ModelField::NotificationEvent, source_id, now_us); }

    void mark_changed_from_sentence(NmeaSentenceId id, const nmea0183_connector::NmeaSentence& sentence, SourceId source_id, uint64_t now_us) {
        switch (id) {
        case NmeaSentenceId::RMC: mark_gnss_position(source_id, now_us); mark_gnss_time(source_id, now_us); mark_gnss_date(source_id, now_us); mark(ModelField::GnssSpeedKn, source_id, now_us); mark(ModelField::GnssTrackDeg, source_id, now_us); break;
        case NmeaSentenceId::GGA: mark_gnss_position(source_id, now_us); mark_gnss_time(source_id, now_us); mark_gnss_fix_quality(source_id, now_us); break;
        case NmeaSentenceId::GLL: mark_gnss_position(source_id, now_us); mark_gnss_time(source_id, now_us); break;
        case NmeaSentenceId::GNS: mark_gnss_position(source_id, now_us); mark_gnss_time(source_id, now_us); mark_gnss_fix_quality(source_id, now_us); break;
        case NmeaSentenceId::ZDA: mark_gnss_time(source_id, now_us); mark_gnss_date(source_id, now_us); break;
        case NmeaSentenceId::VTG: mark(ModelField::GnssSpeedKn, source_id, now_us); mark(ModelField::GnssTrackDeg, source_id, now_us); break;
        case NmeaSentenceId::GSA: mark(ModelField::GnssDopActiveFixMode, source_id, now_us); mark(ModelField::GnssDopActivePdop, source_id, now_us); mark(ModelField::GnssHdop, source_id, now_us); mark(ModelField::GnssDopActiveVdop, source_id, now_us); break;
        case NmeaSentenceId::GFA: mark(ModelField::GnssFixAccuracyHorizontalM, source_id, now_us); mark(ModelField::GnssFixAccuracyVerticalM, source_id, now_us); mark(ModelField::GnssFixAccuracyPdop, source_id, now_us); mark(ModelField::GnssHdop, source_id, now_us); mark(ModelField::GnssFixAccuracyVdop, source_id, now_us); break;
        case NmeaSentenceId::GSV: mark(ModelField::GnssSatellitesInView, source_id, now_us); mark(ModelField::GnssSatellitePrn0, source_id, now_us); mark(ModelField::GnssSatelliteElevationDeg0, source_id, now_us); mark(ModelField::GnssSatelliteAzimuthDeg0, source_id, now_us); mark(ModelField::GnssSatelliteSnrDb0, source_id, now_us); break;
        case NmeaSentenceId::HDT: mark(ModelField::ImuHeadingTrueDeg, source_id, now_us); break;
        case NmeaSentenceId::HDM: mark(ModelField::ImuHeadingMagneticDeg, source_id, now_us); break;
        case NmeaSentenceId::HDG: mark(ModelField::ImuHeadingMagneticDeg, source_id, now_us); mark(ModelField::ImuMagneticDeviationDeg, source_id, now_us); mark(ModelField::ImuMagneticVariationDeg, source_id, now_us); mark(ModelField::GnssDeclinationDeg, source_id, now_us); break;
        case NmeaSentenceId::XDR: mark(ModelField::ImuPitchDeg, source_id, now_us); mark(ModelField::ImuRollDeg, source_id, now_us); break;
        case NmeaSentenceId::MWV: if (sentence.field(1)[0] == 'T') { mark(ModelField::WindTrueDirectionDeg, source_id, now_us); mark(ModelField::WindTrueSpeedKn, source_id, now_us); } else { mark(ModelField::WindApparentDirectionDeg, source_id, now_us); mark(ModelField::WindApparentSpeedKn, source_id, now_us); } break;
        case NmeaSentenceId::MWD: mark(ModelField::WindTrueDirectionDeg, source_id, now_us); mark(ModelField::WindTrueSpeedKn, source_id, now_us); break;
        case NmeaSentenceId::VWR: mark(ModelField::WindApparentDirectionDeg, source_id, now_us); mark(ModelField::WindApparentSpeedKn, source_id, now_us); break;
        case NmeaSentenceId::VWT: mark(ModelField::WindTrueDirectionDeg, source_id, now_us); mark(ModelField::WindTrueSpeedKn, source_id, now_us); break;
        case NmeaSentenceId::MDA:
            mark(ModelField::EnvBarometricPressureInHg, source_id, now_us); mark(ModelField::EnvBarometricPressureBar, source_id, now_us); mark(ModelField::EnvAirTemperatureC, source_id, now_us); mark(ModelField::SeaTemperatureC, source_id, now_us); mark(ModelField::EnvRelativeHumidityPercent, source_id, now_us); mark(ModelField::EnvAbsoluteHumidityPercent, source_id, now_us); mark(ModelField::EnvDewPointC, source_id, now_us); mark(ModelField::WindSurfaceDirectionDeg, source_id, now_us); mark(ModelField::WindTrueDirectionDeg, source_id, now_us); mark(ModelField::WindSurfaceDirectionMagneticDeg, source_id, now_us); mark(ModelField::WindSurfaceSpeedKn, source_id, now_us); mark(ModelField::WindTrueSpeedKn, source_id, now_us); mark(ModelField::WindSurfaceSpeedMs, source_id, now_us); mark(ModelField::WindTrueSpeedMs, source_id, now_us); break;
        case NmeaSentenceId::DBT: mark(ModelField::SeaDepthM, source_id, now_us); break;
        case NmeaSentenceId::DPT: mark(ModelField::SeaDepthM, source_id, now_us); mark(ModelField::SeaDepthOffsetM, source_id, now_us); break;
        case NmeaSentenceId::DBK: mark(ModelField::SeaDepthBelowKeelM, source_id, now_us); break;
        case NmeaSentenceId::DBS: mark(ModelField::SeaDepthBelowSurfaceM, source_id, now_us); break;
        case NmeaSentenceId::MTW: mark(ModelField::SeaTemperatureC, source_id, now_us); break;
        case NmeaSentenceId::VBW: mark(ModelField::SeaLongitudinalWaterSpeedKn, source_id, now_us); mark(ModelField::SeaTransverseWaterSpeedKn, source_id, now_us); mark(ModelField::SeaLongitudinalGroundSpeedKn, source_id, now_us); mark(ModelField::SeaTransverseGroundSpeedKn, source_id, now_us); mark(ModelField::SeaSpeedKn, source_id, now_us); break;
        case NmeaSentenceId::VHW: mark(ModelField::SeaSpeedKn, source_id, now_us); break;
        case NmeaSentenceId::VPW: mark(ModelField::SeaSpeedParallelToWindKn, source_id, now_us); mark(ModelField::SeaSpeedParallelToWindMs, source_id, now_us); break;
        case NmeaSentenceId::LWY: mark(ModelField::SeaLeewayDeg, source_id, now_us); break;
        case NmeaSentenceId::VDR:
        case NmeaSentenceId::CUR: mark_current(source_id, now_us); break;
        case NmeaSentenceId::RPM: mark(ModelField::PropulsionRevolutionsNumber, source_id, now_us); mark(ModelField::PropulsionRevolutionsSpeedRpm, source_id, now_us); mark(ModelField::PropulsionRevolutionsPitchPercent, source_id, now_us); break;
        case NmeaSentenceId::RSA: mark(ModelField::SteeringRudderAngleDeg, source_id, now_us); break;
        case NmeaSentenceId::VLW: mark(ModelField::RouteLogTotalDistanceNmi, source_id, now_us); mark(ModelField::RouteLogTripDistanceNmi, source_id, now_us); break;
        case NmeaSentenceId::AAM: mark_waypoint_arrival(source_id, now_us); break;
        case NmeaSentenceId::APA:
        case NmeaSentenceId::APB: mark_route_course(source_id, now_us); break;
        case NmeaSentenceId::RMB: mark_rmb(source_id, now_us); mark(ModelField::RouteApbXteNmi, source_id, now_us); mark(ModelField::RouteApbOriginToDestinationBearingDeg, source_id, now_us); break;
        case NmeaSentenceId::BOD:
        case NmeaSentenceId::BWW: mark(ModelField::RouteWaypointBearingTrueDeg, source_id, now_us); mark(ModelField::RouteWaypointBearingMagneticDeg, source_id, now_us); mark(ModelField::RouteWaypointToId, source_id, now_us); mark(ModelField::RouteWaypointFromId, source_id, now_us); break;
        case NmeaSentenceId::BEC:
        case NmeaSentenceId::BWC:
        case NmeaSentenceId::BWR: mark_waypoint(source_id, now_us); break;
        case NmeaSentenceId::WPL: mark(ModelField::RouteWaypointLatDeg, source_id, now_us); mark(ModelField::RouteWaypointLonDeg, source_id, now_us); mark(ModelField::RouteWaypointToId, source_id, now_us); break;
        case NmeaSentenceId::WCV: mark(ModelField::RouteRmbClosingVelocityKn, source_id, now_us); mark(ModelField::RouteRmbDestinationId, source_id, now_us); mark(ModelField::RouteWaypointToId, source_id, now_us); break;
        case NmeaSentenceId::WDC:
        case NmeaSentenceId::WDR:
        case NmeaSentenceId::WNC: mark(ModelField::RouteWaypointDistanceNmi, source_id, now_us); mark(ModelField::RouteWaypointToId, source_id, now_us); mark(ModelField::RouteWaypointFromId, source_id, now_us); break;
        case NmeaSentenceId::XTE:
        case NmeaSentenceId::XTR: mark(ModelField::RouteApbXteNmi, source_id, now_us); mark(ModelField::RouteRmbXteNmi, source_id, now_us); break;
        case NmeaSentenceId::R00:
        case NmeaSentenceId::RTE: mark(ModelField::RouteActiveWaypointCount, source_id, now_us); break;
        case NmeaSentenceId::HSC: mark(ModelField::RouteHeadingSteeringCommandTrueDeg, source_id, now_us); mark(ModelField::RouteHeadingSteeringCommandMagneticDeg, source_id, now_us); mark(ModelField::RouteApbHeadingToSteerDeg, source_id, now_us); break;
        case NmeaSentenceId::ZDL: mark(ModelField::RouteWaypointDestinationTimeRemainingS, source_id, now_us); mark(ModelField::RouteWaypointDistanceNmi, source_id, now_us); mark(ModelField::RouteWaypointToId, source_id, now_us); break;
        case NmeaSentenceId::ZFO: mark(ModelField::RouteWaypointOriginUtcTimeS, source_id, now_us); mark(ModelField::RouteWaypointOriginElapsedTimeS, source_id, now_us); mark(ModelField::RouteWaypointFromId, source_id, now_us); break;
        case NmeaSentenceId::ZTG: mark(ModelField::RouteWaypointDestinationUtcTimeS, source_id, now_us); mark(ModelField::RouteWaypointDestinationTimeRemainingS, source_id, now_us); mark(ModelField::RouteWaypointToId, source_id, now_us); break;
        case NmeaSentenceId::HFB:
        case NmeaSentenceId::TDS:
        case NmeaSentenceId::TPC:
        case NmeaSentenceId::TPR:
        case NmeaSentenceId::TPT: mark_trawl(source_id, now_us); break;
        case NmeaSentenceId::VDM:
        case NmeaSentenceId::VDO: mark(ModelField::AisTargetsObject, source_id, now_us); mark(ModelField::AisOwnVesselObject, source_id, now_us); mark(ModelField::AisSafetyObject, source_id, now_us); break;
        case NmeaSentenceId::ABK:
        case NmeaSentenceId::ADS:
        case NmeaSentenceId::AGA:
        case NmeaSentenceId::BCL: mark(ModelField::AisDataLinkStatusObject, source_id, now_us); break;
        case NmeaSentenceId::ASD: mark(ModelField::AisSafetyObject, source_id, now_us); break;
        case NmeaSentenceId::DSC:
        case NmeaSentenceId::DSE: mark(ModelField::DscStructuredNotification, source_id, now_us); break;
        case NmeaSentenceId::NRX:
        case NmeaSentenceId::NRM: mark(ModelField::NavtexStructuredNotification, source_id, now_us); mark_notification_message(source_id, now_us); break;
        case NmeaSentenceId::SM1:
        case NmeaSentenceId::SM2:
        case NmeaSentenceId::SM3:
        case NmeaSentenceId::SM4:
        case NmeaSentenceId::SMB: mark(ModelField::InmarsatSafetyNetStructuredNotification, source_id, now_us); mark_notification_message(source_id, now_us); break;
        case NmeaSentenceId::CAN:
        case NmeaSentenceId::CRQ:
        case NmeaSentenceId::DSM:
        case NmeaSentenceId::TMD: mark(ModelField::InmarsatSafetyNetStructuredNotification, source_id, now_us); break;
        case NmeaSentenceId::ACK:
        case NmeaSentenceId::ALR:
        case NmeaSentenceId::HBT:
        case NmeaSentenceId::FIR: mark(ModelField::AlertStructuredNotification, source_id, now_us); mark_notification_message(source_id, now_us); break;
        case NmeaSentenceId::AKD:
        case NmeaSentenceId::ALA:
        case NmeaSentenceId::ALC:
        case NmeaSentenceId::ALF:
        case NmeaSentenceId::ARC: mark(ModelField::AlertStructuredNotification, source_id, now_us); break;
        case NmeaSentenceId::MOB: mark(ModelField::MobStructuredNotification, source_id, now_us); mark_notification_message(source_id, now_us); break;
        case NmeaSentenceId::SMV: mark(ModelField::MobStructuredNotification, source_id, now_us); break;
        case NmeaSentenceId::FSI:
        case NmeaSentenceId::MSK:
        case NmeaSentenceId::MSS:
        case NmeaSentenceId::RLM:
        case NmeaSentenceId::CEK:
        case NmeaSentenceId::COP:
        case NmeaSentenceId::DCR:
        case NmeaSentenceId::DDC:
        case NmeaSentenceId::DOR: mark(ModelField::LegacyCommObject, source_id, now_us); break;
        case NmeaSentenceId::TXT: mark_notification_message(source_id, now_us); break;
        case NmeaSentenceId::EVE: mark(ModelField::NotificationEvent, source_id, now_us); break;
        case NmeaSentenceId::ETL: mark(ModelField::NotificationEventLog, source_id, now_us); break;
        case NmeaSentenceId::Unknown: break;
        }
    }

    ModelStore<Real>& store_;
    nmea0183_connector::Nmea0183StreamParser parser_;
    nmea0183_connector::Nmea0183RxConnector<Real> rx_;
};

} // namespace signalk_mini
