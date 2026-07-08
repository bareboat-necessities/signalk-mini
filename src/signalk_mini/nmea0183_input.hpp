#pragma once

#include <stdint.h>

#include <nmea0183_connector.hpp>
#include <seatalk.hpp>
#include <ship_data_model.hpp>

#include "model_store.hpp"
#include "seatalk_change_mapper.hpp"
#include "types.hpp"

namespace signalk_mini {

template<typename Real>
class Nmea0183Input {
public:
    explicit Nmea0183Input(ModelStore<Real>& store) : store_(store) {}

    bool feed_line(const char* line, SourceId source_id, uint64_t now_us, bool validate_checksum = true) {
        const nmea0183_connector::NmeaTokenizeResult tokens = nmea0183_connector::tokenize_nmea_line(line);
        const nmea0183_connector::NmeaToken* token = tokens.first_sentence();
        if (!token) return false;

        char sentence_text[nmea0183_connector::NMEA_MAX_SENTENCE_LEN];
        nmea0183_connector::nmea_copy_span(sentence_text, sizeof(sentence_text), token->text);

        nmea0183_connector::NmeaSentence sentence;
        if (!parser_.parse_line(sentence_text, sentence, validate_checksum)) return false;

        const uint32_t decoded_before = rx_.seatalk_receiver().decoded_count();
        const bool applied = rx_.apply_sentence(sentence, store_.model(), now_us, ship_data_model::SensorSource::serial);
        if (!applied) return false;

        if (rx_.seatalk_receiver().decoded_count() != decoded_before) {
            mark_seatalk_changes(store_, rx_.seatalk_receiver().last_decoded(), source_id, now_us);
        } else {
            mark_changed_from_sentence(sentence, source_id, now_us);
        }
        return true;
    }

    const char* last_error() const { return rx_.last_error(); }
    const nmea0183_connector::NmeaMessageState& message_state() const { return rx_.message_state(); }
    const nmea0183_connector::NmeaDscMessageState& dsc_state() const { return rx_.dsc_state(); }
    const seatalk::SeaTalkReceiver<Real>& seatalk_receiver() const { return rx_.seatalk_receiver(); }

private:
    bool is(const nmea0183_connector::NmeaSentence& sentence, const char* id) const {
        return nmea0183_connector::sentence_is(sentence, id);
    }

    void mark(ModelField field, SourceId source_id, uint64_t now_us) {
        store_.mark_changed(field, source_id, now_us);
    }

    void mark_gnss_position(SourceId source_id, uint64_t now_us) {
        mark(ModelField::GnssFixLatDeg, source_id, now_us);
        mark(ModelField::GnssFixLonDeg, source_id, now_us);
    }

    void mark_gnss_time(SourceId source_id, uint64_t now_us) {
        mark(ModelField::GnssTimestampS, source_id, now_us);
    }

    void mark_gnss_date(SourceId source_id, uint64_t now_us) {
        mark(ModelField::GnssDateDay, source_id, now_us);
        mark(ModelField::GnssDateMonth, source_id, now_us);
        mark(ModelField::GnssDateYear, source_id, now_us);
    }

    void mark_gnss_fix_quality(SourceId source_id, uint64_t now_us) {
        mark(ModelField::GnssFixQuality, source_id, now_us);
        mark(ModelField::GnssSatellitesUsed, source_id, now_us);
        mark(ModelField::GnssHdop, source_id, now_us);
        mark(ModelField::GnssFixAltitudeM, source_id, now_us);
        mark(ModelField::GnssGeoidalSeparationM, source_id, now_us);
        mark(ModelField::GnssDgpsAgeS, source_id, now_us);
        mark(ModelField::GnssDgpsStationId, source_id, now_us);
    }

    void mark_route_course(SourceId source_id, uint64_t now_us) {
        mark(ModelField::RouteApbXteNmi, source_id, now_us);
        mark(ModelField::RouteApbOriginToDestinationBearingDeg, source_id, now_us);
        mark(ModelField::RouteApbPresentToDestinationBearingDeg, source_id, now_us);
        mark(ModelField::RouteApbHeadingToSteerDeg, source_id, now_us);
        mark(ModelField::RouteApbArrivalCircleEntered, source_id, now_us);
        mark(ModelField::RouteApbPerpendicularPassed, source_id, now_us);
        mark(ModelField::RouteApbDestinationId, source_id, now_us);
    }

    void mark_rmb(SourceId source_id, uint64_t now_us) {
        mark(ModelField::RouteRmbXteNmi, source_id, now_us);
        mark(ModelField::RouteRmbDestinationLatDeg, source_id, now_us);
        mark(ModelField::RouteRmbDestinationLonDeg, source_id, now_us);
        mark(ModelField::RouteRmbRangeNmi, source_id, now_us);
        mark(ModelField::RouteRmbBearingDeg, source_id, now_us);
        mark(ModelField::RouteRmbClosingVelocityKn, source_id, now_us);
        mark(ModelField::RouteRmbArrived, source_id, now_us);
        mark(ModelField::RouteRmbDestinationId, source_id, now_us);
    }

    void mark_waypoint(SourceId source_id, uint64_t now_us) {
        mark(ModelField::RouteWaypointUtcTimeS, source_id, now_us);
        mark(ModelField::RouteWaypointLatDeg, source_id, now_us);
        mark(ModelField::RouteWaypointLonDeg, source_id, now_us);
        mark(ModelField::RouteWaypointBearingTrueDeg, source_id, now_us);
        mark(ModelField::RouteWaypointBearingMagneticDeg, source_id, now_us);
        mark(ModelField::RouteWaypointDistanceNmi, source_id, now_us);
        mark(ModelField::RouteWaypointToId, source_id, now_us);
        mark(ModelField::RouteWaypointFromId, source_id, now_us);
    }

    void mark_trawl(SourceId source_id, uint64_t now_us) {
        mark(ModelField::TrawlHeadropeToFootropeM, source_id, now_us);
        mark(ModelField::TrawlHeadropeToBottomM, source_id, now_us);
        mark(ModelField::TrawlDoorCenterlineOffsetM, source_id, now_us);
        mark(ModelField::TrawlDoorAlongCenterlineM, source_id, now_us);
        mark(ModelField::TrawlCartesianCenterlineOffsetM, source_id, now_us);
        mark(ModelField::TrawlCartesianAlongCenterlineM, source_id, now_us);
        mark(ModelField::TrawlDepthBelowSurfaceM, source_id, now_us);
        mark(ModelField::TrawlRelativeRangeM, source_id, now_us);
        mark(ModelField::TrawlRelativeBearingDeg, source_id, now_us);
        mark(ModelField::TrawlTrueRangeM, source_id, now_us);
        mark(ModelField::TrawlTrueBearingDeg, source_id, now_us);
    }

    void mark_changed_from_sentence(const nmea0183_connector::NmeaSentence& sentence, SourceId source_id, uint64_t now_us) {
        if (is(sentence, "RMC") || is(sentence, "GGA") || is(sentence, "GLL") || is(sentence, "GNS")) {
            mark_gnss_position(source_id, now_us);
        }
        if (is(sentence, "RMC") || is(sentence, "GLL") || is(sentence, "GGA") || is(sentence, "GNS") || is(sentence, "ZDA")) {
            mark_gnss_time(source_id, now_us);
        }
        if (is(sentence, "RMC") || is(sentence, "ZDA")) {
            mark_gnss_date(source_id, now_us);
        }
        if (is(sentence, "RMC") || is(sentence, "VTG")) {
            mark(ModelField::GnssSpeedKn, source_id, now_us);
            mark(ModelField::GnssTrackDeg, source_id, now_us);
        }
        if (is(sentence, "GGA") || is(sentence, "GNS")) {
            mark_gnss_fix_quality(source_id, now_us);
        }
        if (is(sentence, "GSA")) {
            mark(ModelField::GnssDopActiveFixMode, source_id, now_us);
            mark(ModelField::GnssDopActivePdop, source_id, now_us);
            mark(ModelField::GnssHdop, source_id, now_us);
            mark(ModelField::GnssDopActiveVdop, source_id, now_us);
        }
        if (is(sentence, "GFA")) {
            mark(ModelField::GnssFixAccuracyHorizontalM, source_id, now_us);
            mark(ModelField::GnssFixAccuracyVerticalM, source_id, now_us);
            mark(ModelField::GnssFixAccuracyPdop, source_id, now_us);
            mark(ModelField::GnssHdop, source_id, now_us);
            mark(ModelField::GnssFixAccuracyVdop, source_id, now_us);
        }
        if (is(sentence, "GSV")) {
            mark(ModelField::GnssSatellitesInView, source_id, now_us);
            mark(ModelField::GnssSatellitePrn0, source_id, now_us);
            mark(ModelField::GnssSatelliteElevationDeg0, source_id, now_us);
            mark(ModelField::GnssSatelliteAzimuthDeg0, source_id, now_us);
            mark(ModelField::GnssSatelliteSnrDb0, source_id, now_us);
        }

        if (is(sentence, "HDT")) {
            mark(ModelField::ImuHeadingDeg, source_id, now_us);
            mark(ModelField::ImuHeadingTrueDeg, source_id, now_us);
        }
        if (is(sentence, "HDM") || is(sentence, "HDG")) {
            mark(ModelField::ImuHeadingDeg, source_id, now_us);
            mark(ModelField::ImuHeadingMagneticDeg, source_id, now_us);
        }
        if (is(sentence, "HDG")) {
            mark(ModelField::ImuMagneticDeviationDeg, source_id, now_us);
            mark(ModelField::ImuMagneticVariationDeg, source_id, now_us);
            mark(ModelField::GnssDeclinationDeg, source_id, now_us);
        }
        if (is(sentence, "XDR")) {
            mark(ModelField::ImuPitchDeg, source_id, now_us);
            mark(ModelField::ImuRollDeg, source_id, now_us);
        }

        if (is(sentence, "MWV")) {
            if (sentence.field(1)[0] == 'T') {
                mark(ModelField::WindTrueDirectionDeg, source_id, now_us);
                mark(ModelField::WindTrueSpeedKn, source_id, now_us);
            } else {
                mark(ModelField::WindApparentDirectionDeg, source_id, now_us);
                mark(ModelField::WindApparentSpeedKn, source_id, now_us);
            }
        }
        if (is(sentence, "MWD")) {
            mark(ModelField::WindTrueDirectionDeg, source_id, now_us);
            mark(ModelField::WindTrueSpeedKn, source_id, now_us);
        }
        if (is(sentence, "VWR")) {
            mark(ModelField::WindApparentDirectionDeg, source_id, now_us);
            mark(ModelField::WindApparentSpeedKn, source_id, now_us);
        }
        if (is(sentence, "VWT")) {
            mark(ModelField::WindTrueDirectionDeg, source_id, now_us);
            mark(ModelField::WindTrueSpeedKn, source_id, now_us);
        }
        if (is(sentence, "MDA")) {
            mark(ModelField::EnvBarometricPressureInHg, source_id, now_us);
            mark(ModelField::EnvBarometricPressureBar, source_id, now_us);
            mark(ModelField::EnvAirTemperatureC, source_id, now_us);
            mark(ModelField::SeaTemperatureC, source_id, now_us);
            mark(ModelField::EnvRelativeHumidityPercent, source_id, now_us);
            mark(ModelField::EnvAbsoluteHumidityPercent, source_id, now_us);
            mark(ModelField::EnvDewPointC, source_id, now_us);
            mark(ModelField::WindSurfaceDirectionDeg, source_id, now_us);
            mark(ModelField::WindTrueDirectionDeg, source_id, now_us);
            mark(ModelField::WindSurfaceDirectionMagneticDeg, source_id, now_us);
            mark(ModelField::WindSurfaceSpeedKn, source_id, now_us);
            mark(ModelField::WindTrueSpeedKn, source_id, now_us);
            mark(ModelField::WindSurfaceSpeedMs, source_id, now_us);
            mark(ModelField::WindTrueSpeedMs, source_id, now_us);
        }

        if (is(sentence, "DBT") || is(sentence, "DPT")) {
            mark(ModelField::SeaDepthM, source_id, now_us);
        }
        if (is(sentence, "DPT")) {
            mark(ModelField::SeaDepthOffsetM, source_id, now_us);
        }
        if (is(sentence, "DBK")) {
            mark(ModelField::SeaDepthBelowKeelM, source_id, now_us);
        }
        if (is(sentence, "DBS")) {
            mark(ModelField::SeaDepthBelowSurfaceM, source_id, now_us);
        }
        if (is(sentence, "MTW")) {
            mark(ModelField::SeaTemperatureC, source_id, now_us);
        }
        if (is(sentence, "VBW")) {
            mark(ModelField::SeaLongitudinalWaterSpeedKn, source_id, now_us);
            mark(ModelField::SeaTransverseWaterSpeedKn, source_id, now_us);
            mark(ModelField::SeaLongitudinalGroundSpeedKn, source_id, now_us);
            mark(ModelField::SeaTransverseGroundSpeedKn, source_id, now_us);
            mark(ModelField::SeaSpeedKn, source_id, now_us);
        }
        if (is(sentence, "VHW")) {
            mark(ModelField::SeaSpeedKn, source_id, now_us);
        }
        if (is(sentence, "VPW")) {
            mark(ModelField::SeaSpeedParallelToWindKn, source_id, now_us);
            mark(ModelField::SeaSpeedParallelToWindMs, source_id, now_us);
        }
        if (is(sentence, "LWY")) {
            mark(ModelField::SeaLeewayDeg, source_id, now_us);
        }
        if (is(sentence, "VDR")) {
            mark(ModelField::SeaCurrentDirectionDeg, source_id, now_us);
            mark(ModelField::SeaCurrentDirectionMagneticDeg, source_id, now_us);
            mark(ModelField::SeaCurrentSpeedKn, source_id, now_us);
        }

        if (is(sentence, "RPM")) {
            mark(ModelField::PropulsionRevolutionsNumber, source_id, now_us);
            mark(ModelField::PropulsionRevolutionsSpeedRpm, source_id, now_us);
            mark(ModelField::PropulsionRevolutionsPitchPercent, source_id, now_us);
        }
        if (is(sentence, "RSA")) {
            mark(ModelField::SteeringRudderAngleDeg, source_id, now_us);
        }

        if (is(sentence, "VLW")) {
            mark(ModelField::RouteLogTotalDistanceNmi, source_id, now_us);
            mark(ModelField::RouteLogTripDistanceNmi, source_id, now_us);
        }
        if (is(sentence, "APB") || is(sentence, "APA")) {
            mark_route_course(source_id, now_us);
        }
        if (is(sentence, "RMB")) {
            mark_rmb(source_id, now_us);
            mark(ModelField::RouteApbXteNmi, source_id, now_us);
            mark(ModelField::RouteApbOriginToDestinationBearingDeg, source_id, now_us);
        }
        if (is(sentence, "BOD") || is(sentence, "BWW")) {
            mark(ModelField::RouteWaypointBearingTrueDeg, source_id, now_us);
            mark(ModelField::RouteWaypointBearingMagneticDeg, source_id, now_us);
            mark(ModelField::RouteWaypointToId, source_id, now_us);
            mark(ModelField::RouteWaypointFromId, source_id, now_us);
        }
        if (is(sentence, "BWC") || is(sentence, "BWR")) {
            mark_waypoint(source_id, now_us);
        }
        if (is(sentence, "WPL")) {
            mark(ModelField::RouteWaypointLatDeg, source_id, now_us);
            mark(ModelField::RouteWaypointLonDeg, source_id, now_us);
            mark(ModelField::RouteWaypointToId, source_id, now_us);
        }
        if (is(sentence, "WCV")) {
            mark(ModelField::RouteRmbClosingVelocityKn, source_id, now_us);
            mark(ModelField::RouteRmbDestinationId, source_id, now_us);
            mark(ModelField::RouteWaypointToId, source_id, now_us);
        }
        if (is(sentence, "WDC") || is(sentence, "WDR") || is(sentence, "WNC")) {
            mark(ModelField::RouteWaypointDistanceNmi, source_id, now_us);
            mark(ModelField::RouteWaypointToId, source_id, now_us);
            mark(ModelField::RouteWaypointFromId, source_id, now_us);
        }
        if (is(sentence, "XTE") || is(sentence, "XTR")) {
            mark(ModelField::RouteApbXteNmi, source_id, now_us);
            mark(ModelField::RouteRmbXteNmi, source_id, now_us);
        }
        if (is(sentence, "R00") || is(sentence, "RTE")) {
            mark(ModelField::RouteActiveWaypointCount, source_id, now_us);
        }
        if (is(sentence, "HSC")) {
            mark(ModelField::RouteHeadingSteeringCommandTrueDeg, source_id, now_us);
            mark(ModelField::RouteHeadingSteeringCommandMagneticDeg, source_id, now_us);
            mark(ModelField::RouteApbHeadingToSteerDeg, source_id, now_us);
        }
        if (is(sentence, "ZDL")) {
            mark(ModelField::RouteWaypointDestinationTimeRemainingS, source_id, now_us);
            mark(ModelField::RouteWaypointDistanceNmi, source_id, now_us);
            mark(ModelField::RouteWaypointToId, source_id, now_us);
        }
        if (is(sentence, "ZFO")) {
            mark(ModelField::RouteWaypointOriginUtcTimeS, source_id, now_us);
            mark(ModelField::RouteWaypointOriginElapsedTimeS, source_id, now_us);
            mark(ModelField::RouteWaypointFromId, source_id, now_us);
        }
        if (is(sentence, "ZTG")) {
            mark(ModelField::RouteWaypointDestinationUtcTimeS, source_id, now_us);
            mark(ModelField::RouteWaypointDestinationTimeRemainingS, source_id, now_us);
            mark(ModelField::RouteWaypointToId, source_id, now_us);
        }

        if (is(sentence, "HFB") || is(sentence, "TDS") || is(sentence, "TPC") || is(sentence, "TPR") || is(sentence, "TPT")) {
            mark_trawl(source_id, now_us);
        }
        if (is(sentence, "TXT") || is(sentence, "ALR") || is(sentence, "ACK") || is(sentence, "HBT") || is(sentence, "FIR") ||
            is(sentence, "NRX") || is(sentence, "NRM") || is(sentence, "SM1") || is(sentence, "SM2") || is(sentence, "SM3") ||
            is(sentence, "SM4") || is(sentence, "SMB") || is(sentence, "MOB")) {
            mark(ModelField::NotificationText, source_id, now_us);
            mark(ModelField::NotificationEvent, source_id, now_us);
        }
    }

    ModelStore<Real>& store_;
    nmea0183_connector::Nmea0183StreamParser parser_;
    nmea0183_connector::Nmea0183RxConnector<Real> rx_;
};

} // namespace signalk_mini
