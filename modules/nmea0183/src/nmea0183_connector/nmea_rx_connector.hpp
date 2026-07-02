#pragma once

#include <stdint.h>
#include <string.h>

#include <ship_data_model.hpp>

#include "sentence_parser.hpp"
#include "nmea_message_state.hpp"
#include "nmea_dsc_message_state.hpp"

namespace nmea0183_connector {

template<typename Real = float>
class Nmea0183RxConnector {
public:
    Nmea0183RxConnector()
        : last_error_(""),
          last_apb_mode_(ship_data_model::AutopilotMode::gps) {
        last_apb_sender_id_[0] = '\0';
        last_apb_sender_id_[1] = '\0';
        last_apb_sender_id_[2] = '\0';
    }

    const char* last_error() const { return last_error_; }
    ship_data_model::AutopilotMode last_apb_mode() const { return last_apb_mode_; }
    const char* last_apb_sender_id() const { return last_apb_sender_id_; }
    const NmeaMessageState& message_state() const { return state_; }
    const NmeaDscMessageState& dsc_state() const { return dsc_state_; }

    bool apply_sentence(const NmeaSentence& sentence,
                        ship_data_model::DataModel<Real>& model,
                        uint64_t now_us) {
        return apply_sentence(sentence, model, now_us, ship_data_model::SensorSource::none);
    }

    bool apply_sentence(const NmeaSentence& sentence,
                        ship_data_model::DataModel<Real>& root_model,
                        uint64_t now_us,
                        ship_data_model::SensorSource source) {
        last_error_ = "";
        if (!sentence.valid_checksum) {
            last_error_ = "invalid checksum";
            return false;
        }

        update_multipart_message_state(sentence, now_us, source);
        NmeaModelWriteView model(root_model);

#define NMEA_APPLY(ID, FN) if (sentence_is(sentence, ID)) return FN(sentence, model, now_us, source)
#define NMEA_APPLY_NO_SOURCE(ID, FN) if (sentence_is(sentence, ID)) return FN(sentence, model, now_us)
        NMEA_APPLY("AAM", apply_aam);
        NMEA_APPLY("ACK", apply_ack);
        NMEA_APPLY("ADS", apply_ads);
        NMEA_APPLY("AKD", apply_akd);
        NMEA_APPLY("ALA", apply_ala);
        NMEA_APPLY("ALM", apply_alm);
        NMEA_APPLY("APA", apply_apa);
        NMEA_APPLY("APB", apply_apb);
        NMEA_APPLY("ASD", apply_asd);
        NMEA_APPLY("BEC", apply_bec);
        if (sentence_is(sentence, "BOD")) return apply_bod_bww(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWC")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWR")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWW")) return apply_bod_bww(sentence, model, now_us, source);
        NMEA_APPLY("CEK", apply_cek);
        NMEA_APPLY("COP", apply_cop);
        NMEA_APPLY("CUR", apply_cur);
        NMEA_APPLY("DBK", apply_depth_below_keel);
        NMEA_APPLY("DBS", apply_depth_below_surface);
        NMEA_APPLY("DBT", apply_dbt);
        NMEA_APPLY("DCN", apply_dcn);
        NMEA_APPLY("DCR", apply_dcr);
        NMEA_APPLY("DDC", apply_ddc);
        NMEA_APPLY("DOR", apply_dor);
        NMEA_APPLY("DPT", apply_dpt);
        NMEA_APPLY("DSC", apply_dsc);
        NMEA_APPLY("DSE", apply_dse);
        NMEA_APPLY("DSI", apply_dsi);
        NMEA_APPLY("DSR", apply_dsr);
        NMEA_APPLY("DTM", apply_dtm);
        NMEA_APPLY("ETL", apply_etl);
        NMEA_APPLY("EVE", apply_eve);
        NMEA_APPLY("FIR", apply_fir);
        NMEA_APPLY("FSI", apply_fsi);
        NMEA_APPLY("GBS", apply_gbs);
        NMEA_APPLY("GGA", apply_gga);
        NMEA_APPLY("GLC", apply_glc);
        NMEA_APPLY("GLL", apply_gll);
        NMEA_APPLY("GNS", apply_gns);
        NMEA_APPLY("GRS", apply_grs);
        NMEA_APPLY("GSA", apply_gsa);
        NMEA_APPLY("GST", apply_gst);
        NMEA_APPLY("GSV", apply_gsv);
        NMEA_APPLY("GTD", apply_gtd);
        NMEA_APPLY("GXA", apply_gxa);
        NMEA_APPLY_NO_SOURCE("HDG", apply_hdg);
        NMEA_APPLY_NO_SOURCE("HDM", apply_hdm);
        NMEA_APPLY_NO_SOURCE("HDT", apply_hdt);
        NMEA_APPLY("HFB", apply_hfb);
        NMEA_APPLY("HSC", apply_hsc);
        NMEA_APPLY("ITS", apply_its);
        NMEA_APPLY("LWY", apply_lwy);
        NMEA_APPLY("MDA", apply_mda);
        NMEA_APPLY("MSK", apply_msk);
        NMEA_APPLY("MSS", apply_mss);
        NMEA_APPLY("MTW", apply_mtw);
        NMEA_APPLY("MWD", apply_mwd);
        NMEA_APPLY("MWV", apply_mwv);
        NMEA_APPLY("OLN", apply_oln);
        NMEA_APPLY("OSD", apply_osd);
        NMEA_APPLY("R00", apply_r00);
        NMEA_APPLY("RLM", apply_rlm);
        NMEA_APPLY("RMA", apply_rma);
        NMEA_APPLY("RMB", apply_rmb);
        NMEA_APPLY("RMC", apply_rmc);
        NMEA_APPLY_NO_SOURCE("ROT", apply_rot);
        NMEA_APPLY("RPM", apply_rpm);
        NMEA_APPLY("RSA", apply_rsa);
        NMEA_APPLY("RSD", apply_rsd);
        NMEA_APPLY("RTE", apply_rte);
        NMEA_APPLY("SFI", apply_sfi);
        NMEA_APPLY("STN", apply_stn);
        NMEA_APPLY("TDS", apply_tds);
        NMEA_APPLY("TFI", apply_tfi);
        NMEA_APPLY("TLB", apply_tlb);
        NMEA_APPLY("TLL", apply_tll);
        NMEA_APPLY("TPC", apply_tpc);
        NMEA_APPLY("TPR", apply_tpr);
        NMEA_APPLY("TPT", apply_tpt);
        NMEA_APPLY("TRF", apply_trf);
        NMEA_APPLY("TTM", apply_ttm);
        NMEA_APPLY("TXT", apply_txt);
        NMEA_APPLY("VBW", apply_vbw);
        NMEA_APPLY("VDR", apply_vdr);
        NMEA_APPLY("VHW", apply_vhw);
        NMEA_APPLY("VLW", apply_vlw);
        NMEA_APPLY("VPW", apply_vpw);
        NMEA_APPLY("VTG", apply_vtg);
        if (sentence_is(sentence, "VWR")) return apply_vwr(sentence, model, now_us, source, false);
        if (sentence_is(sentence, "VWT")) return apply_vwr(sentence, model, now_us, source, true);
        NMEA_APPLY("WCV", apply_wcv);
        NMEA_APPLY("WDC", apply_wdc);
        NMEA_APPLY("WDR", apply_wdr);
        NMEA_APPLY("WNC", apply_wnc);
        NMEA_APPLY("WPL", apply_wpl);
        NMEA_APPLY_NO_SOURCE("XDR", apply_xdr);
        NMEA_APPLY("XTE", apply_xte);
        NMEA_APPLY("XTR", apply_xtr);
        NMEA_APPLY("ZDA", apply_zda);
        NMEA_APPLY("ZDL", apply_zdl);
        NMEA_APPLY("ZFO", apply_zfo);
        NMEA_APPLY("ZTG", apply_ztg);
#undef NMEA_APPLY
#undef NMEA_APPLY_NO_SOURCE

        if (sentence.fragment.is_fragmented && accepts_fragment_only_family(sentence.family)) return true;

        last_error_ = "unsupported sentence";
        return false;
    }

private:
    struct NmeaGnssAlmanacWriteView {
        ship_data_model::Setting<ship_data_model::SensorSource>& source;
        ship_data_model::Stamped<int32_t>& total_messages;
        ship_data_model::Stamped<int32_t>& message_number;
        ship_data_model::Stamped<int32_t>& satellite_prn;
        ship_data_model::Stamped<int32_t>& gps_week;
        ship_data_model::Stamped<int32_t>& sv_health;
        ship_data_model::Stamped<Real>& eccentricity;
        ship_data_model::Stamped<Real>& reference_time_s;
        ship_data_model::Stamped<Real>& inclination_rad;
        ship_data_model::Stamped<Real>& right_ascension_rate_rad_s;
        ship_data_model::Stamped<Real>& sqrt_semi_major_axis;
        ship_data_model::Stamped<Real>& argument_of_perigee_rad;
        ship_data_model::Stamped<Real>& longitude_ascension_node_rad;
        ship_data_model::Stamped<Real>& mean_anomaly_rad;
        ship_data_model::Stamped<Real>& clock_f0_s;
        ship_data_model::Stamped<Real>& clock_f1_s_s;
        uint64_t& last_update_us;

        explicit NmeaGnssAlmanacWriteView(ship_data_model::GnssAlmanacData<Real>& data)
            : source(data.source),
              total_messages(data.total_messages),
              message_number(data.message_number),
              satellite_prn(data.satellite_prn),
              gps_week(data.gnss_week),
              sv_health(data.sv_health),
              eccentricity(data.eccentricity),
              reference_time_s(data.reference_time_s),
              inclination_rad(data.inclination_rad),
              right_ascension_rate_rad_s(data.right_ascension_rate_rad_s),
              sqrt_semi_major_axis(data.sqrt_semi_major_axis),
              argument_of_perigee_rad(data.argument_of_perigee_rad),
              longitude_ascension_node_rad(data.longitude_ascension_node_rad),
              mean_anomaly_rad(data.mean_anomaly_rad),
              clock_f0_s(data.clock_f0_s),
              clock_f1_s_s(data.clock_f1_s_s),
              last_update_us(data.last_update_us) {}
    };

    struct NmeaNavigationWriteView {
        ship_data_model::GnssData<Real>& gps;
        NmeaGnssAlmanacWriteView gps_almanac;
        ship_data_model::GnssFaultDetectionData<Real>& gps_fault;
        ship_data_model::GnssRangeResidualData<Real>& gps_range_residual;
        ship_data_model::GnssNoiseStatisticsData<Real>& gps_noise;
        ship_data_model::GnssDopActiveSatellitesData<Real>& gps_dop;
        ship_data_model::GnssSatellitesInViewData<Real>& gps_satellites_in_view;
        ship_data_model::DatumReferenceData<Real>& datum;
        ship_data_model::DeccaPositionData<Real>& decca;
        ship_data_model::LegacyTimingData<Real>& legacy_timing;
        ship_data_model::LegacyDeltaData<Real>& legacy_delta;
        ship_data_model::TransitFixData<Real>& transit_fix;
        ship_data_model::RadioFrequencySetData<Real>& radio_frequency_set;
        ship_data_model::HeadingSteeringCommandData<Real>& heading_steering_command;
        ship_data_model::BeaconReceiverControlData<Real>& beacon_control;
        ship_data_model::BeaconReceiverStatusData<Real>& beacon_status;
        ship_data_model::ReturnLinkMessageData<Real>& return_link_message;
        ship_data_model::OmegaLaneNumbersData<Real>& omega_lane_numbers;
        ship_data_model::OwnShipData<Real>& own_ship;
        ship_data_model::ActiveRouteData<Real>& active_route;
        ship_data_model::RevolutionsData<Real>& revolutions;
        ship_data_model::RadarSystemData<Real>& radar_system;
        ship_data_model::ScanningFrequencyData<Real>& scanning_frequency;
        ship_data_model::MultipleDataIdData<Real>& multiple_data_id;
        ship_data_model::TrackedTargetData<Real>& tracked_target;
        ship_data_model::RmaData<Real>& rma;
        ship_data_model::WaypointArrivalData<Real>& waypoint_arrival;
        ship_data_model::WaypointNavigationData<Real>& waypoint;
        ship_data_model::ApbData<Real>& apb;
        ship_data_model::RmbData<Real>& rmb;

        explicit NmeaNavigationWriteView(ship_data_model::DataModel<Real>& model)
            : gps(model.gnss.fix),
              gps_almanac(model.gnss.almanac),
              gps_fault(model.gnss.fault_detection),
              gps_range_residual(model.gnss.range_residual),
              gps_noise(model.gnss.noise_statistics),
              gps_dop(model.gnss.dop_active_satellites),
              gps_satellites_in_view(model.gnss.satellites_in_view),
              datum(model.nav.datum),
              decca(model.nav.decca),
              legacy_timing(model.nav.legacy_timing),
              legacy_delta(model.nav.legacy_delta),
              transit_fix(model.nav.transit_fix),
              radio_frequency_set(model.comm.radio_frequency_set),
              heading_steering_command(model.route.heading_steering_command),
              beacon_control(model.comm.beacon_control),
              beacon_status(model.comm.beacon_status),
              return_link_message(model.comm.return_link_message),
              omega_lane_numbers(model.nav.omega_lane_numbers),
              own_ship(model.nav.own_ship),
              active_route(model.route.active),
              revolutions(model.propulsion.revolutions),
              radar_system(model.nav.radar_system),
              scanning_frequency(model.nav.scanning_frequency),
              multiple_data_id(model.nav.multiple_data_id),
              tracked_target(model.ais.tracked_target),
              rma(model.nav.rma),
              waypoint_arrival(model.route.waypoint_arrival),
              waypoint(model.route.waypoint),
              apb(model.route.apb),
              rmb(model.route.rmb) {}
    };

    struct NmeaWaterWriteView {
        ship_data_model::Setting<ship_data_model::SensorSource>& source;
        ship_data_model::Setting<ship_data_model::SensorSource>& leeway_source;
        ship_data_model::Setting<ship_data_model::SensorSource>& depth_source;
        ship_data_model::Stamped<Real>& speed_kn;
        ship_data_model::Stamped<Real>& longitudinal_water_speed_kn;
        ship_data_model::Stamped<Real>& transverse_water_speed_kn;
        char& water_speed_status;
        ship_data_model::Stamped<Real>& longitudinal_ground_speed_kn;
        ship_data_model::Stamped<Real>& transverse_ground_speed_kn;
        char& ground_speed_status;
        ship_data_model::Stamped<Real>& speed_parallel_to_wind_kn;
        ship_data_model::Stamped<Real>& speed_parallel_to_wind_m_s;
        ship_data_model::Stamped<Real>& total_distance_nmi;
        ship_data_model::Stamped<Real>& trip_distance_nmi;
        ship_data_model::Stamped<Real>& leeway_deg;
        ship_data_model::Stamped<Real>& current_speed_kn;
        ship_data_model::Stamped<Real>& current_direction_deg;
        ship_data_model::Stamped<Real>& current_direction_magnetic_deg;
        ship_data_model::Stamped<Real>& wind_speed_kn;
        ship_data_model::Stamped<Real>& wind_speed_m_s;
        ship_data_model::Stamped<Real>& wind_direction_deg;
        ship_data_model::Stamped<Real>& wind_direction_magnetic_deg;
        ship_data_model::Stamped<Real>& barometric_pressure_inhg;
        ship_data_model::Stamped<Real>& barometric_pressure_bar;
        ship_data_model::Stamped<Real>& air_temperature_c;
        ship_data_model::Stamped<Real>& relative_humidity_percent;
        ship_data_model::Stamped<Real>& absolute_humidity_percent;
        ship_data_model::Stamped<Real>& dew_point_c;
        ship_data_model::Stamped<Real>& depth_m;
        ship_data_model::Stamped<Real>& depth_below_keel_m;
        ship_data_model::Stamped<Real>& depth_below_surface_m;
        ship_data_model::Stamped<Real>& depth_offset_m;
        ship_data_model::Stamped<Real>& temperature_c;
        ship_data_model::Stamped<Real>& trawl_headrope_to_footrope_m;
        ship_data_model::Stamped<Real>& trawl_headrope_to_bottom_m;
        ship_data_model::Stamped<Real>& trawl_door_centerline_offset_m;
        ship_data_model::Stamped<Real>& trawl_door_along_centerline_m;
        ship_data_model::Stamped<Real>& trawl_cartesian_centerline_offset_m;
        ship_data_model::Stamped<Real>& trawl_cartesian_along_centerline_m;
        ship_data_model::Stamped<int32_t> (&trawl_catch_sensor_status)[3];
        ship_data_model::Stamped<Real>& trawl_depth_below_surface_m;
        ship_data_model::Stamped<Real>& trawl_relative_range_m;
        ship_data_model::Stamped<Real>& trawl_relative_bearing_deg;
        ship_data_model::Stamped<Real>& trawl_true_range_m;
        ship_data_model::Stamped<Real>& trawl_true_bearing_deg;
        uint64_t& last_update_us;

        explicit NmeaWaterWriteView(ship_data_model::DataModel<Real>& model)
            : source(model.sea.source),
              leeway_source(model.sea.leeway_source),
              depth_source(model.sea.depth_source),
              speed_kn(model.sea.speed_kn),
              longitudinal_water_speed_kn(model.sea.longitudinal_water_speed_kn),
              transverse_water_speed_kn(model.sea.transverse_water_speed_kn),
              water_speed_status(model.sea.water_speed_status),
              longitudinal_ground_speed_kn(model.sea.longitudinal_ground_speed_kn),
              transverse_ground_speed_kn(model.sea.transverse_ground_speed_kn),
              ground_speed_status(model.sea.ground_speed_status),
              speed_parallel_to_wind_kn(model.sea.speed_parallel_to_wind_kn),
              speed_parallel_to_wind_m_s(model.sea.speed_parallel_to_wind_m_s),
              total_distance_nmi(model.sea.total_distance_nmi),
              trip_distance_nmi(model.sea.trip_distance_nmi),
              leeway_deg(model.sea.leeway_deg),
              current_speed_kn(model.sea.current_speed_kn),
              current_direction_deg(model.sea.current_direction_deg),
              current_direction_magnetic_deg(model.sea.current_direction_magnetic_deg),
              wind_speed_kn(model.wind.surface.speed_kn),
              wind_speed_m_s(model.wind.surface.speed_m_s),
              wind_direction_deg(model.wind.surface.direction_deg),
              wind_direction_magnetic_deg(model.wind.surface.direction_magnetic_deg),
              barometric_pressure_inhg(model.env.barometric_pressure_inhg),
              barometric_pressure_bar(model.env.barometric_pressure_bar),
              air_temperature_c(model.env.air_temperature_c),
              relative_humidity_percent(model.env.relative_humidity_percent),
              absolute_humidity_percent(model.env.absolute_humidity_percent),
              dew_point_c(model.env.dew_point_c),
              depth_m(model.sea.depth_m),
              depth_below_keel_m(model.sea.depth_below_keel_m),
              depth_below_surface_m(model.sea.depth_below_surface_m),
              depth_offset_m(model.sea.depth_offset_m),
              temperature_c(model.sea.temperature_c),
              trawl_headrope_to_footrope_m(model.sea.trawl_headrope_to_footrope_m),
              trawl_headrope_to_bottom_m(model.sea.trawl_headrope_to_bottom_m),
              trawl_door_centerline_offset_m(model.sea.trawl_door_centerline_offset_m),
              trawl_door_along_centerline_m(model.sea.trawl_door_along_centerline_m),
              trawl_cartesian_centerline_offset_m(model.sea.trawl_cartesian_centerline_offset_m),
              trawl_cartesian_along_centerline_m(model.sea.trawl_cartesian_along_centerline_m),
              trawl_catch_sensor_status(model.sea.trawl_catch_sensor_status),
              trawl_depth_below_surface_m(model.sea.trawl_depth_below_surface_m),
              trawl_relative_range_m(model.sea.trawl_relative_range_m),
              trawl_relative_bearing_deg(model.sea.trawl_relative_bearing_deg),
              trawl_true_range_m(model.sea.trawl_true_range_m),
              trawl_true_bearing_deg(model.sea.trawl_true_bearing_deg),
              last_update_us(model.sea.last_update_us) {}
    };

    struct NmeaModelWriteView {
        NmeaNavigationWriteView navigation;
        NmeaWaterWriteView water;
        ship_data_model::WindData<Real>& wind;
        ship_data_model::ShipImuData<Real>& imu;
        ship_data_model::RudderData<Real>& rudder;

        explicit NmeaModelWriteView(ship_data_model::DataModel<Real>& model)
            : navigation(model), water(model), wind(model.wind), imu(model.imu), rudder(model.steering.rudder) {}
    };

    const char* last_error_;
    ship_data_model::AutopilotMode last_apb_mode_;
    char last_apb_sender_id_[3];
    NmeaMessageState state_;
    NmeaDscMessageState dsc_state_;

    template<typename Setting>
    void set_source(Setting& setting, ship_data_model::SensorSource source) {
        if (source != ship_data_model::SensorSource::none) setting.value = source;
    }

    bool accepts_fragment_only_family(NmeaSentenceFamily family) const {
        return family == NmeaSentenceFamily::Ais ||
               family == NmeaSentenceFamily::NavTex ||
               family == NmeaSentenceFamily::SeaTalk ||
               family == NmeaSentenceFamily::Inmarsat;
    }

    template<typename Multipart>
    void reset_multipart_record(Multipart& record) {
        record.received_mask = 0;
        record.in_progress = false;
        record.complete = false;
        record.overflow = false;
        record.text[0] = '\0';
    }

    template<typename Multipart>
    void update_multipart_record(const NmeaSentence& sentence,
                                 Multipart& record,
                                 uint64_t now_us,
                                 ship_data_model::SensorSource source) {
        if (!sentence.fragment.is_fragmented) return;
        if (sentence.fragment.is_first() || !record.in_progress || record.complete) {
            reset_multipart_record(record);
            nmea_copy_span(record.sentence_id, sizeof(record.sentence_id), sentence.sentence);
            nmea_copy_span(record.message_id, sizeof(record.message_id), sentence.fragment.message_id);
            record.in_progress = true;
        }
        record.total_fragments.set(static_cast<int32_t>(sentence.fragment.total), now_us);
        record.last_fragment_number.set(static_cast<int32_t>(sentence.fragment.number), now_us);
        if (sentence.fragment.number <= 16) {
            record.received_mask = static_cast<uint16_t>(record.received_mask | (1u << (sentence.fragment.number - 1u)));
        }

        size_t current = strlen(record.text);
        const size_t capacity = NMEA_MESSAGE_MULTIPART_TEXT_BYTES;
        size_t append = sentence.fragment.payload.length;
        if (current + append + 1u > capacity) {
            append = current < capacity ? capacity - current - 1u : 0u;
            record.overflow = true;
        }
        if (append > 0u && sentence.fragment.payload.data) {
            memcpy(record.text + current, sentence.fragment.payload.data, append);
            record.text[current + append] = '\0';
            current += append;
        }
        record.text_length.set(static_cast<int32_t>(current), now_us);

        const uint16_t expected = sentence.fragment.total >= 16
            ? 0xffffu
            : static_cast<uint16_t>((1u << sentence.fragment.total) - 1u);
        record.complete = sentence.fragment.total > 0 && (record.received_mask & expected) == expected;
        if (record.complete) record.in_progress = false;
        set_source(record.source, source);
        record.last_update_us = now_us;
    }

    void update_multipart_message_state(const NmeaSentence& sentence,
                                        uint64_t now_us,
                                        ship_data_model::SensorSource source) {
        if (!sentence.fragment.is_fragmented) return;
        if (sentence_is(sentence, "TXT")) update_multipart_record(sentence, state_.text_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Ais) update_multipart_record(sentence, state_.ais_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::NavTex) update_multipart_record(sentence, state_.navtex_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Dsc) update_multipart_record(sentence, dsc_state_.multipart, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::SeaTalk) update_multipart_record(sentence, state_.seatalk_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Inmarsat) update_multipart_record(sentence, state_.inmarsat_message, now_us, source);
        else update_multipart_record(sentence, state_.generic_multipart_message, now_us, source);
    }

#include "nmea_A_E.hpp"
#include "nmea_F_G.hpp"
#include "nmea_H_N.hpp"
#include "nmea_O_Z.hpp"
};

} // namespace nmea0183_connector
