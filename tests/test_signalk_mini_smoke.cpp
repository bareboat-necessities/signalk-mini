#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static std::string sentence(const char* body) {
    char out[256];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body, nmea0183_connector::to_hex((cs >> 4) & 0x0f), nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

static void test_first_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "GPAAM,A,A,0.10,N,WP001", now_us);
    REQUIRE(app.store().model().route.waypoint_arrival.arrival_circle_entered.value);
    REQUIRE(app.store().model().route.waypoint_arrival.perpendicular_passed.value);
    NEAR(app.store().model().route.waypoint_arrival.arrival_radius_nmi.value, 0.10f, 0.0001f);
    REQUIRE(std::strcmp(app.store().model().route.waypoint_arrival.waypoint_id, "WP001") == 0);

    feed(app, "GPALM,1,1,12,2045,0,0.1,200.0,0.2,0.3,5153.5,0.4,0.5,0.6,0.7,0.8", now_us);
    REQUIRE(app.store().model().gnss.almanac.satellite_prn.value == 12);
    REQUIRE(app.store().model().gnss.almanac.gnss_week.value == 2045);
    NEAR(app.store().model().gnss.almanac.sqrt_semi_major_axis.value, 5153.5f, 0.001f);

    feed(app, "GPAPA,A,A,0.05,R,N,A,V,123.4,T,WP002", now_us);
    NEAR(app.store().model().route.apb.xte_nmi.value, 0.05f, 0.0001f);
    REQUIRE(app.store().model().route.apb.arrival_circle_entered.value);
    REQUIRE(!app.store().model().route.apb.perpendicular_passed.value);
    NEAR(app.store().model().route.apb.origin_to_destination_bearing_deg.value, 123.4f, 0.001f);

    feed(app, "GPAPB,A,A,0.02,L,N,A,A,100.0,T,WP003,110.0,T,120.0,T", now_us);
    NEAR(app.store().model().route.apb.xte_nmi.value, -0.02f, 0.0001f);
    NEAR(app.store().model().route.apb.present_to_destination_bearing_deg.value, 110.0f, 0.001f);
    NEAR(app.store().model().route.apb.heading_to_steer_deg.value, 120.0f, 0.001f);

    feed(app, "GPBOD,100.0,T,101.0,M,TO,FROM", now_us);
    NEAR(app.store().model().route.waypoint.bearing_true_deg.value, 100.0f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().route.waypoint.to_waypoint_id, "TO") == 0);

    feed(app, "GPBWC,225444,4917.24,N,12309.57,W,054.7,T,034.4,M,10.5,N,WP004", now_us);
    NEAR(app.store().model().route.waypoint.utc_time_s.value, 82484.0f, 0.001f);
    NEAR(app.store().model().route.waypoint.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().route.waypoint.longitude_deg.value, -123.1595f, 0.001f);
    NEAR(app.store().model().route.waypoint.distance_nmi.value, 10.5f, 0.001f);

    feed(app, "GPBWR,225445,4917.25,N,12309.58,W,055.7,T,035.4,M,11.5,N,WP005", now_us);
    NEAR(app.store().model().route.waypoint.distance_nmi.value, 11.5f, 0.001f);

    feed(app, "GPBWW,200.0,T,201.0,M,TO2,FROM2", now_us);
    NEAR(app.store().model().route.waypoint.bearing_true_deg.value, 200.0f, 0.001f);

    feed(app, "SDDBK,32.8,f,10.0,M,5.5,F", now_us);
    REQUIRE(app.store().model().sea.depth_below_keel_m.valid);
    NEAR(app.store().model().sea.depth_below_keel_m.value, 10.0f, 0.001f);

    feed(app, "SDDBS,19.7,f,6.0,M,3.3,F", now_us);
    REQUIRE(app.store().model().sea.depth_below_surface_m.valid);
    NEAR(app.store().model().sea.depth_below_surface_m.value, 6.0f, 0.001f);
}

static void test_second_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "GPDCN,5B,A,12.3,A,B,23.4,V,C,34.5,A,Y,N,Y,0.8,N,1", now_us);
    REQUIRE(std::strcmp(app.store().model().nav.decca.chain_id, "5B") == 0);
    NEAR(app.store().model().nav.decca.red_line_of_position.value, 12.3f, 0.001f);
    REQUIRE(app.store().model().nav.decca.green_master_status == 'V');
    REQUIRE(app.store().model().nav.decca.fix_data_basis.value == 1);

    feed(app, "GPDTM,W84,,0.10,N,0.20,W,1.5,W84", now_us);
    REQUIRE(std::strcmp(app.store().model().nav.datum.local_datum_code, "W84") == 0);
    NEAR(app.store().model().nav.datum.latitude_offset_min.value, 0.10f, 0.001f);
    NEAR(app.store().model().nav.datum.longitude_offset_min.value, -0.20f, 0.001f);

    feed(app, "GPFSI,156800000,156800000,T,25", now_us);
    NEAR(app.store().model().comm.radio_frequency_set.transmitting_frequency_hz.value, 156800000.0f, 32.0f);
    REQUIRE(app.store().model().comm.radio_frequency_set.communication_mode == 'T');
    REQUIRE(app.store().model().comm.radio_frequency_set.power_level.value == 25);

    feed(app, "GPGBS,225444,1.1,2.2,3.3,12,0.05,4.4,0.6", now_us);
    NEAR(app.store().model().gnss.fault_detection.utc_time_s.value, 82484.0f, 0.001f);
    NEAR(app.store().model().gnss.fault_detection.expected_error_lat_m.value, 1.1f, 0.001f);
    REQUIRE(app.store().model().gnss.fault_detection.failed_satellite_prn.value == 12);
}

static void test_third_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "GPGLC,7980,12345.6,A,11.1,A,22.2,V,33.3,A,44.4,A,55.5,V", now_us);
    NEAR(app.store().model().nav.legacy_timing.gri_us_div_10.value, 7980.0f, 0.001f);
    NEAR(app.store().model().nav.legacy_timing.master_toa_us.value, 12345.6f, 0.001f);
    REQUIRE(app.store().model().nav.legacy_timing.delta_status[1] == 'V');

    feed(app, "GPGRS,225444,1,0.1,-0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2", now_us);
    NEAR(app.store().model().gnss.range_residual.utc_time_s.value, 82484.0f, 0.001f);
    REQUIRE(app.store().model().gnss.range_residual.mode.value == 1);
    NEAR(app.store().model().gnss.range_residual.satellite_residual_m[1].value, -0.2f, 0.001f);

    feed(app, "GPGST,225444,1.1,2.2,3.3,45.0,4.4,5.5,6.6", now_us);
    NEAR(app.store().model().gnss.noise_statistics.rms_range_stddev_m.value, 1.1f, 0.001f);
    NEAR(app.store().model().gnss.noise_statistics.semi_major_orientation_deg.value, 45.0f, 0.001f);

    feed(app, "GPGSA,A,3,04,05,09,12,24,25,29,31,,,,,1.8,1.0,1.5", now_us);
    REQUIRE(app.store().model().gnss.dop_active_satellites.selection_mode == 'A');
    REQUIRE(app.store().model().gnss.dop_active_satellites.fix_mode.value == 3);
    REQUIRE(app.store().model().gnss.dop_active_satellites.satellite_prn[0].value == 4);
    NEAR(app.store().model().gnss.dop_active_satellites.hdop.value, 1.0f, 0.001f);

    feed(app, "GPGSV,2,1,08,01,40,083,41,02,17,308,42,03,22,123,43,04,10,010,35", now_us);
    REQUIRE(app.store().model().gnss.satellites_in_view.total_messages.value == 2);
    REQUIRE(app.store().model().gnss.satellites_in_view.satellite_prn[2].value == 3);
    NEAR(app.store().model().gnss.satellites_in_view.azimuth_true_deg[0].value, 83.0f, 0.001f);

    feed(app, "GPGTD,1.1,2.2,3.3,4.4,5.5", now_us);
    NEAR(app.store().model().nav.legacy_delta.value[4].value, 5.5f, 0.001f);

    feed(app, "GPGXA,225444,4917.24,N,12309.57,W,WPX,7", now_us);
    NEAR(app.store().model().nav.transit_fix.utc_time_s.value, 82484.0f, 0.001f);
    NEAR(app.store().model().nav.transit_fix.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().nav.transit_fix.longitude_deg.value, -123.1595f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().nav.transit_fix.waypoint_id, "WPX") == 0);

    feed(app, "HCHDT,081.7,T", now_us);
    NEAR(app.store().model().ins.imu.heading_true_deg.value, 81.7f, 0.001f);
    feed(app, "HCHDM,082.3,M", now_us);
    NEAR(app.store().model().ins.imu.heading_magnetic_deg.value, 82.3f, 0.001f);
    feed(app, "HCHDG,083.4,1.2,E,13.1,W", now_us);
    NEAR(app.store().model().ins.imu.magnetic_deviation_deg.value, 1.2f, 0.001f);
    NEAR(app.store().model().ins.imu.magnetic_variation_deg.value, -13.1f, 0.001f);
}

static void test_fourth_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "GPHFB,12.3,M,45.6,M", now_us);
    NEAR(app.store().model().sea.trawl_headrope_to_footrope_m.value, 12.3f, 0.001f);
    NEAR(app.store().model().sea.trawl_headrope_to_bottom_m.value, 45.6f, 0.001f);

    feed(app, "GPHSC,123.4,T,121.1,M", now_us);
    NEAR(app.store().model().route.heading_steering_command.heading_true_deg.value, 123.4f, 0.001f);
    NEAR(app.store().model().route.heading_steering_command.heading_magnetic_deg.value, 121.1f, 0.001f);

    feed(app, "GPITS,7980,20.5,1.2,11.1,A,22.2,V,33.3,A,44.4,A,55.5,V", now_us);
    NEAR(app.store().model().nav.legacy_timing.gri_us_div_10.value, 7980.0f, 0.001f);
    NEAR(app.store().model().nav.legacy_timing.master_relative_snr_db.value, 20.5f, 0.001f);
    NEAR(app.store().model().nav.legacy_timing.master_relative_ecd.value, 1.2f, 0.001f);
    REQUIRE(app.store().model().nav.legacy_timing.delta_status[1] == 'V');

    feed(app, "GPMSK,283.5,A,100,M,284.5", now_us);
    NEAR(app.store().model().comm.beacon_control.frequency_khz.value, 283.5f, 0.001f);
    REQUIRE(app.store().model().comm.beacon_control.frequency_mode == 'A');
    REQUIRE(app.store().model().comm.beacon_control.bit_rate_bps.value == 100);
    REQUIRE(app.store().model().comm.beacon_control.bit_rate_mode == 'M');
    NEAR(app.store().model().comm.beacon_control.status_frequency_khz.value, 284.5f, 0.001f);

    feed(app, "GPMSS,55.5,12.2,283.5,100,7", now_us);
    NEAR(app.store().model().comm.beacon_status.signal_strength_db_uv.value, 55.5f, 0.001f);
    NEAR(app.store().model().comm.beacon_status.signal_to_noise_ratio_db.value, 12.2f, 0.001f);
    NEAR(app.store().model().comm.beacon_status.beacon_frequency_khz.value, 283.5f, 0.001f);
    REQUIRE(app.store().model().comm.beacon_status.beacon_bit_rate_bps.value == 100);
    REQUIRE(app.store().model().comm.beacon_status.status.value == 7);

    feed(app, "IIMTW,18.7,C", now_us);
    NEAR(app.store().model().sea.temperature_c.value, 18.7f, 0.001f);
}

static void test_fifth_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "GPOLN,A12,B34,C56", now_us);
    REQUIRE(std::strcmp(app.store().model().nav.omega_lane_numbers.pair[0], "A12") == 0);
    REQUIRE(std::strcmp(app.store().model().nav.omega_lane_numbers.pair[2], "C56") == 0);

    feed(app, "IIOSD,123.4,A,234.5,T,6.7,B,210.0,1.2,N", now_us);
    NEAR(app.store().model().nav.own_ship.heading_true_deg.value, 123.4f, 0.001f);
    NEAR(app.store().model().nav.own_ship.course_deg.value, 234.5f, 0.001f);
    NEAR(app.store().model().nav.own_ship.speed_kn.value, 6.7f, 0.001f);
    NEAR(app.store().model().nav.own_ship.set_true_deg.value, 210.0f, 0.001f);
    NEAR(app.store().model().sea.current_speed_kn.value, 1.2f, 0.001f);

    feed(app, "GPR00,WP001,WP002,WP003", now_us);
    REQUIRE(app.store().model().route.active.waypoint_count.value == 3);
    REQUIRE(std::strcmp(app.store().model().route.active.waypoint_id[1], "WP002") == 0);

    feed(app, "GPRMA,A,4917.24,N,12309.57,W,11.1,22.2,5.5,083.2,13.1,W", now_us);
    NEAR(app.store().model().nav.rma.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().nav.rma.longitude_deg.value, -123.1595f, 0.001f);
    NEAR(app.store().model().nav.rma.time_difference_a_us.value, 11.1f, 0.001f);
    NEAR(app.store().model().nav.rma.speed_kn.value, 5.5f, 0.001f);
    NEAR(app.store().model().nav.rma.track_deg.value, 83.2f, 0.001f);
    NEAR(app.store().model().nav.rma.magnetic_variation_deg.value, -13.1f, 0.001f);
}

static void test_sixth_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "IIRPM,E,1,1234.5,42.0,A", now_us);
    REQUIRE(app.store().model().propulsion.revolutions.source_type == 'E');
    REQUIRE(app.store().model().propulsion.revolutions.number.value == 1);
    NEAR(app.store().model().propulsion.revolutions.speed_rpm.value, 1234.5f, 0.001f);
    NEAR(app.store().model().propulsion.revolutions.propeller_pitch_percent.value, 42.0f, 0.001f);

    feed(app, "RARSD,1.0,010.0,2.0,020.0,3.0,030.0,4.0,040.0,5.0,050.0,12.0,N", now_us);
    NEAR(app.store().model().nav.radar_system.origin_range_nmi[0].value, 1.0f, 0.001f);
    NEAR(app.store().model().nav.radar_system.variable_range_marker_nmi[1].value, 4.0f, 0.001f);
    NEAR(app.store().model().nav.radar_system.cursor_range_nmi.value, 5.0f, 0.001f);
    NEAR(app.store().model().nav.radar_system.cursor_bearing_deg.value, 50.0f, 0.001f);
    NEAR(app.store().model().nav.radar_system.range_scale_nmi.value, 12.0f, 0.001f);

    feed(app, "GPRTE,1,1,c,WP001,WP002,WP003", now_us);
    REQUIRE(app.store().model().route.active.total_messages.value == 1);
    REQUIRE(app.store().model().route.active.message_number.value == 1);
    REQUIRE(app.store().model().route.active.mode == 'c');
    REQUIRE(app.store().model().route.active.waypoint_count.value == 3);
    REQUIRE(std::strcmp(app.store().model().route.active.waypoint_id[2], "WP003") == 0);

    feed(app, "RFSFI,1,1,156800000,A,156850000,M", now_us);
    REQUIRE(app.store().model().nav.scanning_frequency.total_messages.value == 1);
    NEAR(app.store().model().nav.scanning_frequency.frequency_hz[1].value, 156850000.0f, 32.0f);
    REQUIRE(app.store().model().nav.scanning_frequency.mode[1] == 'M');

    feed(app, "GPSTN,12", now_us);
    REQUIRE(app.store().model().nav.multiple_data_id.talker_id_number.value == 12);
}

static void test_seventh_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "IITDS,1.2,M,33.4,M,56.7,M", now_us);
    NEAR(app.store().model().sea.trawl_door_centerline_offset_m.value, 1.2f, 0.001f);
    NEAR(app.store().model().sea.trawl_door_along_centerline_m.value, 33.4f, 0.001f);
    NEAR(app.store().model().sea.trawl_depth_below_surface_m.value, 56.7f, 0.001f);

    feed(app, "IITPR,120.5,M,034.0,,60.0,M", now_us);
    NEAR(app.store().model().sea.trawl_relative_range_m.value, 120.5f, 0.001f);
    NEAR(app.store().model().sea.trawl_relative_bearing_deg.value, 34.0f, 0.001f);
    NEAR(app.store().model().sea.trawl_depth_below_surface_m.value, 60.0f, 0.001f);

    feed(app, "IITPT,130.5,M,210.0,,65.0,M", now_us);
    NEAR(app.store().model().sea.trawl_true_range_m.value, 130.5f, 0.001f);
    NEAR(app.store().model().sea.trawl_true_bearing_deg.value, 210.0f, 0.001f);
    NEAR(app.store().model().sea.trawl_depth_below_surface_m.value, 65.0f, 0.001f);

    feed(app, "GPTRF,123519,010726,4917.24,N,12309.57,W,45.0,3,12,1.5,7,A", now_us);
    REQUIRE(std::strcmp(app.store().model().nav.transit_fix.date_ddmmyy, "010726") == 0);
    NEAR(app.store().model().nav.transit_fix.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().nav.transit_fix.longitude_deg.value, -123.1595f, 0.001f);
    NEAR(app.store().model().nav.transit_fix.elevation_angle_deg.value, 45.0f, 0.001f);
    REQUIRE(app.store().model().nav.transit_fix.iterations.value == 3);
    REQUIRE(app.store().model().nav.transit_fix.doppler_intervals.value == 12);
    NEAR(app.store().model().nav.transit_fix.update_distance_nmi.value, 1.5f, 0.001f);
    REQUIRE(app.store().model().nav.transit_fix.satellite_number.value == 7);
    REQUIRE(app.store().model().nav.transit_fix.data_validity == 'A');

    feed(app, "RATTM,7,2.5,045.0,T,12.3,090.0,T,0.8,15.0,N,TGT7,A,R", now_us);
    REQUIRE(app.store().model().ais.tracked_target.target_number.value == 7);
    NEAR(app.store().model().ais.tracked_target.distance_nmi.value, 2.5f, 0.001f);
    NEAR(app.store().model().ais.tracked_target.bearing_deg.value, 45.0f, 0.001f);
    REQUIRE(app.store().model().ais.tracked_target.bearing_reference == 'T');
    NEAR(app.store().model().ais.tracked_target.speed_kn.value, 12.3f, 0.001f);
    NEAR(app.store().model().ais.tracked_target.course_deg.value, 90.0f, 0.001f);
    NEAR(app.store().model().ais.tracked_target.cpa_distance_nmi.value, 0.8f, 0.001f);
    NEAR(app.store().model().ais.tracked_target.tcpa_min.value, 15.0f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().ais.tracked_target.target_name, "TGT7") == 0);
    REQUIRE(app.store().model().ais.tracked_target.target_status == 'A');
    REQUIRE(app.store().model().ais.tracked_target.reference_target == 'R');
}

static void test_eighth_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "IIVBW,5.1,-0.2,A,5.4,-0.1,A", now_us);
    NEAR(app.store().model().sea.longitudinal_water_speed_kn.value, 5.1f, 0.001f);
    NEAR(app.store().model().sea.transverse_water_speed_kn.value, -0.2f, 0.001f);
    REQUIRE(app.store().model().sea.water_speed_status == 'A');
    NEAR(app.store().model().sea.longitudinal_ground_speed_kn.value, 5.4f, 0.001f);
    NEAR(app.store().model().sea.transverse_ground_speed_kn.value, -0.1f, 0.001f);
    REQUIRE(app.store().model().sea.ground_speed_status == 'A');

    feed(app, "VDVDR,123.4,T,121.1,M,1.8,N", now_us);
    NEAR(app.store().model().sea.current_direction_deg.value, 123.4f, 0.001f);
    NEAR(app.store().model().sea.current_direction_magnetic_deg.value, 121.1f, 0.001f);
    NEAR(app.store().model().sea.current_speed_kn.value, 1.8f, 0.001f);

    feed(app, "IIVLW,1234.5,N,67.8,N", now_us);
    NEAR(app.store().model().sea.total_distance_nmi.value, 1234.5f, 0.001f);
    NEAR(app.store().model().sea.trip_distance_nmi.value, 67.8f, 0.001f);

    feed(app, "IIVPW,4.2,N,2.16,M", now_us);
    NEAR(app.store().model().sea.speed_parallel_to_wind_kn.value, 4.2f, 0.001f);
    NEAR(app.store().model().sea.speed_parallel_to_wind_m_s.value, 2.16f, 0.001f);
}

int main() {
    using Real = float;

    signalk_mini::SignalKMiniConfig default_config;
    REQUIRE(default_config.connector_count == 1);
    REQUIRE(default_config.connectors != nullptr);
    REQUIRE(default_config.connectors[0].enabled);
    REQUIRE(default_config.connectors[0].protocol.kind == signalk_mini::ConnectorProtocol::Nmea0183);
    REQUIRE(default_config.connectors[0].transport.kind == signalk_mini::ConnectorTransport::Udp);
    REQUIRE(std::strcmp(default_config.connectors[0].transport.udp.listen_host, "0.0.0.0") == 0);
    REQUIRE(default_config.connectors[0].transport.udp.listen_port == 10110);
    REQUIRE(default_config.connectors[0].protocol.nmea0183.validate_checksum == false);

    signalk_mini::SignalKMiniApp<Real> app;
    uint64_t now_us = 0;
    feed(app, "GPRMC,142533.00,A,4042.6142,N,07400.4168,W,5.4,083.2,010726,13.1,W,A", now_us);
    feed(app, "HCHDT,081.7,T", now_us);
    feed(app, "IIMWV,045.0,R,12.8,N,A", now_us);
    feed(app, "SDDPT,5.6,0.4", now_us);

    const auto& model = app.store().model();
    REQUIRE(model.gnss.fix.fix_lat_deg.valid);
    REQUIRE(model.gnss.fix.fix_lon_deg.valid);
    REQUIRE(model.gnss.fix.speed_kn.valid);
    REQUIRE(model.ins.imu.heading_deg.valid);
    REQUIRE(model.wind.apparent.speed_kn.valid);
    REQUIRE(model.sea.depth_m.valid);
    NEAR(model.gnss.fix.fix_lat_deg.value, 40.7102367f, 0.0002f);
    NEAR(model.gnss.fix.fix_lon_deg.value, -74.0069467f, 0.0002f);
    NEAR(model.gnss.fix.speed_kn.value, 5.4f, 0.001f);
    NEAR(model.wind.apparent.speed_kn.value, 12.8f, 0.001f);
    NEAR(model.sea.depth_m.value, 5.6f, 0.001f);

    test_first_smart0183_group(app, now_us);
    test_second_smart0183_group(app, now_us);
    test_third_smart0183_group(app, now_us);
    test_fourth_smart0183_group(app, now_us);
    test_fifth_smart0183_group(app, now_us);
    test_sixth_smart0183_group(app, now_us);
    test_seventh_smart0183_group(app, now_us);
    test_eighth_smart0183_group(app, now_us);

    signalk_mini::SignalKMapper<Real> mapper;
    signalk_mini::ModelChange change;
    bool saw_wind_speed = false;
    while (app.store().changes().pop(change)) {
        signalk_mini::SignalKMappedValue<Real> mapped;
        if (mapper.map_change(app.store().model(), change, mapped) && mapped.path && std::strcmp(mapped.path, "environment.wind.speedApparent") == 0) {
            saw_wind_speed = true;
            NEAR(mapped.number, 6.5848889f, 0.001f);
        }
    }
    REQUIRE(saw_wind_speed);
    return 0;
}
