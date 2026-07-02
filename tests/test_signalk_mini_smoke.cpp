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
    REQUIRE(app.store().model().navigation.waypoint_arrival.arrival_circle_entered.value);
    REQUIRE(app.store().model().navigation.waypoint_arrival.perpendicular_passed.value);
    NEAR(app.store().model().navigation.waypoint_arrival.arrival_radius_nmi.value, 0.10f, 0.0001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint_arrival.waypoint_id, "WP001") == 0);

    feed(app, "GPALM,1,1,12,2045,0,0.1,200.0,0.2,0.3,5153.5,0.4,0.5,0.6,0.7,0.8", now_us);
    REQUIRE(app.store().model().navigation.gps_almanac.satellite_prn.value == 12);
    REQUIRE(app.store().model().navigation.gps_almanac.gps_week.value == 2045);
    NEAR(app.store().model().navigation.gps_almanac.sqrt_semi_major_axis.value, 5153.5f, 0.001f);

    feed(app, "GPAPA,A,A,0.05,R,N,A,V,123.4,T,WP002", now_us);
    NEAR(app.store().model().navigation.apb.xte_nmi.value, 0.05f, 0.0001f);
    REQUIRE(app.store().model().navigation.apb.arrival_circle_entered.value);
    REQUIRE(!app.store().model().navigation.apb.perpendicular_passed.value);
    NEAR(app.store().model().navigation.apb.origin_to_destination_bearing_deg.value, 123.4f, 0.001f);

    feed(app, "GPAPB,A,A,0.02,L,N,A,A,100.0,T,WP003,110.0,T,120.0,T", now_us);
    NEAR(app.store().model().navigation.apb.xte_nmi.value, -0.02f, 0.0001f);
    NEAR(app.store().model().navigation.apb.present_to_destination_bearing_deg.value, 110.0f, 0.001f);
    NEAR(app.store().model().navigation.apb.heading_to_steer_deg.value, 120.0f, 0.001f);

    feed(app, "GPBOD,100.0,T,101.0,M,TO,FROM", now_us);
    NEAR(app.store().model().navigation.waypoint.bearing_true_deg.value, 100.0f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "TO") == 0);

    feed(app, "GPBWC,225444,4917.24,N,12309.57,W,054.7,T,034.4,M,10.5,N,WP004", now_us);
    NEAR(app.store().model().navigation.waypoint.utc_time_s.value, 82484.0f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.longitude_deg.value, -123.1595f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.distance_nmi.value, 10.5f, 0.001f);

    feed(app, "GPBWR,225445,4917.25,N,12309.58,W,055.7,T,035.4,M,11.5,N,WP005", now_us);
    NEAR(app.store().model().navigation.waypoint.distance_nmi.value, 11.5f, 0.001f);

    feed(app, "GPBWW,200.0,T,201.0,M,TO2,FROM2", now_us);
    NEAR(app.store().model().navigation.waypoint.bearing_true_deg.value, 200.0f, 0.001f);

    feed(app, "SDDBK,32.8,f,10.0,M,5.5,F", now_us);
    REQUIRE(app.store().model().water.depth_below_keel_m.valid);
    NEAR(app.store().model().water.depth_below_keel_m.value, 10.0f, 0.001f);

    feed(app, "SDDBS,19.7,f,6.0,M,3.3,F", now_us);
    REQUIRE(app.store().model().water.depth_below_surface_m.valid);
    NEAR(app.store().model().water.depth_below_surface_m.value, 6.0f, 0.001f);
}

static void test_second_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "GPDCN,5B,A,12.3,A,B,23.4,V,C,34.5,A,Y,N,Y,0.8,N,1", now_us);
    REQUIRE(std::strcmp(app.store().model().navigation.decca.chain_id, "5B") == 0);
    NEAR(app.store().model().navigation.decca.red_line_of_position.value, 12.3f, 0.001f);
    REQUIRE(app.store().model().navigation.decca.green_master_status == 'V');
    REQUIRE(app.store().model().navigation.decca.fix_data_basis.value == 1);

    feed(app, "GPDTM,W84,,0.10,N,0.20,W,1.5,W84", now_us);
    REQUIRE(std::strcmp(app.store().model().navigation.datum.local_datum_code, "W84") == 0);
    NEAR(app.store().model().navigation.datum.latitude_offset_min.value, 0.10f, 0.001f);
    NEAR(app.store().model().navigation.datum.longitude_offset_min.value, -0.20f, 0.001f);

    feed(app, "GPFSI,156800000,156800000,T,25", now_us);
    NEAR(app.store().model().navigation.radio_frequency_set.transmitting_frequency_hz.value, 156800000.0f, 32.0f);
    REQUIRE(app.store().model().navigation.radio_frequency_set.communication_mode == 'T');
    REQUIRE(app.store().model().navigation.radio_frequency_set.power_level.value == 25);

    feed(app, "GPGBS,225444,1.1,2.2,3.3,12,0.05,4.4,0.6", now_us);
    NEAR(app.store().model().navigation.gps_fault.utc_time_s.value, 82484.0f, 0.001f);
    NEAR(app.store().model().navigation.gps_fault.expected_error_lat_m.value, 1.1f, 0.001f);
    REQUIRE(app.store().model().navigation.gps_fault.failed_satellite_prn.value == 12);
}

static void test_third_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
    feed(app, "GPGLC,7980,12345.6,A,11.1,A,22.2,V,33.3,A,44.4,A,55.5,V", now_us);
    NEAR(app.store().model().navigation.legacy_timing.gri_us_div_10.value, 7980.0f, 0.001f);
    NEAR(app.store().model().navigation.legacy_timing.master_toa_us.value, 12345.6f, 0.001f);
    REQUIRE(app.store().model().navigation.legacy_timing.delta_status[1] == 'V');

    feed(app, "GPGRS,225444,1,0.1,-0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2", now_us);
    NEAR(app.store().model().navigation.gps_range_residual.utc_time_s.value, 82484.0f, 0.001f);
    REQUIRE(app.store().model().navigation.gps_range_residual.mode.value == 1);
    NEAR(app.store().model().navigation.gps_range_residual.satellite_residual_m[1].value, -0.2f, 0.001f);

    feed(app, "GPGST,225444,1.1,2.2,3.3,45.0,4.4,5.5,6.6", now_us);
    NEAR(app.store().model().navigation.gps_noise.rms_range_stddev_m.value, 1.1f, 0.001f);
    NEAR(app.store().model().navigation.gps_noise.semi_major_orientation_deg.value, 45.0f, 0.001f);

    feed(app, "GPGSA,A,3,04,05,09,12,24,25,29,31,,,,,1.8,1.0,1.5", now_us);
    REQUIRE(app.store().model().navigation.gps_dop.selection_mode == 'A');
    REQUIRE(app.store().model().navigation.gps_dop.fix_mode.value == 3);
    REQUIRE(app.store().model().navigation.gps_dop.satellite_prn[0].value == 4);
    NEAR(app.store().model().navigation.gps_dop.hdop.value, 1.0f, 0.001f);

    feed(app, "GPGSV,2,1,08,01,40,083,41,02,17,308,42,03,22,123,43,04,10,010,35", now_us);
    REQUIRE(app.store().model().navigation.gps_satellites_in_view.total_messages.value == 2);
    REQUIRE(app.store().model().navigation.gps_satellites_in_view.satellite_prn[2].value == 3);
    NEAR(app.store().model().navigation.gps_satellites_in_view.azimuth_true_deg[0].value, 83.0f, 0.001f);

    feed(app, "GPGTD,1.1,2.2,3.3,4.4,5.5", now_us);
    NEAR(app.store().model().navigation.legacy_delta.value[4].value, 5.5f, 0.001f);

    feed(app, "GPGXA,225444,4917.24,N,12309.57,W,WPX,7", now_us);
    NEAR(app.store().model().navigation.transit_fix.utc_time_s.value, 82484.0f, 0.001f);
    NEAR(app.store().model().navigation.transit_fix.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().navigation.transit_fix.longitude_deg.value, -123.1595f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.transit_fix.waypoint_id, "WPX") == 0);

    feed(app, "HCHDT,081.7,T", now_us);
    NEAR(app.store().model().imu.heading_true_deg.value, 81.7f, 0.001f);
    feed(app, "HCHDM,082.3,M", now_us);
    NEAR(app.store().model().imu.heading_magnetic_deg.value, 82.3f, 0.001f);
    feed(app, "HCHDG,083.4,1.2,E,13.1,W", now_us);
    NEAR(app.store().model().imu.magnetic_deviation_deg.value, 1.2f, 0.001f);
    NEAR(app.store().model().imu.magnetic_variation_deg.value, -13.1f, 0.001f);
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
    REQUIRE(model.navigation.gps.fix_lat_deg.valid);
    REQUIRE(model.navigation.gps.fix_lon_deg.valid);
    REQUIRE(model.navigation.gps.speed_kn.valid);
    REQUIRE(model.imu.heading_deg.valid);
    REQUIRE(model.wind.apparent.speed_kn.valid);
    REQUIRE(model.water.depth_m.valid);
    NEAR(model.navigation.gps.fix_lat_deg.value, 40.7102367f, 0.0002f);
    NEAR(model.navigation.gps.fix_lon_deg.value, -74.0069467f, 0.0002f);
    NEAR(model.navigation.gps.speed_kn.value, 5.4f, 0.001f);
    NEAR(model.wind.apparent.speed_kn.value, 12.8f, 0.001f);
    NEAR(model.water.depth_m.value, 5.6f, 0.001f);

    test_first_smart0183_group(app, now_us);
    test_second_smart0183_group(app, now_us);
    test_third_smart0183_group(app, now_us);

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
