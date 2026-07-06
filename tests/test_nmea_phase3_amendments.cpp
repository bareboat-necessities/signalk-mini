#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static std::string sentence(const std::string& body) {
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body.c_str());
    std::string out = "$";
    out += body;
    out += "*";
    out.push_back(nmea0183_connector::to_hex((cs >> 4) & 0x0f));
    out.push_back(nmea0183_connector::to_hex(cs & 0x0f));
    return out;
}

static void feed(signalk_mini::SignalKMiniApp<float>& app,
                 const std::string& body,
                 uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    if (!app.nmea0183().feed_line(line.c_str(), 1, now_us)) {
        std::fprintf(stderr, "feed failed: %s\nlast_error=%s\n", line.c_str(), app.nmea0183().last_error());
        std::exit(1);
    }
}

static void require_abk_acknowledgement() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "AIABK,123456789,A,6,3,0", now_us);

    const auto& ack = app.store().model().ais.acknowledgement;
    REQUIRE(ack.mmsi.valid);
    REQUIRE(ack.mmsi.value == 123456789);
    REQUIRE(ack.destination_mmsi[0].value == 123456789);
    REQUIRE(ack.message_type.value == 6);
    REQUIRE(ack.sequence_number[0].value == 3);
    REQUIRE(ack.acknowledgement_count.value == 1);

    const auto& status = app.store().model().ais.data_link_status;
    REQUIRE(std::strcmp(status.station_id, "123456789") == 0);
    REQUIRE(std::strcmp(status.slot_status, "A") == 0);
    REQUIRE(std::strcmp(status.status_text, "acknowledged") == 0);
    REQUIRE(status.field_count.value == 5);
}

static void require_aga_group_assignment() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "AIAGA,123456789,1,31,3745.000,N,12230.000,W,3730.000,N,12245.000,W,10,2,5,A", now_us);

    const auto& group = app.store().model().ais.group_assignment;
    REQUIRE(group.mmsi.value == 123456789);
    REQUIRE(group.station_type.value == 1);
    REQUIRE(group.ship_type.value == 31);
    REQUIRE(group.northeast_lat_deg.valid);
    REQUIRE(group.northeast_lon_deg.valid);
    REQUIRE(group.southwest_lat_deg.valid);
    REQUIRE(group.southwest_lon_deg.valid);
    NEAR(group.northeast_lat_deg.value, 37.75f, 0.001f);
    NEAR(group.southwest_lat_deg.value, 37.50f, 0.001f);
    NEAR(group.northeast_lon_deg.value, -122.50f, 0.001f);
    NEAR(group.southwest_lon_deg.value, -122.75f, 0.001f);
    REQUIRE(group.report_interval.value == 10);
    REQUIRE(group.txrx_mode.value == 2);
    REQUIRE(group.quiet_time_min.value == 5);
}

static void require_bcl_base_station_location() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "AIBCL,987654321,1,,3740.000,N,12220.000,W,1,BASE-A,A", now_us);

    const auto& base = app.store().model().ais.base_station;
    REQUIRE(base.mmsi.value == 987654321);
    REQUIRE(base.epfd_type.value == 1);
    REQUIRE(base.position_accuracy == true);
    REQUIRE(base.latitude_deg.valid);
    REQUIRE(base.longitude_deg.valid);
    NEAR(base.latitude_deg.value, 37.666667f, 0.001f);
    NEAR(base.longitude_deg.value, -122.333333f, 0.001f);
}

static void require_mob_notification() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ECMOB,MOB01,A,123519,4807.038,N,01131.000,E,Button pressed", now_us);

    const auto& mob = app.store().model().notifications.special.smv;
    REQUIRE(std::strcmp(mob.message_id, "MOB01") == 0);
    REQUIRE(std::strcmp(mob.event_type, "MOB") == 0);
    REQUIRE(mob.status == 'A');
    REQUIRE(mob.utc_time_s.valid);
    REQUIRE(mob.latitude_deg.valid);
    REQUIRE(mob.longitude_deg.valid);
    NEAR(mob.utc_time_s.value, 45319.0f, 0.001f);
    NEAR(mob.latitude_deg.value, 48.1173f, 0.001f);
    NEAR(mob.longitude_deg.value, 11.516667f, 0.001f);
    REQUIRE(std::strstr(mob.description, "Button") != nullptr);
    REQUIRE(mob.field_count.value == 8);
}

int main() {
    require_abk_acknowledgement();
    require_aga_group_assignment();
    require_bcl_base_station_location();
    require_mob_notification();
    return 0;
}
