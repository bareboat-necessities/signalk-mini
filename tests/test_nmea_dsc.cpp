#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static std::string sentence(const char* body) {
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::string out = "$";
    out += body;
    out += "*";
    out.push_back(nmea0183_connector::to_hex((cs >> 4) & 0x0f));
    out.push_back(nmea0183_connector::to_hex(cs & 0x0f));
    return out;
}

static void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us, uint64_t delta_us = 1000) {
    now_us += delta_us;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

static bool no_dsc_call(const ship_data_model::DscCommData<float>& dsc) {
    return !dsc.call_count.valid || dsc.call_count.value == 0;
}

static void require_direct_dsc_commit() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CDDSC,120,338040079,100,10,20,0123407356,1234,366123456,00,B", now_us);

    const auto& call = app.store().model().comm.dsc.latest_call;
    REQUIRE(app.store().model().comm.dsc.call_count.value == 1);
    REQUIRE(call.format_specifier.value == 120);
    REQUIRE(std::strcmp(call.sender_mmsi, "338040079") == 0);
    REQUIRE(call.category.value == 100);
    REQUIRE(call.nature_or_first_telecommand.value == 10);
    REQUIRE(call.communication_or_second_telecommand.value == 20);
    REQUIRE(std::strcmp(call.position_code, "0123407356") == 0);
    REQUIRE(call.latitude_deg.valid);
    REQUIRE(call.longitude_deg.valid);
    REQUIRE(call.latitude_deg.value > 12.56f && call.latitude_deg.value < 12.57f);
    REQUIRE(call.longitude_deg.value > 73.93f && call.longitude_deg.value < 73.94f);
    REQUIRE(call.utc_time_s.valid);
    REQUIRE(call.utc_time_s.value == 45240.0f);
    REQUIRE(std::strcmp(call.address_or_distress_mmsi, "366123456") == 0);
    REQUIRE(std::strcmp(call.field10, "00") == 0);
    REQUIRE(call.end_of_sequence == 'B');
    REQUIRE(call.expansion_expected == false);
    REQUIRE(call.expansion_received == false);
    REQUIRE(call.expansion_timeout == false);
}

static void require_dsc_alert_promotion() {
    signalk_mini::SignalKMiniApp<float> distress_app;
    uint64_t now_us = 0;
    feed(distress_app, "CDDSC,112,338040079,112,107,126,0123407356,1234,338040079,00,B", now_us);
    const auto& distress = distress_app.store().model().notifications.dsc.distress;
    REQUIRE(distress.active == true);
    REQUIRE(std::strcmp(distress.sender_mmsi, "338040079") == 0);
    REQUIRE(distress.category.value == 112);
    REQUIRE(distress.nature_or_first_telecommand.value == 107);
    REQUIRE(distress.latitude_deg.valid);
    REQUIRE(distress.longitude_deg.valid);
    REQUIRE(distress_app.store().model().comm.dsc.distress_count.value == 1);

    signalk_mini::SignalKMiniApp<float> urgency_app;
    now_us = 0;
    feed(urgency_app, "CDDSC,120,338040080,110,10,20,0123407356,1234,366123456,00,B", now_us);
    const auto& urgency = urgency_app.store().model().notifications.dsc.urgency;
    REQUIRE(urgency.active == true);
    REQUIRE(std::strcmp(urgency.sender_mmsi, "338040080") == 0);
    REQUIRE(urgency.category.value == 110);
    REQUIRE(urgency_app.store().model().comm.dsc.urgency_count.value == 1);

    signalk_mini::SignalKMiniApp<float> safety_app;
    now_us = 0;
    feed(safety_app, "CDDSC,120,338040081,108,10,20,0123407356,1234,366123456,00,B", now_us);
    const auto& safety = safety_app.store().model().notifications.dsc.safety;
    REQUIRE(safety.active == true);
    REQUIRE(std::strcmp(safety.sender_mmsi, "338040081") == 0);
    REQUIRE(safety.category.value == 108);
    REQUIRE(safety_app.store().model().comm.dsc.safety_count.value == 1);
}

static void require_dsc_dse_merge() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CDDSC,120,338040079,100,10,20,0123407356,1234,366123456,00,B,E", now_us);
    REQUIRE(no_dsc_call(app.store().model().comm.dsc));

    feed(app, "CDDSE,1,1,A,338040079,00,EXPANDED", now_us);
    const auto& call = app.store().model().comm.dsc.latest_call;
    REQUIRE(app.store().model().comm.dsc.call_count.value == 1);
    REQUIRE(call.expansion_expected == true);
    REQUIRE(call.expansion_received == true);
    REQUIRE(call.expansion_timeout == false);
    REQUIRE(call.dse_total_messages.value == 1);
    REQUIRE(call.dse_message_number.value == 1);
    REQUIRE(call.dse_query_flag == 'A');
    REQUIRE(call.dse_expansion_specifier.value == 0);
    REQUIRE(std::strcmp(call.dse_payload, "EXPANDED") == 0);
}

static void require_orphan_dse_ignored() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CDDSE,1,1,A,338040079,00,ORPHAN", now_us);
    REQUIRE(no_dsc_call(app.store().model().comm.dsc));
    REQUIRE(app.nmea0183().dsc_state().expansion.last_update_us == now_us);
}

static void require_dsc_dse_timeout() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CDDSC,120,338040079,100,10,20,0123407356,1234,366123456,00,B,E", now_us);
    REQUIRE(no_dsc_call(app.store().model().comm.dsc));

    feed(app, "GPTXT,1,1,01,TICK", now_us, 600000);
    const auto& call = app.store().model().comm.dsc.latest_call;
    REQUIRE(app.store().model().comm.dsc.call_count.value == 1);
    REQUIRE(app.store().model().comm.dsc.expansion_timeout_count.value == 1);
    REQUIRE(call.expansion_expected == true);
    REQUIRE(call.expansion_received == false);
    REQUIRE(call.expansion_timeout == true);
}

static void require_multipart_dse_waits_until_complete() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CDDSC,120,338040079,100,10,20,0123407356,1234,366123456,00,B,E", now_us);
    feed(app, "CDDSE,2,1,A,338040079,00,HELLO ", now_us);
    REQUIRE(no_dsc_call(app.store().model().comm.dsc));
    REQUIRE(app.nmea0183().dsc_state().multipart.in_progress == true);

    feed(app, "CDDSE,2,2,A,338040079,00,DSC", now_us);
    const auto& call = app.store().model().comm.dsc.latest_call;
    REQUIRE(app.store().model().comm.dsc.call_count.value == 1);
    REQUIRE(call.expansion_received == true);
    REQUIRE(call.dse_total_messages.value == 2);
    REQUIRE(call.dse_message_number.value == 2);
    REQUIRE(std::strcmp(call.dse_payload, "HELLO DSC") == 0);
}

int main() {
    require_direct_dsc_commit();
    require_dsc_alert_promotion();
    require_dsc_dse_merge();
    require_orphan_dse_ignored();
    require_dsc_dse_timeout();
    require_multipart_dse_waits_until_complete();
    return 0;
}
