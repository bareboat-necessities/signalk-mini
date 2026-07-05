#include <cstdio>
#include <cstdlib>
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

static void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

int main() {
    {
        signalk_mini::SignalKMiniApp<float> app;
        uint64_t now_us = 0;
        feed(app, "CDDSC,112,338040079,112,107,126,0123407356,1234,338040079,00,A", now_us);
        const auto& dsc = app.store().model().comm.dsc;
        REQUIRE(dsc.call_count.value == 1);
        REQUIRE(dsc.latest_call.end_signal_type == ship_data_model::DscEndSignalType::ack);
        REQUIRE(dsc.latest_call.orphan_ack == true);
        REQUIRE(dsc.orphan_ack_count.value == 1);
        REQUIRE(!dsc.acknowledged_count.valid || dsc.acknowledged_count.value == 0);
        REQUIRE(app.store().model().notifications.dsc.distress.active == false);
    }

    {
        signalk_mini::SignalKMiniApp<float> app;
        uint64_t now_us = 0;
        feed(app, "CDDSC,112,338040079,112,107,126,0123407356,1234,338040079,00,B", now_us);
        feed(app, "CDDSC,112,338040079,112,107,126,0123407356,1234,338040079,00,A", now_us);
        const auto& dsc = app.store().model().comm.dsc;
        REQUIRE(dsc.latest_call.end_signal_type == ship_data_model::DscEndSignalType::ack);
        REQUIRE(dsc.latest_call.orphan_ack == false);
        REQUIRE(dsc.acknowledged_count.value == 1);
        REQUIRE(!dsc.orphan_ack_count.valid || dsc.orphan_ack_count.value == 0);
        REQUIRE(dsc.distress_count.value == 1);
        REQUIRE(app.store().model().notifications.dsc.distress.active == true);
        REQUIRE(app.store().model().notifications.dsc.distress.acknowledged == true);
    }

    {
        signalk_mini::SignalKMiniApp<float> app;
        uint64_t now_us = 0;
        feed(app, "CDDSC,112,338040079,112,107,126,0123407356,1234,338040079,00,B", now_us);
        feed(app, "CDDSC,112,338040079,112,107,126,0123507356,1234,338040079,00,A", now_us);
        const auto& dsc = app.store().model().comm.dsc;
        REQUIRE(dsc.latest_call.end_signal_type == ship_data_model::DscEndSignalType::ack);
        REQUIRE(dsc.latest_call.orphan_ack == true);
        REQUIRE(dsc.orphan_ack_count.value == 1);
        REQUIRE(!dsc.acknowledged_count.valid || dsc.acknowledged_count.value == 0);
        REQUIRE(dsc.distress_count.value == 1);
        REQUIRE(app.store().model().notifications.dsc.distress.active == true);
        REQUIRE(app.store().model().notifications.dsc.distress.acknowledged == false);
    }

    return 0;
}
