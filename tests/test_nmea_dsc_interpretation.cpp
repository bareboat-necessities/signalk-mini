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
    uint64_t now_us = 0;

    {
        signalk_mini::SignalKMiniApp<float> app;
        feed(app, "CDDSC,116,338040079,108,10,20,0123407356,1234,366123456,00,B", now_us);
        const auto& call = app.store().model().comm.dsc.latest_call;
        REQUIRE(call.address_type == ship_data_model::DscAddressType::all_ships);
        REQUIRE(call.priority == ship_data_model::DscPriority::safety);
        REQUIRE(app.store().model().notifications.dsc.safety.active == true);
        REQUIRE(app.store().model().comm.dsc.safety_count.value == 1);
    }

    {
        signalk_mini::SignalKMiniApp<float> app;
        feed(app, "CDDSC,114,338040079,100,10,20,0123407356,1234,366123456,00,B", now_us);
        const auto& call = app.store().model().comm.dsc.latest_call;
        REQUIRE(call.address_type == ship_data_model::DscAddressType::group);
        REQUIRE(call.priority == ship_data_model::DscPriority::routine);
    }

    {
        signalk_mini::SignalKMiniApp<float> app;
        feed(app, "CDDSC,102,338040079,110,10,20,0123407356,1234,366123456,00,B", now_us);
        const auto& call = app.store().model().comm.dsc.latest_call;
        REQUIRE(call.address_type == ship_data_model::DscAddressType::geographic_area);
        REQUIRE(call.priority == ship_data_model::DscPriority::urgency);
        REQUIRE(app.store().model().notifications.dsc.urgency.active == true);
        REQUIRE(app.store().model().comm.dsc.urgency_count.value == 1);
    }

    return 0;
}
