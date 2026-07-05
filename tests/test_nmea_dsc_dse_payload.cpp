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

static void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

int main() {
    {
        signalk_mini::SignalKMiniApp<float> app;
        uint64_t now_us = 0;
        feed(app, "CDDSC,120,338040079,100,10,20,0123407356,1234,366123456,00,B,E", now_us);
        feed(app, "CDDSE,1,1,A,338040079,00,45894494", now_us);
        const auto& call = app.store().model().comm.dsc.latest_call;
        REQUIRE(call.expansion_received == true);
        REQUIRE(std::strcmp(call.dse_payload, "45894494") == 0);
        REQUIRE(std::strcmp(call.dse_decoded_text, "45894494") == 0);
        REQUIRE(call.dse_payload_type == ship_data_model::DscDsePayloadType::digits);
        REQUIRE(call.dse_ascii_valid == true);
        REQUIRE(call.dse_payload_length.value == 8);
        REQUIRE(call.dse_digit_count.value == 8);
    }

    {
        signalk_mini::SignalKMiniApp<float> app;
        uint64_t now_us = 0;
        feed(app, "CDDSC,120,338040079,100,10,20,0123407356,1234,366123456,00,B,E", now_us);
        feed(app, "CDDSE,1,1,A,338040079,00,EXPANDED", now_us);
        const auto& call = app.store().model().comm.dsc.latest_call;
        REQUIRE(call.dse_payload_type == ship_data_model::DscDsePayloadType::text);
        REQUIRE(call.dse_ascii_valid == true);
        REQUIRE(call.dse_payload_length.value == 8);
        REQUIRE(call.dse_digit_count.value == 0);
        REQUIRE(std::strcmp(call.dse_decoded_text, "EXPANDED") == 0);
    }

    {
        signalk_mini::SignalKMiniApp<float> app;
        uint64_t now_us = 0;
        feed(app, "CDDSC,120,338040079,100,10,20,0123407356,1234,366123456,00,B,E", now_us);
        feed(app, "CDDSE,1,1,A,338040079,00,AB12", now_us);
        const auto& call = app.store().model().comm.dsc.latest_call;
        REQUIRE(call.dse_payload_type == ship_data_model::DscDsePayloadType::mixed_ascii);
        REQUIRE(call.dse_ascii_valid == true);
        REQUIRE(call.dse_payload_length.value == 4);
        REQUIRE(call.dse_digit_count.value == 2);
        REQUIRE(std::strcmp(call.dse_decoded_text, "AB12") == 0);
    }

    {
        signalk_mini::SignalKMiniApp<float> app;
        uint64_t now_us = 0;
        feed(app, "CDDSC,120,338040079,100,10,20,0123407356,1234,366123456,00,B,E", now_us);
        feed(app, "CDDSE,2,1,A,338040079,00,HELLO ", now_us);
        feed(app, "CDDSE,2,2,A,338040079,00,DSC", now_us);
        const auto& call = app.store().model().comm.dsc.latest_call;
        REQUIRE(call.dse_payload_type == ship_data_model::DscDsePayloadType::text);
        REQUIRE(call.dse_ascii_valid == true);
        REQUIRE(call.dse_payload_length.value == 9);
        REQUIRE(call.dse_digit_count.value == 0);
        REQUIRE(std::strcmp(call.dse_payload, "HELLO DSC") == 0);
        REQUIRE(std::strcmp(call.dse_decoded_text, "HELLO DSC") == 0);
    }

    return 0;
}
