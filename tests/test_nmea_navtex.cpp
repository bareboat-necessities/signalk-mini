#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static std::string sentence(const char* body) {
    char out[240];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CRNRX,1,1,42,ZCZC QA12 WEATHER WARNING NNNN", now_us);
    const auto& single = app.store().model().notifications.navtex.received;
    REQUIRE(single.total_fragments.value == 1);
    REQUIRE(single.fragment_number.value == 1);
    REQUIRE(std::strcmp(single.sentence_message_id, "42") == 0);
    REQUIRE(std::strcmp(single.navtex_message_id, "QA12") == 0);
    REQUIRE(single.transmitter_id == 'Q');
    REQUIRE(single.subject_indicator == 'A');
    REQUIRE(single.serial_number.value == 12);
    REQUIRE(std::strcmp(single.message_text, "ZCZC QA12 WEATHER WARNING NNNN") == 0);
    REQUIRE(single.complete == true);
    REQUIRE(single.end_of_message == true);
    REQUIRE(single.last_update_us == now_us);

    feed(app, "CRNRX,2,1,77,ZCZC QB34 GALE ", now_us);
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == false);
    feed(app, "CRNRX,2,2,77,WARNING NNNN", now_us);
    const auto& multi = app.store().model().notifications.navtex.received;
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == true);
    REQUIRE(multi.total_fragments.value == 2);
    REQUIRE(multi.fragment_number.value == 2);
    REQUIRE(std::strcmp(multi.sentence_message_id, "77") == 0);
    REQUIRE(std::strcmp(multi.navtex_message_id, "QB34") == 0);
    REQUIRE(multi.transmitter_id == 'Q');
    REQUIRE(multi.subject_indicator == 'B');
    REQUIRE(multi.serial_number.value == 34);
    REQUIRE(std::strcmp(multi.message_text, "ZCZC QB34 GALE WARNING NNNN") == 0);
    REQUIRE(multi.complete == true);
    REQUIRE(multi.end_of_message == true);

    feed(app, "CRNRM,1,1,5,RX1,ABCDE,FGHIJ,ENABLED", now_us);
    const auto& mask = app.store().model().notifications.navtex.receiver_mask;
    REQUIRE(mask.total_fragments.value == 1);
    REQUIRE(mask.fragment_number.value == 1);
    REQUIRE(std::strcmp(mask.sentence_message_id, "5") == 0);
    REQUIRE(std::strcmp(mask.receiver_id, "RX1") == 0);
    REQUIRE(std::strcmp(mask.station_mask, "ABCDE") == 0);
    REQUIRE(std::strcmp(mask.subject_mask, "FGHIJ") == 0);
    REQUIRE(std::strcmp(mask.status_text, "ENABLED") == 0);
    REQUIRE(mask.complete == true);

    return 0;
}
