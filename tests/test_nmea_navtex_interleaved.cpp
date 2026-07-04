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

static const ship_data_model::NavtexReceivedMessageData<float>* find_navtex_id(
    const ship_data_model::NavtexMessageHistoryData<float>& history,
    const char* id) {
    for (uint8_t i = 0; i < ship_data_model::NAVTEX_MESSAGE_HISTORY_CAPACITY; ++i) {
        if (std::strcmp(history.messages[i].navtex_message_id, id) == 0) return &history.messages[i];
    }
    return nullptr;
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CRNRX,2,1,101,ZCZC QA12 FIRST ", now_us);
    REQUIRE(app.nmea0183().message_state().active_navtex_slot.valid == true);
    const int32_t first_slot = app.nmea0183().message_state().active_navtex_slot.value;
    REQUIRE(first_slot >= 0 && first_slot < nmea0183_connector::NMEA_NAVTEX_MULTIPART_SLOT_COUNT);
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == false);

    feed(app, "CRNRX,2,1,202,ZCZC QB34 SECOND ", now_us);
    REQUIRE(app.nmea0183().message_state().active_navtex_slot.valid == true);
    const int32_t second_slot = app.nmea0183().message_state().active_navtex_slot.value;
    REQUIRE(second_slot >= 0 && second_slot < nmea0183_connector::NMEA_NAVTEX_MULTIPART_SLOT_COUNT);
    REQUIRE(second_slot != first_slot);
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == false);

    feed(app, "CRNRX,2,2,101,MESSAGE NNNN", now_us);
    REQUIRE(app.nmea0183().message_state().active_navtex_slot.value == first_slot);
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == true);
    REQUIRE(std::strcmp(app.store().model().notifications.navtex.received.navtex_message_id, "QA12") == 0);
    REQUIRE(std::strcmp(app.store().model().notifications.navtex.received.body_text, "FIRST MESSAGE") == 0);

    feed(app, "CRNRX,2,2,202,MESSAGE NNNN", now_us);
    REQUIRE(app.nmea0183().message_state().active_navtex_slot.value == second_slot);
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == true);
    REQUIRE(std::strcmp(app.store().model().notifications.navtex.received.navtex_message_id, "QB34") == 0);
    REQUIRE(std::strcmp(app.store().model().notifications.navtex.received.body_text, "SECOND MESSAGE") == 0);

    const auto& history = app.store().model().notifications.navtex.history;
    REQUIRE(history.count.value == 2);
    const auto* first = find_navtex_id(history, "QA12");
    const auto* second = find_navtex_id(history, "QB34");
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(std::strcmp(first->body_text, "FIRST MESSAGE") == 0);
    REQUIRE(std::strcmp(second->body_text, "SECOND MESSAGE") == 0);

    return 0;
}
