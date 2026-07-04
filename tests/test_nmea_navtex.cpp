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

    feed(app, "CRNRX,1,1,42,ZCZC QA12 WEATHER NOTE NNNN", now_us);
    const auto& single = app.store().model().notifications.navtex.received;
    REQUIRE(single.total_fragments.value == 1);
    REQUIRE(single.fragment_number.value == 1);
    REQUIRE(std::strcmp(single.sentence_message_id, "42") == 0);
    REQUIRE(std::strcmp(single.navtex_message_id, "QA12") == 0);
    REQUIRE(single.transmitter_id == 'Q');
    REQUIRE(single.subject_indicator == 'A');
    REQUIRE(single.serial_number.value == 12);
    REQUIRE(std::strcmp(single.message_text, "ZCZC QA12 WEATHER NOTE NNNN") == 0);
    REQUIRE(std::strcmp(single.body_text, "WEATHER NOTE") == 0);
    REQUIRE(single.complete == true);
    REQUIRE(single.end_of_message == true);
    REQUIRE(single.framing_valid == true);
    REQUIRE(single.duplicate == false);
    REQUIRE(single.repeat_count.value == 1);
    REQUIRE(single.last_update_us == now_us);

    const auto& history1 = app.store().model().notifications.navtex.history;
    REQUIRE(history1.count.value == 1);
    REQUIRE(history1.next_index.value == 1);
    REQUIRE(std::strcmp(history1.messages[0].navtex_message_id, "QA12") == 0);
    REQUIRE(std::strcmp(history1.messages[0].body_text, "WEATHER NOTE") == 0);

    feed(app, "CRNRX,2,1,77,ZCZC QB34 ROUTE ", now_us);
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == false);
    feed(app, "CRNRX,2,2,77,UPDATE NNNN", now_us);
    const auto& multi = app.store().model().notifications.navtex.received;
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == true);
    REQUIRE(multi.total_fragments.value == 2);
    REQUIRE(multi.fragment_number.value == 2);
    REQUIRE(std::strcmp(multi.sentence_message_id, "77") == 0);
    REQUIRE(std::strcmp(multi.navtex_message_id, "QB34") == 0);
    REQUIRE(multi.transmitter_id == 'Q');
    REQUIRE(multi.subject_indicator == 'B');
    REQUIRE(multi.serial_number.value == 34);
    REQUIRE(std::strcmp(multi.message_text, "ZCZC QB34 ROUTE UPDATE NNNN") == 0);
    REQUIRE(std::strcmp(multi.body_text, "ROUTE UPDATE") == 0);
    REQUIRE(multi.complete == true);
    REQUIRE(multi.end_of_message == true);
    REQUIRE(app.store().model().notifications.navtex.history.count.value == 2);

    feed(app, "CRNRX,1,1,43,ZCZC QA12 WEATHER NOTE NNNN", now_us);
    const auto& duplicate = app.store().model().notifications.navtex.received;
    REQUIRE(duplicate.duplicate == true);
    REQUIRE(duplicate.repeat_count.value == 2);
    REQUIRE(app.store().model().notifications.navtex.history.count.value == 2);
    REQUIRE(app.store().model().notifications.navtex.history.duplicate_count.value == 1);
    const auto* qa12 = find_navtex_id(app.store().model().notifications.navtex.history, "QA12");
    REQUIRE(qa12 != nullptr);
    REQUIRE(qa12->repeat_count.value == 2);
    REQUIRE(qa12->duplicate == true);

    feed(app, "CRNRX,1,1,1,ZCZC QC01 MSG1 NNNN", now_us);
    feed(app, "CRNRX,1,1,2,ZCZC QD02 MSG2 NNNN", now_us);
    feed(app, "CRNRX,1,1,3,ZCZC QE03 MSG3 NNNN", now_us);
    feed(app, "CRNRX,1,1,4,ZCZC QF04 MSG4 NNNN", now_us);
    feed(app, "CRNRX,1,1,5,ZCZC QG05 MSG5 NNNN", now_us);
    feed(app, "CRNRX,1,1,6,ZCZC QH06 MSG6 NNNN", now_us);
    feed(app, "CRNRX,1,1,7,ZCZC QI07 MSG7 NNNN", now_us);
    feed(app, "CRNRX,1,1,8,ZCZC QJ08 MSG8 NNNN", now_us);

    const auto& history = app.store().model().notifications.navtex.history;
    REQUIRE(history.count.value == ship_data_model::NAVTEX_MESSAGE_HISTORY_CAPACITY);
    REQUIRE(history.overwrite_count.value == 2);
    REQUIRE(history.next_index.value == 2);
    REQUIRE(find_navtex_id(history, "QA12") == nullptr);
    REQUIRE(find_navtex_id(history, "QB34") == nullptr);
    REQUIRE(find_navtex_id(history, "QJ08") != nullptr);
    REQUIRE(std::strcmp(app.store().model().notifications.navtex.received.body_text, "MSG8") == 0);

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
