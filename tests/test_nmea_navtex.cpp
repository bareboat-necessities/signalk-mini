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
    REQUIRE(single.subject_category.value == 1);
    REQUIRE(std::strcmp(single.subject_label, "navigation") == 0);
    REQUIRE(single.subject_is_service == false);
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
    REQUIRE(std::strcmp(history1.messages[0].subject_label, "navigation") == 0);

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
    REQUIRE(multi.subject_category.value == 2);
    REQUIRE(std::strcmp(multi.subject_label, "meteorology") == 0);
    REQUIRE(multi.serial_number.value == 34);
    REQUIRE(std::strcmp(multi.message_text, "ZCZC QB34 ROUTE UPDATE NNNN") == 0);
    REQUIRE(std::strcmp(multi.body_text, "ROUTE UPDATE") == 0);
    REQUIRE(multi.complete == true);
    REQUIRE(multi.end_of_message == true);
    REQUIRE(app.store().model().notifications.navtex.history.count.value == 2);

    feed(app, "CRNRX,2,1,88,ZCZC QE03 ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHIJKLMNOPQRSTUVWXYZ ", now_us);
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == false);
    feed(app, "CRNRX,2,2,88,ABCDEFGHIJKLMNOPQRSTUVWXYZ ABCDEFGHIJKLMNOPQRSTUVWXYZ NNNN", now_us);
    const auto& long_msg = app.store().model().notifications.navtex.received;
    REQUIRE(app.nmea0183().message_state().navtex_message.complete == true);
    REQUIRE(app.nmea0183().message_state().navtex_message.overflow == false);
    REQUIRE(app.nmea0183().message_state().navtex_message.text_length.value > 96);
    REQUIRE(long_msg.text_length.value > 96);
    REQUIRE(long_msg.body_length.value > 96);
    REQUIRE(std::strcmp(long_msg.navtex_message_id, "QE03") == 0);
    REQUIRE(long_msg.subject_category.value == 5);
    REQUIRE(std::strcmp(long_msg.subject_label, "forecast") == 0);
    REQUIRE(long_msg.framing_valid == true);
    REQUIRE(long_msg.end_of_message == true);
    REQUIRE(app.store().model().notifications.navtex.history.count.value == 3);

    feed(app, "CRNRX,1,1,43,ZCZC QA12 WEATHER NOTE NNNN", now_us);
    const auto& duplicate = app.store().model().notifications.navtex.received;
    REQUIRE(duplicate.duplicate == true);
    REQUIRE(duplicate.repeat_count.value == 2);
    REQUIRE(app.store().model().notifications.navtex.history.count.value == 3);
    REQUIRE(app.store().model().notifications.navtex.history.duplicate_count.value == 1);
    const auto* qa12 = find_navtex_id(app.store().model().notifications.navtex.history, "QA12");
    REQUIRE(qa12 != nullptr);
    REQUIRE(qa12->repeat_count.value == 2);
    REQUIRE(qa12->duplicate == true);

    feed(app, "CRNRX,1,1,1,ZCZC QC01 MSG1 NNNN", now_us);
    feed(app, "CRNRX,1,1,2,ZCZC QD02 MSG2 NNNN", now_us);
    feed(app, "CRNRX,1,1,3,ZCZC QE04 MSG3 NNNN", now_us);
    feed(app, "CRNRX,1,1,4,ZCZC QF04 MSG4 NNNN", now_us);
    feed(app, "CRNRX,1,1,5,ZCZC QG05 MSG5 NNNN", now_us);
    feed(app, "CRNRX,1,1,6,ZCZC QH06 MSG6 NNNN", now_us);
    feed(app, "CRNRX,1,1,7,ZCZC QI07 MSG7 NNNN", now_us);
    feed(app, "CRNRX,1,1,8,ZCZC QJ08 MSG8 NNNN", now_us);

    const auto& history = app.store().model().notifications.navtex.history;
    REQUIRE(history.count.value == ship_data_model::NAVTEX_MESSAGE_HISTORY_CAPACITY);
    REQUIRE(history.overwrite_count.value == 3);
    REQUIRE(history.next_index.value == 3);
    REQUIRE(find_navtex_id(history, "QA12") == nullptr);
    REQUIRE(find_navtex_id(history, "QB34") == nullptr);
    REQUIRE(find_navtex_id(history, "QE03") == nullptr);
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
    REQUIRE(mask.enabled_station_count.value == 5);
    REQUIRE(mask.enabled_subject_count.value == 5);
    REQUIRE((mask.station_mask_bits & 0x1fu) == 0x1fu);
    REQUIRE((mask.subject_mask_bits & (1u << ('F' - 'A'))) != 0u);
    REQUIRE((mask.subject_mask_bits & (1u << ('J' - 'A'))) != 0u);
    REQUIRE(std::strcmp(mask.status_text, "ENABLED") == 0);
    REQUIRE(mask.complete == true);

    signalk_mini::SignalKMiniApp<float> malformed_app;
    uint64_t malformed_now_us = 0;
    feed(malformed_app, "CRNRX,1,1,90,QA12 BODY NNNN", malformed_now_us);
    const auto& missing_zczc = malformed_app.store().model().notifications.navtex.received;
    REQUIRE(missing_zczc.framing_valid == false);
    REQUIRE(missing_zczc.end_of_message == true);
    REQUIRE(std::strcmp(missing_zczc.navtex_message_id, "QA12") == 0);
    REQUIRE(std::strcmp(missing_zczc.body_text, "QA12 BODY") == 0);

    feed(malformed_app, "CRNRX,1,1,91,ZCZC QB35 OPEN TEXT", malformed_now_us);
    const auto& missing_eom = malformed_app.store().model().notifications.navtex.received;
    REQUIRE(missing_eom.framing_valid == false);
    REQUIRE(missing_eom.end_of_message == false);
    REQUIRE(std::strcmp(missing_eom.navtex_message_id, "QB35") == 0);
    REQUIRE(std::strcmp(missing_eom.body_text, "OPEN TEXT") == 0);

    signalk_mini::SignalKMiniApp<float> bad_id_app;
    uint64_t bad_id_now_us = 0;
    feed(bad_id_app, "CRNRX,1,1,92,ZCZC QXAB BAD ID NNNN", bad_id_now_us);
    const auto& bad_id = bad_id_app.store().model().notifications.navtex.received;
    REQUIRE(bad_id.framing_valid == true);
    REQUIRE(bad_id.end_of_message == true);
    REQUIRE(bad_id.navtex_message_id[0] == '\0');
    REQUIRE(bad_id.subject_indicator == 0);
    REQUIRE(std::strcmp(bad_id.body_text, "QXAB BAD ID") == 0);

    signalk_mini::SignalKMiniApp<float> lifecycle_app;
    uint64_t lifecycle_now_us = 0;
    feed(lifecycle_app, "CRNRX,1,1,11,ZCZC QC10 FIRST NNNN", lifecycle_now_us);
    feed(lifecycle_app, "CRNRX,1,1,12,ZCZC QD11 SECOND NNNN", lifecycle_now_us);
    auto& lifecycle = lifecycle_app.store().model().notifications.navtex;
    REQUIRE(lifecycle.history.count.value == 2);
    REQUIRE(ship_data_model::navtex_acknowledge_message(lifecycle, "QD11", lifecycle_now_us + 100) == true);
    REQUIRE(find_navtex_id(lifecycle.history, "QD11") != nullptr);
    REQUIRE(find_navtex_id(lifecycle.history, "QD11")->acknowledged == true);
    REQUIRE(lifecycle.received.acknowledged == true);
    REQUIRE(lifecycle.received.last_update_us == lifecycle_now_us + 100);
    REQUIRE(ship_data_model::navtex_acknowledge_message(lifecycle, "QZ99", lifecycle_now_us + 200) == false);

    REQUIRE(ship_data_model::navtex_clear_message(lifecycle, "QC10", lifecycle_now_us + 300) == true);
    REQUIRE(lifecycle.history.count.value == 1);
    REQUIRE(find_navtex_id(lifecycle.history, "QC10") == nullptr);
    REQUIRE(find_navtex_id(lifecycle.history, "QD11") != nullptr);
    REQUIRE(lifecycle.received.navtex_message_id[0] != '\0');

    REQUIRE(ship_data_model::navtex_clear_message(lifecycle, "QD11", lifecycle_now_us + 400) == true);
    REQUIRE(lifecycle.history.count.value == 0);
    REQUIRE(find_navtex_id(lifecycle.history, "QD11") == nullptr);
    REQUIRE(lifecycle.received.navtex_message_id[0] == '\0');
    REQUIRE(ship_data_model::navtex_clear_message(lifecycle, "QD11", lifecycle_now_us + 500) == false);

    feed(lifecycle_app, "CRNRX,1,1,13,ZCZC QE12 THIRD NNNN", lifecycle_now_us);
    REQUIRE(lifecycle.history.count.value == 1);
    ship_data_model::navtex_clear_history(lifecycle);
    REQUIRE(lifecycle.received.navtex_message_id[0] == '\0');
    REQUIRE(lifecycle.history.count.valid == false);
    REQUIRE(find_navtex_id(lifecycle.history, "QE12") == nullptr);

    return 0;
}
