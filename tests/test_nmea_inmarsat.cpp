#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

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
                 uint64_t& now_us,
                 uint64_t delta_us = 1000) {
    now_us += delta_us;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

static bool no_inmarsat_message(const ship_data_model::InmarsatSafetyNetData<float>& safetynet) {
    return !safetynet.message_count.valid || safetynet.message_count.value == 0;
}

static void require_single_message_commit() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ICIMK,MSG01,OK,HELLO INMARSAT", now_us);

    const auto& safetynet = app.store().model().notifications.inmarsat.safetynet;
    const auto& msg = safetynet.latest_message;
    REQUIRE(safetynet.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, "MSG01") == 0);
    REQUIRE(std::strcmp(msg.terminal_id, "IC") == 0);
    REQUIRE(std::strcmp(msg.status, "OK") == 0);
    REQUIRE(std::strcmp(msg.message_text, "HELLO INMARSAT") == 0);
    REQUIRE(msg.total_fragments.value == 1);
    REQUIRE(msg.fragment_number.value == 1);
    REQUIRE(msg.text_length.value == 14);
    REQUIRE(msg.field_count.value == 3);
    REQUIRE(msg.complete == true);
    REQUIRE(msg.overflow == false);
    REQUIRE(msg.first_seen_us == now_us);
    REQUIRE(msg.last_update_us == now_us);
}

static void require_multipart_waits_until_complete() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ICIMK,2,1,MSG42,HELLO ", now_us);
    REQUIRE(no_inmarsat_message(app.store().model().notifications.inmarsat.safetynet));
    REQUIRE(app.nmea0183().message_state().inmarsat_message.in_progress == true);
    REQUIRE(app.nmea0183().message_state().inmarsat_message.complete == false);

    feed(app, "ICIMK,2,2,MSG42,WORLD", now_us);
    const auto& safetynet = app.store().model().notifications.inmarsat.safetynet;
    const auto& msg = safetynet.latest_message;
    REQUIRE(safetynet.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, "MSG42") == 0);
    REQUIRE(std::strcmp(msg.terminal_id, "IC") == 0);
    REQUIRE(std::strcmp(msg.message_text, "HELLO WORLD") == 0);
    REQUIRE(msg.total_fragments.value == 2);
    REQUIRE(msg.fragment_number.value == 2);
    REQUIRE(msg.text_length.value == 11);
    REQUIRE(msg.field_count.value == 4);
    REQUIRE(msg.complete == true);
    REQUIRE(msg.overflow == false);
}

static void require_bad_fragment_count() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ICIMK,2,2,MSG42,WORLD", now_us);
    REQUIRE(no_inmarsat_message(app.store().model().notifications.inmarsat.safetynet));
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.bad_fragment_count.value == 1);
    REQUIRE(app.nmea0183().message_state().inmarsat_message.bad_fragment_count.value == 1);
}

static void require_unsupported_payload_count() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    std::string body = "ICIMK,MSG02,OK,";
    body.push_back(static_cast<char>(0x7f));
    feed(app, body, now_us);

    REQUIRE(no_inmarsat_message(app.store().model().notifications.inmarsat.safetynet));
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.unsupported_count.value == 1);
}

static void require_multipart_overflow_propagates() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;
    const std::string a60(60, 'A');
    const std::string b60(60, 'B');

    feed(app, std::string("ICIMK,2,1,BIG,") + a60, now_us);
    REQUIRE(no_inmarsat_message(app.store().model().notifications.inmarsat.safetynet));

    feed(app, std::string("ICIMK,2,2,BIG,") + b60, now_us);
    const auto& safetynet = app.store().model().notifications.inmarsat.safetynet;
    const auto& msg = safetynet.latest_message;
    REQUIRE(safetynet.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, "BIG") == 0);
    REQUIRE(msg.complete == true);
    REQUIRE(msg.overflow == true);
    REQUIRE(msg.text_length.value == 95);
    REQUIRE(std::strlen(msg.message_text) == 95);
}

static void require_message_fields(const std::string& body,
                                   const char* expected_id,
                                   const char* expected_terminal,
                                   const char* expected_status,
                                   const char* expected_text) {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, body, now_us);

    const auto& safetynet = app.store().model().notifications.inmarsat.safetynet;
    const auto& msg = safetynet.latest_message;
    REQUIRE(safetynet.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, expected_id) == 0);
    REQUIRE(std::strcmp(msg.terminal_id, expected_terminal) == 0);
    REQUIRE(std::strcmp(msg.status, expected_status) == 0);
    REQUIRE(std::strcmp(msg.message_text, expected_text) == 0);
    REQUIRE(msg.text_length.value == static_cast<int32_t>(std::strlen(expected_text)));
    REQUIRE(msg.total_fragments.value == 1);
    REQUIRE(msg.fragment_number.value == 1);
    REQUIRE(msg.complete == true);
}

static void require_connector_only_sentence_interpretation() {
    require_message_fields("ICIMK,MSG-K,OK,KEY READY", "MSG-K", "IC", "OK", "KEY READY");
    require_message_fields("ICIMN,MSG-N,READY,NETWORK READY", "MSG-N", "IC", "READY", "NETWORK READY");
    require_message_fields("ICIMR,MSG-R,RCVD,RETURN READY", "MSG-R", "IC", "RCVD", "RETURN READY");
    require_message_fields("CSIMK,MSG-CS,OK,SATCOM READY", "MSG-CS", "CS", "OK", "SATCOM READY");
    require_message_fields("PINM,MSG-P,OK,PROPRIETARY READY", "MSG-P", "PI", "OK", "PROPRIETARY READY");
    require_message_fields("INM,MSG-I,OK,SHORT PREFIX READY", "MSG-I", "IN", "OK", "SHORT PREFIX READY");
}

int main() {
    require_single_message_commit();
    require_multipart_waits_until_complete();
    require_bad_fragment_count();
    require_unsupported_payload_count();
    require_multipart_overflow_propagates();
    require_connector_only_sentence_interpretation();
    return 0;
}
