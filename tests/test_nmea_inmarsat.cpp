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

static bool no_inmarsat_message(const ship_data_model::InmarsatData<float>& inmarsat) {
    return !inmarsat.message_count.valid || inmarsat.message_count.value == 0;
}

static void require_single_message_commit() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ICIMK,MSG01,OK,HELLO INMARSAT", now_us);

    const auto& msg = app.store().model().comm.inmarsat.latest_message;
    REQUIRE(app.store().model().comm.inmarsat.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, "MSG01") == 0);
    REQUIRE(std::strcmp(msg.terminal_id, "IC") == 0);
    REQUIRE(std::strcmp(msg.message_type, "IMK") == 0);
    REQUIRE(std::strcmp(msg.message_status, "OK") == 0);
    REQUIRE(std::strcmp(msg.decoded_text, "HELLO INMARSAT") == 0);
    REQUIRE(msg.total_fragments.value == 1);
    REQUIRE(msg.last_fragment_number.value == 1);
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
    REQUIRE(no_inmarsat_message(app.store().model().comm.inmarsat));
    REQUIRE(app.nmea0183().message_state().inmarsat_message.in_progress == true);
    REQUIRE(app.nmea0183().message_state().inmarsat_message.complete == false);

    feed(app, "ICIMK,2,2,MSG42,WORLD", now_us);
    const auto& msg = app.store().model().comm.inmarsat.latest_message;
    REQUIRE(app.store().model().comm.inmarsat.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, "MSG42") == 0);
    REQUIRE(std::strcmp(msg.terminal_id, "IC") == 0);
    REQUIRE(std::strcmp(msg.message_type, "IMK") == 0);
    REQUIRE(std::strcmp(msg.decoded_text, "HELLO WORLD") == 0);
    REQUIRE(msg.total_fragments.value == 2);
    REQUIRE(msg.last_fragment_number.value == 2);
    REQUIRE(msg.text_length.value == 11);
    REQUIRE(msg.field_count.value == 4);
    REQUIRE(msg.complete == true);
    REQUIRE(msg.overflow == false);
}

static void require_bad_fragment_count() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ICIMK,2,2,MSG42,WORLD", now_us);
    REQUIRE(no_inmarsat_message(app.store().model().comm.inmarsat));
    REQUIRE(app.store().model().comm.inmarsat.bad_fragment_count.value == 1);
    REQUIRE(app.nmea0183().message_state().inmarsat_message.bad_fragment_count.value == 1);
}

static void require_unsupported_payload_count() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    std::string body = "ICIMK,MSG02,OK,";
    body.push_back(static_cast<char>(0x7f));
    feed(app, body, now_us);

    REQUIRE(no_inmarsat_message(app.store().model().comm.inmarsat));
    REQUIRE(app.store().model().comm.inmarsat.unsupported_count.value == 1);
}

static void require_multipart_overflow_propagates() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;
    const std::string a60(60, 'A');
    const std::string b60(60, 'B');

    feed(app, std::string("ICIMK,2,1,BIG,") + a60, now_us);
    REQUIRE(no_inmarsat_message(app.store().model().comm.inmarsat));

    feed(app, std::string("ICIMK,2,2,BIG,") + b60, now_us);
    const auto& msg = app.store().model().comm.inmarsat.latest_message;
    REQUIRE(app.store().model().comm.inmarsat.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, "BIG") == 0);
    REQUIRE(msg.complete == true);
    REQUIRE(msg.overflow == true);
    REQUIRE(msg.text_length.value == 95);
    REQUIRE(std::strlen(msg.decoded_text) == 95);
}

int main() {
    require_single_message_commit();
    require_multipart_waits_until_complete();
    require_bad_fragment_count();
    require_unsupported_payload_count();
    require_multipart_overflow_propagates();
    return 0;
}
