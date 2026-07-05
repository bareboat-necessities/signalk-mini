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
    REQUIRE(msg.sentence_type == ship_data_model::InmarsatSentenceType::imk);
    REQUIRE(msg.payload_type == ship_data_model::InmarsatPayloadType::text);
    REQUIRE(msg.standard_sentence == true);
    REQUIRE(msg.proprietary_sentence == false);
    REQUIRE(msg.ascii_valid == true);
    REQUIRE(msg.payload_length_chars.value == 14);
    REQUIRE(msg.alpha_count.value == 13);
    REQUIRE(msg.digit_count.value == 0);
    REQUIRE(msg.separator_count.value == 1);
    REQUIRE(msg.token_count.value == 2);
    REQUIRE(msg.key_value_count.value == 0);
    REQUIRE(std::strcmp(msg.first_token, "HELLO") == 0);
    REQUIRE(std::strcmp(msg.second_token, "INMARSAT") == 0);
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
    REQUIRE(msg.sentence_type == ship_data_model::InmarsatSentenceType::imk);
    REQUIRE(msg.payload_type == ship_data_model::InmarsatPayloadType::text);
    REQUIRE(msg.payload_length_chars.value == 11);
    REQUIRE(msg.token_count.value == 2);
    REQUIRE(std::strcmp(msg.first_token, "HELLO") == 0);
    REQUIRE(std::strcmp(msg.second_token, "WORLD") == 0);
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
    REQUIRE(msg.payload_length_chars.value == 95);
    REQUIRE(std::strlen(msg.decoded_text) == 95);
}

static void require_message_fields(const std::string& body,
                                   const char* expected_id,
                                   const char* expected_terminal,
                                   const char* expected_type,
                                   const char* expected_status,
                                   const char* expected_text,
                                   ship_data_model::InmarsatSentenceType expected_sentence_type,
                                   bool expected_standard,
                                   bool expected_proprietary) {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, body, now_us);

    const auto& msg = app.store().model().comm.inmarsat.latest_message;
    REQUIRE(app.store().model().comm.inmarsat.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, expected_id) == 0);
    REQUIRE(std::strcmp(msg.terminal_id, expected_terminal) == 0);
    REQUIRE(std::strcmp(msg.message_type, expected_type) == 0);
    REQUIRE(std::strcmp(msg.message_status, expected_status) == 0);
    REQUIRE(std::strcmp(msg.decoded_text, expected_text) == 0);
    REQUIRE(msg.sentence_type == expected_sentence_type);
    REQUIRE(msg.standard_sentence == expected_standard);
    REQUIRE(msg.proprietary_sentence == expected_proprietary);
    REQUIRE(msg.payload_type == ship_data_model::InmarsatPayloadType::text);
    REQUIRE(msg.ascii_valid == true);
    REQUIRE(msg.payload_length_chars.value == static_cast<int32_t>(std::strlen(expected_text)));
    REQUIRE(msg.total_fragments.value == 1);
    REQUIRE(msg.last_fragment_number.value == 1);
    REQUIRE(msg.text_length.value == static_cast<int32_t>(std::strlen(expected_text)));
    REQUIRE(msg.complete == true);
}

static void require_safe_sentence_interpretation() {
    require_message_fields("ICIMK,MSG-K,OK,KEY READY", "MSG-K", "IC", "IMK", "OK", "KEY READY", ship_data_model::InmarsatSentenceType::imk, true, false);
    require_message_fields("ICIMN,MSG-N,READY,NETWORK READY", "MSG-N", "IC", "IMN", "READY", "NETWORK READY", ship_data_model::InmarsatSentenceType::imn, true, false);
    require_message_fields("ICIMR,MSG-R,RCVD,RETURN READY", "MSG-R", "IC", "IMR", "RCVD", "RETURN READY", ship_data_model::InmarsatSentenceType::imr, true, false);
    require_message_fields("CSIMK,MSG-CS,OK,SATCOM READY", "MSG-CS", "CS", "IMK", "OK", "SATCOM READY", ship_data_model::InmarsatSentenceType::imk, true, false);
    require_message_fields("PINM,MSG-P,OK,PROPRIETARY READY", "MSG-P", "PI", "PINM", "OK", "PROPRIETARY READY", ship_data_model::InmarsatSentenceType::pinm, false, true);
    require_message_fields("INM,MSG-I,OK,SHORT PREFIX READY", "MSG-I", "IN", "INM", "OK", "SHORT PREFIX READY", ship_data_model::InmarsatSentenceType::inm, false, true);
}

static void require_decoded_payload_classification() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ICIMK,NUM,OK,123456", now_us);
    const auto& digits = app.store().model().comm.inmarsat.latest_message;
    REQUIRE(digits.payload_type == ship_data_model::InmarsatPayloadType::digits);
    REQUIRE(digits.payload_length_chars.value == 6);
    REQUIRE(digits.digit_count.value == 6);
    REQUIRE(digits.alpha_count.value == 0);
    REQUIRE(digits.token_count.value == 1);
    REQUIRE(std::strcmp(digits.first_token, "123456") == 0);

    feed(app, "ICIMK,KV,OK,STATE=READY", now_us);
    const auto& key_value = app.store().model().comm.inmarsat.latest_message;
    REQUIRE(key_value.payload_type == ship_data_model::InmarsatPayloadType::key_value);
    REQUIRE(key_value.payload_length_chars.value == 11);
    REQUIRE(key_value.key_value_count.value == 1);
    REQUIRE(key_value.separator_count.value == 1);
    REQUIRE(key_value.token_count.value == 2);
    REQUIRE(std::strcmp(key_value.first_token, "STATE") == 0);
    REQUIRE(std::strcmp(key_value.second_token, "READY") == 0);
    REQUIRE(std::strcmp(key_value.first_key, "STATE") == 0);
    REQUIRE(std::strcmp(key_value.first_value, "READY") == 0);

    feed(app, "ICIMK,DELIM,OK,ONE/TWO", now_us);
    const auto& delimited = app.store().model().comm.inmarsat.latest_message;
    REQUIRE(delimited.payload_type == ship_data_model::InmarsatPayloadType::delimited);
    REQUIRE(delimited.separator_count.value == 1);
    REQUIRE(delimited.token_count.value == 2);
    REQUIRE(std::strcmp(delimited.first_token, "ONE") == 0);
    REQUIRE(std::strcmp(delimited.second_token, "TWO") == 0);

    feed(app, "ICIMK,MIX,OK,AB12", now_us);
    const auto& mixed = app.store().model().comm.inmarsat.latest_message;
    REQUIRE(mixed.payload_type == ship_data_model::InmarsatPayloadType::mixed_ascii);
    REQUIRE(mixed.alpha_count.value == 2);
    REQUIRE(mixed.digit_count.value == 2);
    REQUIRE(mixed.token_count.value == 1);
}

int main() {
    require_single_message_commit();
    require_multipart_waits_until_complete();
    require_bad_fragment_count();
    require_unsupported_payload_count();
    require_multipart_overflow_propagates();
    require_safe_sentence_interpretation();
    require_decoded_payload_classification();
    return 0;
}
