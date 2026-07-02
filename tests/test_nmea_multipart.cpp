#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static std::string sentence(const char* body) {
    char out[192];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static std::string encapsulated_sentence(const char* body) {
    char out[192];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "!%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

static void feed_raw(signalk_mini::SignalKMiniApp<float>& app, const std::string& line, uint64_t& now_us) {
    now_us += 1000;
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "GPTXT,2,1,07,FIRST ", now_us);
    REQUIRE(app.nmea0183().state().text_message.in_progress);
    REQUIRE(!app.nmea0183().state().text_message.complete);
    REQUIRE(app.nmea0183().state().text_message.received_mask == 0x0001);
    REQUIRE(app.nmea0183().state().text_message.total_fragments.value == 2);
    REQUIRE(app.nmea0183().state().text_message.last_fragment_number.value == 1);
    REQUIRE(std::strcmp(app.nmea0183().state().text_message.message_id, "07") == 0);
    REQUIRE(std::strcmp(app.nmea0183().state().text_message.text, "FIRST ") == 0);

    feed(app, "GPTXT,2,2,07,SECOND", now_us);
    REQUIRE(!app.nmea0183().state().text_message.in_progress);
    REQUIRE(app.nmea0183().state().text_message.complete);
    REQUIRE(!app.nmea0183().state().text_message.overflow);
    REQUIRE(app.nmea0183().state().text_message.received_mask == 0x0003);
    REQUIRE(std::strcmp(app.nmea0183().state().text_message.text, "FIRST SECOND") == 0);
    REQUIRE(app.nmea0183().state().text_message.text_length.value == 12);
    REQUIRE(std::strcmp(app.nmea0183().state().text_sentence.sentence_id, "TXT") == 0);

    feed(app, "CDDSE,2,1,A,3380400790,00,HELLO", now_us);
    REQUIRE(app.nmea0183().state().dsc.multipart.in_progress);
    REQUIRE(app.nmea0183().state().dsc.multipart.received_mask == 0x0001);
    REQUIRE(std::strcmp(app.nmea0183().state().dsc.multipart.message_id, "3380400790") == 0);
    feed(app, "CDDSE,2,2,A,3380400790,00, DSC", now_us);
    REQUIRE(app.nmea0183().state().dsc.multipart.complete);
    REQUIRE(std::strcmp(app.nmea0183().state().dsc.multipart.text, "HELLO DSC") == 0);

    feed(app, "NXNRX,2,1,NAV1,NAV ", now_us);
    REQUIRE(app.nmea0183().state().navtex_message.in_progress);
    REQUIRE(std::strcmp(app.nmea0183().state().navtex_message.message_id, "NAV1") == 0);
    feed(app, "NXNRX,2,2,NAV1,TEX", now_us);
    REQUIRE(app.nmea0183().state().navtex_message.complete);
    REQUIRE(std::strcmp(app.nmea0183().state().navtex_message.text, "NAV TEX") == 0);

    feed_raw(app, encapsulated_sentence("AIVDM,2,1,3,A,ABC,0"), now_us);
    REQUIRE(app.nmea0183().state().ais_message.in_progress);
    REQUIRE(app.nmea0183().state().ais_message.received_mask == 0x0001);
    REQUIRE(std::strcmp(app.nmea0183().state().ais_message.message_id, "3") == 0);
    feed_raw(app, encapsulated_sentence("AIVDM,2,2,3,A,DEF,0"), now_us);
    REQUIRE(app.nmea0183().state().ais_message.complete);
    REQUIRE(std::strcmp(app.nmea0183().state().ais_message.text, "ABCDEF") == 0);

    nmea0183_connector::Nmea0183StreamParser parser;
    nmea0183_connector::NmeaSentence parsed;
    const std::string ais = encapsulated_sentence("AIVDM,2,1,3,A,55NBsv02;P?ITpN?N20EHE:0@F22222222220t4p,0");
    REQUIRE(parser.parse_line(ais.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::Ais);
    REQUIRE(parsed.fragment.is_fragmented);
    REQUIRE(parsed.fragment.total == 2);
    REQUIRE(parsed.fragment.number == 1);
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.fragment.message_id, "3"));
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.fragment.payload, "55NBsv02;P?ITpN?N20EHE:0@F22222222220t4p"));

    return 0;
}
