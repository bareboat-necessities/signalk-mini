#include <cstdio>
#include <cstdlib>
#include <string>

#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static std::string normal_sentence(const char* body) {
    char out[160];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static std::string bang_sentence(const char* body) {
    char out[160];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "!%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

struct Counts {
    int query = 0;
    int ais = 0;
    int seatalk = 0;
    int navtex = 0;
    int proprietary = 0;
};

static void count_query(const nmea0183_connector::NmeaSentence&, void* user) { static_cast<Counts*>(user)->query++; }
static void count_ais(const nmea0183_connector::NmeaSentence&, void* user) { static_cast<Counts*>(user)->ais++; }
static void count_seatalk(const nmea0183_connector::NmeaSentence&, void* user) { static_cast<Counts*>(user)->seatalk++; }
static void count_navtex(const nmea0183_connector::NmeaSentence&, void* user) { static_cast<Counts*>(user)->navtex++; }
static void count_proprietary(const nmea0183_connector::NmeaSentence&, void* user) { static_cast<Counts*>(user)->proprietary++; }

int main() {
    nmea0183_connector::Nmea0183StreamParser parser;
    nmea0183_connector::NmeaSentence parsed;
    Counts counts;

    nmea0183_connector::NmeaParserHooks hooks;
    hooks.user = &counts;
    hooks.on_query_sentence = count_query;
    hooks.on_ais_sentence = count_ais;
    hooks.on_seatalk_sentence = count_seatalk;
    hooks.on_navtex_sentence = count_navtex;
    hooks.on_proprietary_sentence = count_proprietary;
    parser.set_hooks(hooks);

    std::string line = normal_sentence("CCGPQ,GGA");
    REQUIRE(parser.parse_line(line.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::Query);
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.query_requester_talker, "CC"));
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.query_target_talker, "GP"));
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.query_requested_sentence, "GGA"));

    line = bang_sentence("AIVDM,1,1,,A,TESTPAYLOAD,0");
    REQUIRE(parser.parse_line(line.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::Ais);

    line = normal_sentence("STALK,20,01,02");
    REQUIRE(parser.parse_line(line.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::SeaTalk);

    line = normal_sentence("NXNRX,1,1,01,A,TEST MESSAGE");
    REQUIRE(parser.parse_line(line.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::NavTex);

    line = normal_sentence("PXYZ,1,2,3");
    REQUIRE(parser.parse_line(line.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::Proprietary);

    REQUIRE(counts.query == 1);
    REQUIRE(counts.ais == 1);
    REQUIRE(counts.seatalk == 1);
    REQUIRE(counts.navtex == 1);
    REQUIRE(counts.proprietary == 1);
    return 0;
}
