#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

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

static void check_sentence_token(const char* raw, nmea0183_connector::NmeaSentenceFamily family, char start_char) {
    const nmea0183_connector::NmeaTokenizeResult tokens = nmea0183_connector::tokenize_nmea_line(raw);
    const nmea0183_connector::NmeaToken* token = tokens.first_sentence();
    REQUIRE(token != nullptr);
    REQUIRE(token->is_sentence);
    REQUIRE(token->family == family);
    REQUIRE(token->text.length > 0);
    REQUIRE(token->text[0] == start_char);
    REQUIRE(token->has_checksum);
}

int main() {
    check_sentence_token("$GPGGA,002153.000,3342.6618,N,11751.3858,W,1,10,1.2,27.0,M,-34.2,M,,0000*5A", nmea0183_connector::NmeaSentenceFamily::Standard, '$');
    check_sentence_token("!AIVDM,1,1,,A,13HOI:0P0000VOHLCnHQKwvL05Ip,0*23", nmea0183_connector::NmeaSentenceFamily::Ais, '!');
    check_sentence_token("$STALK,84,56,e,0,0,0,0,0,8*0F", nmea0183_connector::NmeaSentenceFamily::SeaTalk, '$');
    check_sentence_token("$DSC,12,3380405810,00,1234567890,72,1234567890,00,00,00,00,00,00,00*3B", nmea0183_connector::NmeaSentenceFamily::Dsc, '$');

    const std::string query = sentence("CCGPQ,GGA");
    check_sentence_token(query.c_str(), nmea0183_connector::NmeaSentenceFamily::Query, '$');

    const std::string navtex = sentence("NXNRX,1,1,01,A,TEST MESSAGE");
    check_sentence_token(navtex.c_str(), nmea0183_connector::NmeaSentenceFamily::NavTex, '$');

    const char* inmarsat = "/g:1-9-1234,s:egcterm1,n:213,c:1333636200*hh/$CSSM3,123456,005213,798,0,3,14,00,2012,04,05,14,30,3400,N,076,W,300*hh";
    const nmea0183_connector::NmeaTokenizeResult tokens = nmea0183_connector::tokenize_nmea_line(inmarsat);
    REQUIRE(tokens.count == 2);
    REQUIRE(tokens.tokens[0].family == nmea0183_connector::NmeaSentenceFamily::Inmarsat);
    REQUIRE(!tokens.tokens[0].is_sentence);
    REQUIRE(tokens.tokens[1].family == nmea0183_connector::NmeaSentenceFamily::Inmarsat);
    REQUIRE(tokens.tokens[1].is_sentence);
    REQUIRE(tokens.tokens[1].text[0] == '$');

    nmea0183_connector::Nmea0183StreamParser parser;
    nmea0183_connector::NmeaSentence parsed;
    char embedded[128];
    nmea0183_connector::nmea_copy_span(embedded, sizeof(embedded), tokens.tokens[1].text);
    REQUIRE(parser.parse_line(embedded, parsed, false));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::Inmarsat);
    REQUIRE(nmea0183_connector::talker_is(parsed, "CS"));
    REQUIRE(nmea0183_connector::sentence_is(parsed, "SM3"));

    REQUIRE(parser.parse_line(query.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::Query);
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.query_requester_talker, "CC"));
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.query_target_talker, "GP"));
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.query_requested_sentence, "GGA"));

    REQUIRE(parser.parse_line(navtex.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::NavTex);
    REQUIRE(nmea0183_connector::talker_is(parsed, "NX"));
    REQUIRE(nmea0183_connector::sentence_is(parsed, "NRX"));
    return 0;
}
