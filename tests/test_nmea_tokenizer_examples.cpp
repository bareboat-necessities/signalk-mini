#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

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
    return 0;
}
