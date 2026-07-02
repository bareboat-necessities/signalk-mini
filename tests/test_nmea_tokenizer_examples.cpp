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

static void check_vendor(const char* body, nmea0183_connector::NmeaProprietaryVendor expected, const char* expected_code) {
    const std::string line = sentence(body);
    const nmea0183_connector::NmeaTokenizeResult tokens = nmea0183_connector::tokenize_nmea_line(line.c_str());
    const nmea0183_connector::NmeaToken* token = tokens.first_sentence();
    REQUIRE(token != nullptr);
    REQUIRE(token->family == nmea0183_connector::NmeaSentenceFamily::Proprietary);
    REQUIRE(token->proprietary_vendor == expected);
    REQUIRE(nmea0183_connector::nmea_span_equals(token->proprietary_vendor_code, expected_code));

    nmea0183_connector::Nmea0183StreamParser parser;
    nmea0183_connector::NmeaSentence parsed;
    REQUIRE(parser.parse_line(line.c_str(), parsed));
    REQUIRE(parsed.family == nmea0183_connector::NmeaSentenceFamily::Proprietary);
    REQUIRE(parsed.proprietary_vendor == expected);
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.proprietary_vendor_code, expected_code));
}

int main() {
    check_sentence_token("$GPGGA,002153.000,3342.6618,N,11751.3858,W,1,10,1.2,27.0,M,-34.2,M,,0000*5A", nmea0183_connector::NmeaSentenceFamily::Standard, '$');
    check_sentence_token("!AIVDM,1,1,,A,13HOI:0P0000VOHLCnHQKwvL05Ip,0*23", nmea0183_connector::NmeaSentenceFamily::Ais, '!');
    check_sentence_token("$STALK,84,56,e,0,0,0,0,0,8*0F", nmea0183_connector::NmeaSentenceFamily::SeaTalk, '$');
    const std::string dsc = sentence("CDDSC,12,3380405810,00,1234567890,72,1234567890,00,00,00,00,00,00,00");
    check_sentence_token(dsc.c_str(), nmea0183_connector::NmeaSentenceFamily::Dsc, '$');

    const std::string query = sentence("CCGPQ,GGA");
    check_sentence_token(query.c_str(), nmea0183_connector::NmeaSentenceFamily::Query, '$');

    const std::string navtex = sentence("NXNRX,2,1,01,HELLO ");
    check_sentence_token(navtex.c_str(), nmea0183_connector::NmeaSentenceFamily::NavTex, '$');

    const std::string txt1 = sentence("GPTXT,2,1,07,FIRST ");
    const nmea0183_connector::NmeaTokenizeResult txt_tokens = nmea0183_connector::tokenize_nmea_line(txt1.c_str());
    const nmea0183_connector::NmeaToken* txt_token = txt_tokens.first_sentence();
    REQUIRE(txt_token != nullptr);
    REQUIRE(txt_token->fragment.is_fragmented);
    REQUIRE(txt_token->fragment.total == 2);
    REQUIRE(txt_token->fragment.number == 1);
    REQUIRE(nmea0183_connector::nmea_span_equals(txt_token->fragment.message_id, "07"));
    REQUIRE(nmea0183_connector::nmea_span_equals(txt_token->fragment.payload, "FIRST "));

    check_vendor("PGRME,1.0,M,2.0,M,3.0,M", nmea0183_connector::NmeaProprietaryVendor::Garmin, "GRM");
    check_vendor("PUBX,00,000000,0000.0000,N,00000.0000,E", nmea0183_connector::NmeaProprietaryVendor::Ublox, "UBX");
    check_vendor("PMTK001,314,3", nmea0183_connector::NmeaProprietaryVendor::MediaTek, "MTK");
    check_vendor("PSRF100,1,4800,8,1,0", nmea0183_connector::NmeaProprietaryVendor::SiRf, "SRF");
    check_vendor("PASHR,123519,1.2,T,0.1,R,0.2,P,0.3,0.4,1,0", nmea0183_connector::NmeaProprietaryVendor::Ashtech, "ASH");
    check_vendor("PAMTC,EN,HDG,1", nmea0183_connector::NmeaProprietaryVendor::Airmar, "AMT");
    check_vendor("PFEC,GPatt,1,2,3", nmea0183_connector::NmeaProprietaryVendor::Furuno, "FEC");
    check_vendor("PZZZ,1,2,3", nmea0183_connector::NmeaProprietaryVendor::Unknown, "ZZZ");

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
    REQUIRE(parsed.fragment.is_fragmented);
    REQUIRE(parsed.fragment.total == 2);
    REQUIRE(parsed.fragment.number == 1);
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.fragment.message_id, "01"));
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.fragment.payload, "HELLO "));

    REQUIRE(parser.parse_line(txt1.c_str(), parsed));
    REQUIRE(nmea0183_connector::sentence_is(parsed, "TXT"));
    REQUIRE(parsed.fragment.is_fragmented);
    REQUIRE(parsed.fragment.total == 2);
    REQUIRE(parsed.fragment.number == 1);
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.fragment.message_id, "07"));
    REQUIRE(nmea0183_connector::nmea_span_equals(parsed.fragment.payload, "FIRST "));
    return 0;
}
