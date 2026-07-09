#include <cstdio>
#include <cstdlib>

#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

int main() {
    nmea0183_connector::Nmea0183StreamParser parser;
    nmea0183_connector::NmeaSentence sentence;

    const char hdm_storage[] = {
        '$','I','I','H','D','M',',','1','2','3','.','4',',','M','Z','Z','Z'
    };
    const nmea0183_connector::NmeaSpan hdm_span(hdm_storage, 14);
    REQUIRE(parser.parse_span(hdm_span, sentence, false));
    REQUIRE(nmea0183_connector::sentence_is(sentence, "HDM"));
    REQUIRE(sentence.raw_length == 14);
    REQUIRE(sentence.field_count == 2);
    REQUIRE(nmea0183_connector::nmea_span_equals(sentence.field(0), "123.4"));
    REQUIRE(nmea0183_connector::nmea_span_equals(sentence.field(1), "M"));

    const char rmc_storage[] = {
        '\\','s','x','r','c',',','c','x','*','0','0','\\','/','$','G','P','R','M','C',',','1','2','3','5','1','9',',','A',',','4','8','0','7','.','0','3','8',',','N',',','0','1','1','3','1','.','0','0','0',',','E','\r','\n','X','X'
    };
    const nmea0183_connector::NmeaSpan rmc_span(rmc_storage, 52);
    REQUIRE(parser.parse_span(rmc_span, sentence, false));
    REQUIRE(nmea0183_connector::sentence_is(sentence, "RMC"));
    REQUIRE(nmea0183_connector::talker_is(sentence, "GP"));
    REQUIRE(sentence.raw[0] == '$');
    REQUIRE(sentence.raw[sentence.raw_length] == '\0');

    return 0;
}
