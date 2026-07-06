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
    if (!app.nmea0183().feed_line(line.c_str(), 1, now_us)) {
        std::fprintf(stderr, "feed failed: %s\nlast_error=%s\n", line.c_str(), app.nmea0183().last_error());
        std::exit(1);
    }
}

static bool no_inmarsat_message(const ship_data_model::InmarsatSafetyNetData<float>& safetynet) {
    return !safetynet.message_count.valid || safetynet.message_count.value == 0;
}

static void require_legacy_inmarsat_notification_storage() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ICIMK,MSG01,OK,HELLO INMARSAT", now_us);
    const auto& msg = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(msg.message_id, "MSG01") == 0);
    REQUIRE(std::strcmp(msg.terminal_id, "IC") == 0);
    REQUIRE(std::strcmp(msg.message_text, "HELLO INMARSAT") == 0);
    REQUIRE(msg.complete);

    feed(app, "ICIMK,2,1,MSG42,HELLO ", now_us);
    feed(app, "ICIMK,2,2,MSG42,WORLD", now_us);
    const auto& multipart = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(multipart.message_id, "MSG42") == 0);
    REQUIRE(std::strcmp(multipart.message_text, "HELLO WORLD") == 0);
    REQUIRE(multipart.complete);
    REQUIRE(multipart.total_fragments.value == 2);
}

static void require_bad_fragment_and_unsupported_counts() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ICIMK,2,2,MSG42,WORLD", now_us);
    REQUIRE(no_inmarsat_message(app.store().model().notifications.inmarsat.safetynet));
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.bad_fragment_count.value >= 1);

    std::string body = "ICIMK,MSG02,OK,";
    body.push_back(static_cast<char>(0x7f));
    feed(app, body, now_us);
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.unsupported_count.value >= 1);
}

static void require_safetynet_sm_headers_and_smb_body() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CSSM1,A,100001,000111,321,1,1,31,00,2020,01,02,03,04,05", now_us);
    feed(app, "CSSMB,001,,,100001,NAVAREA MESSAGE", now_us);
    const auto& sm1 = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(sm1.message_id, "100001") == 0);
    REQUIRE(std::strcmp(sm1.address_kind, "navarea_metarea") == 0);
    REQUIRE(sm1.navarea_metarea_code.valid);
    REQUIRE(sm1.complete);

    feed(app, "CSSM2,A,200001,000222,321,1,1,13,00,2021,06,07,08,09,05,D,D", now_us);
    feed(app, "CSSMB,001,,,200001,COASTAL SAR INFO", now_us);
    const auto& sm2 = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(sm2.message_id, "200001") == 0);
    REQUIRE(std::strcmp(sm2.address_kind, "coastal_warning_area") == 0);
    REQUIRE(sm2.coastal_warning_subject == 'D');
    REQUIRE(sm2.mandatory_reception);

    feed(app, "CSSM3,A,123456,005213,798,0,3,14,00,2012,04,05,14,30,3400,N,076,W,300", now_us);
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.message_count.value == 2);
    feed(app, "CSSMB,002,001,0,123456,FROM:MRCC^0D^0ATO: ALL SHIPS ", now_us);
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.message_count.value == 2);
    feed(app, "CSSMB,002,002,0,123456,SAR SITREP", now_us);
    const auto& sm3 = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(sm3.message_id, "123456") == 0);
    REQUIRE(std::strcmp(sm3.address_kind, "circular_area") == 0);
    REQUIRE(sm3.circular_center_lat_deg.valid);
    REQUIRE(sm3.circular_center_lon_deg.valid);
    REQUIRE(sm3.circular_radius_nmi.valid);
    REQUIRE(sm3.requires_alarm);
    REQUIRE(sm3.mandatory_reception);
    REQUIRE(std::strstr(sm3.message_text, "FROM:MRCC") != nullptr);
    REQUIRE(std::strstr(sm3.message_text, "SAR SITREP") != nullptr);

    feed(app, "CSSM4,A,300001,000333,321,2,1,04,00,2022,07,08,09,10,6000,N,01000,W,30,025", now_us);
    feed(app, "CSSMB,001,,,300001,RECTANGULAR AREA MESSAGE", now_us);
    const auto& sm4 = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(sm4.message_id, "300001") == 0);
    REQUIRE(std::strcmp(sm4.address_kind, "rectangular_area") == 0);
    REQUIRE(sm4.rectangle_sw_lat_deg.valid);
    REQUIRE(sm4.rectangle_sw_lon_deg.valid);
    REQUIRE(sm4.rectangle_extent_lat_deg.valid);
    REQUIRE(sm4.rectangle_extent_lon_deg.valid);
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.message_count.value >= 4);
}

static void require_cr_terminal_sentences() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CSSM1,A,910001,000111,321,1,1,31,00,2024,01,02,03,04,05", now_us);
    feed(app, "CSSMB,001,,,910001,PHASE FOUR MESSAGE", now_us);
    REQUIRE(std::strcmp(app.store().model().notifications.inmarsat.safetynet.latest_message.message_id, "910001") == 0);
    REQUIRE(!app.store().model().notifications.inmarsat.safetynet.latest_message.acknowledged);

    feed(app, "CRCAN,910001,C,cleared", now_us);
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.latest_message.acknowledged);
    REQUIRE(std::strcmp(app.store().model().notifications.messages.event.event_id, "910001") == 0);
    REQUIRE(std::strcmp(app.store().model().notifications.messages.event.event_source, "inmarsat") == 0);

    feed(app, "CRCRQ,REQ01,STATUS,ALL,terminal status", now_us);
    REQUIRE(std::strcmp(app.store().model().notifications.messages.text.id, "REQ01") == 0);
    REQUIRE(std::strcmp(app.store().model().notifications.messages.text.code, "STATUS") == 0);
    REQUIRE(std::strcmp(app.store().model().notifications.messages.text.value, "ALL") == 0);

    feed(app, "CRDSM,1,2,31,active", now_us);
    REQUIRE(std::strcmp(app.store().model().notifications.messages.text.id, "DSM") == 0);
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.latest_message.ocean_region_code == '1');
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.latest_message.priority_code == '2');
    REQUIRE(std::strcmp(app.store().model().notifications.inmarsat.safetynet.latest_message.service_code, "31") == 0);

    feed(app, "CRTMD,MSG99,A,terminal,WEATHER UPDATE", now_us);
    REQUIRE(std::strcmp(app.store().model().notifications.inmarsat.safetynet.latest_message.message_id, "MSG99") == 0);
    REQUIRE(std::strcmp(app.store().model().notifications.inmarsat.safetynet.latest_message.message_text, "WEATHER UPDATE") == 0);
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.latest_message.complete);

    feed(app, "CRTMD,2,1,MSG77,HELLO ", now_us);
    feed(app, "CRTMD,2,2,MSG77,WORLD", now_us);
    const auto& tmd = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(tmd.message_id, "MSG77") == 0);
    REQUIRE(std::strcmp(tmd.message_text, "HELLO WORLD") == 0);
    REQUIRE(tmd.total_fragments.value == 2);
    REQUIRE(tmd.complete);
}

int main() {
    require_legacy_inmarsat_notification_storage();
    require_bad_fragment_and_unsupported_counts();
    require_safetynet_sm_headers_and_smb_body();
    require_cr_terminal_sentences();
    return 0;
}
