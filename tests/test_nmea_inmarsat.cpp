#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

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
                                   const char* expected_text) {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, body, now_us);

    const auto& safetynet = app.store().model().notifications.inmarsat.safetynet;
    const auto& msg = safetynet.latest_message;
    REQUIRE(safetynet.message_count.value == 1);
    REQUIRE(std::strcmp(msg.message_id, expected_id) == 0);
    REQUIRE(std::strcmp(msg.terminal_id, expected_terminal) == 0);
    REQUIRE(std::strcmp(msg.message_text, expected_text) == 0);
    REQUIRE(msg.text_length.value == static_cast<int32_t>(std::strlen(expected_text)));
    REQUIRE(msg.total_fragments.value == 1);
    REQUIRE(msg.fragment_number.value == 1);
    REQUIRE(msg.complete == true);
}

static void require_connector_only_sentence_interpretation() {
    require_message_fields("ICIMK,MSG-K,OK,KEY READY", "MSG-K", "IC", "KEY READY");
    require_message_fields("ICIMN,MSG-N,READY,NETWORK READY", "MSG-N", "IC", "NETWORK READY");
    require_message_fields("ICIMR,MSG-R,RCVD,RETURN READY", "MSG-R", "IC", "RETURN READY");
    require_message_fields("CSIMK,MSG-CS,OK,SATCOM READY", "MSG-CS", "CS", "SATCOM READY");
    require_message_fields("PINM,MSG-P,OK,PROPRIETARY READY", "MSG-P", "PI", "PROPRIETARY READY");
    require_message_fields("INM,MSG-I,OK,SHORT PREFIX READY", "MSG-I", "IN", "SHORT PREFIX READY");
}

static void require_safetynet_sm3_with_smb_body() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CSSM3,A,123456,005213,798,0,3,14,00,2012,04,05,14,30,3400,N,076,W,300", now_us);
    REQUIRE(no_inmarsat_message(app.store().model().notifications.inmarsat.safetynet));

    feed(app, "CSSMB,002,001,0,123456,FROM:MRCC^0D^0ATO: ALL SHIPS ", now_us);
    REQUIRE(no_inmarsat_message(app.store().model().notifications.inmarsat.safetynet));

    feed(app, "CSSMB,002,002,0,123456,SAR SITREP", now_us);
    const auto& msg = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(msg.message_id, "123456") == 0);
    REQUIRE(std::strcmp(msg.terminal_id, "CS") == 0);
    REQUIRE(msg.msi_status == 'A');
    REQUIRE(std::strcmp(msg.les_sequence_number, "005213") == 0);
    REQUIRE(std::strcmp(msg.les_id, "798") == 0);
    REQUIRE(msg.ocean_region_code == '0');
    REQUIRE(std::strcmp(msg.ocean_region_label, "AOR-W") == 0);
    REQUIRE(msg.priority_code == '3');
    REQUIRE(std::strcmp(msg.priority_label, "Distress") == 0);
    REQUIRE(std::strcmp(msg.service_code, "14") == 0);
    REQUIRE(std::strcmp(msg.address_kind, "circular_area") == 0);
    REQUIRE(msg.received_year.value == 2012);
    REQUIRE(msg.received_month.value == 4);
    REQUIRE(msg.received_day.value == 5);
    REQUIRE(msg.received_hour.value == 14);
    REQUIRE(msg.received_minute.value == 30);
    REQUIRE(msg.circular_center_lat_deg.valid);
    REQUIRE(msg.circular_center_lon_deg.valid);
    NEAR(msg.circular_center_lat_deg.value, 34.0f, 0.001f);
    NEAR(msg.circular_center_lon_deg.value, -76.0f, 0.001f);
    REQUIRE(msg.circular_radius_nmi.value == 300);
    REQUIRE(msg.total_fragments.value == 2);
    REQUIRE(msg.fragment_number.value == 2);
    REQUIRE(msg.body_complete == true);
    REQUIRE(msg.complete == true);
    REQUIRE(msg.requires_alarm == true);
    REQUIRE(msg.mandatory_reception == true);
    REQUIRE(std::strstr(msg.message_text, "FROM:MRCC\r\nTO: ALL SHIPS SAR SITREP") != nullptr);
}

static void require_safetynet_sm1_sm2_sm4_headers() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "CSSM1,A,100001,000111,321,1,1,31,00,2020,01,02,03,04,05", now_us);
    feed(app, "CSSMB,001,,,100001,NAVAREA MESSAGE", now_us);
    const auto& sm1 = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(sm1.message_id, "100001") == 0);
    REQUIRE(std::strcmp(sm1.address_kind, "navarea_metarea") == 0);
    REQUIRE(sm1.navarea_metarea_code.value == 5);
    REQUIRE(std::strcmp(sm1.service_label, "NAVAREA/METAREA warning or forecast") == 0);

    feed(app, "CSSM2,A,200001,000222,321,1,1,13,00,2021,06,07,08,09,05,D,D", now_us);
    feed(app, "CSSMB,001,,,200001,COASTAL SAR INFO", now_us);
    const auto& sm2 = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(sm2.message_id, "200001") == 0);
    REQUIRE(std::strcmp(sm2.address_kind, "coastal_warning_area") == 0);
    REQUIRE(sm2.coastal_warning_navarea_metarea.value == 5);
    REQUIRE(sm2.coastal_warning_area == 'D');
    REQUIRE(sm2.coastal_warning_subject == 'D');
    REQUIRE(sm2.coastal_warning_subject_mandatory == true);
    REQUIRE(sm2.mandatory_reception == true);

    feed(app, "CSSM4,A,300001,000333,321,2,1,04,00,2022,07,08,09,10,6000,N,01000,W,30,025", now_us);
    feed(app, "CSSMB,001,,,300001,RECTANGULAR AREA MESSAGE", now_us);
    const auto& sm4 = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(sm4.message_id, "300001") == 0);
    REQUIRE(std::strcmp(sm4.address_kind, "rectangular_area") == 0);
    NEAR(sm4.rectangle_sw_lat_deg.value, 60.0f, 0.001f);
    NEAR(sm4.rectangle_sw_lon_deg.value, -10.0f, 0.001f);
    REQUIRE(sm4.rectangle_extent_lat_deg.value == 30);
    REQUIRE(sm4.rectangle_extent_lon_deg.value == 25);
    REQUIRE(app.store().model().notifications.inmarsat.safetynet.message_count.value == 3);
}

int main() {
    require_single_message_commit();
    require_multipart_waits_until_complete();
    require_bad_fragment_count();
    require_unsupported_payload_count();
    require_multipart_overflow_propagates();
    require_connector_only_sentence_interpretation();
    require_safetynet_sm3_with_smb_body();
    require_safetynet_sm1_sm2_sm4_headers();
    return 0;
}
