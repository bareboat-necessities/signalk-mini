#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

struct AisPayload {
    std::string payload;
    int fill_bits = 0;
};

static std::string encapsulated_sentence(const std::string& body) {
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body.c_str());
    std::string out = "!";
    out += body;
    out += "*";
    out.push_back(nmea0183_connector::to_hex((cs >> 4) & 0x0f));
    out.push_back(nmea0183_connector::to_hex(cs & 0x0f));
    return out;
}

static void feed_ais(signalk_mini::SignalKMiniApp<float>& app, const std::string& body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = encapsulated_sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

static void append_bits(std::string& bits, uint32_t value, uint8_t count) {
    for (int i = static_cast<int>(count) - 1; i >= 0; --i) bits.push_back(((value >> i) & 1u) ? '1' : '0');
}

static void append_signed_bits(std::string& bits, int32_t value, uint8_t count) {
    const uint32_t modulus = 1u << count;
    const uint32_t encoded = value < 0 ? static_cast<uint32_t>(static_cast<int64_t>(modulus) + value) : static_cast<uint32_t>(value);
    append_bits(bits, encoded & (modulus - 1u), count);
}

static char ais_payload_char(uint8_t value) {
    return static_cast<char>(value < 40 ? value + 48 : value + 56);
}

static AisPayload make_payload(std::string bits) {
    AisPayload out;
    out.fill_bits = static_cast<int>((6u - bits.size() % 6u) % 6u);
    for (int i = 0; i < out.fill_bits; ++i) bits.push_back('0');
    for (size_t i = 0; i < bits.size(); i += 6u) {
        uint8_t value = 0;
        for (uint8_t b = 0; b < 6; ++b) value = static_cast<uint8_t>((value << 1u) | (bits[i + b] == '1' ? 1u : 0u));
        out.payload.push_back(ais_payload_char(value));
    }
    return out;
}

static uint8_t ais_text_value(char c) {
    if (c >= 'A' && c <= 'Z') return static_cast<uint8_t>(c - 'A' + 1);
    if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0' + 48);
    return 32;
}

static void append_text(std::string& bits, const char* text, uint8_t chars) {
    for (uint8_t i = 0; i < chars; ++i) append_bits(bits, text && text[i] ? ais_text_value(text[i]) : 32, 6);
}

static std::string single_vdm_body(const AisPayload& payload, char channel = 'A') {
    char out[256];
    std::snprintf(out, sizeof(out), "AIVDM,1,1,,%c,%s,%d", channel, payload.payload.c_str(), payload.fill_bits);
    return std::string(out);
}

static std::string single_vdo_body(const AisPayload& payload, char channel = 'A') {
    char out[256];
    std::snprintf(out, sizeof(out), "AIVDO,1,1,,%c,%s,%d", channel, payload.payload.c_str(), payload.fill_bits);
    return std::string(out);
}

static const ship_data_model::AisTargetData<float>* find_target(const ship_data_model::AisTargetTableData<float>& table, int32_t mmsi) {
    for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
        if (table.targets[i].occupied && table.targets[i].mmsi.valid && table.targets[i].mmsi.value == mmsi) return &table.targets[i];
    }
    return nullptr;
}

static AisPayload make_type8_binary_payload() {
    std::string bits;
    append_bits(bits, 8, 6);
    append_bits(bits, 0, 2);
    append_bits(bits, 333444555u, 30);
    append_bits(bits, 0, 2);
    append_bits(bits, 1, 10);
    append_bits(bits, 31, 6);
    append_bits(bits, 0xabcd, 16);
    return make_payload(bits);
}

static AisPayload make_type18_class_b_payload(uint32_t mmsi = 123123123u) {
    std::string bits;
    append_bits(bits, 18, 6);
    append_bits(bits, 0, 2);
    append_bits(bits, mmsi, 30);
    append_bits(bits, 0, 8);
    append_bits(bits, 123, 10);
    append_bits(bits, 1, 1);
    append_signed_bits(bits, static_cast<int32_t>(std::lround(-73.25 * 600000.0)), 28);
    append_signed_bits(bits, static_cast<int32_t>(std::lround(40.50 * 600000.0)), 27);
    append_bits(bits, 900, 12);
    append_bits(bits, 91, 9);
    append_bits(bits, 35, 6);
    append_bits(bits, 0, 2);
    append_bits(bits, 1, 1);
    append_bits(bits, 1, 1);
    append_bits(bits, 0, 1);
    append_bits(bits, 1, 1);
    append_bits(bits, 1, 1);
    append_bits(bits, 1, 1);
    append_bits(bits, 1, 1);
    append_bits(bits, 0, 1);
    append_bits(bits, 0x12345, 19);
    return make_payload(bits);
}

static AisPayload make_type21_aton_payload() {
    std::string bits;
    append_bits(bits, 21, 6);
    append_bits(bits, 0, 2);
    append_bits(bits, 987654321u, 30);
    append_bits(bits, 5, 5);
    append_text(bits, "LIGHTHOUSE", 20);
    append_bits(bits, 1, 1);
    append_signed_bits(bits, static_cast<int32_t>(std::lround(-74.0 * 600000.0)), 28);
    append_signed_bits(bits, static_cast<int32_t>(std::lround(41.0 * 600000.0)), 27);
    append_bits(bits, 10, 9);
    append_bits(bits, 12, 9);
    append_bits(bits, 2, 6);
    append_bits(bits, 3, 6);
    append_bits(bits, 1, 4);
    append_bits(bits, 22, 6);
    append_bits(bits, 0, 1);
    while (bits.size() < 268u) bits.push_back('0');
    append_bits(bits, 1, 1);
    append_bits(bits, 1, 1);
    append_bits(bits, 1, 1);
    append_bits(bits, 0, 1);
    append_text(bits, "EXTNAME", 14);
    return make_payload(bits);
}

static AisPayload make_type24_part_a(uint32_t mmsi, const char* name) {
    std::string bits;
    append_bits(bits, 24, 6);
    append_bits(bits, 0, 2);
    append_bits(bits, mmsi, 30);
    append_bits(bits, 0, 2);
    append_text(bits, name, 20);
    append_bits(bits, 0, 8);
    return make_payload(bits);
}

static AisPayload make_type24_part_b(uint32_t mmsi, uint32_t ship_type, const char* vendor, const char* call_sign) {
    std::string bits;
    append_bits(bits, 24, 6);
    append_bits(bits, 0, 2);
    append_bits(bits, mmsi, 30);
    append_bits(bits, 1, 2);
    append_bits(bits, ship_type, 8);
    append_text(bits, vendor, 7);
    append_text(bits, call_sign, 7);
    append_bits(bits, 11, 9);
    append_bits(bits, 12, 9);
    append_bits(bits, 3, 6);
    append_bits(bits, 4, 6);
    return make_payload(bits);
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed_ais(app, single_vdm_body(make_type8_binary_payload()), now_us);
    const auto& binary = app.store().model().ais.binary_envelope;
    REQUIRE(binary.message_type.value == 8);
    REQUIRE(binary.dac.value == 1);
    REQUIRE(binary.function_id.value == 31);
    REQUIRE(binary.payload_bit_count.value == 16);
    const auto& appbin = app.store().model().ais.binary_application;
    REQUIRE(appbin.message_type.value == 8);
    REQUIRE(appbin.dac.value == 1);
    REQUIRE(appbin.function_id.value == 31);
    REQUIRE(appbin.payload_bit_count.value == 16);
    REQUIRE(appbin.payload_start_bit.value == 56);
    REQUIRE(appbin.first_payload_bits.value == 0xabcd);
    REQUIRE(appbin.known_application == true);
    REQUIRE(std::strcmp(appbin.application_label, "imo_met_hydro") == 0);

    feed_ais(app, single_vdm_body(make_type18_class_b_payload()), now_us);
    const auto& class_b = app.store().model().ais.class_b_position_report;
    REQUIRE(class_b.message_type.value == 18);
    REQUIRE(class_b.mmsi.value == 123123123);
    NEAR(class_b.speed_over_ground_kn.value, 12.3f, 0.001f);
    REQUIRE(class_b.cs_unit == true);
    REQUIRE(class_b.display == true);
    REQUIRE(class_b.dsc == false);
    REQUIRE(class_b.band == true);
    REQUIRE(class_b.accepts_message_22 == true);
    REQUIRE(class_b.assigned_mode == true);
    REQUIRE(class_b.raim == true);
    REQUIRE(class_b.radio_status.value == 0x12345);

    feed_ais(app, single_vdm_body(make_type21_aton_payload()), now_us);
    const auto& aton = app.store().model().ais.aid_to_navigation;
    REQUIRE(aton.message_type.value == 21);
    REQUIRE(aton.aid_type.value == 5);
    REQUIRE(aton.virtual_aid == true);
    REQUIRE(aton.assigned_mode == true);
    REQUIRE(aton.name_extension_available == true);
    REQUIRE(std::strcmp(aton.name_extension, "EXTNAME") == 0);

    signalk_mini::SignalKMiniApp<float> own_app;
    uint64_t own_us = 0;
    feed_ais(own_app, single_vdo_body(make_type18_class_b_payload(321321321u)), own_us);
    REQUIRE(own_app.store().model().ais.own_vessel.valid == true);
    REQUIRE(own_app.store().model().ais.own_vessel.mmsi.value == 321321321);
    REQUIRE(own_app.store().model().ais.own_vessel.class_b == true);
    REQUIRE(own_app.store().model().ais.targets.target_count.valid == false);
    REQUIRE(find_target(own_app.store().model().ais.targets, 321321321) == nullptr);

    signalk_mini::SignalKMiniApp<float> type24_app;
    uint64_t type24_us = 0;
    feed_ais(type24_app, single_vdm_body(make_type24_part_a(444000111u, "CLASSB ONE")), type24_us);
    const auto& part_a = type24_app.store().model().ais.class_b_static;
    REQUIRE(part_a.message_type.value == 24);
    REQUIRE(part_a.part_number.value == 0);
    REQUIRE(part_a.merge_mmsi.value == 444000111);
    REQUIRE(part_a.part_a_received == true);
    REQUIRE(part_a.part_b_received == false);
    REQUIRE(part_a.complete == false);
    REQUIRE(std::strcmp(part_a.vessel_name, "CLASSB ONE") == 0);

    feed_ais(type24_app, single_vdm_body(make_type24_part_b(444000111u, 36, "VEN1234", "CALL999")), type24_us);
    const auto& part_b = type24_app.store().model().ais.class_b_static;
    REQUIRE(part_b.part_number.value == 1);
    REQUIRE(part_b.merge_mmsi.value == 444000111);
    REQUIRE(part_b.part_a_received == true);
    REQUIRE(part_b.part_b_received == true);
    REQUIRE(part_b.complete == true);
    REQUIRE(part_b.ship_type.value == 36);
    REQUIRE(std::strcmp(part_b.call_sign, "CALL999") == 0);
    REQUIRE(std::strcmp(part_b.vessel_name, "CLASSB ONE") == 0);

    feed_ais(type24_app, single_vdm_body(make_type24_part_b(555000222u, 37, "VEN5678", "CALL222")), type24_us);
    const auto& changed = type24_app.store().model().ais.class_b_static;
    REQUIRE(changed.merge_mmsi.value == 555000222);
    REQUIRE(changed.part_a_received == false);
    REQUIRE(changed.part_b_received == true);
    REQUIRE(changed.complete == false);
    REQUIRE(changed.ship_type.value == 37);

    signalk_mini::SignalKMiniApp<float> lifecycle_app;
    uint64_t lifecycle_us = 0;
    feed_ais(lifecycle_app, single_vdm_body(make_type8_binary_payload()), lifecycle_us);
    feed_ais(lifecycle_app, single_vdm_body(make_type18_class_b_payload(123123123u)), lifecycle_us);
    auto& targets = lifecycle_app.store().model().ais.targets;
    REQUIRE(targets.target_count.value == 2);
    REQUIRE(ship_data_model::ais_mark_targets_stale_older_than(targets, 1500, lifecycle_us + 100) == 1);
    REQUIRE(find_target(targets, 333444555) != nullptr);
    REQUIRE(find_target(targets, 333444555)->stale == true);
    REQUIRE(find_target(targets, 123123123)->stale == false);
    REQUIRE(ship_data_model::ais_expire_targets_older_than(targets, 1500, lifecycle_us + 200) == 1);
    REQUIRE(targets.target_count.value == 1);
    REQUIRE(find_target(targets, 333444555) == nullptr);
    REQUIRE(find_target(targets, 123123123) != nullptr);
    REQUIRE(ship_data_model::ais_clear_target(targets, 123123123, lifecycle_us + 300) == true);
    REQUIRE(targets.target_count.value == 0);
    REQUIRE(ship_data_model::ais_clear_target(targets, 123123123, lifecycle_us + 400) == false);
    feed_ais(lifecycle_app, single_vdm_body(make_type8_binary_payload()), lifecycle_us);
    REQUIRE(targets.target_count.value == 1);
    ship_data_model::ais_clear_targets(targets);
    REQUIRE(targets.target_count.valid == false);
    REQUIRE(find_target(targets, 333444555) == nullptr);

    signalk_mini::SignalKMiniApp<float> hardening;
    uint64_t hardening_us = 0;
    const std::string payload = std::string("51mg=5@2Fe3t<4hk7;=@E=B1HE=<Dh000000") +
                                std::string("000U1PR347?os;B0DPSlP00000000000000");
    const std::string p1 = payload.substr(0, 24);
    const std::string p2 = payload.substr(24, 24);
    const std::string p3 = payload.substr(48);
    feed_ais(hardening, "AIVDM,3,1,71,B," + p1 + ",0", hardening_us);
    feed_ais(hardening, "AIVDM,3,2,71,B," + p2 + ",0", hardening_us);
    feed_ais(hardening, "AIVDM,3,2,71,B," + p2 + ",0", hardening_us);
    REQUIRE(hardening.nmea0183().message_state().ais_message.duplicate_fragment_count.value == 1);
    feed_ais(hardening, "AIVDM,3,3,71,B," + p3 + ",2", hardening_us);
    REQUIRE(hardening.nmea0183().message_state().ais_message.complete == true);
    REQUIRE(hardening.nmea0183().message_state().ais_message.duplicate_fragment_count.value == 1);
    REQUIRE(hardening.store().model().ais.static_voyage.message_type.value == 5);

    signalk_mini::SignalKMiniApp<float> orphan;
    uint64_t orphan_us = 0;
    feed_ais(orphan, "AIVDM,2,2,72,B," + p3 + ",2", orphan_us);
    REQUIRE(orphan.nmea0183().message_state().ais_message.bad_fragment_count.value == 1);
    REQUIRE(orphan.nmea0183().message_state().ais_message.complete == false);

    signalk_mini::SignalKMiniApp<float> stale;
    uint64_t stale_us = 0;
    feed_ais(stale, "AIVDM,2,1,73,B," + p1 + ",0", stale_us);
    REQUIRE(stale.nmea0183().message_state().ais_message.in_progress == true);
    stale_us += nmea0183_connector::NMEA_AIS_MULTIPART_STALE_TIMEOUT_US + 1u;
    feed_ais(stale, "AIVDM,2,1,74,B," + p1 + ",0", stale_us);
    REQUIRE(stale.nmea0183().message_state().ais_multipart_stale_count.value == 1);

    return 0;
}
