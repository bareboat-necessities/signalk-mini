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

static std::string encapsulated_sentence(const char* body) {
    char out[192];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "!%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void feed_ais(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = encapsulated_sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

static void append_bits(std::string& bits, uint32_t value, uint8_t count) {
    for (int i = static_cast<int>(count) - 1; i >= 0; --i) {
        bits.push_back(((value >> i) & 1u) ? '1' : '0');
    }
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

static std::string single_vdm_body(const AisPayload& payload, char channel = 'A') {
    char out[192];
    std::snprintf(out, sizeof(out), "AIVDM,1,1,,%c,%s,%d", channel, payload.payload.c_str(), payload.fill_bits);
    return std::string(out);
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

static AisPayload make_type9_sar_payload() {
    std::string bits;
    append_bits(bits, 9, 6);
    append_bits(bits, 0, 2);
    append_bits(bits, 111222333u, 30);
    append_bits(bits, 1234, 12);
    append_bits(bits, 456, 10);
    append_bits(bits, 1, 1);
    append_signed_bits(bits, static_cast<int32_t>(std::lround(-73.9857 * 600000.0)), 28);
    append_signed_bits(bits, static_cast<int32_t>(std::lround(40.7484 * 600000.0)), 27);
    append_bits(bits, 1234, 12);
    append_bits(bits, 27, 6);
    append_bits(bits, 0, 1);
    while (bits.size() < 148u) bits.push_back('0');
    append_bits(bits, 1, 1);
    while (bits.size() < 168u) bits.push_back('0');
    return make_payload(bits);
}

static AisPayload make_type27_long_range_payload() {
    std::string bits;
    append_bits(bits, 27, 6);
    append_bits(bits, 0, 2);
    append_bits(bits, 222333444u, 30);
    append_bits(bits, 1, 1);
    append_bits(bits, 1, 1);
    append_bits(bits, 0, 4);
    append_signed_bits(bits, static_cast<int32_t>(std::lround(-73.9 * 600.0)), 18);
    append_signed_bits(bits, static_cast<int32_t>(std::lround(40.7 * 600.0)), 17);
    append_bits(bits, 45, 6);
    append_bits(bits, 123, 9);
    append_bits(bits, 1, 1);
    return make_payload(bits);
}

static const ship_data_model::AisTargetData<float>* find_target(const ship_data_model::AisTargetTableData<float>& table, int32_t mmsi) {
    for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
        if (table.targets[i].occupied && table.targets[i].mmsi.valid && table.targets[i].mmsi.value == mmsi) {
            return &table.targets[i];
        }
    }
    return nullptr;
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed_ais(app, "AIVDM,1,1,,A,11mg=5@P0oreDP8GD@44lRmD0000,0", now_us);

    const auto& pos = app.store().model().ais.position_report;
    REQUIRE(pos.message_type.value == 1);
    REQUIRE(pos.repeat_indicator.value == 0);
    REQUIRE(pos.mmsi.value == 123456789);
    REQUIRE(pos.navigation_status.value == 0);
    NEAR(pos.speed_over_ground_kn.value, 5.5f, 0.001f);
    NEAR(pos.longitude_deg.value, -73.9857f, 0.0001f);
    NEAR(pos.latitude_deg.value, 40.7484f, 0.0001f);
    NEAR(pos.course_over_ground_deg.value, 123.4f, 0.001f);
    NEAR(pos.true_heading_deg.value, 90.0f, 0.001f);
    REQUIRE(pos.timestamp_s.value == 42);
    REQUIRE(app.store().model().ais.tracked_target.target_number.value == 123456789);
    NEAR(app.store().model().ais.tracked_target.latitude_deg.value, 40.7484f, 0.0001f);

    const auto* target = find_target(app.store().model().ais.targets, 123456789);
    REQUIRE(target != nullptr);
    NEAR(target->latitude_deg.value, 40.7484f, 0.0001f);
    REQUIRE(app.store().model().ais.targets.target_count.value == 1);

    feed_ais(app, "AIVDM,2,1,7,B,51mg=5@2Fe3t<4hk7;=@E=B1HE=<Dh000000,0", now_us);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == false);
    feed_ais(app, "AIVDM,2,2,7,B,000U1PR347?os;B0DPSlP00000000000000,2", now_us);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == true);

    const auto& stat = app.store().model().ais.static_voyage;
    REQUIRE(stat.message_type.value == 5);
    REQUIRE(stat.repeat_indicator.value == 0);
    REQUIRE(stat.mmsi.value == 123456789);
    REQUIRE(stat.imo_number.value == 9876543);
    REQUIRE(std::strcmp(stat.call_sign, "CALL123") == 0);
    REQUIRE(std::strcmp(stat.vessel_name, "TEST VESSEL") == 0);
    REQUIRE(stat.ship_type.value == 37);
    REQUIRE(stat.dimension_to_bow_m.value == 12);
    REQUIRE(stat.dimension_to_stern_m.value == 34);
    REQUIRE(stat.dimension_to_port_m.value == 3);
    REQUIRE(stat.dimension_to_starboard_m.value == 4);
    REQUIRE(stat.epfd_type.value == 1);
    REQUIRE(stat.eta_month.value == 12);
    REQUIRE(stat.eta_day.value == 31);
    REQUIRE(stat.eta_hour.value == 23);
    REQUIRE(stat.eta_minute.value == 59);
    NEAR(stat.draught_m.value, 4.5f, 0.001f);
    REQUIRE(std::strcmp(stat.destination, "HARBOR") == 0);
    REQUIRE(stat.dte_ready == true);
    REQUIRE(std::strcmp(app.store().model().ais.tracked_target.target_name, "TEST VESSEL") == 0);
    target = find_target(app.store().model().ais.targets, 123456789);
    REQUIRE(target != nullptr);
    REQUIRE(std::strcmp(target->vessel_name, "TEST VESSEL") == 0);
    REQUIRE(std::strcmp(target->destination, "HARBOR") == 0);

    const std::string type8 = single_vdm_body(make_type8_binary_payload());
    feed_ais(app, type8.c_str(), now_us);
    const auto& binary = app.store().model().ais.binary_envelope;
    REQUIRE(binary.message_type.value == 8);
    REQUIRE(binary.mmsi.value == 333444555);
    REQUIRE(binary.dac.value == 1);
    REQUIRE(binary.function_id.value == 31);
    REQUIRE(binary.payload_bit_count.value == 16);
    REQUIRE(binary.addressed == false);
    REQUIRE(find_target(app.store().model().ais.targets, 333444555) != nullptr);

    const std::string type9 = single_vdm_body(make_type9_sar_payload());
    feed_ais(app, type9.c_str(), now_us);
    const auto& sar = app.store().model().ais.sar_aircraft_position;
    REQUIRE(sar.message_type.value == 9);
    REQUIRE(sar.mmsi.value == 111222333);
    REQUIRE(sar.altitude_m.value == 1234);
    NEAR(sar.speed_over_ground_kn.value, 45.6f, 0.001f);
    NEAR(sar.longitude_deg.value, -73.9857f, 0.0001f);
    NEAR(sar.latitude_deg.value, 40.7484f, 0.0001f);
    NEAR(sar.course_over_ground_deg.value, 123.4f, 0.001f);
    REQUIRE(sar.timestamp_s.value == 27);
    REQUIRE(sar.raim == true);
    target = find_target(app.store().model().ais.targets, 111222333);
    REQUIRE(target != nullptr);
    REQUIRE(target->sar_aircraft == true);
    NEAR(target->speed_over_ground_kn.value, 45.6f, 0.001f);

    const std::string type27 = single_vdm_body(make_type27_long_range_payload());
    feed_ais(app, type27.c_str(), now_us);
    const auto& lr = app.store().model().ais.long_range_broadcast;
    REQUIRE(lr.message_type.value == 27);
    REQUIRE(lr.mmsi.value == 222333444);
    REQUIRE(lr.navigation_status.value == 0);
    NEAR(lr.longitude_deg.value, -73.9f, 0.001f);
    NEAR(lr.latitude_deg.value, 40.7f, 0.001f);
    NEAR(lr.speed_over_ground_kn.value, 45.0f, 0.001f);
    NEAR(lr.course_over_ground_deg.value, 123.0f, 0.001f);
    REQUIRE(lr.raim == true);
    target = find_target(app.store().model().ais.targets, 222333444);
    REQUIRE(target != nullptr);
    NEAR(target->longitude_deg.value, -73.9f, 0.001f);
    REQUIRE(app.store().model().ais.targets.target_count.value == 4);

    return 0;
}
