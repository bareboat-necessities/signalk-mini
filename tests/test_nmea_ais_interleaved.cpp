#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

struct AisPayload {
    std::string payload;
    int fill_bits = 0;
};

static std::string encapsulated_sentence(const char* body) {
    char out[220];
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
    for (int i = static_cast<int>(count) - 1; i >= 0; --i) bits.push_back(((value >> i) & 1u) ? '1' : '0');
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

static std::string multipart_body(const char* talker_sentence,
                                  int total,
                                  int number,
                                  int sequence_id,
                                  char channel,
                                  const std::string& payload,
                                  int fill_bits) {
    char out[220];
    std::snprintf(out, sizeof(out), "%s,%d,%d,%d,%c,%s,%d",
                  talker_sentence, total, number, sequence_id, channel, payload.c_str(), fill_bits);
    return std::string(out);
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    const char* type5_part1 = "AIVDM,2,1,7,B,51mg=5@2Fe3t<4hk7;=@E=B1HE=<Dh000000,0";
    const char* type5_part2 = "AIVDM,2,2,7,B,000U1PR347?os;B0DPSlP00000000000000,2";

    const AisPayload type8 = make_type8_binary_payload();
    const size_t split = type8.payload.size() / 2u;
    const std::string type8_part1_payload = type8.payload.substr(0, split);
    const std::string type8_part2_payload = type8.payload.substr(split);
    const std::string type8_part1 = multipart_body("AIVDM", 2, 1, 8, 'A', type8_part1_payload, 0);
    const std::string type8_part2 = multipart_body("AIVDM", 2, 2, 8, 'A', type8_part2_payload, type8.fill_bits);

    feed_ais(app, type5_part1, now_us);
    const int32_t static_slot = app.nmea0183().message_state().active_ais_slot.value;
    REQUIRE(static_slot >= 0 && static_slot < nmea0183_connector::NMEA_AIS_MULTIPART_SLOT_COUNT);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == false);

    feed_ais(app, type8_part1.c_str(), now_us);
    const int32_t binary_slot = app.nmea0183().message_state().active_ais_slot.value;
    REQUIRE(binary_slot >= 0 && binary_slot < nmea0183_connector::NMEA_AIS_MULTIPART_SLOT_COUNT);
    REQUIRE(binary_slot != static_slot);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == false);

    feed_ais(app, type5_part2, now_us);
    REQUIRE(app.nmea0183().message_state().active_ais_slot.value == static_slot);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == true);
    REQUIRE(app.store().model().ais.static_voyage.message_type.value == 5);
    REQUIRE(app.store().model().ais.static_voyage.mmsi.value == 123456789);
    REQUIRE(std::strcmp(app.store().model().ais.static_voyage.vessel_name, "TEST VESSEL") == 0);

    feed_ais(app, type8_part2.c_str(), now_us);
    REQUIRE(app.nmea0183().message_state().active_ais_slot.value == binary_slot);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == true);
    REQUIRE(app.store().model().ais.binary_envelope.message_type.value == 8);
    REQUIRE(app.store().model().ais.binary_envelope.mmsi.value == 333444555);
    REQUIRE(app.store().model().ais.binary_envelope.dac.value == 1);
    REQUIRE(app.store().model().ais.binary_envelope.function_id.value == 31);

    return 0;
}
