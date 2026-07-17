#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <ubx.hpp>
#include "ubx_test_helpers.hpp"

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

int main() {
    ship_data_model::DataModel<float> model;
    ubx::Receiver<float> receiver;

    const std::vector<uint8_t> ack = ubx_test::frame(0x05, 0x01, {0x06, 0x8a});
    for (size_t i = 0; i < ack.size(); ++i) receiver.accept_octets(&ack[i], 1, model, 1000 + i);
    REQUIRE(receiver.diagnostics().frame_count == 1);
    REQUIRE(receiver.info().ack_valid);
    REQUIRE(receiver.info().last_ack_accepted);
    REQUIRE(receiver.info().last_ack_class == 0x06);
    REQUIRE(receiver.info().last_ack_id == 0x8a);

    std::vector<uint8_t> bad = ack;
    bad.back() ^= 0xffu;
    receiver.accept_octets(bad.data(), bad.size(), model, 2000);
    REQUIRE(receiver.diagnostics().checksum_error_count == 1);

    const uint8_t noise[] = {0x00, 0x11, 0xB5, 0x00, 0xB5};
    receiver.accept_octets(noise, sizeof(noise), model, 3000);
    receiver.accept_octets(ack.data() + 1, ack.size() - 1, model, 3001);
    REQUIRE(receiver.diagnostics().frame_count == 2);
    REQUIRE(receiver.diagnostics().discarded_byte_count >= 3);

    const std::vector<uint8_t> version = ubx_test::mon_ver();
    receiver.accept_octets(version.data(), 7, model, 4000);
    receiver.accept_octets(version.data() + 7, version.size() - 7, model, 4001);
    REQUIRE(receiver.diagnostics().frame_count == 3);
    REQUIRE(std::strcmp(receiver.info().software_version, "EXT CORE 1.00") == 0);
    REQUIRE(std::strcmp(receiver.info().hardware_version, "00190000") == 0);
    REQUIRE(std::strcmp(receiver.info().protocol_version, "27.31") == 0);

    ubx::Receiver<float, 16> small;
    const std::vector<uint8_t> oversized = ubx_test::frame(0x01, 0x99, std::vector<uint8_t>(20, 0x55));
    small.accept_octets(oversized.data(), oversized.size(), model, 5000);
    REQUIRE(small.diagnostics().oversized_frame_count == 1);
    REQUIRE(small.diagnostics().frame_count == 0);
    small.accept_octets(ack.data(), ack.size(), model, 5001);
    REQUIRE(small.diagnostics().frame_count == 1);

    ubx::Receiver<float, 16> corrupt_length;
    const uint8_t corrupt_header[] = {0xB5, 0x62, 0x01, 0x07, 0xff, 0xff};
    corrupt_length.accept_octets(corrupt_header, sizeof(corrupt_header), model, 5500);
    corrupt_length.accept_octets(ack.data(), ack.size(), model, 5501);
    REQUIRE(corrupt_length.diagnostics().oversized_frame_count == 1);
    REQUIRE(corrupt_length.diagnostics().frame_count == 1);

    const std::vector<uint8_t> unknown = ubx_test::frame(0x99, 0x01, {});
    receiver.accept_octets(unknown.data(), unknown.size(), model, 6000);
    REQUIRE(receiver.diagnostics().unsupported_frame_count == 1);

    return 0;
}
