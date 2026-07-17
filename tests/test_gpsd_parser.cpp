#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <gpsd.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

int main() {
    char watch[256];
    REQUIRE(gpsd::make_watch_command(watch, sizeof(watch), "/dev/ttyACM0"));
    REQUIRE(std::strstr(watch, "?WATCH=") == watch);
    REQUIRE(std::strstr(watch, "\"device\":\"/dev/ttyACM0\"") != nullptr);
    REQUIRE(!gpsd::make_watch_command(watch, sizeof(watch), "/dev/bad\"device"));

    ship_data_model::DataModel<float> model;
    gpsd::Client<float> client("/dev/ttyACM0", true, true);
    client.on_connected();
    REQUIRE(client.state() == gpsd::SessionState::WaitingForVersion);

    const char* version = "{\"class\":\"VERSION\",\"release\":\"3.25\",\"rev\":\"test\",\"proto_major\":3,\"proto_minor\":14}\n";
    client.accept_octets(reinterpret_cast<const uint8_t*>(version), 17, model, 1000);
    const auto updates = client.accept_octets(reinterpret_cast<const uint8_t*>(version + 17), std::strlen(version) - 17, model, 1001);
    REQUIRE(gpsd::has_update(updates, gpsd::UpdateKind::Version));
    REQUIRE(client.server_info().valid);
    REQUIRE(std::strcmp(client.server_info().release, "3.25") == 0);

    const char* mismatch = "{\"class\":\"TPV\",\"device\":\"/dev/ttyUSB9\",\"mode\":3,\"lat\":1,\"lon\":2}\n";
    client.accept_octets(reinterpret_cast<const uint8_t*>(mismatch), std::strlen(mismatch), model, 2000);
    REQUIRE(client.diagnostics().device_mismatch_count == 1);
    REQUIRE(!model.gnss.fix.fix_lat_deg.valid);

    const char* malformed = "{not-json}\n";
    client.accept_octets(reinterpret_cast<const uint8_t*>(malformed), std::strlen(malformed), model, 3000);
    REQUIRE(client.diagnostics().malformed_record_count == 1);

    gpsd::Client<float, 32> small;
    std::string oversized(40, 'x');
    oversized.push_back('\n');
    small.accept_octets(reinterpret_cast<const uint8_t*>(oversized.data()), oversized.size(), model, 4000);
    REQUIRE(small.diagnostics().oversized_record_count == 1);
    const char* watch_response = "{\"class\":\"WATCH\",\"enable\":true}\n";
    small.accept_octets(reinterpret_cast<const uint8_t*>(watch_response), std::strlen(watch_response), model, 4001);
    REQUIRE(small.state() == gpsd::SessionState::Watching);
    const char* watch_disabled = "{\"class\":\"WATCH\",\"enable\":false}\n";
    small.on_connected();
    small.accept_octets(reinterpret_cast<const uint8_t*>(watch_disabled), std::strlen(watch_disabled), model, 4002);
    REQUIRE(small.state() == gpsd::SessionState::WaitingForVersion);
    return 0;
}
