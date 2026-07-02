#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d\n", __FILE__, __LINE__); std::exit(2); } } while (0)

static std::string sentence(const char* body) {
    char out[160];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body, nmea0183_connector::to_hex((cs >> 4) & 0x0f), nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

int main() {
    using Real = float;

    signalk_mini::SignalKMiniConfig default_config;
    REQUIRE(default_config.connector_count == 1);
    REQUIRE(default_config.connectors != nullptr);
    REQUIRE(default_config.connectors[0].enabled);
    REQUIRE(default_config.connectors[0].protocol.kind == signalk_mini::ConnectorProtocol::Nmea0183);
    REQUIRE(default_config.connectors[0].transport.kind == signalk_mini::ConnectorTransport::Udp);
    REQUIRE(std::strcmp(default_config.connectors[0].transport.udp.listen_host, "0.0.0.0") == 0);
    REQUIRE(default_config.connectors[0].transport.udp.listen_port == 10110);
    REQUIRE(default_config.connectors[0].protocol.nmea0183.validate_checksum == false);

    signalk_mini::SignalKMiniApp<Real> app;
    const char* bodies[] = {
        "GPRMC,142533.00,A,4042.6142,N,07400.4168,W,5.4,083.2,010726,13.1,W,A",
        "HCHDT,081.7,T",
        "IIMWV,045.0,R,12.8,N,A",
        "SDDPT,5.6,0.4"
    };
    uint64_t now_us = 0;
    for (const char* body : bodies) {
        const std::string line = sentence(body);
        now_us += 100000;
        REQUIRE(app.nmea0183().feed_line(line.c_str(), 7, now_us));
    }
    const auto& model = app.store().model();
    REQUIRE(model.navigation.gps.fix_lat_deg.valid);
    REQUIRE(model.navigation.gps.fix_lon_deg.valid);
    REQUIRE(model.navigation.gps.speed_kn.valid);
    REQUIRE(model.imu.heading_deg.valid);
    REQUIRE(model.wind.apparent.speed_kn.valid);
    REQUIRE(model.water.depth_m.valid);
    NEAR(model.navigation.gps.fix_lat_deg.value, 40.7102367f, 0.0002f);
    NEAR(model.navigation.gps.fix_lon_deg.value, -74.0069467f, 0.0002f);
    NEAR(model.navigation.gps.speed_kn.value, 5.4f, 0.001f);
    NEAR(model.wind.apparent.speed_kn.value, 12.8f, 0.001f);
    NEAR(model.water.depth_m.value, 5.6f, 0.001f);
    signalk_mini::SignalKMapper<Real> mapper;
    signalk_mini::ModelChange change;
    bool saw_wind_speed = false;
    while (app.store().changes().pop(change)) {
        signalk_mini::SignalKMappedValue<Real> mapped;
        if (mapper.map_change(app.store().model(), change, mapped) && mapped.path && std::strcmp(mapped.path, "environment.wind.speedApparent") == 0) {
            saw_wind_speed = true;
            NEAR(mapped.number, 6.5848889f, 0.001f);
        }
    }
    REQUIRE(saw_wind_speed);
    return 0;
}
