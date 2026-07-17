#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <signalk_mini/gpsd_input.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a)-(b))) > (e)) { std::fprintf(stderr, "NEAR FAILED %s:%d\n", __FILE__, __LINE__); std::exit(2); } } while (0)

int main() {
    signalk_mini::ModelStore<float> store;
    signalk_mini::GpsdInput<float> input(store, "/dev/ttyACM0", true, true);
    auto& model = store.model();

    const char* tpv = "{\"class\":\"TPV\",\"device\":\"/dev/ttyACM0\",\"mode\":3,\"time\":\"2026-07-17T05:06:07.500Z\",\"lat\":40.7654321,\"lon\":-74.1234567,\"altMSL\":12.3,\"altHAE\":43.2,\"speed\":5.0,\"track\":123.4,\"climb\":-0.5,\"epx\":1.2,\"epy\":1.5,\"epv\":2.5,\"eps\":0.2,\"epd\":0.8,\"ept\":0.01}\n";
    REQUIRE(input.feed_octets(reinterpret_cast<const uint8_t*>(tpv), std::strlen(tpv), 10, 1000));
    REQUIRE(store.pending_change_count() > 0);
    REQUIRE(model.gnss.fix.fix_valid.value);
    NEAR(model.gnss.fix.fix_lat_deg.value, 40.7654321f, 0.00001);
    NEAR(model.gnss.fix.fix_lon_deg.value, -74.1234567f, 0.00001);
    NEAR(model.gnss.fix.speed_kn.value, 9.71922f, 0.001);
    NEAR(model.gnss.fix.timestamp_s.value, 18367.5f, 0.001);
    REQUIRE(model.gnss.fix.date_year.value == 2026);
    NEAR(model.gnss.fix.geoidal_separation_m.value, 30.9f, 0.001);
    NEAR(model.gnss.fix_accuracy.horizontal_accuracy_m.value, 1.5f, 0.001);

    const float saved_lat = model.gnss.fix.fix_lat_deg.value;
    const char* nofix = "{\"class\":\"TPV\",\"device\":\"/dev/ttyACM0\",\"mode\":1,\"lat\":0,\"lon\":0}\n";
    input.feed_octets(reinterpret_cast<const uint8_t*>(nofix), std::strlen(nofix), 10, 1100);
    REQUIRE(!model.gnss.fix.fix_valid.value);
    NEAR(model.gnss.fix.fix_lat_deg.value, saved_lat, 0.000001);

    const char* sky = "{\"class\":\"SKY\",\"device\":\"/dev/ttyACM0\",\"gdop\":1.7,\"pdop\":1.2,\"tdop\":0.8,\"hdop\":0.9,\"vdop\":1.1,\"xdop\":0.6,\"ydop\":0.7,\"satellites\":[{\"PRN\":3,\"gnssid\":0,\"sigid\":1,\"el\":45,\"az\":120,\"ss\":38.5,\"used\":true},{\"PRN\":8,\"el\":20,\"az\":200,\"ss\":30,\"used\":false}]}\n";
    REQUIRE(input.feed_octets(reinterpret_cast<const uint8_t*>(sky), std::strlen(sky), 10, 1200));
    REQUIRE(model.gnss.sky_view.observation_count == 2);
    REQUIRE(model.gnss.sky_view.satellites_used.value == 1);
    REQUIRE(model.gnss.sky_view.observations[0].satellite_id == 3);
    NEAR(model.gnss.dop_active_satellites.pdop.value, 1.2f, 0.001);

    const char* gst = "{\"class\":\"GST\",\"device\":\"/dev/ttyACM0\",\"time\":\"2026-07-17T05:06:08Z\",\"major\":2.1,\"minor\":1.1,\"orient\":22.0,\"lat\":1.4,\"lon\":1.2,\"alt\":2.8}\n";
    REQUIRE(input.feed_octets(reinterpret_cast<const uint8_t*>(gst), std::strlen(gst), 10, 1300));
    NEAR(model.gnss.fix_accuracy.semi_major_accuracy_m.value, 2.1f, 0.001);
    NEAR(model.gnss.fix_accuracy.horizontal_accuracy_m.value, 1.4f, 0.001);
    return 0;
}
