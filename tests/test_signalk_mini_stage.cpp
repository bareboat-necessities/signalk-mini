#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <signalk_mini.hpp>

#define OK(expr) do { if (!(expr)) { std::fprintf(stderr, "failed %s:%d: %s\n", __FILE__, __LINE__, #expr); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { long double da=(long double)(a); long double db=(long double)(b); if (fabsl(da-db)>(long double)(e)) { std::fprintf(stderr, "near failed %s:%d: %.12Lf vs %.12Lf\n", __FILE__, __LINE__, da, db); std::exit(2); } } while (0)

int main() {
    using Real = float;
    signalk_mini::ModelStore<Real> store;
    signalk_mini::Nmea0183Input<Real> input(store);

    OK(input.feed_line("$GPRMC,142533.00,A,4042.6142,N,07400.4168,W,5.4,083.2,010726,13.1,W,A*05", 7, 100));
    OK(input.feed_line("$HCHDT,081.7,T*27", 7, 200));
    OK(input.feed_line("$IIMWV,045.0,R,12.8,N,A*07", 7, 300));
    OK(input.feed_line("$SDDPT,5.6,0.4*50", 7, 400));

    const auto& model = store.model();
    NEAR(model.navigation.gps.fix_lat_deg.value, 40.7102367L, 0.0001L);
    NEAR(model.navigation.gps.fix_lon_deg.value, -74.0069467L, 0.0001L);
    NEAR(model.navigation.gps.speed_kn.value, 5.4L, 0.0001L);
    NEAR(model.navigation.gps.track_deg.value, 83.2L, 0.0001L);
    NEAR(model.imu.heading_deg.value, 81.7L, 0.0001L);
    NEAR(model.wind.apparent.direction_deg.value, 45.0L, 0.0001L);
    NEAR(model.wind.apparent.speed_kn.value, 12.8L, 0.0001L);
    NEAR(model.water.depth_m.value, 5.6L, 0.0001L);

    signalk_mini::SignalKMapper<Real> mapper;
    signalk_mini::ModelChange change;
    bool got_wind_angle = false;
    bool got_wind_speed = false;
    bool got_depth = false;
    while (store.changes().pop(change)) {
        signalk_mini::SignalKMappedValue<Real> mapped;
        if (!mapper.map_change(store.model(), change, mapped)) continue;
        if (std::strcmp(mapped.path, "environment.wind.angleApparent") == 0) {
            got_wind_angle = true;
            NEAR(mapped.number, signalk_mini::deg_to_rad<Real>(Real(45)), 0.0001L);
        }
        if (std::strcmp(mapped.path, "environment.wind.speedApparent") == 0) {
            got_wind_speed = true;
            NEAR(mapped.number, signalk_mini::knots_to_mps<Real>(Real(12.8)), 0.0001L);
        }
        if (std::strcmp(mapped.path, "environment.depth.belowTransducer") == 0) {
            got_depth = true;
            NEAR(mapped.number, 5.6L, 0.0001L);
        }
    }
    OK(got_wind_angle);
    OK(got_wind_speed);
    OK(got_depth);
    return 0;
}
