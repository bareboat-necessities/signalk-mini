#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got=%f expected=%f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static bool pop_path(signalk_mini::ModelStore<float>& store, const char* path, signalk_mini::SignalKMappedValue<float>& found) {
    signalk_mini::SignalKMapper<float> mapper;
    signalk_mini::ModelChange change;
    bool ok = false;
    while (store.changes().pop(change)) {
        signalk_mini::SignalKMappedValue<float> mapped;
        if (!mapper.map_change(store.model(), change, mapped) || !mapped.path) continue;
        if (std::strcmp(mapped.path, path) == 0) {
            found = mapped;
            ok = true;
        }
    }
    return ok;
}

static void mark(signalk_mini::ModelStore<float>& store, signalk_mini::ModelField field) {
    store.mark_changed(field, 1, 1000000);
}

int main() {
    signalk_mini::ModelStore<float> store;
    signalk_mini::SignalKMappedValue<float> mapped;

    store.model().env.barometric_pressure_bar.set(1.012f, 1000);
    mark(store, signalk_mini::ModelField::EnvBarometricPressureBar);
    REQUIRE(pop_path(store, "environment.outside.pressure", mapped));
    NEAR(mapped.number, 101200.0f, 20.0f);

    store.model().sea.transverse_water_speed_kn.set(0.2f, 1000);
    mark(store, signalk_mini::ModelField::SeaTransverseWaterSpeedKn);
    REQUIRE(pop_path(store, "navigation.speedThroughWaterTransverse", mapped));
    NEAR(mapped.number, 0.2f * 0.514444444f, 0.0001f);

    store.model().route.rmb.destination_lat_deg.set(49.274166f, 1000);
    mark(store, signalk_mini::ModelField::RouteRmbDestinationLatDeg);
    REQUIRE(pop_path(store, "navigation.courseGreatCircle.nextPoint.position.latitude", mapped));
    NEAR(mapped.number, 49.274166f, 0.0002f);

    store.model().fishing.trawl.headrope_to_footrope_m.set(3.2f, 1000);
    mark(store, signalk_mini::ModelField::TrawlHeadropeToFootropeM);
    REQUIRE(pop_path(store, "fishing.trawl.headropeToFootrope", mapped));
    NEAR(mapped.number, 3.2f, 0.0001f);

    store.model().ins.imu.pitch_deg.set(1.2f, 1000);
    mark(store, signalk_mini::ModelField::ImuPitchDeg);
    REQUIRE(pop_path(store, "navigation.attitude.pitch", mapped));
    NEAR(mapped.number, 1.2f * 3.14159265358979323846f / 180.0f, 0.0001f);

    auto& target = store.model().ais.targets.targets[0];
    target.occupied = true;
    target.mmsi.set(123456789, 1000);
    target.latitude_deg.set(40.1f, 1000);
    target.longitude_deg.set(-73.9f, 1000);
    store.model().ais.targets.target_count.set(1, 1000);
    mark(store, signalk_mini::ModelField::AisTargetsObject);
    REQUIRE(pop_path(store, "navigation.ais.targets", mapped));
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Object);

    auto& navtex = store.model().notifications.navtex.received;
    std::strcpy(navtex.navtex_message_id, "A1");
    std::strcpy(navtex.body_text, "navtex body");
    navtex.first_seen_us = 3000;
    mark(store, signalk_mini::ModelField::NavtexStructuredNotification);
    REQUIRE(pop_path(store, "notifications.navtex", mapped));
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Object);

    return 0;
}
