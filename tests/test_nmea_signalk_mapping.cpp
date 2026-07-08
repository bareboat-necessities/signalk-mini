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

static void require_object_json(signalk_mini::ModelStore<float>& store,
                                signalk_mini::ModelField field,
                                const char* path,
                                const char* expected_fragment) {
    store.mark_changed(field, 1, 1000000);
    signalk_mini::SignalKMappedValue<float> mapped;
    REQUIRE(pop_path(store, path, mapped));
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Object);
    char json[1024];
    signalk_mini::SignalKDeltaWriter<float> writer;
    const int n = writer.write_mapped(json, sizeof(json), "test", store.model(), mapped);
    REQUIRE(n > 0);
    REQUIRE(std::strstr(json, expected_fragment) != nullptr);
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    signalk_mini::SignalKMappedValue<float> mapped;
    uint64_t now_us = 1000;

    REQUIRE(app.nmea0183().feed_line("$WIMDA,29.884,I,1.012,B,32.1,C,,,12.8,,0.0,C,243.3,T,241.1,M,2.5,N,1.3,M*00", 1, now_us, false));
    REQUIRE(pop_path(app.store(), "environment.outside.pressure", mapped));
    NEAR(mapped.number, 101200.0f, 20.0f);

    now_us += 1000;
    REQUIRE(app.nmea0183().feed_line("$IIVBW,1.0,0.2,A,2.0,0.3,A*00", 1, now_us, false));
    REQUIRE(pop_path(app.store(), "navigation.speedThroughWaterTransverse", mapped));
    NEAR(mapped.number, 0.2f * 0.514444444f, 0.0001f);

    now_us += 1000;
    REQUIRE(app.nmea0183().feed_line("$GPRMB,A,0.66,L,ORIG,DEST,4916.45,N,12311.12,W,5.5,054.7,2.1,A*00", 1, now_us, false));
    REQUIRE(pop_path(app.store(), "navigation.courseGreatCircle.nextPoint.position.latitude", mapped));
    NEAR(mapped.number, 49.274166f, 0.0002f);

    now_us += 1000;
    REQUIRE(app.nmea0183().feed_line("$IIHFB,3.2,M,5.4,M*00", 1, now_us, false));
    REQUIRE(pop_path(app.store(), "environment.trawl.headropeToFootrope", mapped));
    NEAR(mapped.number, 3.2f, 0.0001f);

    now_us += 1000;
    REQUIRE(app.nmea0183().feed_line("$IIXDR,A,1.2,D,PTCH,A,-2.3,D,ROLL*00", 1, now_us, false));
    REQUIRE(pop_path(app.store(), "navigation.attitude.pitch", mapped));
    NEAR(mapped.number, 1.2f * 3.14159265358979323846f / 180.0f, 0.0001f);

    signalk_mini::ModelStore<float> store;
    auto& target = store.model().ais.targets.targets[0];
    target.occupied = true;
    target.mmsi.set(123456789, 1000);
    target.latitude_deg.set(40.1f, 1000);
    target.longitude_deg.set(-73.9f, 1000);
    target.speed_over_ground_kn.set(6.2f, 1000);
    std::strcpy(target.vessel_name, "TESTVESSEL");
    store.model().ais.targets.target_count.set(1, 1000);
    require_object_json(store, signalk_mini::ModelField::AisTargetsObject, "navigation.ais.targets", "123456789");

    auto& dsc = store.model().notifications.dsc.distress;
    std::strcpy(dsc.sender_mmsi, "111222333");
    std::strcpy(dsc.alert_text, "distress");
    dsc.active = true;
    dsc.last_update_us = 2000;
    require_object_json(store, signalk_mini::ModelField::DscStructuredNotification, "notifications.dsc", "distress");

    auto& navtex = store.model().notifications.navtex.received;
    std::strcpy(navtex.navtex_message_id, "A1");
    std::strcpy(navtex.body_text, "navtex body");
    navtex.first_seen_us = 3000;
    require_object_json(store, signalk_mini::ModelField::NavtexStructuredNotification, "notifications.navtex", "navtex body");

    return 0;
}
