#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got=%f expected=%f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static bool feed_and_pop(const char* line,
                         const char* path,
                         signalk_mini::SignalKMappedValue<float>& found,
                         char* json,
                         size_t json_size) {
    signalk_mini::ModelStore<float> store;
    signalk_mini::Nmea0183Input<float> input(store);
    REQUIRE(input.feed_line(line, 7, 1000000ULL, false));

    signalk_mini::SignalKMapper<float> mapper;
    signalk_mini::SignalKDeltaWriter<float> writer;
    signalk_mini::ModelChange change;

    while (store.changes().pop(change)) {
        signalk_mini::SignalKMappedValue<float> mapped;
        if (!mapper.map_change(store.model(), change, mapped) || !mapped.path) continue;
        if (std::strcmp(mapped.path, path) != 0) continue;

        const int len = writer.write_mapped(json, json_size, "audit", store.model(), mapped);
        REQUIRE(len > 0);
        found = mapped;
        return true;
    }

    return false;
}

static void require_path_contains(const char* line, const char* path, const char* needle) {
    signalk_mini::SignalKMappedValue<float> mapped;
    char json[2048];
    REQUIRE(feed_and_pop(line, path, mapped, json, sizeof(json)));
    REQUIRE(std::strstr(json, needle) != nullptr);
}

int main() {
    signalk_mini::SignalKMappedValue<float> mapped;
    char json[2048];

    REQUIRE(feed_and_pop("$GPAAM,A,A,0.10,N,WPT1", "navigation.courseGreatCircle.arrivalCircleRadius", mapped, json, sizeof(json)));
    NEAR(mapped.number, 185.2f, 0.2f);

    REQUIRE(feed_and_pop("$GPBEC,123519,4807.038,N,01131.000,E,54.7,T,34.4,M,1.25,N,WPT2", "navigation.courseGreatCircle.nextPoint.distance", mapped, json, sizeof(json)));
    NEAR(mapped.number, 2315.0f, 1.0f);

    REQUIRE(feed_and_pop("$IICUR,A,123.4,T,1.2,N", "environment.current.speed", mapped, json, sizeof(json)));
    NEAR(mapped.number, 1.2f * 0.514444444f, 0.0001f);

    require_path_contains("!AIADS,ST01,SLOT,OK", "navigation.ais.dataLinkStatus", "SLOT");
    require_path_contains("!AIASD,1,1,SEQ,123456789,A,Safety text", "notifications.ais.safety", "Safety text");
    require_path_contains("$IIARC,ALARM1,2,CMD,01,Refused text", "notifications.alerts", "commandRefused");
    require_path_contains("$IIDCR,EQ1,CMD1,A,OK", "communication.legacy", "controlResponse");

    REQUIRE(feed_and_pop("$IIEVE,EV1,SRC,A,Event text", "notifications.message.event", mapped, json, sizeof(json)));
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Text);
    REQUIRE(std::strcmp(mapped.text, "Event text") == 0);

    REQUIRE(feed_and_pop("$IIETL,EV1,TYPE,A,Event log text", "notifications.message.eventLog", mapped, json, sizeof(json)));
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Text);
    REQUIRE(std::strcmp(mapped.text, "Event log text") == 0);

    return 0;
}
