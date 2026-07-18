#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <signalk_mini/config.hpp>
#include <signalk_mini/model_store.hpp>
#include <signalk_mini/signalk_mapper.hpp>
#include <signalk_mini/signalk_typed_view.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

int main() {
    signalk_mini::SignalKMiniConfig config;
    config.publisher.current_value_timeout_ms = 100;

    signalk_mini::ModelStore<float> store;
    auto& model = store.model();
    constexpr signalk_mini::SourceId source = signalk_mini::FirstConnectorSourceId;

    std::strcpy(model.route.apb.destination_id, "OLD");
    model.route.apb.last_update_us = 1000000;
    store.mark_changed(signalk_mini::ModelField::RouteApbDestinationId, source, 1000000);

    signalk_mini::SignalKMappedValue<float> mapped;
    signalk_mini::SignalKTypedModelView<float> fresh_apb(store, config, 1050000);
    REQUIRE(fresh_apb.current_value(signalk_mini::ModelField::RouteApbDestinationId, mapped));
    REQUIRE(mapped.last_update_us == 1000000);
    REQUIRE(std::strcmp(mapped.text, "OLD") == 0);

    signalk_mini::SignalKTypedModelView<float> expired_apb(store, config, 1100001);
    REQUIRE(!expired_apb.current_value(signalk_mini::ModelField::RouteApbDestinationId, mapped));

    model.route.apb.last_update_us = 1950000;
    store.mark_changed(signalk_mini::ModelField::RouteApbDestinationId, source, 1950000);
    std::strcpy(model.route.rmb.destination_id, "NEW");
    model.route.rmb.last_update_us = 2000000;
    store.mark_changed(signalk_mini::ModelField::RouteRmbDestinationId, source, 2000000);
    signalk_mini::SignalKTypedModelView<float> duplicate_view(store, config, 2050000);
    size_t next_point_count = 0;
    const char* selected = nullptr;
    duplicate_view.for_each_current_value([&](signalk_mini::ModelField,
                                               const signalk_mini::SignalKMappedValue<float>& value) {
        if (std::strcmp(value.path, signalk_mini::signalk_path::NavigationCourseGreatCircleNextPointId) == 0) {
            ++next_point_count;
            selected = value.text;
        }
    });
    REQUIRE(next_point_count == 1);
    REQUIRE(selected && std::strcmp(selected, "NEW") == 0);

    model.gnss.sky_view.observation_count = 1;
    model.gnss.sky_view.observations[0].satellite_id = 22;
    model.gnss.sky_view.observations[0].satellite_id_valid = true;
    model.gnss.sky_view.last_update_us = 3000000;
    store.mark_changed(signalk_mini::ModelField::GnssSatellitePrn0, source, 3000000);
    signalk_mini::SignalKTypedModelView<float> fresh_satellite(store, config, 3050000);
    REQUIRE(fresh_satellite.current_value(signalk_mini::ModelField::GnssSatellitePrn0, mapped));
    REQUIRE(mapped.last_update_us == 3000000);
    signalk_mini::SignalKTypedModelView<float> expired_satellite(store, config, 3100001);
    REQUIRE(!expired_satellite.current_value(signalk_mini::ModelField::GnssSatellitePrn0, mapped));

    model.ais.data_link_status.last_update_us = 4000000;
    store.mark_changed(signalk_mini::ModelField::AisDataLinkStatusObject, source, 4000000);
    signalk_mini::SignalKTypedModelView<float> fresh_object(store, config, 4050000);
    REQUIRE(fresh_object.current_value(signalk_mini::ModelField::AisDataLinkStatusObject, mapped));
    REQUIRE(mapped.last_update_us == 4000000);
    signalk_mini::SignalKTypedModelView<float> expired_object(store, config, 4100001);
    REQUIRE(!expired_object.current_value(signalk_mini::ModelField::AisDataLinkStatusObject, mapped));

    return 0;
}
