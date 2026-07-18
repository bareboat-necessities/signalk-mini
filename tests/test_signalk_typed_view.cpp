#include <ArduinoJson.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <signalk_mini/config.hpp>
#include <signalk_mini/model_store.hpp>
#include <signalk_mini/publisher.hpp>
#include <signalk_mini/signalk_full_model_writer.hpp>
#include <signalk_mini/signalk_identity.hpp>
#include <signalk_mini/signalk_mapper.hpp>
#include <signalk_mini/signalk_typed_view.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a)-(b))) > (e)) { std::fprintf(stderr, "NEAR FAILED %s:%d got=%f expected=%f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

namespace {
constexpr const char* SelfContext = "vessels.urn:mrn:signalk:uuid:11111111-2222-4333-8444-555555555555";
constexpr const char* SelfKey = "urn:mrn:signalk:uuid:11111111-2222-4333-8444-555555555555";

void mark(signalk_mini::ModelStore<float>& store,
          signalk_mini::ModelField field,
          signalk_mini::SourceId source,
          uint64_t now_us) {
    store.mark_changed(field, source, now_us);
}

struct CollectSink {
    std::vector<std::string> deltas;
    void write_signal_k_delta(const char* json, size_t len) {
        deltas.emplace_back(json, len);
    }
};

bool find_delta_value(const CollectSink& sink,
                      const char* expected_context,
                      const char* expected_path,
                      JsonDocument& result) {
    for (const std::string& text : sink.deltas) {
        JsonDocument doc;
        if (deserializeJson(doc, text)) continue;
        if (std::strcmp(doc["context"] | "", expected_context) != 0) continue;
        JsonArray updates = doc["updates"].as<JsonArray>();
        for (JsonObject update : updates) {
            JsonArray values = update["values"].as<JsonArray>();
            for (JsonObject value : values) {
                if (std::strcmp(value["path"] | "", expected_path) == 0) {
                    result = doc;
                    return true;
                }
            }
        }
    }
    return false;
}

JsonVariantConst delta_value(JsonDocument& doc, const char* path) {
    JsonArray updates = doc["updates"].as<JsonArray>();
    for (JsonObject update : updates) {
        JsonArray values = update["values"].as<JsonArray>();
        for (JsonObject value : values) {
            if (std::strcmp(value["path"] | "", path) == 0) return value["value"];
        }
    }
    return JsonVariantConst{};
}
}

int main() {
    REQUIRE(signalk_mini::signalk_valid_self_context(SelfContext));
    REQUIRE(signalk_mini::signalk_valid_self_context("vessels.urn:mrn:signalk:uuid:11111111-2222-5333-8444-555555555555"));
    REQUIRE(!signalk_mini::signalk_valid_self_context("vessels.self"));
    REQUIRE(!signalk_mini::signalk_valid_self_context("vessels.urn:mrn:signalk:uuid:not-a-uuid"));
    REQUIRE(!signalk_mini::signalk_valid_self_context("vessels.http://"));
    REQUIRE(!signalk_mini::signalk_valid_self_context("vessels.https://"));
    REQUIRE(!signalk_mini::signalk_valid_self_context("vessels.mailto:"));
    REQUIRE(!signalk_mini::signalk_valid_self_context("vessels.tel:"));
    REQUIRE(signalk_mini::signalk_valid_self_context("vessels.https://example.test/vessel"));
    char generated_context[80];
    REQUIRE(signalk_mini::signalk_make_uuid_context(generated_context, sizeof(generated_context), 0x1234ULL, 0x5678ULL));
    REQUIRE(signalk_mini::signalk_valid_self_context(generated_context));
    REQUIRE(std::strstr(generated_context, "-4") != nullptr);

    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "gps-primary";
    connector.protocol.kind = signalk_mini::ConnectorProtocol::Ubx;

    signalk_mini::SignalKMiniConfig config;
    config.identity.server_name = "schema-view-test";
    config.identity.server_version = "0.1-test";
    config.identity.signalk_version = "1.8.2";
    config.identity.self = SelfContext;
    config.publisher.source_label = "internal";
    config.publisher.json_buffer_size = 4096;
    config.connectors = &connector;
    config.connector_count = 1;

    signalk_mini::ModelStore<float> store;
    const signalk_mini::SourceId source = signalk_mini::FirstConnectorSourceId;
    auto& model = store.model();

    model.gnss.fix.fix_lat_deg.set(40.765432f, 1000000);
    model.gnss.fix.fix_lon_deg.set(-74.123456f, 1000000);
    model.gnss.fix.fix_alt_hae_m.set(31.25f, 1000000);
    mark(store, signalk_mini::ModelField::GnssFixLatDeg, source, 1000000);
    mark(store, signalk_mini::ModelField::GnssFixLonDeg, source, 1000000);
    mark(store, signalk_mini::ModelField::GnssFixAltitudeHaeM, source, 1000000);

    model.gnss.fix.date_year.set(2026, 1100000);
    model.gnss.fix.date_month.set(7, 1100000);
    model.gnss.fix.date_day.set(18, 1100000);
    model.gnss.fix.timestamp_s.set(6 * 3600.0f + 7 * 60.0f + 8.5f, 1100000);
    mark(store, signalk_mini::ModelField::GnssDateYear, source, 1100000);
    mark(store, signalk_mini::ModelField::GnssDateMonth, source, 1100000);
    mark(store, signalk_mini::ModelField::GnssDateDay, source, 1100000);
    mark(store, signalk_mini::ModelField::GnssTimestampS, source, 1100000);

    model.ins.imu.pitch_deg.set(2.0f, 1200000);
    model.ins.imu.roll_deg.set(-3.0f, 1200000);
    mark(store, signalk_mini::ModelField::ImuPitchDeg, source, 1200000);
    mark(store, signalk_mini::ModelField::ImuRollDeg, source, 1200000);

    model.gnss.fix.speed_kn.set(4.0f, 1300000);
    mark(store, signalk_mini::ModelField::GnssSpeedKn, source, 1300000);

    model.gnss.fix.fix_quality.set(4, 1310000);
    mark(store, signalk_mini::ModelField::GnssFixQuality, source, 1310000);

    model.sea.current_speed_kn.set(2.0f, 1320000);
    model.sea.current_direction_deg.set(135.0f, 1320000);
    model.sea.current_direction_magnetic_deg.set(130.0f, 1320000);
    mark(store, signalk_mini::ModelField::SeaCurrentSpeedKn, source, 1320000);
    mark(store, signalk_mini::ModelField::SeaCurrentDirectionDeg, source, 1320000);
    mark(store, signalk_mini::ModelField::SeaCurrentDirectionMagneticDeg, source, 1320000);

    model.autopilot.controller.enabled.value = true;
    model.autopilot.controller.mode.value = ship_data_model::AutopilotMode::wind;
    mark(store, signalk_mini::ModelField::AutopilotEnabled, source, 1330000);
    mark(store, signalk_mini::ModelField::AutopilotMode, source, 1330000);

    auto& target = model.ais.targets.targets[0];
    target.occupied = true;
    target.mmsi.set(234567890, 1400000);
    target.latitude_deg.set(41.0f, 1400000);
    target.longitude_deg.set(-73.0f, 1400000);
    target.speed_over_ground_kn.set(6.0f, 1400000);
    target.last_seen_us = 1400000;
    target.last_position_update_us = 1400000;
    target.last_static_update_us = 1400000;
    std::strcpy(target.vessel_name, "TEST TARGET");
    model.ais.targets.target_count.set(1, 1400000);
    mark(store, signalk_mini::ModelField::AisTargetsObject, source, 1400000);

    auto& non_vessel_target = model.ais.targets.targets[1];
    non_vessel_target.occupied = true;
    non_vessel_target.mmsi.set(111000001, 1400000);
    non_vessel_target.latitude_deg.set(39.0f, 1400000);
    auto& incomplete_vessel = model.ais.targets.targets[2];
    incomplete_vessel.occupied = true;
    incomplete_vessel.mmsi.set(235000001, 1400000);
    incomplete_vessel.latitude_deg.set(38.0f, 1400000);

    signalk_mini::SignalKMapper<float> mapper;
    signalk_mini::SignalKMappedValue<float> mapped;
    REQUIRE(mapper.map_field(model, signalk_mini::ModelField::GnssFixLatDeg, mapped));
    REQUIRE(std::strcmp(mapped.path, "navigation.position") == 0);
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Object);
    REQUIRE(mapped.object_kind == signalk_mini::SignalKObjectKind::Position);

    REQUIRE(mapper.map_field(model, signalk_mini::ModelField::ImuPitchDeg, mapped));
    REQUIRE(std::strcmp(mapped.path, "navigation.attitude") == 0);
    REQUIRE(mapped.object_kind == signalk_mini::SignalKObjectKind::Attitude);

    REQUIRE(mapper.map_field(model, signalk_mini::ModelField::GnssDateYear, mapped));
    REQUIRE(std::strcmp(mapped.path, "navigation.datetime") == 0);
    REQUIRE(mapped.object_kind == signalk_mini::SignalKObjectKind::DateTime);

    signalk_mini::SignalKTypedModelView<float> view(store, config, 1500000);
    REQUIRE(sizeof(view) <= 64);
    REQUIRE(view.current_value(signalk_mini::ModelField::GnssFixLatDeg, mapped));
    REQUIRE(mapped.source_id == source);
    REQUIRE(std::strcmp(view.source_label(mapped.source_id), "gps-primary") == 0);

    char full[32768];
    signalk_mini::SignalKFullModelWriter<float> full_writer;
    const int full_len = full_writer.write(full, sizeof(full), store, config, 1500000);
    REQUIRE(full_len > 0);
    REQUIRE(static_cast<size_t>(full_len) < sizeof(full));

    JsonDocument full_doc;
    REQUIRE(!deserializeJson(full_doc, full, static_cast<size_t>(full_len)));
    REQUIRE(std::strcmp(full_doc["version"] | "", "1.8.2") == 0);
    REQUIRE(std::strcmp(full_doc["self"] | "", SelfKey) == 0);

    JsonObject self = full_doc["vessels"][SelfKey].as<JsonObject>();
    REQUIRE(!self.isNull());
    REQUIRE(std::strcmp(self["uuid"] | "", SelfKey) == 0);
    JsonObject position = self["navigation"]["position"]["value"].as<JsonObject>();
    REQUIRE(!position.isNull());
    NEAR(position["latitude"].as<float>(), 40.765432f, 0.00001f);
    NEAR(position["longitude"].as<float>(), -74.123456f, 0.00001f);
    NEAR(position["altitude"].as<float>(), 31.25f, 0.001f);
    REQUIRE(self["navigation"]["position"]["value"]["value"].isNull());
    REQUIRE(std::strcmp(self["navigation"]["datetime"]["value"] | "", "2026-07-18T06:07:08.500Z") == 0);
    NEAR(self["navigation"]["attitude"]["value"]["pitch"].as<float>(), 2.0f * 3.14159265358979323846f / 180.0f, 0.00001f);
    NEAR(self["navigation"]["attitude"]["value"]["roll"].as<float>(), -3.0f * 3.14159265358979323846f / 180.0f, 0.00001f);
    REQUIRE(std::strcmp(self["navigation"]["position"]["$source"] | "", "connector0") == 0);
    REQUIRE(std::strcmp(self["navigation"]["gnss"]["methodQuality"]["value"] | "", "RTK fixed integer") == 0);
    NEAR(self["environment"]["current"]["value"]["drift"].as<float>(), 2.0f * 0.5144444444f, 0.00001f);
    NEAR(self["environment"]["current"]["value"]["setTrue"].as<float>(), 135.0f * 3.14159265358979323846f / 180.0f, 0.00001f);
    REQUIRE(std::strcmp(self["steering"]["autopilot"]["state"]["value"] | "", "wind") == 0);
    REQUIRE(std::strcmp(full_doc["sources"]["connector0"]["label"] | "", "gps-primary") == 0);

    signalk_mini::ModelStore<float> duplicate_store;
    duplicate_store.model().env.barometric_pressure_bar.set(1.0f, 1000000);
    mark(duplicate_store, signalk_mini::ModelField::EnvBarometricPressureBar, source, 1000000);
    duplicate_store.model().env.barometric_pressure_inhg.set(29.92f, 2000000);
    mark(duplicate_store, signalk_mini::ModelField::EnvBarometricPressureInHg, source, 2000000);
    signalk_mini::SignalKTypedModelView<float> duplicate_view(duplicate_store, config, 2000000);
    size_t pressure_count = 0;
    float selected_pressure = 0.0f;
    duplicate_view.for_each_current_value([&](signalk_mini::ModelField, const signalk_mini::SignalKMappedValue<float>& value) {
        if (std::strcmp(value.path, "environment.outside.pressure") == 0) {
            ++pressure_count;
            selected_pressure = value.number;
        }
    });
    REQUIRE(pressure_count == 1);
    NEAR(selected_pressure, 29.92f * 3386.389f, 0.1f);
    char duplicate_full[4096];
    REQUIRE(full_writer.write(duplicate_full, sizeof(duplicate_full), duplicate_store, config, 2000000) > 0);
    JsonDocument duplicate_doc;
    REQUIRE(!deserializeJson(duplicate_doc, duplicate_full));
    NEAR(duplicate_doc["vessels"][SelfKey]["environment"]["outside"]["pressure"]["value"].as<float>(),
         29.92f * 3386.389f, 0.1f);

    signalk_mini::ModelStore<float> datetime_source_store;
    auto& datetime_fix = datetime_source_store.model().gnss.fix;
    datetime_fix.timestamp_s.set(3600.0f, 1000000);
    datetime_fix.date_year.set(2026, 1000000);
    datetime_fix.date_month.set(7, 3000000);
    datetime_fix.date_day.set(18, 2000000);
    mark(datetime_source_store, signalk_mini::ModelField::GnssTimestampS, source, 1000000);
    mark(datetime_source_store, signalk_mini::ModelField::GnssDateYear, source, 1000000);
    const signalk_mini::SourceId newer_source = static_cast<signalk_mini::SourceId>(source + 1);
    mark(datetime_source_store, signalk_mini::ModelField::GnssDateDay, newer_source, 2000000);
    mark(datetime_source_store, signalk_mini::ModelField::GnssDateMonth, newer_source, 3000000);
    signalk_mini::SignalKTypedModelView<float> datetime_source_view(datetime_source_store, config, 3000000);
    REQUIRE(datetime_source_view.current_value(signalk_mini::ModelField::GnssDateYear, mapped));
    REQUIRE(mapped.source_id == newer_source);

    char datetime_buffer[32];
    REQUIRE(!signalk_mini::format_signalk_datetime_utc(datetime_buffer, sizeof(datetime_buffer), 2025, 2, 29, 0.0));
    REQUIRE(!signalk_mini::format_signalk_datetime_utc(datetime_buffer, sizeof(datetime_buffer), 2026, 4, 31, 0.0));
    REQUIRE(!signalk_mini::format_signalk_datetime_utc(datetime_buffer, sizeof(datetime_buffer), 2026, 7, 18, std::numeric_limits<double>::quiet_NaN()));
    REQUIRE(signalk_mini::format_signalk_datetime_utc(datetime_buffer, sizeof(datetime_buffer), 2024, 2, 29, 0.0));

    char too_small[32];
    REQUIRE(full_writer.write(too_small, sizeof(too_small), store, config, 1500000) == 0);

    signalk_mini::ModelStore<float> empty_store;
    char empty_full[1024];
    const int empty_len = full_writer.write(empty_full, sizeof(empty_full), empty_store, config, 1500000);
    REQUIRE(empty_len > 0);
    JsonDocument empty_doc;
    REQUIRE(!deserializeJson(empty_doc, empty_full, static_cast<size_t>(empty_len)));
    REQUIRE(std::strcmp(empty_doc["version"] | "", "1.8.2") == 0);
    REQUIRE(std::strcmp(empty_doc["self"] | "", SelfKey) == 0);
    REQUIRE(!empty_doc["vessels"][SelfKey].isNull());
    REQUIRE(empty_doc["vessels"][SelfKey]["navigation"].isNull());

    JsonObject ais = full_doc["vessels"]["urn:mrn:imo:mmsi:234567890"].as<JsonObject>();
    REQUIRE(!ais.isNull());
    REQUIRE(std::strcmp(ais["name"] | "", "TEST TARGET") == 0);
    REQUIRE(std::strcmp(ais["mmsi"] | "", "234567890") == 0);
    NEAR(ais["navigation"]["position"]["value"]["latitude"].as<float>(), 41.0f, 0.0001f);

    const size_t queued_before_snapshot = store.pending_change_count();
    signalk_mini::SignalKPublisher<float, 4096> publisher(store, config);
    CollectSink first_snapshot;
    publisher.publish_current(first_snapshot, 1500000, true);
    REQUIRE(store.pending_change_count() == queued_before_snapshot);
    REQUIRE(publisher.published_snapshot_count() == 1);

    JsonDocument position_delta;
    REQUIRE(find_delta_value(first_snapshot, SelfContext, "navigation.position", position_delta));
    JsonVariantConst position_value = delta_value(position_delta, "navigation.position");
    NEAR(position_value["latitude"].as<float>(), 40.765432f, 0.00001f);

    JsonDocument ais_delta;
    REQUIRE(find_delta_value(first_snapshot, "vessels.urn:mrn:imo:mmsi:234567890", "navigation.position", ais_delta));
    REQUIRE(!find_delta_value(first_snapshot, SelfContext, "navigation.ais.targets", ais_delta));
    JsonDocument ais_static_delta;
    REQUIRE(find_delta_value(first_snapshot, "vessels.urn:mrn:imo:mmsi:234567890", "", ais_static_delta));
    REQUIRE(std::strcmp(delta_value(ais_static_delta, "")["mmsi"] | "", "234567890") == 0);
    REQUIRE(!find_delta_value(first_snapshot, "vessels.urn:mrn:imo:mmsi:111000001", "navigation.position", ais_delta));
    REQUIRE(!find_delta_value(first_snapshot, "vessels.urn:mrn:imo:mmsi:235000001", "navigation.position", ais_delta));

    JsonDocument quality_delta;
    REQUIRE(find_delta_value(first_snapshot, SelfContext, "navigation.gnss.methodQuality", quality_delta));
    REQUIRE(std::strcmp(delta_value(quality_delta, "navigation.gnss.methodQuality") | "", "RTK fixed integer") == 0);

    JsonDocument current_delta;
    REQUIRE(find_delta_value(first_snapshot, SelfContext, "environment.current", current_delta));
    NEAR(delta_value(current_delta, "environment.current")["drift"].as<float>(), 2.0f * 0.5144444444f, 0.00001f);

    signalk_mini::ModelStore<float> incomplete_store;
    incomplete_store.model().gnss.fix.fix_lat_deg.set(10.0f, 1000);
    mark(incomplete_store, signalk_mini::ModelField::GnssFixLatDeg, source, 1000);
    signalk_mini::SignalKTypedModelView<float> incomplete_view(incomplete_store, config, 1000);
    REQUIRE(!incomplete_view.current_value(signalk_mini::ModelField::GnssFixLatDeg, mapped));
    char incomplete_full[2048];
    REQUIRE(full_writer.write(incomplete_full, sizeof(incomplete_full), incomplete_store, config, 1000) > 0);
    JsonDocument incomplete_doc;
    REQUIRE(!deserializeJson(incomplete_doc, incomplete_full));
    REQUIRE(incomplete_doc["vessels"][SelfKey]["navigation"]["position"].isNull());

    model.gnss.fix.fix_lat_deg.set(42.25f, 2000000);
    mark(store, signalk_mini::ModelField::GnssFixLatDeg, source, 2000000);
    CollectSink second_snapshot;
    publisher.publish_current(second_snapshot, 2000000, true);
    JsonDocument changed_position_delta;
    REQUIRE(find_delta_value(second_snapshot, SelfContext, "navigation.position", changed_position_delta));
    NEAR(delta_value(changed_position_delta, "navigation.position")["latitude"].as<float>(), 42.25f, 0.00001f);

    signalk_mini::SignalKMiniConfig expiring_config = config;
    expiring_config.publisher.current_value_timeout_ms = 100;
    signalk_mini::SignalKTypedModelView<float> expired_view(store, expiring_config, 2050000);
    REQUIRE(!expired_view.current_value(signalk_mini::ModelField::GnssSpeedKn, mapped));
    REQUIRE(expired_view.current_value(signalk_mini::ModelField::GnssFixLatDeg, mapped));
    REQUIRE(!expired_view.current_value(signalk_mini::ModelField::AutopilotMode, mapped));
    REQUIRE(!expired_view.ais_target_is_current(target));
    CollectSink expired_snapshot;
    signalk_mini::SignalKPublisher<float, 4096> expiring_publisher(store, expiring_config);
    expiring_publisher.publish_current(expired_snapshot, 2050000, true);
    REQUIRE(!find_delta_value(expired_snapshot, "vessels.urn:mrn:imo:mmsi:234567890", "navigation.position", ais_delta));
    char expired_full[32768];
    REQUIRE(full_writer.write(expired_full, sizeof(expired_full), store, expiring_config, 2050000) > 0);
    JsonDocument expired_doc;
    REQUIRE(!deserializeJson(expired_doc, expired_full));
    REQUIRE(expired_doc["vessels"]["urn:mrn:imo:mmsi:234567890"].isNull());
    REQUIRE(signalk_mini::ModelStore<float>::PresenceBytes < signalk_mini::ModelStore<float>::FieldCount);

    return 0;
}
