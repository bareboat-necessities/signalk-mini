#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <signalk_mini/ubx_input.hpp>
#include "ubx_test_helpers.hpp"

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got=%f expected=%f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

int main() {
    signalk_mini::ModelStore<float> store;
    signalk_mini::UbxInput<float> input(store);

    const auto pvt = ubx_test::nav_pvt(true);
    REQUIRE(!input.feed_octets(pvt.data(), 9, 10, 1000));
    REQUIRE(input.feed_octets(pvt.data() + 9, pvt.size() - 9, 10, 1001));
    const auto& fix = store.model().gnss.fix;
    REQUIRE(fix.fix_valid.valid && fix.fix_valid.value);
    REQUIRE(fix.fix_type.value == 3);
    REQUIRE(fix.satellites_used.value == 12);
    NEAR(fix.timestamp_s.value, 45296.125f, 0.01f);
    REQUIRE(fix.date_year.value == 2026 && fix.date_month.value == 7 && fix.date_day.value == 17);
    NEAR(fix.fix_lat_deg.value, 40.7654321f, 0.00001f);
    NEAR(fix.fix_lon_deg.value, -74.1234567f, 0.00001f);
    NEAR(fix.fix_alt_hae_m.value, 15.34f, 0.001f);
    NEAR(fix.fix_alt_msl_m.value, -18.2f, 0.001f);
    NEAR(fix.geoidal_separation_m.value, 33.54f, 0.001f);
    NEAR(fix.vertical_speed_m_s.value, 0.3f, 0.001f);
    NEAR(fix.track_deg.value, 12.34567f, 0.0001f);
    NEAR(fix.declination_deg.value, -13.5f, 0.001f);
    NEAR(store.model().gnss.fix_accuracy.horizontal_accuracy_m.value, 0.8f, 0.001f);
    NEAR(store.model().gnss.fix_accuracy.track_accuracy_deg.value, 0.25f, 0.001f);
    REQUIRE(fix.source.value == ship_data_model::SensorSource::ubx);
    const float previous_lat = fix.fix_lat_deg.value;
    const uint64_t previous_lat_update_us = fix.fix_lat_deg.last_update_us;

    const auto no_fix = ubx_test::nav_pvt(false);
    REQUIRE(input.feed_octets(no_fix.data(), no_fix.size(), 10, 2000));
    REQUIRE(!fix.fix_valid.value);
    NEAR(fix.fix_lat_deg.value, previous_lat, 0.00001f);
    REQUIRE(fix.fix_lat_deg.last_update_us == previous_lat_update_us);

    const auto dop = ubx_test::nav_dop();
    REQUIRE(input.feed_octets(dop.data(), dop.size(), 10, 3000));
    NEAR(store.model().gnss.dop_active_satellites.gdop.value, 2.5f, 0.001f);
    NEAR(store.model().gnss.dop_active_satellites.hdop.value, 0.95f, 0.001f);
    NEAR(store.model().gnss.dop_active_satellites.east_dop.value, 0.65f, 0.001f);
    NEAR(store.model().gnss.fix_accuracy.gdop.value, 2.5f, 0.001f);
    NEAR(store.model().gnss.fix_accuracy.tdop.value, 1.2f, 0.001f);
    NEAR(store.model().gnss.fix_accuracy.north_dop.value, 0.7f, 0.001f);
    NEAR(store.model().gnss.fix_accuracy.east_dop.value, 0.65f, 0.001f);
    NEAR(store.model().gnss.fix.hdop.value, 0.95f, 0.001f);

    const auto sat = ubx_test::nav_sat();
    REQUIRE(input.feed_datagram(sat.data(), sat.size(), 10, 4000));
    const auto& sky = store.model().gnss.sky_view;
    REQUIRE(sky.complete);
    REQUIRE(sky.observation_count == 3);
    REQUIRE(sky.satellites_in_view.value == 3);
    REQUIRE(sky.satellites_used.value == 2);
    REQUIRE(sky.observations[0].used);
    REQUIRE(sky.observations[0].healthy);
    REQUIRE(sky.observations[0].differential_corrections);
    REQUIRE(!sky.observations[1].used);
    REQUIRE(sky.observations[2].system_id == 2);

    const auto large_sat = ubx_test::nav_sat(70);
    REQUIRE(input.feed_datagram(large_sat.data(), large_sat.size(), 10, 5000));
    REQUIRE(sky.observation_count == sky.capacity());
    REQUIRE(sky.satellites_in_view.value == 70);
    REQUIRE(sky.satellites_used.value == 69);

    REQUIRE(store.marked_change_count() > 20);
    return 0;
}
