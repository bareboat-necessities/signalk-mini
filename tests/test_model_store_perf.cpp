#include <cstdio>
#include <cstdlib>

#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

int main() {
    REQUIRE(sizeof(signalk_mini::ModelChange) <= 12);
    REQUIRE(signalk_mini::DefaultModelChangeQueueCapacity > 0);
    REQUIRE(signalk_mini::DefaultModelChangeQueueCapacity <= signalk_mini::MaxSupportedModelChangeQueueCapacity);
    REQUIRE(signalk_mini::DefaultSignalKJsonBufferSize > 0);
    REQUIRE(signalk_mini::DefaultSignalKBatchValues > 0);

    signalk_mini::ModelStore<float, 4> store;

    REQUIRE(store.marked_change_count() == 0);
    REQUIRE(store.enqueued_change_count() == 0);
    REQUIRE(store.coalesced_change_count() == 0);
    REQUIRE(store.dropped_change_count() == 0);
    REQUIRE(store.pending_change_count() == 0);
    REQUIRE(store.change_queue_capacity() == 4);
    REQUIRE(store.change_queue_high_watermark() == 0);

    store.mark_changed(signalk_mini::ModelField::WindApparentSpeedKn, 10, 1000);
    store.mark_changed(signalk_mini::ModelField::WindApparentSpeedKn, 10, 2000);
    store.mark_changed(signalk_mini::ModelField::WindApparentSpeedKn, 10, 3000);

    REQUIRE(store.marked_change_count() == 3);
    REQUIRE(store.enqueued_change_count() == 1);
    REQUIRE(store.coalesced_change_count() == 2);
    REQUIRE(store.dropped_change_count() == 0);
    REQUIRE(store.pending_change_count() == 1);
    REQUIRE(store.change_queue_high_watermark() == 1);
    REQUIRE(store.sequence() == 3);

    signalk_mini::ModelChange change;
    REQUIRE(store.changes().pop(change));
    REQUIRE(change.field == signalk_mini::ModelField::WindApparentSpeedKn);
    REQUIRE(change.source_id == 10);
    REQUIRE(change.timestamp_ms == 3);
    REQUIRE(change.sequence == 3);
    REQUIRE(store.pending_change_count() == 0);

    store.mark_changed(signalk_mini::ModelField::WindApparentSpeedKn, 10, 4000);
    store.mark_changed(signalk_mini::ModelField::WindApparentSpeedKn, 11, 5000);
    store.mark_changed(signalk_mini::ModelField::WindApparentDirectionDeg, 10, 6000);

    REQUIRE(store.marked_change_count() == 6);
    REQUIRE(store.enqueued_change_count() == 4);
    REQUIRE(store.coalesced_change_count() == 2);
    REQUIRE(store.pending_change_count() == 3);
    REQUIRE(store.change_queue_high_watermark() == 3);

    REQUIRE(store.changes().pop(change));
    REQUIRE(change.field == signalk_mini::ModelField::WindApparentSpeedKn);
    REQUIRE(change.source_id == 10);
    REQUIRE(change.timestamp_ms == 4);

    REQUIRE(store.changes().pop(change));
    REQUIRE(change.field == signalk_mini::ModelField::WindApparentSpeedKn);
    REQUIRE(change.source_id == 11);
    REQUIRE(change.timestamp_ms == 5);

    REQUIRE(store.changes().pop(change));
    REQUIRE(change.field == signalk_mini::ModelField::WindApparentDirectionDeg);
    REQUIRE(change.source_id == 10);
    REQUIRE(change.timestamp_ms == 6);

    REQUIRE(!store.changes().pop(change));

    signalk_mini::ModelStore<float, 2> tiny;
    tiny.mark_changed(signalk_mini::ModelField::GnssSpeedKn, 1, 1000);
    tiny.mark_changed(signalk_mini::ModelField::GnssTrackDeg, 1, 2000);
    tiny.mark_changed(signalk_mini::ModelField::GnssFixLatDeg, 1, 3000);

    REQUIRE(tiny.marked_change_count() == 3);
    REQUIRE(tiny.enqueued_change_count() == 3);
    REQUIRE(tiny.coalesced_change_count() == 0);
    REQUIRE(tiny.dropped_change_count() == 1);
    REQUIRE(tiny.pending_change_count() == 2);
    REQUIRE(tiny.change_queue_high_watermark() == 2);

    REQUIRE(tiny.changes().pop(change));
    REQUIRE(change.field == signalk_mini::ModelField::GnssTrackDeg);
    REQUIRE(tiny.changes().pop(change));
    REQUIRE(change.field == signalk_mini::ModelField::GnssFixLatDeg);
    REQUIRE(!tiny.changes().pop(change));

    return 0;
}
