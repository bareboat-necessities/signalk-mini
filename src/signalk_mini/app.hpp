#pragma once

#include "server.hpp"

namespace signalk_mini {

template<typename Real = float>
class SignalKMiniApp {
public:
    explicit SignalKMiniApp(const SignalKMiniConfig& config = SignalKMiniConfig{})
        : server_(config) {}

    bool begin() { return server_.begin(); }
    void tick() { server_.tick(); }
    void run_forever() { server_.run_forever(); }
    ModelStore<Real>& store() { return server_.store(); }
    const ModelStore<Real>& store() const { return server_.store(); }
    Nmea0183Input<Real>& nmea0183() { return server_.nmea0183(); }
    SeaTalkInput<Real>& seatalk() { return server_.seatalk(); }
    UbxInput<Real>& ubx() { return server_.ubx(); }

    uint64_t dropped_change_count() const { return server_.dropped_change_count(); }
    uint64_t dropped_publish_count() const { return server_.dropped_publish_count(); }
    uint64_t published_delta_count() const { return server_.published_delta_count(); }
    typename MiniSignalKServer<Real>::StartupError last_startup_error() const { return server_.last_startup_error(); }
    size_t last_failed_connector_index() const { return server_.last_failed_connector_index(); }
    uint32_t connector_start_failure_count() const { return server_.connector_start_failure_count(); }
    uint32_t connector_reconnect_count() const { return server_.connector_reconnect_count(); }
    uint32_t ubx_frame_count() const { return server_.ubx_frame_count(); }
    uint32_t ubx_checksum_error_count() const { return server_.ubx_checksum_error_count(); }
    uint32_t ubx_oversized_frame_count() const { return server_.ubx_oversized_frame_count(); }
    uint32_t ubx_unsupported_frame_count() const { return server_.ubx_unsupported_frame_count(); }

    bool transmit_seatalk_frame(const seatalk::SeaTalkFrame& frame) {
        return server_.transmit_seatalk_frame(frame);
    }

    bool transmit_seatalk_pilot_key(seatalk::SeaTalkPilotKey key, bool long_press = false) {
        return server_.transmit_seatalk_pilot_key(key, long_press);
    }

    bool transmit_seatalk_depth_m(Real depth_m, bool alarm = false) {
        return server_.transmit_seatalk_depth_m(depth_m, alarm);
    }

    bool transmit_seatalk_apparent_wind_angle_deg(Real angle_deg) {
        return server_.transmit_seatalk_apparent_wind_angle_deg(angle_deg);
    }

    bool transmit_seatalk_apparent_wind_speed_kn(Real speed_kn) {
        return server_.transmit_seatalk_apparent_wind_speed_kn(speed_kn);
    }

    bool transmit_seatalk_speed_through_water_kn(Real speed_kn) {
        return server_.transmit_seatalk_speed_through_water_kn(speed_kn);
    }

private:
    MiniSignalKServer<Real> server_;
};

} // namespace signalk_mini
