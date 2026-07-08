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

    bool transmit_seatalk_frame(const seatalk::SeaTalkFrame& frame) {
        return server_.transmit_seatalk_frame(frame);
    }

    bool transmit_seatalk_pilot_key(seatalk::SeaTalkPilotKey key, bool long_press = false) {
        return server_.transmit_seatalk_pilot_key(key, long_press);
    }

    bool transmit_seatalk_depth_m(float depth_m, bool alarm = false) {
        return server_.transmit_seatalk_depth_m(depth_m, alarm);
    }

    bool transmit_seatalk_apparent_wind_angle_deg(float angle_deg) {
        return server_.transmit_seatalk_apparent_wind_angle_deg(angle_deg);
    }

    bool transmit_seatalk_apparent_wind_speed_kn(float speed_kn) {
        return server_.transmit_seatalk_apparent_wind_speed_kn(speed_kn);
    }

    bool transmit_seatalk_speed_through_water_kn(float speed_kn) {
        return server_.transmit_seatalk_speed_through_water_kn(speed_kn);
    }

private:
    MiniSignalKServer<Real> server_;
};

} // namespace signalk_mini
