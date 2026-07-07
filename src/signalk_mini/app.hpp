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

private:
    MiniSignalKServer<Real> server_;
};

} // namespace signalk_mini
