#pragma once

#if defined(SIGNALK_MINI_ENABLE_NATIVE_TCP) || defined(ARDUINO)
#include "server.hpp"
#else
#include "model_store.hpp"
#include "nmea0183_input.hpp"
#endif

namespace signalk_mini {

template<typename Real = float>
class SignalKMiniApp {
public:
#if defined(SIGNALK_MINI_ENABLE_NATIVE_TCP) || defined(ARDUINO)
    explicit SignalKMiniApp(const SignalKMiniConfig& config = SignalKMiniConfig{})
        : server_(config) {}

    bool begin() { return server_.begin(); }
    void tick() { server_.tick(); }
    void run_forever() { server_.run_forever(); }
    ModelStore<Real>& store() { return server_.store(); }
    const ModelStore<Real>& store() const { return server_.store(); }
    Nmea0183Input<Real>& nmea0183() { return server_.nmea0183(); }

private:
    MiniSignalKServer<Real> server_;
#else
    using Store = ModelStore<Real>;
    bool begin() { return true; }
    void tick() {}
    void run_forever() {}
    Store& store() { return store_; }
    const Store& store() const { return store_; }
    Nmea0183Input<Real>& nmea0183() { return nmea0183_; }

private:
    Store store_{};
    Nmea0183Input<Real> nmea0183_{store_};
#endif
};

} // namespace signalk_mini
