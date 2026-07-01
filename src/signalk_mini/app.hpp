#pragma once

#include "model_store.hpp"
#include "nmea0183_input.hpp"

namespace signalk_mini {

template<typename Real = float>
class SignalKMiniApp {
public:
    using Store = ModelStore<Real>;
    Store& store() { return store_; }
    const Store& store() const { return store_; }
    Nmea0183Input<Real>& nmea0183() { return nmea0183_; }

private:
    Store store_{};
    Nmea0183Input<Real> nmea0183_{store_};
};

} // namespace signalk_mini
