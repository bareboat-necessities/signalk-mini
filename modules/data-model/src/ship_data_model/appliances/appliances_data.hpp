#pragma once

namespace ship_data_model {

template<typename Real = float>
struct FridgesData {
};

template<typename Real = float>
struct AcHeatingData {
};

template<typename Real = float>
struct WaterHeatersData {
};

template<typename Real = float>
struct WatermakerData {
};

template<typename Real = float>
struct StovesData {
};

template<typename Real = float>
struct LiveWellData {
};

template<typename Real = float>
struct AppliancesData {
    FridgesData<Real> fridges;
    AcHeatingData<Real> ac_heating;
    WaterHeatersData<Real> water_heaters;
    WatermakerData<Real> watermaker;
    StovesData<Real> stoves;
    LiveWellData<Real> live_well;
};

} // namespace ship_data_model
