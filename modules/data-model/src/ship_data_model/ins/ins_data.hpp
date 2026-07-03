#pragma once

#include "ship_imu_data.hpp"

namespace ship_data_model {

template<typename Real = float>
struct InsData {
    ShipImuData<Real> imu;
    ImuStateData<Real> imu_state;
    ImuCalibrationData<Real> imu_calibration;
};

} // namespace ship_data_model
