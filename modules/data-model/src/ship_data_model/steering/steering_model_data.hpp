#pragma once

#include "steering_data.hpp"

namespace ship_data_model {

template<typename Real = float>
struct SteeringModelData {
    RudderData<Real> rudder;
    ServoData<Real> servo;
    ServoTelemetryData<Real> servo_telemetry;
    ServoCalibrationData<Real> servo_calibration;
};

} // namespace ship_data_model
