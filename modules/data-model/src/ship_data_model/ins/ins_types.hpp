#pragma once

#include <stdint.h>

namespace ship_data_model {

enum class ImuCalibrationState : uint8_t {
    unknown,
    invalid,
    learning,
    valid
};

template<typename Real = float>
struct Vec3 {
    Real x = Real(0);
    Real y = Real(0);
    Real z = Real(0);
};

template<typename Real = float>
struct Quat {
    Real w = Real(1);
    Real x = Real(0);
    Real y = Real(0);
    Real z = Real(0);
};

} // namespace ship_data_model
