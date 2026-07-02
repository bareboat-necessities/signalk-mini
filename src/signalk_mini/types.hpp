#pragma once

#include <stdint.h>

namespace signalk_mini {

using SourceId = uint16_t;

enum class ModelField : uint16_t {
    None = 0,
    NavigationGpsFixLatDeg,
    NavigationGpsFixLonDeg,
    NavigationGpsSpeedKn,
    NavigationGpsTrackDeg,
    ImuHeadingDeg,
    WindApparentDirectionDeg,
    WindApparentSpeedKn,
    WaterDepthM,
    WaterDepthBelowKeelM,
    WaterDepthBelowSurfaceM,
};

} // namespace signalk_mini
