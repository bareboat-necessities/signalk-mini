#pragma once

#include <stdint.h>

namespace signalk_mini {

using SourceId = uint16_t;

enum class ModelField : uint16_t {
    None = 0,
    GnssFixLatDeg,
    GnssFixLonDeg,
    GnssSpeedKn,
    GnssTrackDeg,
    ImuHeadingDeg,
    WindApparentDirectionDeg,
    WindApparentSpeedKn,
    SeaDepthM,
    SeaDepthBelowKeelM,
    SeaDepthBelowSurfaceM,
};

} // namespace signalk_mini
