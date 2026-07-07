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
    GnssTimestampS,
    GnssDateDay,
    GnssDateMonth,
    GnssDateYear,
    GnssFixQuality,
    GnssSatellitesUsed,
    GnssHdop,
    GnssDgpsStationId,
    GnssDeclinationDeg,
    GnssSatellitesInView,
    GnssSatellitePrn0,
    GnssSatelliteElevationDeg0,
    GnssSatelliteAzimuthDeg0,
    GnssSatelliteSnrDb0,

    ImuHeadingDeg,
    ImuHeadingMagneticDeg,
    ImuMagneticVariationDeg,

    WindApparentDirectionDeg,
    WindApparentSpeedKn,

    SeaSpeedKn,
    SeaLongitudinalWaterSpeedKn,
    SeaLongitudinalGroundSpeedKn,
    SeaTemperatureC,
    SeaDepthM,
    SeaDepthBelowKeelM,
    SeaDepthBelowSurfaceM,

    SteeringRudderAngleDeg,

    PropulsionRevolutionsNumber,
    PropulsionRevolutionsSpeedRpm,
    PropulsionRevolutionsPitchPercent,

    AutopilotMode,
    AutopilotEnabled,
    AutopilotHeadingDeg,
    AutopilotHeadingCommandDeg,
    AutopilotWarnings,

    RouteLogTotalDistanceNmi,
    RouteLogTripDistanceNmi,
    RouteApbXteNmi,
    RouteApbOriginToDestinationBearingDeg,
    RouteApbPresentToDestinationBearingDeg,
    RouteApbHeadingToSteerDeg,
    RouteApbArrivalCircleEntered,
    RouteApbPerpendicularPassed,
    RouteApbDestinationId,
    RouteRmbRangeNmi,
    RouteRmbArrived,
    RouteRmbDestinationId,
    RouteWaypointBearingMagneticDeg,
    RouteWaypointDistanceNmi,
    RouteWaypointToId,
    RouteWaypointArrivalCircleEntered,
    RouteWaypointPerpendicularPassed,
    RouteWaypointArrivalId,

    NotificationText,
    NotificationEvent,
};

} // namespace signalk_mini
