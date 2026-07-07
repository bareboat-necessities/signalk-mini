#pragma once

#include <seatalk.hpp>

#include "model_store.hpp"
#include "types.hpp"

namespace signalk_mini {

template<typename Real, size_t QueueCapacity>
void mark_seatalk_changes(ModelStore<Real, QueueCapacity>& store,
                          const seatalk::SeaTalkDecoded<Real>& decoded,
                          SourceId source_id,
                          uint64_t now_us) {
    using Kind = seatalk::SeaTalkDecodedKind;

    switch (decoded.kind) {
    case Kind::depth:
        store.mark_changed(ModelField::SeaDepthM, source_id, now_us);
        store.mark_changed(ModelField::SeaDepthBelowSurfaceM, source_id, now_us);
        break;

    case Kind::engine_rpm_pitch:
        store.mark_changed(ModelField::PropulsionRevolutionsSpeedRpm, source_id, now_us);
        store.mark_changed(ModelField::PropulsionRevolutionsPitchPercent, source_id, now_us);
        store.mark_changed(ModelField::PropulsionRevolutionsNumber, source_id, now_us);
        break;

    case Kind::apparent_wind_angle:
        store.mark_changed(ModelField::WindApparentDirectionDeg, source_id, now_us);
        break;

    case Kind::apparent_wind_speed:
        store.mark_changed(ModelField::WindApparentSpeedKn, source_id, now_us);
        break;

    case Kind::speed_through_water:
        store.mark_changed(ModelField::SeaSpeedKn, source_id, now_us);
        store.mark_changed(ModelField::SeaLongitudinalWaterSpeedKn, source_id, now_us);
        break;

    case Kind::water_temperature:
        store.mark_changed(ModelField::SeaTemperatureC, source_id, now_us);
        break;

    case Kind::heading_magnetic:
        store.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
        store.mark_changed(ModelField::ImuHeadingMagneticDeg, source_id, now_us);
        break;

    case Kind::rudder_angle:
        store.mark_changed(ModelField::SteeringRudderAngleDeg, source_id, now_us);
        store.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
        store.mark_changed(ModelField::ImuHeadingMagneticDeg, source_id, now_us);
        break;

    case Kind::trip_distance:
        store.mark_changed(ModelField::RouteLogTripDistanceNmi, source_id, now_us);
        break;

    case Kind::total_distance:
        store.mark_changed(ModelField::RouteLogTotalDistanceNmi, source_id, now_us);
        break;

    case Kind::trip_total:
        store.mark_changed(ModelField::RouteLogTotalDistanceNmi, source_id, now_us);
        store.mark_changed(ModelField::RouteLogTripDistanceNmi, source_id, now_us);
        break;

    case Kind::display_units:
    case Kind::lamp_intensity:
    case Kind::waypoint_definition:
    case Kind::device_status:
    case Kind::observed_unknown:
        store.mark_changed(ModelField::NotificationText, source_id, now_us);
        break;

    case Kind::position_latitude:
        store.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us);
        break;

    case Kind::position_longitude:
        store.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us);
        break;

    case Kind::position_lat_lon:
        store.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us);
        store.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us);
        break;

    case Kind::speed_over_ground:
        store.mark_changed(ModelField::GnssSpeedKn, source_id, now_us);
        store.mark_changed(ModelField::SeaLongitudinalGroundSpeedKn, source_id, now_us);
        break;

    case Kind::course_over_ground:
        store.mark_changed(ModelField::GnssTrackDeg, source_id, now_us);
        break;

    case Kind::time_utc:
        store.mark_changed(ModelField::GnssTimestampS, source_id, now_us);
        break;

    case Kind::date_utc:
        store.mark_changed(ModelField::GnssDateDay, source_id, now_us);
        store.mark_changed(ModelField::GnssDateMonth, source_id, now_us);
        store.mark_changed(ModelField::GnssDateYear, source_id, now_us);
        break;

    case Kind::satellite_info:
        store.mark_changed(ModelField::GnssSatellitesUsed, source_id, now_us);
        store.mark_changed(ModelField::GnssSatellitesInView, source_id, now_us);
        store.mark_changed(ModelField::GnssHdop, source_id, now_us);
        break;

    case Kind::satellite_detail:
        store.mark_changed(ModelField::GnssFixQuality, source_id, now_us);
        store.mark_changed(ModelField::GnssSatellitesUsed, source_id, now_us);
        store.mark_changed(ModelField::GnssHdop, source_id, now_us);
        store.mark_changed(ModelField::GnssDgpsStationId, source_id, now_us);
        store.mark_changed(ModelField::GnssSatellitePrn0, source_id, now_us);
        store.mark_changed(ModelField::GnssSatelliteAzimuthDeg0, source_id, now_us);
        store.mark_changed(ModelField::GnssSatelliteElevationDeg0, source_id, now_us);
        store.mark_changed(ModelField::GnssSatelliteSnrDb0, source_id, now_us);
        store.mark_changed(ModelField::NotificationText, source_id, now_us);
        break;

    case Kind::differential_detail:
        store.mark_changed(ModelField::GnssSatellitePrn0, source_id, now_us);
        store.mark_changed(ModelField::GnssSatelliteAzimuthDeg0, source_id, now_us);
        store.mark_changed(ModelField::GnssSatelliteElevationDeg0, source_id, now_us);
        store.mark_changed(ModelField::GnssSatelliteSnrDb0, source_id, now_us);
        store.mark_changed(ModelField::NotificationText, source_id, now_us);
        break;

    case Kind::autopilot_state:
        store.mark_changed(ModelField::AutopilotMode, source_id, now_us);
        store.mark_changed(ModelField::AutopilotEnabled, source_id, now_us);
        store.mark_changed(ModelField::AutopilotHeadingDeg, source_id, now_us);
        store.mark_changed(ModelField::AutopilotHeadingCommandDeg, source_id, now_us);
        store.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
        store.mark_changed(ModelField::ImuHeadingMagneticDeg, source_id, now_us);
        store.mark_changed(ModelField::SteeringRudderAngleDeg, source_id, now_us);
        store.mark_changed(ModelField::AutopilotWarnings, source_id, now_us);
        store.mark_changed(ModelField::NotificationEvent, source_id, now_us);
        break;

    case Kind::autopilot_key:
        store.mark_changed(ModelField::NotificationEvent, source_id, now_us);
        break;

    case Kind::navigation_to_waypoint:
        store.mark_changed(ModelField::RouteApbXteNmi, source_id, now_us);
        store.mark_changed(ModelField::RouteApbPresentToDestinationBearingDeg, source_id, now_us);
        store.mark_changed(ModelField::RouteApbHeadingToSteerDeg, source_id, now_us);
        store.mark_changed(ModelField::RouteApbOriginToDestinationBearingDeg, source_id, now_us);
        store.mark_changed(ModelField::RouteWaypointBearingMagneticDeg, source_id, now_us);
        store.mark_changed(ModelField::RouteWaypointDistanceNmi, source_id, now_us);
        store.mark_changed(ModelField::RouteRmbRangeNmi, source_id, now_us);
        break;

    case Kind::compass_variation:
        store.mark_changed(ModelField::ImuMagneticVariationDeg, source_id, now_us);
        store.mark_changed(ModelField::GnssDeclinationDeg, source_id, now_us);
        break;

    case Kind::waypoint_id:
    case Kind::waypoint_name:
        store.mark_changed(ModelField::RouteWaypointToId, source_id, now_us);
        store.mark_changed(ModelField::RouteApbDestinationId, source_id, now_us);
        store.mark_changed(ModelField::RouteRmbDestinationId, source_id, now_us);
        break;

    case Kind::arrival_info:
        store.mark_changed(ModelField::RouteWaypointArrivalId, source_id, now_us);
        store.mark_changed(ModelField::RouteWaypointArrivalCircleEntered, source_id, now_us);
        store.mark_changed(ModelField::RouteWaypointPerpendicularPassed, source_id, now_us);
        store.mark_changed(ModelField::RouteApbDestinationId, source_id, now_us);
        store.mark_changed(ModelField::RouteApbArrivalCircleEntered, source_id, now_us);
        store.mark_changed(ModelField::RouteApbPerpendicularPassed, source_id, now_us);
        store.mark_changed(ModelField::RouteRmbArrived, source_id, now_us);
        break;

    case Kind::none:
    default:
        break;
    }
}

} // namespace signalk_mini
