#pragma once

#include <stdint.h>
#include <ship_data_model.hpp>
#include "model_store.hpp"
#include "signalk_paths.hpp"
#include "units.hpp"

namespace signalk_mini {

enum class SignalKObjectKind : uint8_t {
    None,
    Position,
    Attitude,
    DateTime,
    Current,
    AisTargets,
    AisOwnVessel,
    AisSafety,
    AisDataLinkStatus,
    Dsc,
    InmarsatSafetyNet,
    Navtex,
    Alerts,
    Mob,
    LegacyComm,
};

enum class SignalKMappedValueKind : uint8_t { Number, Bool, Text, Object };

template<typename Real>
struct SignalKMappedValue {
    const char* path = nullptr;
    SignalKMappedValueKind kind = SignalKMappedValueKind::Number;
    SignalKObjectKind object_kind = SignalKObjectKind::None;
    Real number{};
    bool boolean = false;
    const char* text = nullptr;
    SourceId source_id = 0;
    uint64_t last_update_us = 0;
};

inline const char* signalk_gnss_method_quality(int32_t quality) {
    switch (quality) {
    case 0: return "no GPS";
    case 1: return "GNSS Fix";
    case 2: return "DGNSS fix";
    case 3: return "Precise GNSS";
    case 4: return "RTK fixed integer";
    case 5: return "RTK float";
    case 6: return "Estimated (DR) mode";
    case 7: return "Manual input";
    case 8: return "Simulator mode";
    default: return "Error";
    }
}

template<typename Real>
inline const char* signalk_autopilot_state(const ship_data_model::AutopilotData<Real>& controller) {
    if (!controller.enabled.value) return "standby";
    switch (controller.mode.value) {
    case ship_data_model::AutopilotMode::wind:
    case ship_data_model::AutopilotMode::true_wind: return "wind";
    case ship_data_model::AutopilotMode::gps:
    case ship_data_model::AutopilotMode::nav: return "route";
    case ship_data_model::AutopilotMode::compass: return "auto";
    }
    return "auto";
}

template<typename Real>
class SignalKMapper {
public:
    bool map_change(const ship_data_model::DataModel<Real>& model, const ModelChange& change, SignalKMappedValue<Real>& out) const {
        if (!map_field(model, change.field, out)) return false;
        out.source_id = change.source_id;
        return true;
    }

    bool map_field(const ship_data_model::DataModel<Real>& model, ModelField field, SignalKMappedValue<Real>& out) const {
        out = SignalKMappedValue<Real>{};
        switch (field) {
        case ModelField::GnssFixLatDeg:
        case ModelField::GnssFixLonDeg:
        case ModelField::GnssFixAltitudeHaeM: return map_object_if(model.gnss.fix.fix_lat_deg.valid && model.gnss.fix.fix_lon_deg.valid, latest_timestamp(model.gnss.fix.fix_lat_deg.last_update_us, model.gnss.fix.fix_lon_deg.last_update_us, model.gnss.fix.fix_alt_hae_m.last_update_us), signalk_path::NavigationPosition, SignalKObjectKind::Position, out);
        case ModelField::GnssSpeedKn: return map_number(model.gnss.fix.speed_kn, signalk_path::NavigationSpeedOverGround, out, UnitTransform::KnotsToMps);
        case ModelField::GnssTrackDeg: return map_number(model.gnss.fix.track_deg, signalk_path::NavigationCourseOverGroundTrue, out, UnitTransform::DegToRad);
        case ModelField::GnssVerticalSpeedMs: return map_number(model.gnss.fix.vertical_speed_m_s, signalk_path::NavigationVerticalSpeed, out);
        case ModelField::GnssTimestampS:
        case ModelField::GnssDateDay:
        case ModelField::GnssDateMonth:
        case ModelField::GnssDateYear: return map_object_if(model.gnss.fix.timestamp_s.valid && model.gnss.fix.date_day.valid && model.gnss.fix.date_month.valid && model.gnss.fix.date_year.valid, latest_timestamp(model.gnss.fix.timestamp_s.last_update_us, model.gnss.fix.date_day.last_update_us, latest_timestamp(model.gnss.fix.date_month.last_update_us, model.gnss.fix.date_year.last_update_us)), signalk_path::NavigationDateTime, SignalKObjectKind::DateTime, out);
        case ModelField::GnssFixQuality: return map_stamped_text(model.gnss.fix.fix_quality, signalk_gnss_method_quality(model.gnss.fix.fix_quality.value), signalk_path::NavigationGnssMethodQuality, out);
        case ModelField::GnssFixType: return map_int(model.gnss.fix.fix_type, signalk_path::NavigationGnssFixType, out);
        case ModelField::GnssFixValid: return map_stamped_bool(model.gnss.fix.fix_valid, signalk_path::NavigationGnssFixValid, out);
        case ModelField::GnssSatellitesUsed: return map_int(model.gnss.fix.satellites_used, signalk_path::NavigationGnssSatellites, out);
        case ModelField::GnssHdop: return map_number(model.gnss.fix.hdop, signalk_path::NavigationGnssHorizontalDilution, out);
        case ModelField::GnssDgpsStationId: return map_int(model.gnss.fix.dgps_station_id, signalk_path::NavigationGnssDifferentialReference, out);
        case ModelField::GnssDeclinationDeg: return map_number(model.gnss.fix.declination_deg, signalk_path::NavigationMagneticVariation, out, UnitTransform::DegToRad);
        case ModelField::GnssFixAltitudeMslM: return map_number(model.gnss.fix.fix_alt_msl_m, signalk_path::NavigationGnssAntennaAltitude, out);
        case ModelField::GnssGeoidalSeparationM: return map_number(model.gnss.fix.geoidal_separation_m, signalk_path::NavigationGnssGeoidalSeparation, out);
        case ModelField::GnssDgpsAgeS: return map_number(model.gnss.fix.dgps_age_s, signalk_path::NavigationGnssDifferentialAge, out);
        case ModelField::GnssFixAccuracyHorizontalM: return map_number(model.gnss.fix_accuracy.horizontal_accuracy_m, signalk_path::NavigationGnssHorizontalAccuracy, out);
        case ModelField::GnssFixAccuracyVerticalM: return map_number(model.gnss.fix_accuracy.vertical_accuracy_m, signalk_path::NavigationGnssVerticalAccuracy, out);
        case ModelField::GnssFixAccuracySpeedMs: return map_number(model.gnss.fix_accuracy.speed_accuracy_m_s, signalk_path::NavigationGnssSpeedAccuracy, out);
        case ModelField::GnssFixAccuracyTrackDeg: return map_number(model.gnss.fix_accuracy.track_accuracy_deg, signalk_path::NavigationGnssTrackAccuracy, out, UnitTransform::DegToRad);
        case ModelField::GnssFixAccuracyTimeS: return map_number(model.gnss.fix_accuracy.time_accuracy_s, signalk_path::NavigationGnssTimeAccuracy, out);
        case ModelField::GnssFixAccuracyPdop: return map_number(model.gnss.fix_accuracy.pdop, signalk_path::NavigationGnssPositionDilution, out);
        case ModelField::GnssFixAccuracyVdop: return map_number(model.gnss.fix_accuracy.vdop, signalk_path::NavigationGnssVerticalDilution, out);
        case ModelField::GnssDopActiveFixMode: return map_int(model.gnss.dop_active_satellites.fix_mode, signalk_path::NavigationGnssFixType, out);
        case ModelField::GnssDopActivePdop: return map_number(model.gnss.dop_active_satellites.pdop, signalk_path::NavigationGnssPositionDilution, out);
        case ModelField::GnssDopActiveVdop: return map_number(model.gnss.dop_active_satellites.vdop, signalk_path::NavigationGnssVerticalDilution, out);
        case ModelField::GnssSatellitesInView: return map_satellites_in_view(model, out);
        case ModelField::GnssSatellitePrn0: return map_sky_satellite_id(model, 0, signalk_path::NavigationGnssSatellite0Prn, out);
        case ModelField::GnssSatelliteElevationDeg0: return map_sky_satellite_number(model, 0, SkyField::Elevation, signalk_path::NavigationGnssSatellite0Elevation, out, UnitTransform::DegToRad);
        case ModelField::GnssSatelliteAzimuthDeg0: return map_sky_satellite_number(model, 0, SkyField::Azimuth, signalk_path::NavigationGnssSatellite0Azimuth, out, UnitTransform::DegToRad);
        case ModelField::GnssSatelliteSnrDb0: return map_sky_satellite_number(model, 0, SkyField::Cn0, signalk_path::NavigationGnssSatellite0Snr, out);
        case ModelField::ImuHeadingTrueDeg: return map_number(model.ins.imu.heading_true_deg, signalk_path::NavigationHeadingTrue, out, UnitTransform::DegToRad);
        case ModelField::ImuHeadingMagneticDeg: return map_number(model.ins.imu.heading_magnetic_deg, signalk_path::NavigationHeadingMagnetic, out, UnitTransform::DegToRad);
        case ModelField::ImuMagneticVariationDeg: return map_number(model.ins.imu.magnetic_variation_deg, signalk_path::NavigationMagneticVariation, out, UnitTransform::DegToRad);
        case ModelField::ImuMagneticDeviationDeg: return map_number(model.ins.imu.magnetic_deviation_deg, signalk_path::NavigationMagneticDeviation, out, UnitTransform::DegToRad);
        case ModelField::ImuPitchDeg:
        case ModelField::ImuRollDeg: return map_object_if(model.ins.imu.pitch_deg.valid || model.ins.imu.roll_deg.valid, latest_timestamp(model.ins.imu.pitch_deg.last_update_us, model.ins.imu.roll_deg.last_update_us), signalk_path::NavigationAttitude, SignalKObjectKind::Attitude, out);
        case ModelField::WindApparentDirectionDeg: return map_number(model.wind.apparent.direction_deg, signalk_path::EnvironmentWindAngleApparent, out, UnitTransform::DegToRad);
        case ModelField::WindApparentSpeedKn: return map_number(model.wind.apparent.speed_kn, signalk_path::EnvironmentWindSpeedApparent, out, UnitTransform::KnotsToMps);
        case ModelField::WindTrueDirectionDeg: return map_number(model.wind.truewind.direction_deg, signalk_path::EnvironmentWindDirectionTrue, out, UnitTransform::DegToRad);
        case ModelField::WindTrueDirectionMagneticDeg: return map_number(model.wind.truewind.direction_magnetic_deg, signalk_path::EnvironmentWindDirectionMagnetic, out, UnitTransform::DegToRad);
        case ModelField::WindTrueSpeedKn: return map_number(model.wind.truewind.speed_kn, signalk_path::EnvironmentWindSpeedTrue, out, UnitTransform::KnotsToMps);
        case ModelField::WindTrueSpeedMs: return map_number(model.wind.truewind.speed_m_s, signalk_path::EnvironmentWindSpeedTrue, out);
        case ModelField::WindSurfaceDirectionDeg: return map_number(model.wind.surface.direction_deg, signalk_path::EnvironmentWindSurfaceDirectionTrue, out, UnitTransform::DegToRad);
        case ModelField::WindSurfaceDirectionMagneticDeg: return map_number(model.wind.surface.direction_magnetic_deg, signalk_path::EnvironmentWindSurfaceDirectionMagnetic, out, UnitTransform::DegToRad);
        case ModelField::WindSurfaceSpeedKn: return map_number(model.wind.surface.speed_kn, signalk_path::EnvironmentWindSurfaceSpeed, out, UnitTransform::KnotsToMps);
        case ModelField::WindSurfaceSpeedMs: return map_number(model.wind.surface.speed_m_s, signalk_path::EnvironmentWindSurfaceSpeed, out);
        case ModelField::EnvBarometricPressureBar: return map_number(model.env.barometric_pressure_bar, signalk_path::EnvironmentOutsidePressure, out, UnitTransform::BarToPa);
        case ModelField::EnvBarometricPressureInHg: return map_number(model.env.barometric_pressure_inhg, signalk_path::EnvironmentOutsidePressure, out, UnitTransform::InHgToPa);
        case ModelField::EnvAirTemperatureC: return map_number(model.env.air_temperature_c, signalk_path::EnvironmentOutsideTemperature, out, UnitTransform::CelsiusToKelvin);
        case ModelField::EnvRelativeHumidityPercent: return map_number(model.env.relative_humidity_percent, signalk_path::EnvironmentOutsideRelativeHumidity, out, UnitTransform::PercentToRatio);
        case ModelField::EnvAbsoluteHumidityPercent: return map_number(model.env.absolute_humidity_percent, signalk_path::EnvironmentOutsideAbsoluteHumidity, out, UnitTransform::PercentToRatio);
        case ModelField::EnvDewPointC: return map_number(model.env.dew_point_c, signalk_path::EnvironmentOutsideDewPointTemperature, out, UnitTransform::CelsiusToKelvin);
        case ModelField::SeaSpeedKn: return map_number(model.sea.speed_kn, signalk_path::NavigationSpeedThroughWater, out, UnitTransform::KnotsToMps);
        case ModelField::SeaLongitudinalWaterSpeedKn: return map_number(model.sea.longitudinal_water_speed_kn, signalk_path::NavigationSpeedThroughWaterLongitudinal, out, UnitTransform::KnotsToMps);
        case ModelField::SeaTransverseWaterSpeedKn: return map_number(model.sea.transverse_water_speed_kn, signalk_path::NavigationSpeedThroughWaterTransverse, out, UnitTransform::KnotsToMps);
        case ModelField::SeaLongitudinalGroundSpeedKn: return map_number(model.sea.longitudinal_ground_speed_kn, signalk_path::NavigationSpeedOverGroundLongitudinal, out, UnitTransform::KnotsToMps);
        case ModelField::SeaTransverseGroundSpeedKn: return map_number(model.sea.transverse_ground_speed_kn, signalk_path::NavigationSpeedOverGroundTransverse, out, UnitTransform::KnotsToMps);
        case ModelField::SeaSpeedParallelToWindKn: return map_number(model.sea.speed_parallel_to_wind_kn, signalk_path::NavigationSpeedParallelToWind, out, UnitTransform::KnotsToMps);
        case ModelField::SeaSpeedParallelToWindMs: return map_number(model.sea.speed_parallel_to_wind_m_s, signalk_path::NavigationSpeedParallelToWind, out);
        case ModelField::SeaLeewayDeg: return map_number(model.sea.leeway_deg, signalk_path::NavigationLeewayAngle, out, UnitTransform::DegToRad);
        case ModelField::SeaCurrentSpeedKn:
        case ModelField::SeaCurrentDirectionDeg:
        case ModelField::SeaCurrentDirectionMagneticDeg: return map_object_if(model.sea.current_speed_kn.valid || model.sea.current_direction_deg.valid || model.sea.current_direction_magnetic_deg.valid, latest_timestamp(model.sea.current_speed_kn.last_update_us, model.sea.current_direction_deg.last_update_us, model.sea.current_direction_magnetic_deg.last_update_us), signalk_path::EnvironmentCurrent, SignalKObjectKind::Current, out);
        case ModelField::SeaTemperatureC: return map_number(model.sea.temperature_c, signalk_path::EnvironmentWaterTemperature, out, UnitTransform::CelsiusToKelvin);
        case ModelField::SeaDepthM: return map_number(model.sea.depth_m, signalk_path::EnvironmentDepthBelowTransducer, out);
        case ModelField::SeaDepthBelowKeelM: return map_number(model.sea.depth_below_keel_m, signalk_path::EnvironmentDepthBelowKeel, out);
        case ModelField::SeaDepthBelowSurfaceM: return map_number(model.sea.depth_below_surface_m, signalk_path::EnvironmentDepthBelowSurface, out);
        case ModelField::SeaDepthOffsetM: return map_number(model.sea.depth_offset_m, signalk_path::EnvironmentDepthSurfaceToTransducer, out);
        case ModelField::SteeringRudderAngleDeg: return map_number(model.steering.rudder.angle_deg, signalk_path::SteeringRudderAngle, out, UnitTransform::DegToRad);
        case ModelField::PropulsionRevolutionsNumber: return map_int(model.propulsion.revolutions.number, signalk_path::PropulsionMainEngineInstance, out);
        case ModelField::PropulsionRevolutionsSpeedRpm: return map_number(model.propulsion.revolutions.speed_rpm, signalk_path::PropulsionMainRevolutions, out, UnitTransform::RpmToHz);
        case ModelField::PropulsionRevolutionsPitchPercent: return map_number(model.propulsion.revolutions.propeller_pitch_percent, signalk_path::PropulsionMainPitch, out, UnitTransform::PercentToRatio);
        case ModelField::AutopilotMode:
        case ModelField::AutopilotEnabled: return map_text(signalk_autopilot_state(model.autopilot.controller), signalk_path::SteeringAutopilotState, out);
        case ModelField::AutopilotHeadingDeg: return map_number(model.autopilot.controller.heading_deg, signalk_path::SteeringAutopilotHeadingMagnetic, out, UnitTransform::DegToRad);
        case ModelField::AutopilotHeadingCommandDeg: return map_number(model.autopilot.controller.heading_command_deg, signalk_path::SteeringAutopilotTargetHeadingMagnetic, out, UnitTransform::DegToRad);
        case ModelField::AutopilotWarnings: return map_uint(model.autopilot.status.warnings, signalk_path::SteeringAutopilotWarnings, out);
        case ModelField::RouteActiveWaypointCount: return map_int(model.route.active.waypoint_count, signalk_path::NavigationRouteWaypointCount, out);
        case ModelField::RouteLogTotalDistanceNmi: return map_number(model.route.log.total_distance_nmi, signalk_path::NavigationLog, out, UnitTransform::NmiToM);
        case ModelField::RouteLogTripDistanceNmi: return map_number(model.route.log.trip_distance_nmi, signalk_path::NavigationTripLog, out, UnitTransform::NmiToM);
        case ModelField::RouteHeadingSteeringCommandTrueDeg: return map_number(model.route.heading_steering_command.heading_true_deg, signalk_path::SteeringAutopilotTargetHeadingTrue, out, UnitTransform::DegToRad);
        case ModelField::RouteHeadingSteeringCommandMagneticDeg: return map_number(model.route.heading_steering_command.heading_magnetic_deg, signalk_path::SteeringAutopilotTargetHeadingMagnetic, out, UnitTransform::DegToRad);
        case ModelField::RouteApbXteNmi: return map_number(model.route.apb.xte_nmi, signalk_path::NavigationCourseGreatCircleCrossTrackError, out, UnitTransform::NmiToM);
        case ModelField::RouteApbOriginToDestinationBearingDeg: return map_number(model.route.apb.origin_to_destination_bearing_deg, signalk_path::NavigationCourseGreatCircleBearingTrackTrue, out, UnitTransform::DegToRad);
        case ModelField::RouteApbPresentToDestinationBearingDeg: return map_number(model.route.apb.present_to_destination_bearing_deg, signalk_path::NavigationCourseGreatCircleBearingTrue, out, UnitTransform::DegToRad);
        case ModelField::RouteApbHeadingToSteerDeg: return map_number(model.route.apb.heading_to_steer_deg, signalk_path::SteeringAutopilotTargetHeadingMagnetic, out, UnitTransform::DegToRad);
        case ModelField::RouteApbArrivalCircleEntered: return map_stamped_bool(model.route.apb.arrival_circle_entered, signalk_path::NavigationCourseGreatCircleArrivalCircleEntered, out);
        case ModelField::RouteApbPerpendicularPassed: return map_stamped_bool(model.route.apb.perpendicular_passed, signalk_path::NavigationCourseGreatCirclePerpendicularPassed, out);
        case ModelField::RouteApbDestinationId: return map_text(model.route.apb.destination_id, signalk_path::NavigationCourseGreatCircleNextPointId, out);
        case ModelField::RouteRmbXteNmi: return map_number(model.route.rmb.xte_nmi, signalk_path::NavigationCourseGreatCircleCrossTrackError, out, UnitTransform::NmiToM);
        case ModelField::RouteRmbDestinationLatDeg: return map_number(model.route.rmb.destination_lat_deg, signalk_path::NavigationCourseGreatCircleNextPointPositionLatitude, out);
        case ModelField::RouteRmbDestinationLonDeg: return map_number(model.route.rmb.destination_lon_deg, signalk_path::NavigationCourseGreatCircleNextPointPositionLongitude, out);
        case ModelField::RouteRmbRangeNmi: return map_number(model.route.rmb.range_nmi, signalk_path::NavigationCourseGreatCircleNextPointDistance, out, UnitTransform::NmiToM);
        case ModelField::RouteRmbBearingDeg: return map_number(model.route.rmb.bearing_deg, signalk_path::NavigationCourseGreatCircleBearingTrue, out, UnitTransform::DegToRad);
        case ModelField::RouteRmbClosingVelocityKn: return map_number(model.route.rmb.closing_velocity_kn, signalk_path::NavigationCourseGreatCircleNextPointVelocityMadeGood, out, UnitTransform::KnotsToMps);
        case ModelField::RouteRmbArrived: return map_stamped_bool(model.route.rmb.arrived, signalk_path::NavigationCourseGreatCircleArrived, out);
        case ModelField::RouteRmbDestinationId: return map_text(model.route.rmb.destination_id, signalk_path::NavigationCourseGreatCircleNextPointId, out);
        case ModelField::RouteWaypointUtcTimeS: return map_number(model.route.waypoint.utc_time_s, signalk_path::NavigationCourseGreatCircleTimeOfFix, out);
        case ModelField::RouteWaypointOriginUtcTimeS: return map_number(model.route.waypoint.origin_utc_time_s, signalk_path::NavigationCourseGreatCircleOriginTime, out);
        case ModelField::RouteWaypointOriginElapsedTimeS: return map_number(model.route.waypoint.origin_elapsed_time_s, signalk_path::NavigationCourseGreatCircleOriginElapsedTime, out);
        case ModelField::RouteWaypointDestinationUtcTimeS: return map_number(model.route.waypoint.destination_utc_time_s, signalk_path::NavigationCourseGreatCircleDestinationTime, out);
        case ModelField::RouteWaypointDestinationTimeRemainingS: return map_number(model.route.waypoint.destination_time_remaining_s, signalk_path::NavigationCourseGreatCircleTimeToGo, out);
        case ModelField::RouteWaypointLatDeg: return map_number(model.route.waypoint.latitude_deg, signalk_path::NavigationCourseGreatCircleNextPointPositionLatitude, out);
        case ModelField::RouteWaypointLonDeg: return map_number(model.route.waypoint.longitude_deg, signalk_path::NavigationCourseGreatCircleNextPointPositionLongitude, out);
        case ModelField::RouteWaypointBearingTrueDeg: return map_number(model.route.waypoint.bearing_true_deg, signalk_path::NavigationCourseGreatCircleBearingTrue, out, UnitTransform::DegToRad);
        case ModelField::RouteWaypointBearingMagneticDeg: return map_number(model.route.waypoint.bearing_magnetic_deg, signalk_path::NavigationCourseGreatCircleBearingMagnetic, out, UnitTransform::DegToRad);
        case ModelField::RouteWaypointDistanceNmi: return map_number(model.route.waypoint.distance_nmi, signalk_path::NavigationCourseGreatCircleNextPointDistance, out, UnitTransform::NmiToM);
        case ModelField::RouteWaypointToId: return map_text(model.route.waypoint.to_waypoint_id, signalk_path::NavigationCourseGreatCircleNextPointId, out);
        case ModelField::RouteWaypointFromId: return map_text(model.route.waypoint.from_waypoint_id, signalk_path::NavigationCourseGreatCirclePreviousPointId, out);
        case ModelField::RouteWaypointArrivalCircleEntered: return map_stamped_bool(model.route.waypoint_arrival.arrival_circle_entered, signalk_path::NavigationCourseGreatCircleArrivalCircleEntered, out);
        case ModelField::RouteWaypointPerpendicularPassed: return map_stamped_bool(model.route.waypoint_arrival.perpendicular_passed, signalk_path::NavigationCourseGreatCirclePerpendicularPassed, out);
        case ModelField::RouteWaypointArrivalRadiusNmi: return map_number(model.route.waypoint_arrival.arrival_radius_nmi, signalk_path::NavigationCourseGreatCircleArrivalCircleRadius, out, UnitTransform::NmiToM);
        case ModelField::RouteWaypointArrivalId: return map_text(model.route.waypoint_arrival.waypoint_id, signalk_path::NavigationCourseGreatCircleNextPointId, out);
        case ModelField::TrawlHeadropeToFootropeM: return map_number(model.fishing.trawl.headrope_to_footrope_m, signalk_path::FishingTrawlHeadropeToFootrope, out);
        case ModelField::TrawlHeadropeToBottomM: return map_number(model.fishing.trawl.headrope_to_bottom_m, signalk_path::FishingTrawlHeadropeToBottom, out);
        case ModelField::TrawlDoorCenterlineOffsetM: return map_number(model.fishing.trawl.door_centerline_offset_m, signalk_path::FishingTrawlDoorCenterlineOffset, out);
        case ModelField::TrawlDoorAlongCenterlineM: return map_number(model.fishing.trawl.door_along_centerline_m, signalk_path::FishingTrawlDoorAlongCenterline, out);
        case ModelField::TrawlCartesianCenterlineOffsetM: return map_number(model.fishing.trawl.cartesian_centerline_offset_m, signalk_path::FishingTrawlCartesianCenterlineOffset, out);
        case ModelField::TrawlCartesianAlongCenterlineM: return map_number(model.fishing.trawl.cartesian_along_centerline_m, signalk_path::FishingTrawlCartesianAlongCenterline, out);
        case ModelField::TrawlDepthBelowSurfaceM: return map_number(model.fishing.trawl.depth_below_surface_m, signalk_path::FishingTrawlDepthBelowSurface, out);
        case ModelField::TrawlRelativeRangeM: return map_number(model.fishing.trawl.relative_range_m, signalk_path::FishingTrawlRelativeRange, out);
        case ModelField::TrawlRelativeBearingDeg: return map_number(model.fishing.trawl.relative_bearing_deg, signalk_path::FishingTrawlRelativeBearing, out, UnitTransform::DegToRad);
        case ModelField::TrawlTrueRangeM: return map_number(model.fishing.trawl.true_range_m, signalk_path::FishingTrawlTrueRange, out);
        case ModelField::TrawlTrueBearingDeg: return map_number(model.fishing.trawl.true_bearing_deg, signalk_path::FishingTrawlTrueBearing, out, UnitTransform::DegToRad);
        case ModelField::AisTargetsObject: return false;
        case ModelField::AisOwnVesselObject: return map_object_if(model.ais.own_vessel.valid, latest_timestamp(model.ais.own_vessel.last_seen_us, model.ais.own_vessel.last_static_update_us, model.ais.own_vessel.last_position_update_us), signalk_path::NavigationAisOwnVessel, SignalKObjectKind::AisOwnVessel, out);
        case ModelField::AisSafetyObject: return map_object(signalk_path::NotificationsAisSafety, SignalKObjectKind::AisSafety, out);
        case ModelField::AisDataLinkStatusObject: return map_object(signalk_path::NavigationAisDataLinkStatus, SignalKObjectKind::AisDataLinkStatus, out);
        case ModelField::DscStructuredNotification: return map_object(signalk_path::NotificationsDsc, SignalKObjectKind::Dsc, out);
        case ModelField::InmarsatSafetyNetStructuredNotification: return map_object(signalk_path::NotificationsInmarsatSafetyNet, SignalKObjectKind::InmarsatSafetyNet, out);
        case ModelField::NavtexStructuredNotification: return map_object(signalk_path::NotificationsNavtex, SignalKObjectKind::Navtex, out);
        case ModelField::AlertStructuredNotification: return map_object(signalk_path::NotificationsAlerts, SignalKObjectKind::Alerts, out);
        case ModelField::MobStructuredNotification: return map_object(signalk_path::NotificationsMob, SignalKObjectKind::Mob, out);
        case ModelField::LegacyCommObject: return map_object(signalk_path::CommunicationLegacy, SignalKObjectKind::LegacyComm, out);
        case ModelField::CommServerClockS: return map_uint(model.comm.server.clock_s, signalk_path::CommunicationServerClock, out);
        case ModelField::NotificationText: return map_text(model.notifications.messages.text.text[0] ? model.notifications.messages.text.text : model.notifications.messages.text.value, signalk_path::NotificationsMessageText, out);
        case ModelField::NotificationEvent: return map_text(model.notifications.messages.event.event_text, signalk_path::NotificationsMessageEvent, out);
        case ModelField::NotificationEventLog: return map_text(model.notifications.messages.event_log.event_text, signalk_path::NotificationsMessageEventLog, out);
        default: return false;
        }
    }

private:
    enum class UnitTransform : uint8_t {
        None,
        DegToRad,
        KnotsToMps,
        NmiToM,
        CelsiusToKelvin,
        RpmToHz,
        PercentToRatio,
        BarToPa,
        InHgToPa,
    };

    enum class SkyField : uint8_t { Elevation, Azimuth, Cn0 };

    static uint64_t latest_timestamp(uint64_t a, uint64_t b) { return a > b ? a : b; }
    static uint64_t latest_timestamp(uint64_t a, uint64_t b, uint64_t c) { return latest_timestamp(latest_timestamp(a, b), c); }

    template<typename StampedReal>
    bool map_number(const StampedReal& stamped, const char* path, SignalKMappedValue<Real>& out, UnitTransform transform = UnitTransform::None) const {
        if (!stamped.valid) return false;
        const bool mapped = map_number_value(static_cast<Real>(stamped.value), path, out, transform);
        if (mapped) out.last_update_us = stamped.last_update_us;
        return mapped;
    }

    bool map_number_value(Real value, const char* path, SignalKMappedValue<Real>& out, UnitTransform transform = UnitTransform::None) const {
        out.path = path;
        out.kind = SignalKMappedValueKind::Number;
        out.number = transform_number(value, transform);
        return true;
    }

    template<typename StampedInt>
    bool map_int(const StampedInt& stamped, const char* path, SignalKMappedValue<Real>& out) const {
        if (!stamped.valid) return false;
        const bool mapped = map_number_value(static_cast<Real>(stamped.value), path, out);
        if (mapped) out.last_update_us = stamped.last_update_us;
        return mapped;
    }

    template<typename StampedUInt>
    bool map_uint(const StampedUInt& stamped, const char* path, SignalKMappedValue<Real>& out) const {
        if (!stamped.valid) return false;
        const bool mapped = map_number_value(static_cast<Real>(stamped.value), path, out);
        if (mapped) out.last_update_us = stamped.last_update_us;
        return mapped;
    }

    bool map_stamped_bool(const ship_data_model::Setting<bool>& setting, const char* path, SignalKMappedValue<Real>& out) const {
        return map_bool(setting.value, path, out);
    }

    template<typename StampedBool>
    bool map_stamped_bool(const StampedBool& stamped, const char* path, SignalKMappedValue<Real>& out) const {
        if (!stamped.valid) return false;
        const bool mapped = map_bool(stamped.value, path, out);
        if (mapped) out.last_update_us = stamped.last_update_us;
        return mapped;
    }

    bool map_bool(bool value, const char* path, SignalKMappedValue<Real>& out) const {
        out.path = path;
        out.kind = SignalKMappedValueKind::Bool;
        out.boolean = value;
        return true;
    }

    bool map_text(const char* text, const char* path, SignalKMappedValue<Real>& out) const {
        if (!text || text[0] == '\0') return false;
        out.path = path;
        out.kind = SignalKMappedValueKind::Text;
        out.text = text;
        return true;
    }

    template<typename StampedValue>
    bool map_stamped_text(const StampedValue& stamped, const char* text, const char* path, SignalKMappedValue<Real>& out) const {
        if (!stamped.valid || !map_text(text, path, out)) return false;
        out.last_update_us = stamped.last_update_us;
        return true;
    }

    bool map_object(const char* path, SignalKObjectKind kind, SignalKMappedValue<Real>& out) const {
        out.path = path;
        out.kind = SignalKMappedValueKind::Object;
        out.object_kind = kind;
        return true;
    }

    bool map_object_if(bool available, uint64_t last_update_us, const char* path, SignalKObjectKind kind, SignalKMappedValue<Real>& out) const {
        if (!available || !map_object(path, kind, out)) return false;
        out.last_update_us = last_update_us;
        return true;
    }

    bool map_satellites_in_view(const ship_data_model::DataModel<Real>& model, SignalKMappedValue<Real>& out) const {
        if (model.gnss.sky_view.satellites_in_view.valid) {
            return map_int(model.gnss.sky_view.satellites_in_view, signalk_path::NavigationGnssSatellitesInView, out);
        }
        return map_int(model.gnss.satellites_in_view.satellites_in_view, signalk_path::NavigationGnssSatellitesInView, out);
    }

    bool map_sky_satellite_id(const ship_data_model::DataModel<Real>& model, size_t index, const char* path, SignalKMappedValue<Real>& out) const {
        const auto& sky = model.gnss.sky_view;
        if (index < sky.observation_count && sky.observations[index].satellite_id_valid) {
            return map_number_value(static_cast<Real>(sky.observations[index].satellite_id), path, out);
        }
        if (index < 4) return map_int(model.gnss.satellites_in_view.satellite_prn[index], path, out);
        return false;
    }

    bool map_sky_satellite_number(const ship_data_model::DataModel<Real>& model,
                                  size_t index,
                                  SkyField field,
                                  const char* path,
                                  SignalKMappedValue<Real>& out,
                                  UnitTransform transform = UnitTransform::None) const {
        const auto& sky = model.gnss.sky_view;
        if (index < sky.observation_count) {
            const auto& observation = sky.observations[index];
            if (field == SkyField::Elevation && observation.elevation_valid) return map_number_value(observation.elevation_deg, path, out, transform);
            if (field == SkyField::Azimuth && observation.azimuth_valid) return map_number_value(observation.azimuth_true_deg, path, out, transform);
            if (field == SkyField::Cn0 && observation.cn0_valid) return map_number_value(observation.cn0_db_hz, path, out, transform);
        }
        if (index >= 4) return false;
        if (field == SkyField::Elevation) return map_number(model.gnss.satellites_in_view.elevation_deg[index], path, out, transform);
        if (field == SkyField::Azimuth) return map_number(model.gnss.satellites_in_view.azimuth_true_deg[index], path, out, transform);
        return map_number(model.gnss.satellites_in_view.snr_db[index], path, out, transform);
    }

    Real transform_number(Real value, UnitTransform transform) const {
        switch (transform) {
        case UnitTransform::None: return value;
        case UnitTransform::DegToRad: return deg_to_rad<Real>(value);
        case UnitTransform::KnotsToMps: return knots_to_mps<Real>(value);
        case UnitTransform::NmiToM: return nmi_to_m<Real>(value);
        case UnitTransform::CelsiusToKelvin: return celsius_to_kelvin<Real>(value);
        case UnitTransform::RpmToHz: return rpm_to_hz<Real>(value);
        case UnitTransform::PercentToRatio: return percent_to_ratio<Real>(value);
        case UnitTransform::BarToPa: return bar_to_pa<Real>(value);
        case UnitTransform::InHgToPa: return inhg_to_pa<Real>(value);
        }
        return value;
    }
};

} // namespace signalk_mini
