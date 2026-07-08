#pragma once

#include <stdint.h>
#include <ship_data_model.hpp>
#include "model_store.hpp"
#include "units.hpp"

namespace signalk_mini {

enum class SignalKObjectKind : uint8_t {
    None,
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
};

inline const char* signalk_autopilot_mode(ship_data_model::AutopilotMode mode) {
    switch (mode) {
    case ship_data_model::AutopilotMode::compass: return "compass";
    case ship_data_model::AutopilotMode::gps: return "gps";
    case ship_data_model::AutopilotMode::nav: return "nav";
    case ship_data_model::AutopilotMode::wind: return "wind";
    case ship_data_model::AutopilotMode::true_wind: return "trueWind";
    }
    return "unknown";
}

template<typename Real>
class SignalKMapper {
public:
    bool map_change(const ship_data_model::DataModel<Real>& model, const ModelChange& change, SignalKMappedValue<Real>& out) const {
        switch (change.field) {
        case ModelField::GnssFixLatDeg: return map_number(model.gnss.fix.fix_lat_deg, "navigation.position.value.latitude", out);
        case ModelField::GnssFixLonDeg: return map_number(model.gnss.fix.fix_lon_deg, "navigation.position.value.longitude", out);
        case ModelField::GnssSpeedKn: return map_number(model.gnss.fix.speed_kn, "navigation.speedOverGround", out, UnitTransform::KnotsToMps);
        case ModelField::GnssTrackDeg: return map_number(model.gnss.fix.track_deg, "navigation.courseOverGroundTrue", out, UnitTransform::DegToRad);
        case ModelField::GnssTimestampS: return map_number(model.gnss.fix.timestamp_s, "navigation.datetime.timeOfDay", out);
        case ModelField::GnssDateDay: return map_int(model.gnss.fix.date_day, "navigation.datetime.day", out);
        case ModelField::GnssDateMonth: return map_int(model.gnss.fix.date_month, "navigation.datetime.month", out);
        case ModelField::GnssDateYear: return map_int(model.gnss.fix.date_year, "navigation.datetime.year", out);
        case ModelField::GnssFixQuality: return map_int(model.gnss.fix.fix_quality, "navigation.gnss.methodQuality", out);
        case ModelField::GnssSatellitesUsed: return map_int(model.gnss.fix.satellites_used, "navigation.gnss.satellites", out);
        case ModelField::GnssHdop: return map_number(model.gnss.fix.hdop, "navigation.gnss.horizontalDilution", out);
        case ModelField::GnssDgpsStationId: return map_int(model.gnss.fix.dgps_station_id, "navigation.gnss.differentialReference", out);
        case ModelField::GnssDeclinationDeg: return map_number(model.gnss.fix.declination_deg, "navigation.gnss.declination", out, UnitTransform::DegToRad);
        case ModelField::GnssFixAltitudeM: return map_number(model.gnss.fix.fix_alt_m, "navigation.gnss.antennaAltitude", out);
        case ModelField::GnssGeoidalSeparationM: return map_number(model.gnss.fix.geoidal_separation_m, "navigation.gnss.geoidalSeparation", out);
        case ModelField::GnssDgpsAgeS: return map_number(model.gnss.fix.dgps_age_s, "navigation.gnss.differentialAge", out);
        case ModelField::GnssFixAccuracyHorizontalM: return map_number(model.gnss.fix_accuracy.horizontal_accuracy_m, "navigation.gnss.horizontalAccuracy", out);
        case ModelField::GnssFixAccuracyVerticalM: return map_number(model.gnss.fix_accuracy.vertical_accuracy_m, "navigation.gnss.verticalAccuracy", out);
        case ModelField::GnssFixAccuracyPdop: return map_number(model.gnss.fix_accuracy.pdop, "navigation.gnss.positionDilution", out);
        case ModelField::GnssFixAccuracyVdop: return map_number(model.gnss.fix_accuracy.vdop, "navigation.gnss.verticalDilution", out);
        case ModelField::GnssDopActiveFixMode: return map_int(model.gnss.dop_active_satellites.fix_mode, "navigation.gnss.fixType", out);
        case ModelField::GnssDopActivePdop: return map_number(model.gnss.dop_active_satellites.pdop, "navigation.gnss.positionDilution", out);
        case ModelField::GnssDopActiveVdop: return map_number(model.gnss.dop_active_satellites.vdop, "navigation.gnss.verticalDilution", out);
        case ModelField::GnssSatellitesInView: return map_int(model.gnss.satellites_in_view.satellites_in_view, "navigation.gnss.satellitesInView", out);
        case ModelField::GnssSatellitePrn0: return map_int(model.gnss.satellites_in_view.satellite_prn[0], "navigation.gnss.satellite.0.prn", out);
        case ModelField::GnssSatelliteElevationDeg0: return map_number(model.gnss.satellites_in_view.elevation_deg[0], "navigation.gnss.satellite.0.elevation", out, UnitTransform::DegToRad);
        case ModelField::GnssSatelliteAzimuthDeg0: return map_number(model.gnss.satellites_in_view.azimuth_true_deg[0], "navigation.gnss.satellite.0.azimuth", out, UnitTransform::DegToRad);
        case ModelField::GnssSatelliteSnrDb0: return map_number(model.gnss.satellites_in_view.snr_db[0], "navigation.gnss.satellite.0.snr", out);
        case ModelField::ImuHeadingDeg: return map_number(model.ins.imu.heading_deg, "navigation.headingTrue", out, UnitTransform::DegToRad);
        case ModelField::ImuHeadingTrueDeg: return map_number(model.ins.imu.heading_true_deg, "navigation.headingTrue", out, UnitTransform::DegToRad);
        case ModelField::ImuHeadingMagneticDeg: return map_number(model.ins.imu.heading_magnetic_deg, "navigation.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::ImuMagneticVariationDeg: return map_number(model.ins.imu.magnetic_variation_deg, "navigation.magneticVariation", out, UnitTransform::DegToRad);
        case ModelField::ImuMagneticDeviationDeg: return map_number(model.ins.imu.magnetic_deviation_deg, "navigation.magneticDeviation", out, UnitTransform::DegToRad);
        case ModelField::ImuPitchDeg: return map_number(model.ins.imu.pitch_deg, "navigation.attitude.pitch", out, UnitTransform::DegToRad);
        case ModelField::ImuRollDeg: return map_number(model.ins.imu.roll_deg, "navigation.attitude.roll", out, UnitTransform::DegToRad);
        case ModelField::WindApparentDirectionDeg: return map_number(model.wind.apparent.direction_deg, "environment.wind.angleApparent", out, UnitTransform::DegToRad);
        case ModelField::WindApparentSpeedKn: return map_number(model.wind.apparent.speed_kn, "environment.wind.speedApparent", out, UnitTransform::KnotsToMps);
        case ModelField::WindTrueDirectionDeg: return map_number(model.wind.truewind.direction_deg, "environment.wind.directionTrue", out, UnitTransform::DegToRad);
        case ModelField::WindTrueDirectionMagneticDeg: return map_number(model.wind.truewind.direction_magnetic_deg, "environment.wind.directionMagnetic", out, UnitTransform::DegToRad);
        case ModelField::WindTrueSpeedKn: return map_number(model.wind.truewind.speed_kn, "environment.wind.speedTrue", out, UnitTransform::KnotsToMps);
        case ModelField::WindTrueSpeedMs: return map_number(model.wind.truewind.speed_m_s, "environment.wind.speedTrue", out);
        case ModelField::WindSurfaceDirectionDeg: return map_number(model.wind.surface.direction_deg, "environment.wind.surface.directionTrue", out, UnitTransform::DegToRad);
        case ModelField::WindSurfaceDirectionMagneticDeg: return map_number(model.wind.surface.direction_magnetic_deg, "environment.wind.surface.directionMagnetic", out, UnitTransform::DegToRad);
        case ModelField::WindSurfaceSpeedKn: return map_number(model.wind.surface.speed_kn, "environment.wind.surface.speed", out, UnitTransform::KnotsToMps);
        case ModelField::WindSurfaceSpeedMs: return map_number(model.wind.surface.speed_m_s, "environment.wind.surface.speed", out);
        case ModelField::EnvBarometricPressureBar: return map_number(model.env.barometric_pressure_bar, "environment.outside.pressure", out, UnitTransform::BarToPa);
        case ModelField::EnvBarometricPressureInHg: return map_number(model.env.barometric_pressure_inhg, "environment.outside.pressure", out, UnitTransform::InHgToPa);
        case ModelField::EnvAirTemperatureC: return map_number(model.env.air_temperature_c, "environment.outside.temperature", out, UnitTransform::CelsiusToKelvin);
        case ModelField::EnvRelativeHumidityPercent: return map_number(model.env.relative_humidity_percent, "environment.outside.relativeHumidity", out, UnitTransform::PercentToRatio);
        case ModelField::EnvAbsoluteHumidityPercent: return map_number(model.env.absolute_humidity_percent, "environment.outside.absoluteHumidity", out, UnitTransform::PercentToRatio);
        case ModelField::EnvDewPointC: return map_number(model.env.dew_point_c, "environment.outside.dewPointTemperature", out, UnitTransform::CelsiusToKelvin);
        case ModelField::SeaSpeedKn: return map_number(model.sea.speed_kn, "navigation.speedThroughWater", out, UnitTransform::KnotsToMps);
        case ModelField::SeaLongitudinalWaterSpeedKn: return map_number(model.sea.longitudinal_water_speed_kn, "navigation.speedThroughWaterLongitudinal", out, UnitTransform::KnotsToMps);
        case ModelField::SeaTransverseWaterSpeedKn: return map_number(model.sea.transverse_water_speed_kn, "navigation.speedThroughWaterTransverse", out, UnitTransform::KnotsToMps);
        case ModelField::SeaLongitudinalGroundSpeedKn: return map_number(model.sea.longitudinal_ground_speed_kn, "navigation.speedOverGroundLongitudinal", out, UnitTransform::KnotsToMps);
        case ModelField::SeaTransverseGroundSpeedKn: return map_number(model.sea.transverse_ground_speed_kn, "navigation.speedOverGroundTransverse", out, UnitTransform::KnotsToMps);
        case ModelField::SeaSpeedParallelToWindKn: return map_number(model.sea.speed_parallel_to_wind_kn, "navigation.speedParallelToWind", out, UnitTransform::KnotsToMps);
        case ModelField::SeaSpeedParallelToWindMs: return map_number(model.sea.speed_parallel_to_wind_m_s, "navigation.speedParallelToWind", out);
        case ModelField::SeaLeewayDeg: return map_number(model.sea.leeway_deg, "navigation.leewayAngle", out, UnitTransform::DegToRad);
        case ModelField::SeaCurrentSpeedKn: return map_number(model.sea.current_speed_kn, "environment.current.speed", out, UnitTransform::KnotsToMps);
        case ModelField::SeaCurrentDirectionDeg: return map_number(model.sea.current_direction_deg, "environment.current.directionTrue", out, UnitTransform::DegToRad);
        case ModelField::SeaCurrentDirectionMagneticDeg: return map_number(model.sea.current_direction_magnetic_deg, "environment.current.directionMagnetic", out, UnitTransform::DegToRad);
        case ModelField::SeaTemperatureC: return map_number(model.sea.temperature_c, "environment.water.temperature", out, UnitTransform::CelsiusToKelvin);
        case ModelField::SeaDepthM: return map_number(model.sea.depth_m, "environment.depth.belowTransducer", out);
        case ModelField::SeaDepthBelowKeelM: return map_number(model.sea.depth_below_keel_m, "environment.depth.belowKeel", out);
        case ModelField::SeaDepthBelowSurfaceM: return map_number(model.sea.depth_below_surface_m, "environment.depth.belowSurface", out);
        case ModelField::SeaDepthOffsetM: return map_number(model.sea.depth_offset_m, "environment.depth.transducerOffset", out);
        case ModelField::SteeringRudderAngleDeg: return map_number(model.steering.rudder.angle_deg, "steering.rudderAngle", out, UnitTransform::DegToRad);
        case ModelField::PropulsionRevolutionsNumber: return map_int(model.propulsion.revolutions.number, "propulsion.main.engineInstance", out);
        case ModelField::PropulsionRevolutionsSpeedRpm: return map_number(model.propulsion.revolutions.speed_rpm, "propulsion.main.revolutions", out, UnitTransform::RpmToHz);
        case ModelField::PropulsionRevolutionsPitchPercent: return map_number(model.propulsion.revolutions.propeller_pitch_percent, "propulsion.main.pitch", out, UnitTransform::PercentToRatio);
        case ModelField::AutopilotMode: return map_text(signalk_autopilot_mode(model.autopilot.controller.mode.value), "steering.autopilot.mode", out);
        case ModelField::AutopilotEnabled: return map_bool(model.autopilot.controller.enabled.value, "steering.autopilot.enabled", out);
        case ModelField::AutopilotHeadingDeg: return map_number(model.autopilot.controller.heading_deg, "steering.autopilot.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::AutopilotHeadingCommandDeg: return map_number(model.autopilot.controller.heading_command_deg, "steering.autopilot.target.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::AutopilotWarnings: return map_uint(model.autopilot.status.warnings, "steering.autopilot.warnings", out);
        case ModelField::RouteActiveWaypointCount: return map_int(model.route.active.waypoint_count, "navigation.route.waypointCount", out);
        case ModelField::RouteLogTotalDistanceNmi: return map_number(model.route.log.total_distance_nmi, "navigation.log", out, UnitTransform::NmiToM);
        case ModelField::RouteLogTripDistanceNmi: return map_number(model.route.log.trip_distance_nmi, "navigation.trip.log", out, UnitTransform::NmiToM);
        case ModelField::RouteHeadingSteeringCommandTrueDeg: return map_number(model.route.heading_steering_command.heading_true_deg, "steering.autopilot.target.headingTrue", out, UnitTransform::DegToRad);
        case ModelField::RouteHeadingSteeringCommandMagneticDeg: return map_number(model.route.heading_steering_command.heading_magnetic_deg, "steering.autopilot.target.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::RouteApbXteNmi: return map_number(model.route.apb.xte_nmi, "navigation.courseGreatCircle.crossTrackError", out, UnitTransform::NmiToM);
        case ModelField::RouteApbOriginToDestinationBearingDeg: return map_number(model.route.apb.origin_to_destination_bearing_deg, "navigation.courseGreatCircle.bearingTrackTrue", out, UnitTransform::DegToRad);
        case ModelField::RouteApbPresentToDestinationBearingDeg: return map_number(model.route.apb.present_to_destination_bearing_deg, "navigation.courseGreatCircle.bearingTrue", out, UnitTransform::DegToRad);
        case ModelField::RouteApbHeadingToSteerDeg: return map_number(model.route.apb.heading_to_steer_deg, "steering.autopilot.target.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::RouteApbArrivalCircleEntered: return map_bool(model.route.apb.arrival_circle_entered.value, "navigation.courseGreatCircle.arrivalCircleEntered", out);
        case ModelField::RouteApbPerpendicularPassed: return map_bool(model.route.apb.perpendicular_passed.value, "navigation.courseGreatCircle.perpendicularPassed", out);
        case ModelField::RouteApbDestinationId: return map_text(model.route.apb.destination_id, "navigation.courseGreatCircle.nextPoint.id", out);
        case ModelField::RouteRmbXteNmi: return map_number(model.route.rmb.xte_nmi, "navigation.courseGreatCircle.crossTrackError", out, UnitTransform::NmiToM);
        case ModelField::RouteRmbDestinationLatDeg: return map_number(model.route.rmb.destination_lat_deg, "navigation.courseGreatCircle.nextPoint.position.latitude", out);
        case ModelField::RouteRmbDestinationLonDeg: return map_number(model.route.rmb.destination_lon_deg, "navigation.courseGreatCircle.nextPoint.position.longitude", out);
        case ModelField::RouteRmbRangeNmi: return map_number(model.route.rmb.range_nmi, "navigation.courseGreatCircle.nextPoint.distance", out, UnitTransform::NmiToM);
        case ModelField::RouteRmbBearingDeg: return map_number(model.route.rmb.bearing_deg, "navigation.courseGreatCircle.bearingTrue", out, UnitTransform::DegToRad);
        case ModelField::RouteRmbClosingVelocityKn: return map_number(model.route.rmb.closing_velocity_kn, "navigation.courseGreatCircle.nextPoint.velocityMadeGood", out, UnitTransform::KnotsToMps);
        case ModelField::RouteRmbArrived: return map_bool(model.route.rmb.arrived.value, "navigation.courseGreatCircle.arrived", out);
        case ModelField::RouteRmbDestinationId: return map_text(model.route.rmb.destination_id, "navigation.courseGreatCircle.nextPoint.id", out);
        case ModelField::RouteWaypointUtcTimeS: return map_number(model.route.waypoint.utc_time_s, "navigation.courseGreatCircle.timeOfFix", out);
        case ModelField::RouteWaypointOriginUtcTimeS: return map_number(model.route.waypoint.origin_utc_time_s, "navigation.courseGreatCircle.origin.time", out);
        case ModelField::RouteWaypointOriginElapsedTimeS: return map_number(model.route.waypoint.origin_elapsed_time_s, "navigation.courseGreatCircle.origin.elapsedTime", out);
        case ModelField::RouteWaypointDestinationUtcTimeS: return map_number(model.route.waypoint.destination_utc_time_s, "navigation.courseGreatCircle.destination.time", out);
        case ModelField::RouteWaypointDestinationTimeRemainingS: return map_number(model.route.waypoint.destination_time_remaining_s, "navigation.courseGreatCircle.timeToGo", out);
        case ModelField::RouteWaypointLatDeg: return map_number(model.route.waypoint.latitude_deg, "navigation.courseGreatCircle.nextPoint.position.latitude", out);
        case ModelField::RouteWaypointLonDeg: return map_number(model.route.waypoint.longitude_deg, "navigation.courseGreatCircle.nextPoint.position.longitude", out);
        case ModelField::RouteWaypointBearingTrueDeg: return map_number(model.route.waypoint.bearing_true_deg, "navigation.courseGreatCircle.bearingTrue", out, UnitTransform::DegToRad);
        case ModelField::RouteWaypointBearingMagneticDeg: return map_number(model.route.waypoint.bearing_magnetic_deg, "navigation.courseGreatCircle.bearingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::RouteWaypointDistanceNmi: return map_number(model.route.waypoint.distance_nmi, "navigation.courseGreatCircle.nextPoint.distance", out, UnitTransform::NmiToM);
        case ModelField::RouteWaypointToId: return map_text(model.route.waypoint.to_waypoint_id, "navigation.courseGreatCircle.nextPoint.id", out);
        case ModelField::RouteWaypointFromId: return map_text(model.route.waypoint.from_waypoint_id, "navigation.courseGreatCircle.previousPoint.id", out);
        case ModelField::RouteWaypointArrivalCircleEntered: return map_bool(model.route.waypoint_arrival.arrival_circle_entered.value, "navigation.courseGreatCircle.arrivalCircleEntered", out);
        case ModelField::RouteWaypointPerpendicularPassed: return map_bool(model.route.waypoint_arrival.perpendicular_passed.value, "navigation.courseGreatCircle.perpendicularPassed", out);
        case ModelField::RouteWaypointArrivalRadiusNmi: return map_number(model.route.waypoint_arrival.arrival_radius_nmi, "navigation.courseGreatCircle.arrivalCircleRadius", out, UnitTransform::NmiToM);
        case ModelField::RouteWaypointArrivalId: return map_text(model.route.waypoint_arrival.waypoint_id, "navigation.courseGreatCircle.nextPoint.id", out);
        case ModelField::TrawlHeadropeToFootropeM: return map_number(model.trawl.headrope_to_footrope_m, "environment.trawl.headropeToFootrope", out);
        case ModelField::TrawlHeadropeToBottomM: return map_number(model.trawl.headrope_to_bottom_m, "environment.trawl.headropeToBottom", out);
        case ModelField::TrawlDoorCenterlineOffsetM: return map_number(model.trawl.door_centerline_offset_m, "environment.trawl.door.centerlineOffset", out);
        case ModelField::TrawlDoorAlongCenterlineM: return map_number(model.trawl.door_along_centerline_m, "environment.trawl.door.alongCenterline", out);
        case ModelField::TrawlCartesianCenterlineOffsetM: return map_number(model.trawl.cartesian_centerline_offset_m, "environment.trawl.cartesian.centerlineOffset", out);
        case ModelField::TrawlCartesianAlongCenterlineM: return map_number(model.trawl.cartesian_along_centerline_m, "environment.trawl.cartesian.alongCenterline", out);
        case ModelField::TrawlDepthBelowSurfaceM: return map_number(model.trawl.depth_below_surface_m, "environment.trawl.depthBelowSurface", out);
        case ModelField::TrawlRelativeRangeM: return map_number(model.trawl.relative_range_m, "environment.trawl.relative.range", out);
        case ModelField::TrawlRelativeBearingDeg: return map_number(model.trawl.relative_bearing_deg, "environment.trawl.relative.bearing", out, UnitTransform::DegToRad);
        case ModelField::TrawlTrueRangeM: return map_number(model.trawl.true_range_m, "environment.trawl.true.range", out);
        case ModelField::TrawlTrueBearingDeg: return map_number(model.trawl.true_bearing_deg, "environment.trawl.true.bearing", out, UnitTransform::DegToRad);
        case ModelField::AisTargetsObject: return map_object("navigation.ais.targets", SignalKObjectKind::AisTargets, out);
        case ModelField::AisOwnVesselObject: return map_object("navigation.ais.ownVessel", SignalKObjectKind::AisOwnVessel, out);
        case ModelField::AisSafetyObject: return map_object("notifications.ais.safety", SignalKObjectKind::AisSafety, out);
        case ModelField::AisDataLinkStatusObject: return map_object("navigation.ais.dataLinkStatus", SignalKObjectKind::AisDataLinkStatus, out);
        case ModelField::DscStructuredNotification: return map_object("notifications.dsc", SignalKObjectKind::Dsc, out);
        case ModelField::InmarsatSafetyNetStructuredNotification: return map_object("notifications.inmarsat.safetynet", SignalKObjectKind::InmarsatSafetyNet, out);
        case ModelField::NavtexStructuredNotification: return map_object("notifications.navtex", SignalKObjectKind::Navtex, out);
        case ModelField::AlertStructuredNotification: return map_object("notifications.alerts", SignalKObjectKind::Alerts, out);
        case ModelField::MobStructuredNotification: return map_object("notifications.mob", SignalKObjectKind::Mob, out);
        case ModelField::LegacyCommObject: return map_object("communication.legacy", SignalKObjectKind::LegacyComm, out);
        case ModelField::NotificationText: return map_text(model.notifications.messages.text.text[0] ? model.notifications.messages.text.text : model.notifications.messages.text.value, "notifications.message.text", out);
        case ModelField::NotificationEvent: return map_text(model.notifications.messages.event.event_text, "notifications.message.event", out);
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

    template<typename StampedReal>
    bool map_number(const StampedReal& stamped, const char* path, SignalKMappedValue<Real>& out, UnitTransform transform = UnitTransform::None) const {
        if (!stamped.valid) return false;
        out.path = path;
        out.kind = SignalKMappedValueKind::Number;
        out.number = transform_number(static_cast<Real>(stamped.value), transform);
        return true;
    }

    template<typename StampedInt>
    bool map_int(const StampedInt& stamped, const char* path, SignalKMappedValue<Real>& out) const {
        if (!stamped.valid) return false;
        out.path = path;
        out.kind = SignalKMappedValueKind::Number;
        out.number = static_cast<Real>(stamped.value);
        return true;
    }

    template<typename StampedUInt>
    bool map_uint(const StampedUInt& stamped, const char* path, SignalKMappedValue<Real>& out) const {
        if (!stamped.valid) return false;
        out.path = path;
        out.kind = SignalKMappedValueKind::Number;
        out.number = static_cast<Real>(stamped.value);
        return true;
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

    bool map_object(const char* path, SignalKObjectKind kind, SignalKMappedValue<Real>& out) const {
        out.path = path;
        out.kind = SignalKMappedValueKind::Object;
        out.object_kind = kind;
        return true;
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
