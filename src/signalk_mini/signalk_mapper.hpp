#pragma once

#include <stdint.h>
#include <ship_data_model.hpp>
#include "model_store.hpp"
#include "units.hpp"

namespace signalk_mini {

enum class SignalKMappedValueKind : uint8_t { Number, Bool, Text };

template<typename Real>
struct SignalKMappedValue {
    const char* path = nullptr;
    SignalKMappedValueKind kind = SignalKMappedValueKind::Number;
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
        case ModelField::GnssSatellitesInView: return map_int(model.gnss.satellites_in_view.satellites_in_view, "navigation.gnss.satellitesInView", out);
        case ModelField::GnssSatellitePrn0: return map_int(model.gnss.satellites_in_view.satellite_prn[0], "navigation.gnss.satellite.0.prn", out);
        case ModelField::GnssSatelliteElevationDeg0: return map_number(model.gnss.satellites_in_view.elevation_deg[0], "navigation.gnss.satellite.0.elevation", out, UnitTransform::DegToRad);
        case ModelField::GnssSatelliteAzimuthDeg0: return map_number(model.gnss.satellites_in_view.azimuth_true_deg[0], "navigation.gnss.satellite.0.azimuth", out, UnitTransform::DegToRad);
        case ModelField::GnssSatelliteSnrDb0: return map_number(model.gnss.satellites_in_view.snr_db[0], "navigation.gnss.satellite.0.snr", out);
        case ModelField::ImuHeadingDeg: return map_number(model.ins.imu.heading_deg, "navigation.headingTrue", out, UnitTransform::DegToRad);
        case ModelField::ImuHeadingMagneticDeg: return map_number(model.ins.imu.heading_magnetic_deg, "navigation.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::ImuMagneticVariationDeg: return map_number(model.ins.imu.magnetic_variation_deg, "navigation.magneticVariation", out, UnitTransform::DegToRad);
        case ModelField::WindApparentDirectionDeg: return map_number(model.wind.apparent.direction_deg, "environment.wind.angleApparent", out, UnitTransform::DegToRad);
        case ModelField::WindApparentSpeedKn: return map_number(model.wind.apparent.speed_kn, "environment.wind.speedApparent", out, UnitTransform::KnotsToMps);
        case ModelField::SeaSpeedKn: return map_number(model.sea.speed_kn, "navigation.speedThroughWater", out, UnitTransform::KnotsToMps);
        case ModelField::SeaLongitudinalWaterSpeedKn: return map_number(model.sea.longitudinal_water_speed_kn, "navigation.speedThroughWaterLongitudinal", out, UnitTransform::KnotsToMps);
        case ModelField::SeaLongitudinalGroundSpeedKn: return map_number(model.sea.longitudinal_ground_speed_kn, "navigation.speedOverGroundLongitudinal", out, UnitTransform::KnotsToMps);
        case ModelField::SeaTemperatureC: return map_number(model.sea.temperature_c, "environment.water.temperature", out, UnitTransform::CelsiusToKelvin);
        case ModelField::SeaDepthM: return map_number(model.sea.depth_m, "environment.depth.belowTransducer", out);
        case ModelField::SeaDepthBelowKeelM: return map_number(model.sea.depth_below_keel_m, "environment.depth.belowKeel", out);
        case ModelField::SeaDepthBelowSurfaceM: return map_number(model.sea.depth_below_surface_m, "environment.depth.belowSurface", out);
        case ModelField::SteeringRudderAngleDeg: return map_number(model.steering.rudder.angle_deg, "steering.rudderAngle", out, UnitTransform::DegToRad);
        case ModelField::PropulsionRevolutionsNumber: return map_int(model.propulsion.revolutions.number, "propulsion.main.seatalkEngineId", out);
        case ModelField::PropulsionRevolutionsSpeedRpm: return map_number(model.propulsion.revolutions.speed_rpm, "propulsion.main.revolutions", out, UnitTransform::RpmToHz);
        case ModelField::PropulsionRevolutionsPitchPercent: return map_number(model.propulsion.revolutions.propeller_pitch_percent, "propulsion.main.pitch", out, UnitTransform::PercentToRatio);
        case ModelField::AutopilotMode: return map_text(signalk_autopilot_mode(model.autopilot.controller.mode.value), "steering.autopilot.mode", out);
        case ModelField::AutopilotEnabled: return map_bool(model.autopilot.controller.enabled.value, "steering.autopilot.enabled", out);
        case ModelField::AutopilotHeadingDeg: return map_number(model.autopilot.controller.heading_deg, "steering.autopilot.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::AutopilotHeadingCommandDeg: return map_number(model.autopilot.controller.heading_command_deg, "steering.autopilot.target.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::AutopilotWarnings: return map_uint(model.autopilot.status.warnings, "steering.autopilot.warnings", out);
        case ModelField::RouteLogTotalDistanceNmi: return map_number(model.route.log.total_distance_nmi, "navigation.log", out, UnitTransform::NmiToM);
        case ModelField::RouteLogTripDistanceNmi: return map_number(model.route.log.trip_distance_nmi, "navigation.trip.log", out, UnitTransform::NmiToM);
        case ModelField::RouteApbXteNmi: return map_number(model.route.apb.xte_nmi, "navigation.courseGreatCircle.crossTrackError", out, UnitTransform::NmiToM);
        case ModelField::RouteApbOriginToDestinationBearingDeg: return map_number(model.route.apb.origin_to_destination_bearing_deg, "navigation.courseGreatCircle.bearingTrackTrue", out, UnitTransform::DegToRad);
        case ModelField::RouteApbPresentToDestinationBearingDeg: return map_number(model.route.apb.present_to_destination_bearing_deg, "navigation.courseGreatCircle.bearingTrue", out, UnitTransform::DegToRad);
        case ModelField::RouteApbHeadingToSteerDeg: return map_number(model.route.apb.heading_to_steer_deg, "steering.autopilot.target.headingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::RouteApbArrivalCircleEntered: return map_bool(model.route.apb.arrival_circle_entered.value, "navigation.courseGreatCircle.arrivalCircleEntered", out);
        case ModelField::RouteApbPerpendicularPassed: return map_bool(model.route.apb.perpendicular_passed.value, "navigation.courseGreatCircle.perpendicularPassed", out);
        case ModelField::RouteApbDestinationId: return map_text(model.route.apb.destination_id, "navigation.courseGreatCircle.nextPoint.id", out);
        case ModelField::RouteRmbRangeNmi: return map_number(model.route.rmb.range_nmi, "navigation.courseGreatCircle.nextPoint.distance", out, UnitTransform::NmiToM);
        case ModelField::RouteRmbArrived: return map_bool(model.route.rmb.arrived.value, "navigation.courseGreatCircle.arrived", out);
        case ModelField::RouteRmbDestinationId: return map_text(model.route.rmb.destination_id, "navigation.courseGreatCircle.nextPoint.id", out);
        case ModelField::RouteWaypointBearingMagneticDeg: return map_number(model.route.waypoint.bearing_magnetic_deg, "navigation.courseGreatCircle.bearingMagnetic", out, UnitTransform::DegToRad);
        case ModelField::RouteWaypointDistanceNmi: return map_number(model.route.waypoint.distance_nmi, "navigation.courseGreatCircle.nextPoint.distance", out, UnitTransform::NmiToM);
        case ModelField::RouteWaypointToId: return map_text(model.route.waypoint.to_waypoint_id, "navigation.courseGreatCircle.nextPoint.id", out);
        case ModelField::RouteWaypointArrivalCircleEntered: return map_bool(model.route.waypoint_arrival.arrival_circle_entered.value, "navigation.courseGreatCircle.arrivalCircleEntered", out);
        case ModelField::RouteWaypointPerpendicularPassed: return map_bool(model.route.waypoint_arrival.perpendicular_passed.value, "navigation.courseGreatCircle.perpendicularPassed", out);
        case ModelField::RouteWaypointArrivalId: return map_text(model.route.waypoint_arrival.waypoint_id, "navigation.courseGreatCircle.nextPoint.id", out);
        case ModelField::NotificationText: return map_text(model.notifications.messages.text.text[0] ? model.notifications.messages.text.text : model.notifications.messages.text.value, "notifications.seatalk.message", out);
        case ModelField::NotificationEvent: return map_text(model.notifications.messages.event.event_text, "notifications.seatalk.event", out);
        default: return false;
        }
    }

private:
    enum class UnitTransform : uint8_t { None, DegToRad, KnotsToMps, NmiToM, CelsiusToKelvin, RpmToHz, PercentToRatio };

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

    Real transform_number(Real value, UnitTransform transform) const {
        switch (transform) {
        case UnitTransform::None: return value;
        case UnitTransform::DegToRad: return deg_to_rad<Real>(value);
        case UnitTransform::KnotsToMps: return knots_to_mps<Real>(value);
        case UnitTransform::NmiToM: return nmi_to_m<Real>(value);
        case UnitTransform::CelsiusToKelvin: return celsius_to_kelvin<Real>(value);
        case UnitTransform::RpmToHz: return rpm_to_hz<Real>(value);
        case UnitTransform::PercentToRatio: return value / static_cast<Real>(100);
        }
        return value;
    }
};

} // namespace signalk_mini
