#pragma once

#include <ship_data_model.hpp>

namespace nmea0183_connector {

template<typename Real>
struct NmeaGnssAlmanacWriteView {
    ship_data_model::Setting<ship_data_model::SensorSource>& source;
    ship_data_model::Stamped<int32_t>& total_messages;
    ship_data_model::Stamped<int32_t>& message_number;
    ship_data_model::Stamped<int32_t>& satellite_prn;
    ship_data_model::Stamped<int32_t>& gps_week;
    ship_data_model::Stamped<int32_t>& sv_health;
    ship_data_model::Stamped<Real>& eccentricity;
    ship_data_model::Stamped<Real>& reference_time_s;
    ship_data_model::Stamped<Real>& inclination_rad;
    ship_data_model::Stamped<Real>& right_ascension_rate_rad_s;
    ship_data_model::Stamped<Real>& sqrt_semi_major_axis;
    ship_data_model::Stamped<Real>& argument_of_perigee_rad;
    ship_data_model::Stamped<Real>& longitude_ascension_node_rad;
    ship_data_model::Stamped<Real>& mean_anomaly_rad;
    ship_data_model::Stamped<Real>& clock_f0_s;
    ship_data_model::Stamped<Real>& clock_f1_s_s;
    uint64_t& last_update_us;

    explicit NmeaGnssAlmanacWriteView(ship_data_model::GnssAlmanacData<Real>& data)
        : source(data.source),
          total_messages(data.total_messages),
          message_number(data.message_number),
          satellite_prn(data.satellite_prn),
          gps_week(data.gnss_week),
          sv_health(data.sv_health),
          eccentricity(data.eccentricity),
          reference_time_s(data.reference_time_s),
          inclination_rad(data.inclination_rad),
          right_ascension_rate_rad_s(data.right_ascension_rate_rad_s),
          sqrt_semi_major_axis(data.sqrt_semi_major_axis),
          argument_of_perigee_rad(data.argument_of_perigee_rad),
          longitude_ascension_node_rad(data.longitude_ascension_node_rad),
          mean_anomaly_rad(data.mean_anomaly_rad),
          clock_f0_s(data.clock_f0_s),
          clock_f1_s_s(data.clock_f1_s_s),
          last_update_us(data.last_update_us) {}
};

template<typename Real>
struct NmeaNavigationWriteView {
    ship_data_model::GnssData<Real>& gps;
    NmeaGnssAlmanacWriteView<Real> gps_almanac;
    ship_data_model::GnssFaultDetectionData<Real>& gps_fault;
    ship_data_model::GnssRangeResidualData<Real>& gps_range_residual;
    ship_data_model::GnssNoiseStatisticsData<Real>& gps_noise;
    ship_data_model::GnssDopActiveSatellitesData<Real>& gps_dop;
    ship_data_model::GnssSatellitesInViewData<Real>& gps_satellites_in_view;
    ship_data_model::DatumReferenceData<Real>& datum;
    ship_data_model::DeccaPositionData<Real>& decca;
    ship_data_model::LegacyTimingData<Real>& legacy_timing;
    ship_data_model::LegacyDeltaData<Real>& legacy_delta;
    ship_data_model::TransitFixData<Real>& transit_fix;
    ship_data_model::RadioFrequencySetData<Real>& radio_frequency_set;
    ship_data_model::HeadingSteeringCommandData<Real>& heading_steering_command;
    ship_data_model::BeaconReceiverControlData<Real>& beacon_control;
    ship_data_model::BeaconReceiverStatusData<Real>& beacon_status;
    ship_data_model::ReturnLinkMessageData<Real>& return_link_message;
    ship_data_model::OmegaLaneNumbersData<Real>& omega_lane_numbers;
    ship_data_model::OwnShipData<Real>& own_ship;
    ship_data_model::ActiveRouteData<Real>& active_route;
    ship_data_model::RevolutionsData<Real>& revolutions;
    ship_data_model::RadarSystemData<Real>& radar_system;
    ship_data_model::ScanningFrequencyData<Real>& scanning_frequency;
    ship_data_model::MultipleDataIdData<Real>& multiple_data_id;
    ship_data_model::TrackedTargetData<Real>& tracked_target;
    ship_data_model::RmaData<Real>& rma;
    ship_data_model::WaypointArrivalData<Real>& waypoint_arrival;
    ship_data_model::WaypointNavigationData<Real>& waypoint;
    ship_data_model::ApbData<Real>& apb;
    ship_data_model::RmbData<Real>& rmb;

    explicit NmeaNavigationWriteView(ship_data_model::DataModel<Real>& model)
        : gps(model.gnss.fix),
          gps_almanac(model.gnss.almanac),
          gps_fault(model.gnss.fault_detection),
          gps_range_residual(model.gnss.range_residual),
          gps_noise(model.gnss.noise_statistics),
          gps_dop(model.gnss.dop_active_satellites),
          gps_satellites_in_view(model.gnss.satellites_in_view),
          datum(model.nav.datum),
          decca(model.nav.decca),
          legacy_timing(model.nav.legacy_timing),
          legacy_delta(model.nav.legacy_delta),
          transit_fix(model.nav.transit_fix),
          radio_frequency_set(model.comm.radio_frequency_set),
          heading_steering_command(model.route.heading_steering_command),
          beacon_control(model.comm.beacon_control),
          beacon_status(model.comm.beacon_status),
          return_link_message(model.comm.return_link_message),
          omega_lane_numbers(model.nav.omega_lane_numbers),
          own_ship(model.nav.own_ship),
          active_route(model.route.active),
          revolutions(model.propulsion.revolutions),
          radar_system(model.nav.radar_system),
          scanning_frequency(model.nav.scanning_frequency),
          multiple_data_id(model.nav.multiple_data_id),
          tracked_target(model.ais.tracked_target),
          rma(model.nav.rma),
          waypoint_arrival(model.route.waypoint_arrival),
          waypoint(model.route.waypoint),
          apb(model.route.apb),
          rmb(model.route.rmb) {}
};

template<typename Real>
struct NmeaWaterWriteView {
    ship_data_model::Setting<ship_data_model::SensorSource>& source;
    ship_data_model::Setting<ship_data_model::SensorSource>& leeway_source;
    ship_data_model::Setting<ship_data_model::SensorSource>& depth_source;
    ship_data_model::Stamped<Real>& speed_kn;
    ship_data_model::Stamped<Real>& longitudinal_water_speed_kn;
    ship_data_model::Stamped<Real>& transverse_water_speed_kn;
    char& water_speed_status;
    ship_data_model::Stamped<Real>& longitudinal_ground_speed_kn;
    ship_data_model::Stamped<Real>& transverse_ground_speed_kn;
    char& ground_speed_status;
    ship_data_model::Stamped<Real>& speed_parallel_to_wind_kn;
    ship_data_model::Stamped<Real>& speed_parallel_to_wind_m_s;
    ship_data_model::Stamped<Real>& total_distance_nmi;
    ship_data_model::Stamped<Real>& trip_distance_nmi;
    ship_data_model::Stamped<Real>& leeway_deg;
    ship_data_model::Stamped<Real>& current_speed_kn;
    ship_data_model::Stamped<Real>& current_direction_deg;
    ship_data_model::Stamped<Real>& current_direction_magnetic_deg;
    ship_data_model::Stamped<Real>& wind_speed_kn;
    ship_data_model::Stamped<Real>& wind_speed_m_s;
    ship_data_model::Stamped<Real>& wind_direction_deg;
    ship_data_model::Stamped<Real>& wind_direction_magnetic_deg;
    ship_data_model::Stamped<Real>& barometric_pressure_inhg;
    ship_data_model::Stamped<Real>& barometric_pressure_bar;
    ship_data_model::Stamped<Real>& air_temperature_c;
    ship_data_model::Stamped<Real>& relative_humidity_percent;
    ship_data_model::Stamped<Real>& absolute_humidity_percent;
    ship_data_model::Stamped<Real>& dew_point_c;
    ship_data_model::Stamped<Real>& depth_m;
    ship_data_model::Stamped<Real>& depth_below_keel_m;
    ship_data_model::Stamped<Real>& depth_below_surface_m;
    ship_data_model::Stamped<Real>& depth_offset_m;
    ship_data_model::Stamped<Real>& temperature_c;
    ship_data_model::Stamped<Real>& trawl_headrope_to_footrope_m;
    ship_data_model::Stamped<Real>& trawl_headrope_to_bottom_m;
    ship_data_model::Stamped<Real>& trawl_door_centerline_offset_m;
    ship_data_model::Stamped<Real>& trawl_door_along_centerline_m;
    ship_data_model::Stamped<Real>& trawl_cartesian_centerline_offset_m;
    ship_data_model::Stamped<Real>& trawl_cartesian_along_centerline_m;
    ship_data_model::Stamped<int32_t> (&trawl_catch_sensor_status)[3];
    ship_data_model::Stamped<Real>& trawl_depth_below_surface_m;
    ship_data_model::Stamped<Real>& trawl_relative_range_m;
    ship_data_model::Stamped<Real>& trawl_relative_bearing_deg;
    ship_data_model::Stamped<Real>& trawl_true_range_m;
    ship_data_model::Stamped<Real>& trawl_true_bearing_deg;
    uint64_t& last_update_us;

    explicit NmeaWaterWriteView(ship_data_model::DataModel<Real>& model)
        : source(model.sea.source),
          leeway_source(model.sea.leeway_source),
          depth_source(model.sea.depth_source),
          speed_kn(model.sea.speed_kn),
          longitudinal_water_speed_kn(model.sea.longitudinal_water_speed_kn),
          transverse_water_speed_kn(model.sea.transverse_water_speed_kn),
          water_speed_status(model.sea.water_speed_status),
          longitudinal_ground_speed_kn(model.sea.longitudinal_ground_speed_kn),
          transverse_ground_speed_kn(model.sea.transverse_ground_speed_kn),
          ground_speed_status(model.sea.ground_speed_status),
          speed_parallel_to_wind_kn(model.sea.speed_parallel_to_wind_kn),
          speed_parallel_to_wind_m_s(model.sea.speed_parallel_to_wind_m_s),
          total_distance_nmi(model.sea.total_distance_nmi),
          trip_distance_nmi(model.sea.trip_distance_nmi),
          leeway_deg(model.sea.leeway_deg),
          current_speed_kn(model.sea.current_speed_kn),
          current_direction_deg(model.sea.current_direction_deg),
          current_direction_magnetic_deg(model.sea.current_direction_magnetic_deg),
          wind_speed_kn(model.wind.surface.speed_kn),
          wind_speed_m_s(model.wind.surface.speed_m_s),
          wind_direction_deg(model.wind.surface.direction_deg),
          wind_direction_magnetic_deg(model.wind.surface.direction_magnetic_deg),
          barometric_pressure_inhg(model.env.barometric_pressure_inhg),
          barometric_pressure_bar(model.env.barometric_pressure_bar),
          air_temperature_c(model.env.air_temperature_c),
          relative_humidity_percent(model.env.relative_humidity_percent),
          absolute_humidity_percent(model.env.absolute_humidity_percent),
          dew_point_c(model.env.dew_point_c),
          depth_m(model.sea.depth_m),
          depth_below_keel_m(model.sea.depth_below_keel_m),
          depth_below_surface_m(model.sea.depth_below_surface_m),
          depth_offset_m(model.sea.depth_offset_m),
          temperature_c(model.sea.temperature_c),
          trawl_headrope_to_footrope_m(model.sea.trawl_headrope_to_footrope_m),
          trawl_headrope_to_bottom_m(model.sea.trawl_headrope_to_bottom_m),
          trawl_door_centerline_offset_m(model.sea.trawl_door_centerline_offset_m),
          trawl_door_along_centerline_m(model.sea.trawl_door_along_centerline_m),
          trawl_cartesian_centerline_offset_m(model.sea.trawl_cartesian_centerline_offset_m),
          trawl_cartesian_along_centerline_m(model.sea.trawl_cartesian_along_centerline_m),
          trawl_catch_sensor_status(model.sea.trawl_catch_sensor_status),
          trawl_depth_below_surface_m(model.sea.trawl_depth_below_surface_m),
          trawl_relative_range_m(model.sea.trawl_relative_range_m),
          trawl_relative_bearing_deg(model.sea.trawl_relative_bearing_deg),
          trawl_true_range_m(model.sea.trawl_true_range_m),
          trawl_true_bearing_deg(model.sea.trawl_true_bearing_deg),
          last_update_us(model.sea.last_update_us) {}
};

template<typename Real>
struct NmeaModelWriteView {
    NmeaNavigationWriteView<Real> navigation;
    NmeaWaterWriteView<Real> water;
    ship_data_model::WindData<Real>& wind;
    ship_data_model::ShipImuData<Real>& imu;
    ship_data_model::RudderData<Real>& rudder;

    explicit NmeaModelWriteView(ship_data_model::DataModel<Real>& model)
        : navigation(model), water(model), wind(model.wind), imu(model.imu), rudder(model.steering.rudder) {}
};

} // namespace nmea0183_connector
