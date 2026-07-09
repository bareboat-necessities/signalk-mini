#pragma once

namespace signalk_mini::signalk_path {

inline constexpr char NavigationPositionLatitude[] = "navigation.position.value.latitude";
inline constexpr char NavigationPositionLongitude[] = "navigation.position.value.longitude";
inline constexpr char NavigationSpeedOverGround[] = "navigation.speedOverGround";
inline constexpr char NavigationCourseOverGroundTrue[] = "navigation.courseOverGroundTrue";
inline constexpr char NavigationHeadingTrue[] = "navigation.headingTrue";
inline constexpr char NavigationHeadingMagnetic[] = "navigation.headingMagnetic";
inline constexpr char NavigationSpeedThroughWater[] = "navigation.speedThroughWater";
inline constexpr char EnvironmentWindAngleApparent[] = "environment.wind.angleApparent";
inline constexpr char EnvironmentWindSpeedApparent[] = "environment.wind.speedApparent";
inline constexpr char EnvironmentWindDirectionTrue[] = "environment.wind.directionTrue";
inline constexpr char EnvironmentWindSpeedTrue[] = "environment.wind.speedTrue";
inline constexpr char EnvironmentWaterTemperature[] = "environment.water.temperature";
inline constexpr char EnvironmentDepthBelowTransducer[] = "environment.depth.belowTransducer";
inline constexpr char EnvironmentDepthBelowKeel[] = "environment.depth.belowKeel";
inline constexpr char EnvironmentDepthBelowSurface[] = "environment.depth.belowSurface";
inline constexpr char EnvironmentOutsidePressure[] = "environment.outside.pressure";
inline constexpr char EnvironmentOutsideTemperature[] = "environment.outside.temperature";
inline constexpr char SteeringRudderAngle[] = "steering.rudderAngle";
inline constexpr char CommunicationServerClock[] = "communication.server.clock";

} // namespace signalk_mini::signalk_path
