#pragma once
#include <stdint.h>
namespace ship_data_model{
enum class FieldId:uint16_t{Invalid,NavigationLatitudeDeg,NavigationLongitudeDeg,NavigationSogMps,NavigationCogDeg,NavigationHeadingTrueDeg,WindApparentAngleDeg,WindApparentSpeedMps,WaterDepthBelowTransducerM};
template<typename Real>struct Value{bool valid=false;Real value{};uint64_t timestamp_us=0;uint16_t source_id=0;void set(Real v,uint64_t t,uint16_t s){valid=true;value=v;timestamp_us=t;source_id=s;}};
template<typename Real>struct DataModel{struct{Value<Real>latitude_deg,longitude_deg,sog_mps,cog_deg,heading_true_deg;}navigation;struct{Value<Real>apparent_angle_deg,apparent_speed_mps;}wind;struct{Value<Real>depth_below_transducer_m;}water;};}
