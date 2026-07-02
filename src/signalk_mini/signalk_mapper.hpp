#pragma once

#include <ship_data_model.hpp>
#include "model_store.hpp"
#include "units.hpp"

namespace signalk_mini {

template<typename Real>
struct SignalKMappedValue {
    const char* path = nullptr;
    Real number{};
};

template<typename Real>
class SignalKMapper {
public:
    bool map_change(const ship_data_model::DataModel<Real>& model, const ModelChange& change, SignalKMappedValue<Real>& out) const {
        switch (change.field) {
        case ModelField::GnssFixLatDeg:
            out.path = "navigation.position.value.latitude";
            out.number = model.gnss.fix.fix_lat_deg.value;
            return model.gnss.fix.fix_lat_deg.valid;
        case ModelField::GnssFixLonDeg:
            out.path = "navigation.position.value.longitude";
            out.number = model.gnss.fix.fix_lon_deg.value;
            return model.gnss.fix.fix_lon_deg.valid;
        case ModelField::GnssSpeedKn:
            out.path = "navigation.speedOverGround";
            out.number = knots_to_mps<Real>(model.gnss.fix.speed_kn.value);
            return model.gnss.fix.speed_kn.valid;
        case ModelField::GnssTrackDeg:
            out.path = "navigation.courseOverGroundTrue";
            out.number = deg_to_rad<Real>(model.gnss.fix.track_deg.value);
            return model.gnss.fix.track_deg.valid;
        case ModelField::ImuHeadingDeg:
            out.path = "navigation.headingTrue";
            out.number = deg_to_rad<Real>(model.imu.heading_deg.value);
            return model.imu.heading_deg.valid;
        case ModelField::WindApparentDirectionDeg:
            out.path = "environment.wind.angleApparent";
            out.number = deg_to_rad<Real>(model.wind.apparent.direction_deg.value);
            return model.wind.apparent.direction_deg.valid;
        case ModelField::WindApparentSpeedKn:
            out.path = "environment.wind.speedApparent";
            out.number = knots_to_mps<Real>(model.wind.apparent.speed_kn.value);
            return model.wind.apparent.speed_kn.valid;
        case ModelField::SeaDepthM:
            out.path = "environment.depth.belowTransducer";
            out.number = model.sea.depth_m.value;
            return model.sea.depth_m.valid;
        default:
            return false;
        }
    }
};

} // namespace signalk_mini
