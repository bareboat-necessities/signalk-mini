#pragma once

#include "trawl_data.hpp"

namespace ship_data_model {

template<typename Real = float>
struct FishingData {
    TrawlData<Real> trawl;
};

} // namespace ship_data_model
