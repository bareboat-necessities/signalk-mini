#pragma once

namespace signalk_mini {

template<typename Real>
constexpr Real pi_v() { return static_cast<Real>(3.1415926535897932384626433832795L); }

template<typename Real>
constexpr Real deg_to_rad(Real deg) { return deg * pi_v<Real>() / static_cast<Real>(180); }

template<typename Real>
constexpr Real knots_to_mps(Real knots) { return knots * static_cast<Real>(0.51444444444444444444L); }

} // namespace signalk_mini
