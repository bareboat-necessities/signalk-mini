#pragma once

#include <stddef.h>
#include <stdio.h>
#include <time.h>

namespace signalk_mini {

template<typename Real>
constexpr Real pi_v() { return static_cast<Real>(3.1415926535897932384626433832795L); }

template<typename Real>
constexpr Real deg_to_rad(Real deg) { return deg * pi_v<Real>() / static_cast<Real>(180); }

template<typename Real>
constexpr Real knots_to_mps(Real knots) { return knots * static_cast<Real>(0.51444444444444444444L); }

template<typename Real>
constexpr Real nmi_to_m(Real nmi) { return nmi * static_cast<Real>(1852); }

template<typename Real>
constexpr Real celsius_to_kelvin(Real celsius) { return celsius + static_cast<Real>(273.15); }

template<typename Real>
constexpr Real rpm_to_hz(Real rpm) { return rpm / static_cast<Real>(60); }

template<typename Real>
constexpr Real bar_to_pa(Real bar) { return bar * static_cast<Real>(100000); }

template<typename Real>
constexpr Real inhg_to_pa(Real inhg) { return inhg * static_cast<Real>(3386.389); }

template<typename Real>
constexpr Real percent_to_ratio(Real percent) { return percent / static_cast<Real>(100); }

inline bool format_signalk_timestamp_utc(char* dst,
                                         size_t dst_size,
                                         time_t epoch_seconds = time(nullptr)) {
    if (!dst || dst_size < 25) return false;

    struct tm utc{};
#if defined(_WIN32)
    if (gmtime_s(&utc, &epoch_seconds) != 0) return false;
#else
    if (!gmtime_r(&epoch_seconds, &utc)) return false;
#endif

    const int len = snprintf(dst,
                             dst_size,
                             "%04d-%02d-%02dT%02d:%02d:%02d.000Z",
                             utc.tm_year + 1900,
                             utc.tm_mon + 1,
                             utc.tm_mday,
                             utc.tm_hour,
                             utc.tm_min,
                             utc.tm_sec);
    return len == 24 && static_cast<size_t>(len) < dst_size;
}

} // namespace signalk_mini
