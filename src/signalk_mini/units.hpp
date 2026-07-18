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

inline bool format_signalk_datetime_utc(char* dst,
                                        size_t dst_size,
                                        int32_t year,
                                        int32_t month,
                                        int32_t day,
                                        double seconds_of_day) {
    if (!dst || dst_size < 25 || year < 1 || month < 1 || month > 12 || day < 1 || day > 31 ||
        seconds_of_day < 0.0 || seconds_of_day >= 86401.0) return false;
    uint32_t millis_of_day = static_cast<uint32_t>(seconds_of_day * 1000.0 + 0.5);
    if (millis_of_day >= 86400000u) millis_of_day = 86399999u;
    const uint32_t hour = millis_of_day / 3600000u;
    millis_of_day %= 3600000u;
    const uint32_t minute = millis_of_day / 60000u;
    millis_of_day %= 60000u;
    const uint32_t second = millis_of_day / 1000u;
    const uint32_t millis = millis_of_day % 1000u;
    const int len = snprintf(dst, dst_size, "%04ld-%02ld-%02ldT%02lu:%02lu:%02lu.%03luZ",
                             static_cast<long>(year), static_cast<long>(month), static_cast<long>(day),
                             static_cast<unsigned long>(hour), static_cast<unsigned long>(minute),
                             static_cast<unsigned long>(second), static_cast<unsigned long>(millis));
    return len == 24 && static_cast<size_t>(len) < dst_size;
}

} // namespace signalk_mini
