#pragma once

#include <stddef.h>
#include <stdio.h>
#include <time.h>

namespace signalk_mini {

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
