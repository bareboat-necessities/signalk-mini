#include <cstring>
#include <signalk_mini/units.hpp>

int main() {
    char timestamp[32];
    if (!signalk_mini::format_signalk_timestamp_utc(timestamp, sizeof(timestamp), 0)) return 1;
    return std::strcmp(timestamp, "1970-01-01T00:00:00.000Z") == 0 ? 0 : 2;
}
