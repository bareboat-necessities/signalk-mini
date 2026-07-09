#pragma once

#include <stddef.h>

namespace signalk_mini {

#if defined(ARDUINO)
static constexpr size_t DefaultModelChangeQueueCapacity = 256;
static constexpr size_t DefaultSignalKJsonBufferSize = 512;
static constexpr size_t DefaultSignalKBatchValues = 16;
#else
static constexpr size_t DefaultModelChangeQueueCapacity = 512;
static constexpr size_t DefaultSignalKJsonBufferSize = 1024;
static constexpr size_t DefaultSignalKBatchValues = 32;
#endif

static constexpr size_t MaxSupportedModelChangeQueueCapacity = 65535;

} // namespace signalk_mini
