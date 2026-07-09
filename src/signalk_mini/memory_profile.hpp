#pragma once

#include <stddef.h>
#include <stdint.h>

namespace signalk_mini {

#if defined(ARDUINO)
static constexpr size_t DefaultModelChangeQueueCapacity = 128;
static constexpr size_t DefaultSignalKJsonBufferSize = 384;
static constexpr size_t DefaultSignalKBatchValues = 8;
static constexpr uint16_t DefaultMaxChangesPerTick = 16;
static constexpr uint16_t DefaultSignalKMaxConnections = 4;
#else
static constexpr size_t DefaultModelChangeQueueCapacity = 512;
static constexpr size_t DefaultSignalKJsonBufferSize = 1024;
static constexpr size_t DefaultSignalKBatchValues = 32;
static constexpr uint16_t DefaultMaxChangesPerTick = 32;
static constexpr uint16_t DefaultSignalKMaxConnections = 8;
#endif

static constexpr size_t MaxSupportedModelChangeQueueCapacity = 65535;

} // namespace signalk_mini
