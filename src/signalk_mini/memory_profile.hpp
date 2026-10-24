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
// REST responses that did not fit the transmit buffer and are being resumed.
static constexpr size_t DefaultMaxResumableRestResponses = 1;
// Latest values kept for paths that more than one source reports.
static constexpr size_t DefaultSourceValueCapacity = 8;
#else
static constexpr size_t DefaultModelChangeQueueCapacity = 512;
static constexpr size_t DefaultSignalKJsonBufferSize = 1024;
static constexpr size_t DefaultSignalKBatchValues = 32;
static constexpr uint16_t DefaultMaxChangesPerTick = 32;
static constexpr uint16_t DefaultSignalKMaxConnections = 8;
static constexpr size_t DefaultMaxResumableRestResponses = 4;
static constexpr size_t DefaultSourceValueCapacity = 32;
#endif

static constexpr size_t MaxSupportedModelChangeQueueCapacity = 65535;

} // namespace signalk_mini
