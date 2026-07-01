#pragma once

#include <gpiod.h>
#include <stdint.h>
#include <stdlib.h>

#include "async_event_loop/pin_io.hpp"

#ifndef ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION
#define ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION 1
#endif

namespace async_event_loop {

/**
 * Linux GPIO edge-event source backed by libgpiod.
 *
 * CMake selects libgpiod v2 when available and falls back to the v1.6 event API
 * otherwise. v2 supports kernel debounce configuration; v1.6 ignores the
 * debounce_us argument because that API has no equivalent per-line setting.
 */
class LinuxGpiodPinEventSource final : public IPinEventSource {
public:
    LinuxGpiodPinEventSource(const char* chip_path,
                             unsigned int line_offset,
                             PinEventType event_type,
                             const char* consumer = "pypilot-event-loop",
                             unsigned long debounce_us = 0,
                             size_t event_buffer_capacity = 16)
        : line_offset_(line_offset), requested_type_(event_type) {
        open(chip_path, consumer, debounce_us, event_buffer_capacity);
    }

    LinuxGpiodPinEventSource(const LinuxGpiodPinEventSource&) = delete;
    LinuxGpiodPinEventSource& operator=(const LinuxGpiodPinEventSource&) = delete;

    ~LinuxGpiodPinEventSource() override { close(); }

    bool valid() const override {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        return request_ != nullptr && buffer_ != nullptr;
#else
        return chip_ != nullptr && line_ != nullptr;
#endif
    }

    int native_fd() const override {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        if (!request_) {
            return -1;
        }
        return gpiod_line_request_get_fd(request_);
#else
        if (!line_) {
            return -1;
        }
        return gpiod_line_event_get_fd(line_);
#endif
    }

    bool read_event(PinEvent& event) override {
        if (!valid()) {
            return false;
        }

#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        const int n = gpiod_line_request_read_edge_events(request_, buffer_, 1);
        if (n <= 0) {
            return false;
        }

        struct gpiod_edge_event* gevent = gpiod_edge_event_buffer_get_event(buffer_, 0);
        if (!gevent) {
            return false;
        }

        const enum gpiod_edge_event_type type = gpiod_edge_event_get_event_type(gevent);
        event.type = (type == GPIOD_EDGE_EVENT_RISING_EDGE) ? PinEventType::RisingEdge
                                                            : PinEventType::FallingEdge;
        event.timestamp_us = gpiod_edge_event_get_timestamp_ns(gevent) / 1000ULL;
        event.source_id = gpiod_edge_event_get_line_offset(gevent);
        event.sequence = static_cast<uint32_t>(gpiod_edge_event_get_global_seqno(gevent));
        event.level = event.type == PinEventType::RisingEdge;
        return true;
#else
        struct gpiod_line_event gevent;
        const int rc = gpiod_line_event_read(line_, &gevent);
        if (rc != 0) {
            return false;
        }
        event.type = (gevent.event_type == GPIOD_LINE_EVENT_RISING_EDGE) ? PinEventType::RisingEdge
                                                                         : PinEventType::FallingEdge;
        event.timestamp_us = static_cast<uint64_t>(gevent.ts.tv_sec) * 1000000ULL +
                             static_cast<uint64_t>(gevent.ts.tv_nsec) / 1000ULL;
        event.source_id = line_offset_;
        event.sequence = ++sequence_;
        event.level = event.type == PinEventType::RisingEdge;
        return true;
#endif
    }

    unsigned int line_offset() const { return line_offset_; }
    PinEventType requested_type() const { return requested_type_; }

private:
    void close() {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        if (buffer_) {
            gpiod_edge_event_buffer_free(buffer_);
            buffer_ = nullptr;
        }
        if (request_) {
            gpiod_line_request_release(request_);
            request_ = nullptr;
        }
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
#else
        if (line_) {
            gpiod_line_release(line_);
            line_ = nullptr;
        }
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
#endif
    }

#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
    static enum gpiod_line_edge to_gpiod_edge(PinEventType type) {
        switch (type) {
        case PinEventType::RisingEdge:
            return GPIOD_LINE_EDGE_RISING;
        case PinEventType::FallingEdge:
            return GPIOD_LINE_EDGE_FALLING;
        case PinEventType::Change:
        case PinEventType::HighLevel:
        case PinEventType::LowLevel:
            return GPIOD_LINE_EDGE_BOTH;
        }
        return GPIOD_LINE_EDGE_BOTH;
    }
#else
    static int request_edge_events_v1(struct gpiod_line* line, PinEventType type, const char* consumer) {
        switch (type) {
        case PinEventType::RisingEdge:
            return gpiod_line_request_rising_edge_events(line, consumer);
        case PinEventType::FallingEdge:
            return gpiod_line_request_falling_edge_events(line, consumer);
        case PinEventType::Change:
        case PinEventType::HighLevel:
        case PinEventType::LowLevel:
            return gpiod_line_request_both_edges_events(line, consumer);
        }
        return gpiod_line_request_both_edges_events(line, consumer);
    }
#endif

    void open(const char* chip_path,
              const char* consumer,
              unsigned long debounce_us,
              size_t event_buffer_capacity) {
        chip_ = gpiod_chip_open(chip_path);
        if (!chip_) {
            return;
        }

#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        struct gpiod_line_settings* settings = gpiod_line_settings_new();
        struct gpiod_line_config* line_config = gpiod_line_config_new();
        struct gpiod_request_config* request_config = gpiod_request_config_new();
        if (!settings || !line_config || !request_config) {
            cleanup_setup(settings, line_config, request_config);
            return;
        }

        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_edge_detection(settings, to_gpiod_edge(requested_type_));
        if (debounce_us != 0) {
            gpiod_line_settings_set_debounce_period_us(settings, debounce_us);
        }

        unsigned int offset = line_offset_;
        if (gpiod_line_config_add_line_settings(line_config, &offset, 1, settings) != 0) {
            cleanup_setup(settings, line_config, request_config);
            return;
        }

        gpiod_request_config_set_consumer(request_config, consumer ? consumer : "pypilot-event-loop");
        request_ = gpiod_chip_request_lines(chip_, request_config, line_config);
        cleanup_setup(settings, line_config, request_config);

        if (!request_) {
            return;
        }

        buffer_ = gpiod_edge_event_buffer_new(event_buffer_capacity);
#else
        (void)debounce_us;
        (void)event_buffer_capacity;
        line_ = gpiod_chip_get_line(chip_, line_offset_);
        if (!line_) {
            close();
            return;
        }
        if (request_edge_events_v1(line_, requested_type_, consumer ? consumer : "pypilot-event-loop") != 0) {
            close();
        }
#endif
    }

#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
    static void cleanup_setup(struct gpiod_line_settings* settings,
                              struct gpiod_line_config* line_config,
                              struct gpiod_request_config* request_config) {
        if (request_config) {
            gpiod_request_config_free(request_config);
        }
        if (line_config) {
            gpiod_line_config_free(line_config);
        }
        if (settings) {
            gpiod_line_settings_free(settings);
        }
    }
#endif

    struct gpiod_chip* chip_ = nullptr;
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
    struct gpiod_line_request* request_ = nullptr;
    struct gpiod_edge_event_buffer* buffer_ = nullptr;
#else
    struct gpiod_line* line_ = nullptr;
    uint32_t sequence_ = 0;
#endif
    unsigned int line_offset_ = 0;
    PinEventType requested_type_ = PinEventType::Change;
};

} // namespace async_event_loop
