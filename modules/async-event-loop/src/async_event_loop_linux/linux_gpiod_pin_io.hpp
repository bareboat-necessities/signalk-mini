#pragma once

#if defined(__linux__) && defined(ASYNC_EVENT_LOOP_ENABLE_LINUX_GPIOD_PIN_IO)
#include <gpiod.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "async_event_loop/pin_io.hpp"

#ifndef ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION
#define ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION 1
#endif

namespace async_event_loop {

namespace detail {

inline bool gpiod_chip_path(const char* chip_name, char* dst, size_t dst_size) {
    if (!chip_name || !*chip_name || !dst || dst_size == 0) {
        return false;
    }
    if (chip_name[0] == '/') {
        snprintf(dst, dst_size, "%s", chip_name);
    } else {
        snprintf(dst, dst_size, "/dev/%s", chip_name);
    }
    return true;
}

#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
inline struct gpiod_line_request* request_one_gpiod_line_v2(const char* chip_name,
                                                             unsigned int line_offset,
                                                             enum gpiod_line_direction direction,
                                                             enum gpiod_line_value output_value,
                                                             const char* consumer) {
    char chip_path[128];
    if (!gpiod_chip_path(chip_name, chip_path, sizeof(chip_path))) {
        return nullptr;
    }

    struct gpiod_chip* chip = gpiod_chip_open(chip_path);
    if (!chip) {
        return nullptr;
    }

    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    struct gpiod_line_config* line_config = gpiod_line_config_new();
    struct gpiod_request_config* request_config = gpiod_request_config_new();
    struct gpiod_line_request* request = nullptr;

    if (!settings || !line_config || !request_config) {
        goto cleanup;
    }

    gpiod_line_settings_set_direction(settings, direction);
    if (direction == GPIOD_LINE_DIRECTION_OUTPUT) {
        gpiod_line_settings_set_output_value(settings, output_value);
    }

    if (gpiod_line_config_add_line_settings(line_config, &line_offset, 1, settings) != 0) {
        goto cleanup;
    }

    gpiod_request_config_set_consumer(request_config, consumer ? consumer : "async_event_loop");
    request = gpiod_chip_request_lines(chip, request_config, line_config);

cleanup:
    if (request_config) {
        gpiod_request_config_free(request_config);
    }
    if (line_config) {
        gpiod_line_config_free(line_config);
    }
    if (settings) {
        gpiod_line_settings_free(settings);
    }
    gpiod_chip_close(chip);
    return request;
}
#endif

} // namespace detail

/**
 * Linux libgpiod digital input pin.
 *
 * CMake selects libgpiod v2 when available and falls back to the v1.6 line API
 * otherwise. The chip argument accepts either a full device path such as
 * `/dev/gpiochip0` or a chip name such as `gpiochip0`.
 */
class LinuxGpiodDigitalInputPin final : public IDigitalInputPin {
public:
    LinuxGpiodDigitalInputPin(const char* chip_name,
                              unsigned int line_offset,
                              const char* consumer = "async_event_loop")
        : line_offset_(line_offset) {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        request_ = detail::request_one_gpiod_line_v2(chip_name,
                                                     line_offset_,
                                                     GPIOD_LINE_DIRECTION_INPUT,
                                                     GPIOD_LINE_VALUE_INACTIVE,
                                                     consumer);
#else
        char chip_path[128];
        if (!detail::gpiod_chip_path(chip_name, chip_path, sizeof(chip_path))) {
            return;
        }
        chip_ = gpiod_chip_open(chip_path);
        if (!chip_) {
            return;
        }
        line_ = gpiod_chip_get_line(chip_, line_offset_);
        if (!line_) {
            close();
            return;
        }
        if (gpiod_line_request_input(line_, consumer ? consumer : "async_event_loop") != 0) {
            close();
        }
#endif
    }

    LinuxGpiodDigitalInputPin(const LinuxGpiodDigitalInputPin&) = delete;
    LinuxGpiodDigitalInputPin& operator=(const LinuxGpiodDigitalInputPin&) = delete;

    ~LinuxGpiodDigitalInputPin() override { close(); }

    bool valid() const override {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        return request_ != nullptr;
#else
        return chip_ != nullptr && line_ != nullptr;
#endif
    }

    bool read() override {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        if (!request_) {
            return false;
        }
        const enum gpiod_line_value value = gpiod_line_request_get_value(request_, line_offset_);
        return value == GPIOD_LINE_VALUE_ACTIVE;
#else
        if (!line_) {
            return false;
        }
        return gpiod_line_get_value(line_) > 0;
#endif
    }

private:
    void close() {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        if (request_) {
            gpiod_line_request_release(request_);
            request_ = nullptr;
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

    unsigned int line_offset_ = 0;
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
    struct gpiod_line_request* request_ = nullptr;
#else
    struct gpiod_chip* chip_ = nullptr;
    struct gpiod_line* line_ = nullptr;
#endif
};

/**
 * Linux libgpiod digital output pin.
 *
 * CMake selects libgpiod v2 when available and falls back to the v1.6 line API
 * otherwise.
 */
class LinuxGpiodDigitalOutputPin final : public IDigitalOutputPin {
public:
    LinuxGpiodDigitalOutputPin(const char* chip_name,
                               unsigned int line_offset,
                               bool initial = false,
                               const char* consumer = "async_event_loop")
        : line_offset_(line_offset) {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        request_ = detail::request_one_gpiod_line_v2(chip_name,
                                                     line_offset_,
                                                     GPIOD_LINE_DIRECTION_OUTPUT,
                                                     initial ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE,
                                                     consumer);
#else
        char chip_path[128];
        if (!detail::gpiod_chip_path(chip_name, chip_path, sizeof(chip_path))) {
            return;
        }
        chip_ = gpiod_chip_open(chip_path);
        if (!chip_) {
            return;
        }
        line_ = gpiod_chip_get_line(chip_, line_offset_);
        if (!line_) {
            close();
            return;
        }
        if (gpiod_line_request_output(line_, consumer ? consumer : "async_event_loop", initial ? 1 : 0) != 0) {
            close();
        }
#endif
    }

    LinuxGpiodDigitalOutputPin(const LinuxGpiodDigitalOutputPin&) = delete;
    LinuxGpiodDigitalOutputPin& operator=(const LinuxGpiodDigitalOutputPin&) = delete;

    ~LinuxGpiodDigitalOutputPin() override { close(); }

    bool valid() const override {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        return request_ != nullptr;
#else
        return chip_ != nullptr && line_ != nullptr;
#endif
    }

    bool write(bool high) override {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        if (!request_) {
            return false;
        }
        return gpiod_line_request_set_value(request_, line_offset_, high ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE) == 0;
#else
        if (!line_) {
            return false;
        }
        return gpiod_line_set_value(line_, high ? 1 : 0) == 0;
#endif
    }

private:
    void close() {
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
        if (request_) {
            gpiod_line_request_release(request_);
            request_ = nullptr;
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

    unsigned int line_offset_ = 0;
#if ASYNC_EVENT_LOOP_LINUX_GPIOD_API_VERSION >= 2
    struct gpiod_line_request* request_ = nullptr;
#else
    struct gpiod_chip* chip_ = nullptr;
    struct gpiod_line* line_ = nullptr;
#endif
};

} // namespace async_event_loop
#endif
