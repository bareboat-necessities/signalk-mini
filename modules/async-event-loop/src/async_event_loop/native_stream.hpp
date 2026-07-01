#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#include "async_event_loop_arduino/arduino_serial_stream.hpp"
#else
#include <unistd.h>
#include "async_event_loop_linux/linux_fd_stream.hpp"
#include "async_event_loop_linux/linux_serial_stream.hpp"
#endif

namespace async_event_loop {

#if defined(ARDUINO)
using NativeSerialStream = ArduinoSerialStream;

class NativeConsoleStream final : public IByteStream {
public:
    NativeConsoleStream() : serial_(Serial) {}

    void begin(unsigned long baud = 115200) { Serial.begin(baud); }

    int read(uint8_t* dst, size_t max_len) override { return serial_.read(dst, max_len); }
    int write(const uint8_t* src, size_t len) override { return serial_.write(src, len); }
    bool readable() const override { return serial_.readable(); }
    bool writable() const override { return serial_.writable(); }
    bool valid() const override { return serial_.valid(); }

private:
    ArduinoSerialStream serial_;
};
#else
using NativeSerialStream = LinuxSerialStream;

class NativeConsoleStream final : public IByteStream {
public:
    NativeConsoleStream() : input_(STDIN_FILENO), output_(STDOUT_FILENO) {}

    void begin(unsigned long = 0) {}

    int read(uint8_t* dst, size_t max_len) override { return input_.read(dst, max_len); }
    int write(const uint8_t* src, size_t len) override { return output_.write(src, len); }
    bool readable() const override { return input_.readable(); }
    bool writable() const override { return output_.writable(); }
    bool valid() const override { return input_.valid() && output_.valid(); }
    int native_fd() const override { return input_.native_fd(); }

private:
    LinuxFdStream input_;
    LinuxFdStream output_;
};
#endif

} // namespace async_event_loop
