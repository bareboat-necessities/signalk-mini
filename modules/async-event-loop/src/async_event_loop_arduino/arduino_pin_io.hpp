#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#include "async_event_loop/pin_io.hpp"

namespace async_event_loop {

class ArduinoDigitalInputPin final : public IDigitalInputPin {
public:
    ArduinoDigitalInputPin(uint8_t pin, DigitalPinMode mode = DigitalPinMode::Input) : pin_(pin) {
        pinMode(pin_, mode == DigitalPinMode::InputPullup ? INPUT_PULLUP : INPUT);
    }
    bool valid() const override { return true; }
    bool read() override { return digitalRead(pin_) == HIGH; }
private:
    uint8_t pin_;
};

class ArduinoDigitalOutputPin final : public IDigitalOutputPin {
public:
    explicit ArduinoDigitalOutputPin(uint8_t pin, bool initial = false) : pin_(pin) {
        pinMode(pin_, OUTPUT);
        write(initial);
    }
    bool valid() const override { return true; }
    bool write(bool high) override {
        digitalWrite(pin_, high ? HIGH : LOW);
        return true;
    }
private:
    uint8_t pin_;
};

class ArduinoAnalogInputPin final : public IAnalogInputPin {
public:
    explicit ArduinoAnalogInputPin(uint8_t pin, int max_value = 1023) : pin_(pin), max_value_(max_value) {}
    bool valid() const override { return true; }
    int read() override { return analogRead(pin_); }
    int max_value() const override { return max_value_; }
private:
    uint8_t pin_;
    int max_value_;
};

class ArduinoAnalogOutputPin final : public IAnalogOutputPin {
public:
    explicit ArduinoAnalogOutputPin(uint8_t pin, int max_value = 255) : pin_(pin), max_value_(max_value) {
        pinMode(pin_, OUTPUT);
    }
    bool valid() const override { return true; }
    bool write(int value) override {
        if (value < 0) {
            value = 0;
        }
        if (value > max_value_) {
            value = max_value_;
        }
        analogWrite(pin_, value);
        return true;
    }
    int max_value() const override { return max_value_; }
private:
    uint8_t pin_;
    int max_value_;
};

} // namespace async_event_loop
#endif
