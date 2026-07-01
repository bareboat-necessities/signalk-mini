#pragma once

#include <Arduino.h>

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

namespace async_event_loop {

/**
 * Small RAII interrupt guard for copying ISR-updated counters in loop context.
 *
 * AVR saves/restores SREG so nested interrupt state is preserved. Other Arduino
 * cores use the portable noInterrupts()/interrupts() fallback.
 */
class ArduinoInterruptGuard final {
public:
    ArduinoInterruptGuard() {
#if defined(__AVR__)
        sreg_ = SREG;
        cli();
#else
        noInterrupts();
#endif
    }

    ~ArduinoInterruptGuard() {
#if defined(__AVR__)
        SREG = sreg_;
#else
        interrupts();
#endif
    }

    ArduinoInterruptGuard(const ArduinoInterruptGuard&) = delete;
    ArduinoInterruptGuard& operator=(const ArduinoInterruptGuard&) = delete;

private:
#if defined(__AVR__)
    uint8_t sreg_ = 0;
#endif
};

} // namespace async_event_loop
