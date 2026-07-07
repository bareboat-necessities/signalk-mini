#pragma once

#include <stddef.h>
#include <stdint.h>

#include <ship_data_model.hpp>

#include "seatalk/seatalk_apply.hpp"
#include "seatalk/seatalk_frame.hpp"

namespace seatalk {

template<typename Real = float>
class SeaTalkReceiver {
public:
    const SeaTalkRxState& state() const { return state_; }
    const SeaTalkDecoded<Real>& last_decoded() const { return last_decoded_; }
    uint32_t decoded_count() const { return decoded_count_; }
    uint32_t unsupported_count() const { return unsupported_count_; }
    uint32_t bad_frame_count() const { return state_.bad_frame_count; }

    bool accept_datagram(const uint8_t* bytes,
                         size_t length,
                         ship_data_model::DataModel<Real>& model,
                         uint64_t now_us,
                         ship_data_model::SensorSource source) {
        SeaTalkFrame frame;
        if (!seatalk_frame_from_bytes(bytes, length, frame)) {
            ++state_.bad_frame_count;
            return false;
        }
        return apply_frame(frame, model, now_us, source);
    }

    bool accept_octet(uint8_t byte,
                      ship_data_model::DataModel<Real>& model,
                      uint64_t now_us,
                      ship_data_model::SensorSource source) {
        SeaTalkFrame frame;
        if (!state_.feed_byte(byte, frame)) return true;
        return apply_frame(frame, model, now_us, source);
    }

    bool accept_octets(const uint8_t* bytes,
                       size_t length,
                       ship_data_model::DataModel<Real>& model,
                       uint64_t now_us,
                       ship_data_model::SensorSource source) {
        if (!bytes) return false;
        bool ok = true;
        for (size_t i = 0; i < length; ++i) {
            if (!accept_octet(bytes[i], model, now_us, source)) ok = false;
        }
        return ok;
    }

private:
    bool apply_frame(const SeaTalkFrame& frame,
                     ship_data_model::DataModel<Real>& model,
                     uint64_t now_us,
                     ship_data_model::SensorSource source) {
        SeaTalkDecoded<Real> decoded;
        if (!decode_seatalk_frame(frame, decoded)) {
            ++unsupported_count_;
            return true;
        }
        last_decoded_ = decoded;
        if (apply_seatalk_decoded(decoded, model, now_us, source)) {
            ++decoded_count_;
            return true;
        }
        ++unsupported_count_;
        return true;
    }

    SeaTalkRxState state_;
    SeaTalkDecoded<Real> last_decoded_;
    uint32_t decoded_count_ = 0;
    uint32_t unsupported_count_ = 0;
};

} // namespace seatalk
