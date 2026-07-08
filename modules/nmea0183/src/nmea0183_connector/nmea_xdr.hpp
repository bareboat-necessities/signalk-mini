#pragma once

// Included inside Nmea0183RxConnector.
// NMEA XDR transducer measurements are grouped separately because XDR has
// variable field groups and does not fit cleanly with the simple alphabetical
// handler headers.

template<typename Model>
bool apply_xdr(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    bool any = false;
    float v = 0.0f;
    for (uint8_t i = 0; i + 3 < sentence.field_count; i = static_cast<uint8_t>(i + 4)) {
        if (sentence.field(i)[0] != 'A' || !parse_real(sentence.field(i + 1), v) || sentence.field(i + 2)[0] != 'D') {
            continue;
        }
        if (nmea_span_equals(sentence.field(i + 3), "PTCH")) {
            model.ins.imu.pitch_deg.set(static_cast<Real>(v), now_us);
            any = true;
        } else if (nmea_span_equals(sentence.field(i + 3), "ROLL")) {
            model.ins.imu.roll_deg.set(static_cast<Real>(v), now_us);
            any = true;
        }
    }
    if (!any) last_error_ = "bad XDR";
    return any;
}
