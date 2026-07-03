#pragma once

#include <stdint.h>
#include <string.h>

#include <ship_data_model.hpp>

#include "sentence_parser.hpp"
#include "nmea_message_state.hpp"
#include "nmea_dsc_message_state.hpp"

namespace nmea0183_connector {

template<typename Real = float>
class Nmea0183RxConnector {
public:
    Nmea0183RxConnector()
        : last_error_(""),
          last_apb_mode_(ship_data_model::AutopilotMode::gps) {
        last_apb_sender_id_[0] = '\0';
        last_apb_sender_id_[1] = '\0';
        last_apb_sender_id_[2] = '\0';
    }

    const char* last_error() const { return last_error_; }
    ship_data_model::AutopilotMode last_apb_mode() const { return last_apb_mode_; }
    const char* last_apb_sender_id() const { return last_apb_sender_id_; }
    const NmeaMessageState& message_state() const { return state_; }
    const NmeaDscMessageState& dsc_state() const { return dsc_state_; }

    bool apply_sentence(const NmeaSentence& sentence,
                        ship_data_model::DataModel<Real>& model,
                        uint64_t now_us) {
        return apply_sentence(sentence, model, now_us, ship_data_model::SensorSource::none);
    }

    bool apply_sentence(const NmeaSentence& sentence,
                        ship_data_model::DataModel<Real>& model,
                        uint64_t now_us,
                        ship_data_model::SensorSource source) {
        last_error_ = "";
        update_multipart_message_state(sentence, now_us, source);

#define NMEA_APPLY(ID, FN) if (sentence_is(sentence, ID)) return FN(sentence, model, now_us, source)
#define NMEA_APPLY_NO_SOURCE(ID, FN) if (sentence_is(sentence, ID)) return FN(sentence, model, now_us)
        NMEA_APPLY("AAM", apply_aam);
        NMEA_APPLY("ACK", apply_ack);
        NMEA_APPLY("ADS", apply_ads);
        NMEA_APPLY("AKD", apply_akd);
        NMEA_APPLY("ALA", apply_ala);
        NMEA_APPLY("ALM", apply_alm);
        NMEA_APPLY("APA", apply_apa);
        NMEA_APPLY("APB", apply_apb);
        NMEA_APPLY("ASD", apply_asd);
        NMEA_APPLY("BEC", apply_bec);
        if (sentence_is(sentence, "BOD")) return apply_bod_bww(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWC")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWR")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWW")) return apply_bod_bww(sentence, model, now_us, source);
        NMEA_APPLY("CEK", apply_cek);
        NMEA_APPLY("COP", apply_cop);
        NMEA_APPLY("CUR", apply_cur);
        NMEA_APPLY("DBK", apply_depth_below_keel);
        NMEA_APPLY("DBS", apply_depth_below_surface);
        NMEA_APPLY("DBT", apply_dbt);
        NMEA_APPLY("DCN", apply_dcn);
        NMEA_APPLY("DCR", apply_dcr);
        NMEA_APPLY("DDC", apply_ddc);
        NMEA_APPLY("DOR", apply_dor);
        NMEA_APPLY("DPT", apply_dpt);
        NMEA_APPLY("DSC", apply_dsc);
        NMEA_APPLY("DSE", apply_dse);
        NMEA_APPLY("DSI", apply_dsi);
        NMEA_APPLY("DSR", apply_dsr);
        NMEA_APPLY("DTM", apply_dtm);
        NMEA_APPLY("ETL", apply_etl);
        NMEA_APPLY("EVE", apply_eve);
        NMEA_APPLY("FIR", apply_fir);
        NMEA_APPLY("FSI", apply_fsi);
        NMEA_APPLY("GBS", apply_gbs);
        NMEA_APPLY("GGA", apply_gga);
        NMEA_APPLY("GLC", apply_glc);
        NMEA_APPLY("GLL", apply_gll);
        NMEA_APPLY("GNS", apply_gns);
        NMEA_APPLY("GRS", apply_grs);
        NMEA_APPLY("GSA", apply_gsa);
        NMEA_APPLY("GST", apply_gst);
        NMEA_APPLY("GSV", apply_gsv);
        NMEA_APPLY("GTD", apply_gtd);
        NMEA_APPLY("GXA", apply_gxa);
        NMEA_APPLY_NO_SOURCE("HDG", apply_hdg);
        NMEA_APPLY_NO_SOURCE("HDM", apply_hdm);
        NMEA_APPLY_NO_SOURCE("HDT", apply_hdt);
        NMEA_APPLY("HFB", apply_hfb);
        NMEA_APPLY("HSC", apply_hsc);
        NMEA_APPLY("ITS", apply_its);
        NMEA_APPLY("LWY", apply_lwy);
        NMEA_APPLY("MDA", apply_mda);
        NMEA_APPLY("MSK", apply_msk);
        NMEA_APPLY("MSS", apply_mss);
        NMEA_APPLY("MTW", apply_mtw);
        NMEA_APPLY("MWD", apply_mwd);
        NMEA_APPLY("MWV", apply_mwv);
        NMEA_APPLY("OLN", apply_oln);
        NMEA_APPLY("OSD", apply_osd);
        NMEA_APPLY("R00", apply_r00);
        NMEA_APPLY("RLM", apply_rlm);
        NMEA_APPLY("RMA", apply_rma);
        NMEA_APPLY("RMB", apply_rmb);
        NMEA_APPLY("RMC", apply_rmc);
        NMEA_APPLY_NO_SOURCE("ROT", apply_rot);
        NMEA_APPLY("RPM", apply_rpm);
        NMEA_APPLY("RSA", apply_rsa);
        NMEA_APPLY("RSD", apply_rsd);
        NMEA_APPLY("RTE", apply_rte);
        NMEA_APPLY("SFI", apply_sfi);
        NMEA_APPLY("STN", apply_stn);
        NMEA_APPLY("TDS", apply_tds);
        NMEA_APPLY("TFI", apply_tfi);
        NMEA_APPLY("TLB", apply_tlb);
        NMEA_APPLY("TLL", apply_tll);
        NMEA_APPLY("TPC", apply_tpc);
        NMEA_APPLY("TPR", apply_tpr);
        NMEA_APPLY("TPT", apply_tpt);
        NMEA_APPLY("TRF", apply_trf);
        NMEA_APPLY("TTM", apply_ttm);
        NMEA_APPLY("TXT", apply_txt);
        NMEA_APPLY("VBW", apply_vbw);
        NMEA_APPLY("VDR", apply_vdr);
        NMEA_APPLY("VHW", apply_vhw);
        NMEA_APPLY("VLW", apply_vlw);
        NMEA_APPLY("VPW", apply_vpw);
        NMEA_APPLY("VTG", apply_vtg);
        if (sentence_is(sentence, "VWR")) return apply_vwr(sentence, model, now_us, source, false);
        if (sentence_is(sentence, "VWT")) return apply_vwr(sentence, model, now_us, source, true);
        NMEA_APPLY("WCV", apply_wcv);
        NMEA_APPLY("WDC", apply_wdc);
        NMEA_APPLY("WDR", apply_wdr);
        NMEA_APPLY("WNC", apply_wnc);
        NMEA_APPLY("WPL", apply_wpl);
        NMEA_APPLY_NO_SOURCE("XDR", apply_xdr);
        NMEA_APPLY("XTE", apply_xte);
        NMEA_APPLY("XTR", apply_xtr);
        NMEA_APPLY("ZDA", apply_zda);
        NMEA_APPLY("ZDL", apply_zdl);
        NMEA_APPLY("ZFO", apply_zfo);
        NMEA_APPLY("ZTG", apply_ztg);
#undef NMEA_APPLY
#undef NMEA_APPLY_NO_SOURCE

        if (sentence.fragment.is_fragmented && accepts_fragment_only_family(sentence.family)) return true;

        last_error_ = "unsupported sentence";
        return false;
    }

private:
    const char* last_error_;
    ship_data_model::AutopilotMode last_apb_mode_;
    char last_apb_sender_id_[3];
    NmeaMessageState state_;
    NmeaDscMessageState dsc_state_;

    template<typename Setting>
    void set_source(Setting& setting, ship_data_model::SensorSource source) {
        if (source != ship_data_model::SensorSource::none) setting.value = source;
    }

    template<typename Record>
    bool apply_notification_text_record(const NmeaSentence& sentence,
                                        Record& record,
                                        uint64_t now_us,
                                        ship_data_model::SensorSource source) {
        if (sentence.field_count > 0) nmea_copy_span(record.id, sizeof(record.id), sentence.field(0));
        if (sentence.field_count > 1) nmea_copy_span(record.code, sizeof(record.code), sentence.field(1));
        if (sentence.field_count > 2) nmea_copy_span(record.value, sizeof(record.value), sentence.field(2));
        if (sentence.field_count > 3) nmea_copy_span(record.text, sizeof(record.text), sentence.field(3));
        else if (sentence.field_count > 2) nmea_copy_span(record.text, sizeof(record.text), sentence.field(2));
        record.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
        set_source(record.source, source);
        record.last_update_us = now_us;
        return true;
    }

    bool accepts_fragment_only_family(NmeaSentenceFamily family) const {
        return family == NmeaSentenceFamily::Ais ||
               family == NmeaSentenceFamily::NavTex ||
               family == NmeaSentenceFamily::SeaTalk ||
               family == NmeaSentenceFamily::Inmarsat;
    }

    template<typename Multipart>
    void reset_multipart_record(Multipart& record) {
        record.received_mask = 0;
        record.in_progress = false;
        record.complete = false;
        record.overflow = false;
        record.text[0] = '\0';
    }

    template<typename Multipart>
    void update_multipart_record(const NmeaSentence& sentence,
                                 Multipart& record,
                                 uint64_t now_us,
                                 ship_data_model::SensorSource source) {
        if (!sentence.fragment.is_fragmented) return;
        if (sentence.fragment.is_first() || !record.in_progress || record.complete) {
            reset_multipart_record(record);
            nmea_copy_span(record.sentence_id, sizeof(record.sentence_id), sentence.sentence);
            nmea_copy_span(record.message_id, sizeof(record.message_id), sentence.fragment.message_id);
            record.in_progress = true;
        }
        record.total_fragments.set(static_cast<int32_t>(sentence.fragment.total), now_us);
        record.last_fragment_number.set(static_cast<int32_t>(sentence.fragment.number), now_us);
        if (sentence.fragment.number <= 16) {
            record.received_mask = static_cast<uint16_t>(record.received_mask | (1u << (sentence.fragment.number - 1u)));
        }

        size_t current = strlen(record.text);
        const size_t capacity = NMEA_MESSAGE_MULTIPART_TEXT_BYTES;
        size_t append = sentence.fragment.payload.length;
        if (current + append + 1u > capacity) {
            append = current < capacity ? capacity - current - 1u : 0u;
            record.overflow = true;
        }
        if (append > 0u && sentence.fragment.payload.data) {
            memcpy(record.text + current, sentence.fragment.payload.data, append);
            record.text[current + append] = '\0';
            current += append;
        }
        record.text_length.set(static_cast<int32_t>(current), now_us);

        const uint16_t expected = sentence.fragment.total >= 16
            ? 0xffffu
            : static_cast<uint16_t>((1u << sentence.fragment.total) - 1u);
        record.complete = sentence.fragment.total > 0 && (record.received_mask & expected) == expected;
        if (record.complete) record.in_progress = false;
        set_source(record.source, source);
        record.last_update_us = now_us;
    }

    void update_multipart_message_state(const NmeaSentence& sentence,
                                        uint64_t now_us,
                                        ship_data_model::SensorSource source) {
        if (!sentence.fragment.is_fragmented) return;
        if (sentence_is(sentence, "TXT")) update_multipart_record(sentence, state_.text_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Ais) update_multipart_record(sentence, state_.ais_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::NavTex) update_multipart_record(sentence, state_.navtex_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Dsc) update_multipart_record(sentence, dsc_state_.multipart, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::SeaTalk) update_multipart_record(sentence, state_.seatalk_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Inmarsat) update_multipart_record(sentence, state_.inmarsat_message, now_us, source);
        else update_multipart_record(sentence, state_.generic_multipart_message, now_us, source);
    }

#include "nmea_A_E.hpp"
#include "nmea_F_G.hpp"
#include "nmea_H_N.hpp"
#include "nmea_O_Z.hpp"
};

} // namespace nmea0183_connector
