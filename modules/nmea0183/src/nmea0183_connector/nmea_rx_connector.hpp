#pragma once

#include <stdint.h>
#include <string.h>

#include <ship_data_model.hpp>

#include "sentence_parser.hpp"
#include "nmea_rx_helpers.hpp"
#include "nmea_message_state.hpp"
#include "nmea_dsc_message_state.hpp"

namespace nmea0183_connector {

struct NmeaMessageStateRefs {
    NmeaMessageState& messages;
    NmeaDscMessageState& dsc;

    NmeaRawMessageRecord& alarm_acknowledgement;
    NmeaRawMessageRecord& automatic_device_status;
    NmeaRawMessageRecord& acknowledge_detail_alarm;
    NmeaRawMessageRecord& set_detail_alarm;
    NmeaRawMessageRecord& autopilot_system;
    NmeaRawMessageRecord& bearing_distance_dead_reckoning;
    NmeaRawMessageRecord& encryption_key_command;
    NmeaRawMessageRecord& operational_period_command;
    NmeaCurrentLayerMessageRecord& current_layer;
    NmeaRawMessageRecord& device_capability_report;
    NmeaRawMessageRecord& display_dimming_control;
    NmeaRawMessageRecord& door_status;
    NmeaRawMessageRecord& engine_telegraph;
    NmeaRawMessageRecord& event_message;
    NmeaRawMessageRecord& fire_detection;
    NmeaRawMessageRecord& waypoint_distance_rhumb;
    NmeaRawMessageRecord& waypoint_distance_great_circle;
    NmeaVariablePointMessageRecord& variable_point;
    NmeaRawMessageRecord& text_sentence;
    NmeaMultipartMessageRecord& text_message;
    NmeaMultipartMessageRecord& ais_message;
    NmeaMultipartMessageRecord& navtex_message;
    NmeaMultipartMessageRecord& seatalk_message;
    NmeaMultipartMessageRecord& inmarsat_message;
    NmeaMultipartMessageRecord& generic_multipart_message;

    NmeaMessageStateRefs(NmeaMessageState& messages_ref, NmeaDscMessageState& dsc_ref)
        : messages(messages_ref),
          dsc(dsc_ref),
          alarm_acknowledgement(messages_ref.alarm_acknowledgement),
          automatic_device_status(messages_ref.automatic_device_status),
          acknowledge_detail_alarm(messages_ref.acknowledge_detail_alarm),
          set_detail_alarm(messages_ref.set_detail_alarm),
          autopilot_system(messages_ref.autopilot_system),
          bearing_distance_dead_reckoning(messages_ref.bearing_distance_dead_reckoning),
          encryption_key_command(messages_ref.encryption_key_command),
          operational_period_command(messages_ref.operational_period_command),
          current_layer(messages_ref.current_layer),
          device_capability_report(messages_ref.device_capability_report),
          display_dimming_control(messages_ref.display_dimming_control),
          door_status(messages_ref.door_status),
          engine_telegraph(messages_ref.engine_telegraph),
          event_message(messages_ref.event_message),
          fire_detection(messages_ref.fire_detection),
          waypoint_distance_rhumb(messages_ref.waypoint_distance_rhumb),
          waypoint_distance_great_circle(messages_ref.waypoint_distance_great_circle),
          variable_point(messages_ref.variable_point),
          text_sentence(messages_ref.text_sentence),
          text_message(messages_ref.text_message),
          ais_message(messages_ref.ais_message),
          navtex_message(messages_ref.navtex_message),
          seatalk_message(messages_ref.seatalk_message),
          inmarsat_message(messages_ref.inmarsat_message),
          generic_multipart_message(messages_ref.generic_multipart_message) {}
};

template<typename Real = float>
class Nmea0183RxConnector {
public:
    Nmea0183RxConnector()
        : last_error_(""),
          last_apb_mode_(ship_data_model::AutopilotMode::gps),
          state_(message_state_, dsc_state_) {
        last_apb_sender_id_[0] = '\0';
        last_apb_sender_id_[1] = '\0';
        last_apb_sender_id_[2] = '\0';
    }

    const char* last_error() const { return last_error_; }
    ship_data_model::AutopilotMode last_apb_mode() const { return last_apb_mode_; }
    const char* last_apb_sender_id() const { return last_apb_sender_id_; }
    const NmeaMessageState& message_state() const { return message_state_; }
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
        if (!sentence.valid_checksum) {
            last_error_ = "invalid checksum";
            return false;
        }

        update_multipart_message_state(sentence, now_us, source);

        if (sentence_is(sentence, "AAM")) return apply_aam(sentence, model, now_us, source);
        if (sentence_is(sentence, "ACK")) return apply_ack(sentence, model, now_us, source);
        if (sentence_is(sentence, "ADS")) return apply_ads(sentence, model, now_us, source);
        if (sentence_is(sentence, "AKD")) return apply_akd(sentence, model, now_us, source);
        if (sentence_is(sentence, "ALA")) return apply_ala(sentence, model, now_us, source);
        if (sentence_is(sentence, "ALM")) return apply_alm(sentence, model, now_us, source);
        if (sentence_is(sentence, "APA")) return apply_apa(sentence, model, now_us, source);
        if (sentence_is(sentence, "APB")) return apply_apb(sentence, model, now_us, source);
        if (sentence_is(sentence, "ASD")) return apply_asd(sentence, model, now_us, source);
        if (sentence_is(sentence, "BEC")) return apply_bec(sentence, model, now_us, source);
        if (sentence_is(sentence, "BOD")) return apply_bod_bww(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWC")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWR")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWW")) return apply_bod_bww(sentence, model, now_us, source);
        if (sentence_is(sentence, "CEK")) return apply_cek(sentence, model, now_us, source);
        if (sentence_is(sentence, "COP")) return apply_cop(sentence, model, now_us, source);
        if (sentence_is(sentence, "CUR")) return apply_cur(sentence, model, now_us, source);
        if (sentence_is(sentence, "DBK")) return apply_depth_below_keel(sentence, model, now_us, source);
        if (sentence_is(sentence, "DBS")) return apply_depth_below_surface(sentence, model, now_us, source);
        if (sentence_is(sentence, "DBT")) return apply_dbt(sentence, model, now_us, source);
        if (sentence_is(sentence, "DCN")) return apply_dcn(sentence, model, now_us, source);
        if (sentence_is(sentence, "DCR")) return apply_dcr(sentence, model, now_us, source);
        if (sentence_is(sentence, "DDC")) return apply_ddc(sentence, model, now_us, source);
        if (sentence_is(sentence, "DOR")) return apply_dor(sentence, model, now_us, source);
        if (sentence_is(sentence, "DPT")) return apply_dpt(sentence, model, now_us, source);
        if (sentence_is(sentence, "DSC")) return apply_dsc(sentence, model, now_us, source);
        if (sentence_is(sentence, "DSE")) return apply_dse(sentence, model, now_us, source);
        if (sentence_is(sentence, "DSI")) return apply_dsi(sentence, model, now_us, source);
        if (sentence_is(sentence, "DSR")) return apply_dsr(sentence, model, now_us, source);
        if (sentence_is(sentence, "DTM")) return apply_dtm(sentence, model, now_us, source);
        if (sentence_is(sentence, "ETL")) return apply_etl(sentence, model, now_us, source);
        if (sentence_is(sentence, "EVE")) return apply_eve(sentence, model, now_us, source);
        if (sentence_is(sentence, "FIR")) return apply_fir(sentence, model, now_us, source);
        if (sentence_is(sentence, "FSI")) return apply_fsi(sentence, model, now_us, source);
        if (sentence_is(sentence, "GBS")) return apply_gbs(sentence, model, now_us, source);
        if (sentence_is(sentence, "GGA")) return apply_gga(sentence, model, now_us, source);
        if (sentence_is(sentence, "GLC")) return apply_glc(sentence, model, now_us, source);
        if (sentence_is(sentence, "GLL")) return apply_gll(sentence, model, now_us, source);
        if (sentence_is(sentence, "GNS")) return apply_gns(sentence, model, now_us, source);
        if (sentence_is(sentence, "GRS")) return apply_grs(sentence, model, now_us, source);
        if (sentence_is(sentence, "GSA")) return apply_gsa(sentence, model, now_us, source);
        if (sentence_is(sentence, "GST")) return apply_gst(sentence, model, now_us, source);
        if (sentence_is(sentence, "GSV")) return apply_gsv(sentence, model, now_us, source);
        if (sentence_is(sentence, "GTD")) return apply_gtd(sentence, model, now_us, source);
        if (sentence_is(sentence, "GXA")) return apply_gxa(sentence, model, now_us, source);
        if (sentence_is(sentence, "HDG")) return apply_hdg(sentence, model, now_us);
        if (sentence_is(sentence, "HDM")) return apply_hdm(sentence, model, now_us);
        if (sentence_is(sentence, "HDT")) return apply_hdt(sentence, model, now_us);
        if (sentence_is(sentence, "HFB")) return apply_hfb(sentence, model, now_us, source);
        if (sentence_is(sentence, "HSC")) return apply_hsc(sentence, model, now_us, source);
        if (sentence_is(sentence, "ITS")) return apply_its(sentence, model, now_us, source);
        if (sentence_is(sentence, "LWY")) return apply_lwy(sentence, model, now_us, source);
        if (sentence_is(sentence, "MDA")) return apply_mda(sentence, model, now_us, source);
        if (sentence_is(sentence, "MSK")) return apply_msk(sentence, model, now_us, source);
        if (sentence_is(sentence, "MSS")) return apply_mss(sentence, model, now_us, source);
        if (sentence_is(sentence, "MTW")) return apply_mtw(sentence, model, now_us, source);
        if (sentence_is(sentence, "MWD")) return apply_mwd(sentence, model, now_us, source);
        if (sentence_is(sentence, "MWV")) return apply_mwv(sentence, model, now_us, source);
        if (sentence_is(sentence, "OLN")) return apply_oln(sentence, model, now_us, source);
        if (sentence_is(sentence, "OSD")) return apply_osd(sentence, model, now_us, source);
        if (sentence_is(sentence, "R00")) return apply_r00(sentence, model, now_us, source);
        if (sentence_is(sentence, "RLM")) return apply_rlm(sentence, model, now_us, source);
        if (sentence_is(sentence, "RMA")) return apply_rma(sentence, model, now_us, source);
        if (sentence_is(sentence, "RMB")) return apply_rmb(sentence, model, now_us, source);
        if (sentence_is(sentence, "RMC")) return apply_rmc(sentence, model, now_us, source);
        if (sentence_is(sentence, "ROT")) return apply_rot(sentence, model, now_us);
        if (sentence_is(sentence, "RPM")) return apply_rpm(sentence, model, now_us, source);
        if (sentence_is(sentence, "RSA")) return apply_rsa(sentence, model, now_us, source);
        if (sentence_is(sentence, "RSD")) return apply_rsd(sentence, model, now_us, source);
        if (sentence_is(sentence, "RTE")) return apply_rte(sentence, model, now_us, source);
        if (sentence_is(sentence, "SFI")) return apply_sfi(sentence, model, now_us, source);
        if (sentence_is(sentence, "STN")) return apply_stn(sentence, model, now_us, source);
        if (sentence_is(sentence, "TDS")) return apply_tds(sentence, model, now_us, source);
        if (sentence_is(sentence, "TFI")) return apply_tfi(sentence, model, now_us, source);
        if (sentence_is(sentence, "TLB")) return apply_tlb(sentence, model, now_us, source);
        if (sentence_is(sentence, "TLL")) return apply_tll(sentence, model, now_us, source);
        if (sentence_is(sentence, "TPC")) return apply_tpc(sentence, model, now_us, source);
        if (sentence_is(sentence, "TPR")) return apply_tpr(sentence, model, now_us, source);
        if (sentence_is(sentence, "TPT")) return apply_tpt(sentence, model, now_us, source);
        if (sentence_is(sentence, "TRF")) return apply_trf(sentence, model, now_us, source);
        if (sentence_is(sentence, "TTM")) return apply_ttm(sentence, model, now_us, source);
        if (sentence_is(sentence, "TXT")) return apply_txt(sentence, model, now_us, source);
        if (sentence_is(sentence, "VBW")) return apply_vbw(sentence, model, now_us, source);
        if (sentence_is(sentence, "VDR")) return apply_vdr(sentence, model, now_us, source);
        if (sentence_is(sentence, "VHW")) return apply_vhw(sentence, model, now_us, source);
        if (sentence_is(sentence, "VLW")) return apply_vlw(sentence, model, now_us, source);
        if (sentence_is(sentence, "VPW")) return apply_vpw(sentence, model, now_us, source);
        if (sentence_is(sentence, "VTG")) return apply_vtg(sentence, model, now_us, source);
        if (sentence_is(sentence, "VWR")) return apply_vwr(sentence, model, now_us, source, false);
        if (sentence_is(sentence, "VWT")) return apply_vwr(sentence, model, now_us, source, true);
        if (sentence_is(sentence, "WCV")) return apply_wcv(sentence, model, now_us, source);
        if (sentence_is(sentence, "WDC")) return apply_wdc(sentence, model, now_us, source);
        if (sentence_is(sentence, "WDR")) return apply_wdr(sentence, model, now_us, source);
        if (sentence_is(sentence, "WNC")) return apply_wnc(sentence, model, now_us, source);
        if (sentence_is(sentence, "WPL")) return apply_wpl(sentence, model, now_us, source);
        if (sentence_is(sentence, "XDR")) return apply_xdr(sentence, model, now_us);
        if (sentence_is(sentence, "XTE")) return apply_xte(sentence, model, now_us, source);
        if (sentence_is(sentence, "XTR")) return apply_xtr(sentence, model, now_us, source);
        if (sentence_is(sentence, "ZDA")) return apply_zda(sentence, model, now_us, source);
        if (sentence_is(sentence, "ZDL")) return apply_zdl(sentence, model, now_us, source);
        if (sentence_is(sentence, "ZFO")) return apply_zfo(sentence, model, now_us, source);
        if (sentence_is(sentence, "ZTG")) return apply_ztg(sentence, model, now_us, source);

        if (sentence.fragment.is_fragmented && accepts_fragment_only_family(sentence.family)) return true;

        last_error_ = "unsupported sentence";
        return false;
    }

private:
    const char* last_error_;
    ship_data_model::AutopilotMode last_apb_mode_;
    char last_apb_sender_id_[3];
    NmeaMessageState message_state_;
    NmeaDscMessageState dsc_state_;
    NmeaMessageStateRefs state_;

    template<typename Setting>
    void set_source(Setting& setting, ship_data_model::SensorSource source) {
        if (source != ship_data_model::SensorSource::none) setting.value = source;
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
        if (sentence_is(sentence, "TXT")) update_multipart_record(sentence, message_state_.text_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Ais) update_multipart_record(sentence, message_state_.ais_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::NavTex) update_multipart_record(sentence, message_state_.navtex_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Dsc) update_multipart_record(sentence, dsc_state_.multipart, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::SeaTalk) update_multipart_record(sentence, message_state_.seatalk_message, now_us, source);
        else if (sentence.family == NmeaSentenceFamily::Inmarsat) update_multipart_record(sentence, message_state_.inmarsat_message, now_us, source);
        else update_multipart_record(sentence, message_state_.generic_multipart_message, now_us, source);
    }

#include "nmea_A_E.hpp"
#include "nmea_F_G.hpp"
#include "nmea_H_N.hpp"
#include "nmea_O_Z.hpp"
};

} // namespace nmea0183_connector
