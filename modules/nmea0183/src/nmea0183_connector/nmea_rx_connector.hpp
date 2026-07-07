#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <ship_data_model.hpp>
#include <seatalk.hpp>

#include "sentence_parser.hpp"
#include "nmea_message_state.hpp"
#include "nmea_dsc_message_state.hpp"

namespace nmea0183_connector {

template<typename Real = float>
class Nmea0183RxConnector {
public:
    Nmea0183RxConnector()
        : last_error_(""),
          last_apb_mode_(ship_data_model::AutopilotMode::gps),
          dsc_pending_commit_(false),
          dsc_pending_started_us_(0),
          dsc_pending_source_(ship_data_model::SensorSource::none) {
        last_apb_sender_id_[0] = '\0';
        last_apb_sender_id_[1] = '\0';
        last_apb_sender_id_[2] = '\0';
    }

    const char* last_error() const { return last_error_; }
    ship_data_model::AutopilotMode last_apb_mode() const { return last_apb_mode_; }
    const char* last_apb_sender_id() const { return last_apb_sender_id_; }
    const NmeaMessageState& message_state() const { return state_; }
    const NmeaDscMessageState& dsc_state() const { return dsc_state_; }
    const seatalk::SeaTalkReceiver<Real>& seatalk_receiver() const { return seatalk_receiver_; }

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
        expire_pending_dsc_if_needed(model, now_us);
        const bool bad_inmarsat_fragment_before_update = inmarsat_bad_non_first_fragment_before_update(sentence);
        update_multipart_message_state(sentence, now_us, source);

#define NMEA_APPLY(ID, FN) if (sentence_is(sentence, ID)) return FN(sentence, model, now_us, source)
#define NMEA_APPLY_NO_SOURCE(ID, FN) if (sentence_is(sentence, ID)) return FN(sentence, model, now_us)
        if (sentence_is(sentence, "VDM") || sentence_is(sentence, "VDO")) return apply_ais_vdm_vdo_with_own_vessel(sentence, model, now_us, source);
        NMEA_APPLY("AAM", apply_aam);
        NMEA_APPLY("ABK", apply_abk);
        NMEA_APPLY("ACK", apply_ack);
        NMEA_APPLY("ADS", apply_ads);
        NMEA_APPLY("AGA", apply_aga);
        NMEA_APPLY("AKD", apply_akd);
        NMEA_APPLY("ALA", apply_ala);
        NMEA_APPLY("ALC", apply_alc);
        NMEA_APPLY("ALF", apply_alf);
        NMEA_APPLY("ALM", apply_alm);
        NMEA_APPLY("ALR", apply_alr);
        NMEA_APPLY("APA", apply_apa);
        NMEA_APPLY("APB", apply_apb);
        NMEA_APPLY("ARC", apply_arc);
        NMEA_APPLY("ASD", apply_asd);
        NMEA_APPLY("BCL", apply_bcl);
        NMEA_APPLY("BEC", apply_bec);
        if (sentence_is(sentence, "BOD")) return apply_bod_bww(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWC")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWR")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWW")) return apply_bod_bww(sentence, model, now_us, source);
        NMEA_APPLY("CAN", apply_inmarsat_can);
        NMEA_APPLY("CEK", apply_cek);
        NMEA_APPLY("COP", apply_cop);
        NMEA_APPLY("CRQ", apply_inmarsat_crq);
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
        NMEA_APPLY("DSM", apply_inmarsat_dsm);
        NMEA_APPLY("DSR", apply_dsr);
        NMEA_APPLY("DTM", apply_dtm);
        NMEA_APPLY("ETL", apply_etl);
        NMEA_APPLY("EVE", apply_eve);
        NMEA_APPLY("FIR", apply_fir);
        NMEA_APPLY("FSI", apply_fsi);
        NMEA_APPLY("GBS", apply_gbs);
        NMEA_APPLY("GFA", apply_gfa);
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
        NMEA_APPLY("HBT", apply_hbt);
        NMEA_APPLY_NO_SOURCE("HDG", apply_hdg);
        NMEA_APPLY_NO_SOURCE("HDM", apply_hdm);
        NMEA_APPLY_NO_SOURCE("HDT", apply_hdt);
        NMEA_APPLY("HFB", apply_hfb);
        NMEA_APPLY("HSC", apply_hsc);
        NMEA_APPLY("ITS", apply_its);
        NMEA_APPLY("LWY", apply_lwy);
        NMEA_APPLY("MDA", apply_mda);
        NMEA_APPLY("MOB", apply_mob);
        NMEA_APPLY("MSK", apply_msk);
        NMEA_APPLY("MSS", apply_mss);
        NMEA_APPLY("MTW", apply_mtw);
        NMEA_APPLY("MWD", apply_mwd);
        NMEA_APPLY("MWV", apply_mwv);
        NMEA_APPLY("NRM", apply_nrm);
        NMEA_APPLY("NRX", apply_nrx);
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
        NMEA_APPLY("SMV", apply_smv);
        NMEA_APPLY("STN", apply_stn);
        NMEA_APPLY("TDS", apply_tds);
        NMEA_APPLY("TFI", apply_tfi);
        NMEA_APPLY("TLB", apply_tlb);
        NMEA_APPLY("TLL", apply_tll);
        NMEA_APPLY("TMD", apply_inmarsat_tmd);
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
        if (sentence.family == NmeaSentenceFamily::SeaTalk) {
            return apply_seatalk_carried(sentence, model, now_us, source);
        }
        if (sentence.family == NmeaSentenceFamily::Inmarsat) {
            return apply_inmarsat(sentence, model, now_us, source, bad_inmarsat_fragment_before_update);
        }
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
    seatalk::SeaTalkReceiver<Real> seatalk_receiver_;
    bool dsc_pending_commit_;
    uint64_t dsc_pending_started_us_;
    ship_data_model::SensorSource dsc_pending_source_;

#include "nmea_rx_multipart.hpp"
#include "nmea_seatalk.hpp"
#include "nmea_dsc.hpp"
#include "nmea_inmarsat.hpp"
#include "nmea_ais.hpp"
#include "nmea_ais_own_vessel.hpp"
#include "nmea_A_E.hpp"
#include "nmea_F_G.hpp"
#include "nmea_H_N.hpp"
#include "nmea_O_Z.hpp"

    static const char* abk_ack_label(int32_t value) {
        switch (value) {
        case 0: return "acknowledged";
        case 1: return "requested_message_not_available";
        case 2: return "requested_message_not_supported";
        case 3: return "addressed_binary_acknowledged";
        case 4: return "broadcast_binary_acknowledged";
        default: return "unknown";
        }
    }

    template<typename StampedInt>
    static void parse_int_to_stamped(NmeaSpan field, StampedInt& dst, uint64_t now_us) {
        int32_t parsed = 0;
        if (parse_int32(field, parsed)) dst.set(parsed, now_us);
    }

    template<typename Model>
    bool apply_abk(const NmeaSentence& sentence,
                   Model& model,
                   uint64_t now_us,
                   ship_data_model::SensorSource source) {
        if (sentence.field_count < 5) { last_error_ = "short ABK"; return false; }

        auto& rec = model.ais.acknowledgement;
        int32_t parsed = 0;
        if (parse_int32(sentence.field(0), parsed)) {
            rec.mmsi.set(parsed, now_us);
            rec.destination_mmsi[0].set(parsed, now_us);
        }
        if (parse_int32(sentence.field(2), parsed)) rec.message_type.set(parsed, now_us);
        if (parse_int32(sentence.field(3), parsed)) rec.sequence_number[0].set(parsed, now_us);
        rec.acknowledgement_count.set(1, now_us);

        auto& status = model.ais.data_link_status;
        nmea_copy_span(status.station_id, sizeof(status.station_id), sentence.field(0));
        nmea_copy_span(status.slot_status, sizeof(status.slot_status), sentence.field(1));
        if (parse_int32(sentence.field(4), parsed)) {
            nmea_copy_cstr(status.status_text, sizeof(status.status_text), abk_ack_label(parsed));
        } else {
            nmea_copy_span(status.status_text, sizeof(status.status_text), sentence.field(4));
        }
        status.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
        set_source(status.source, source);
        status.last_update_us = now_us;

        set_source(rec.source, source);
        rec.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_aga(const NmeaSentence& sentence,
                   Model& model,
                   uint64_t now_us,
                   ship_data_model::SensorSource source) {
        if (sentence.field_count < 15) { last_error_ = "short AGA"; return false; }

        auto& rec = model.ais.group_assignment;
        parse_int_to_stamped(sentence.field(0), rec.mmsi, now_us);
        parse_int_to_stamped(sentence.field(1), rec.station_type, now_us);
        parse_int_to_stamped(sentence.field(2), rec.ship_type, now_us);

        float lat1 = 0.0f;
        float lon1 = 0.0f;
        float lat2 = 0.0f;
        float lon2 = 0.0f;
        const bool have_lat1 = parse_lat_lon(sentence.field(3), sentence.field(4), lat1);
        const bool have_lon1 = parse_lat_lon(sentence.field(5), sentence.field(6), lon1);
        const bool have_lat2 = parse_lat_lon(sentence.field(7), sentence.field(8), lat2);
        const bool have_lon2 = parse_lat_lon(sentence.field(9), sentence.field(10), lon2);
        if (have_lat1 && have_lat2) {
            const float north = lat1 > lat2 ? lat1 : lat2;
            const float south = lat1 > lat2 ? lat2 : lat1;
            rec.northeast_lat_deg.set(static_cast<Real>(north), now_us);
            rec.southwest_lat_deg.set(static_cast<Real>(south), now_us);
        }
        if (have_lon1 && have_lon2) {
            const float east = lon1 > lon2 ? lon1 : lon2;
            const float west = lon1 > lon2 ? lon2 : lon1;
            rec.northeast_lon_deg.set(static_cast<Real>(east), now_us);
            rec.southwest_lon_deg.set(static_cast<Real>(west), now_us);
        }

        parse_int_to_stamped(sentence.field(11), rec.report_interval, now_us);
        parse_int_to_stamped(sentence.field(12), rec.txrx_mode, now_us);
        parse_int_to_stamped(sentence.field(13), rec.quiet_time_min, now_us);
        set_source(rec.source, source);
        rec.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_bcl(const NmeaSentence& sentence,
                   Model& model,
                   uint64_t now_us,
                   ship_data_model::SensorSource source) {
        if (sentence.field_count < 8) { last_error_ = "short BCL"; return false; }

        auto& rec = model.ais.base_station;
        parse_int_to_stamped(sentence.field(0), rec.mmsi, now_us);
        parse_int_to_stamped(sentence.field(1), rec.epfd_type, now_us);

        const uint8_t lat_index = sentence.field_count >= 10 ? 3 : 2;
        float value = 0.0f;
        if (parse_lat_lon(sentence.field(lat_index), sentence.field(lat_index + 1), value)) {
            rec.latitude_deg.set(static_cast<Real>(value), now_us);
        }
        if (parse_lat_lon(sentence.field(lat_index + 2), sentence.field(lat_index + 3), value)) {
            rec.longitude_deg.set(static_cast<Real>(value), now_us);
        }
        const NmeaSpan accuracy = sentence.field_count > lat_index + 4 ? sentence.field(lat_index + 4) : NmeaSpan();
        if (!accuracy.empty()) rec.position_accuracy = accuracy[0] == '1' || accuracy[0] == 'A' || accuracy[0] == 'H';

        set_source(rec.source, source);
        rec.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_mob(const NmeaSentence& sentence,
                   Model& model,
                   uint64_t now_us,
                   ship_data_model::SensorSource source) {
        if (sentence.field_count < 2) { last_error_ = "short MOB"; return false; }

        auto& rec = model.notifications.special.smv;
        nmea_copy_span(rec.message_id, sizeof(rec.message_id), sentence.field(0));
        nmea_copy_cstr(rec.event_type, sizeof(rec.event_type), "MOB");
        rec.status = sentence.field(1)[0];

        float value = 0.0f;
        if (sentence.field_count > 2 && parse_utc_time_of_day_s(sentence.field(2), value)) {
            rec.utc_time_s.set(static_cast<Real>(value), now_us);
        }
        if (sentence.field_count > 4 && parse_lat_lon(sentence.field(3), sentence.field(4), value)) {
            rec.latitude_deg.set(static_cast<Real>(value), now_us);
        }
        if (sentence.field_count > 6 && parse_lat_lon(sentence.field(5), sentence.field(6), value)) {
            rec.longitude_deg.set(static_cast<Real>(value), now_us);
        }
        if (sentence.field_count > 7) nmea_copy_span(rec.description, sizeof(rec.description), sentence.field(7));
        else nmea_copy_cstr(rec.description, sizeof(rec.description), "MOB event");
        rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
        set_source(rec.source, source);
        rec.last_update_us = now_us;
        return true;
    }

    bool inmarsat_cr_sentence_is_from_terminal(const NmeaSentence& sentence) {
        if (!talker_is(sentence, "CR")) {
            last_error_ = "unsupported CR terminal sentence talker";
            return false;
        }
        return true;
    }

    template<typename Message>
    bool inmarsat_cr_message_matches_id(const Message& message, const char* message_id) const {
        return message_id && message_id[0] != '\0' && message.message_id[0] != '\0' && strcmp(message.message_id, message_id) == 0;
    }

    template<typename SafetyNetData>
    void inmarsat_cr_mark_message_acknowledged(SafetyNetData& safetynet, const char* message_id, uint64_t now_us) const {
        if (!message_id || message_id[0] == '\0') return;
        if (inmarsat_cr_message_matches_id(safetynet.latest_message, message_id)) {
            safetynet.latest_message.acknowledged = true;
            safetynet.latest_message.last_update_us = now_us;
        }
        for (uint8_t i = 0; i < ship_data_model::INMARSAT_SAFETYNET_MESSAGE_HISTORY_CAPACITY; ++i) {
            auto& slot = safetynet.recent_messages[i];
            if (inmarsat_cr_message_matches_id(slot, message_id)) {
                slot.acknowledged = true;
                slot.last_update_us = now_us;
            }
        }
    }

    template<typename Model>
    bool apply_inmarsat_can(const NmeaSentence& sentence,
                            Model& model,
                            uint64_t now_us,
                            ship_data_model::SensorSource source) {
        if (!inmarsat_cr_sentence_is_from_terminal(sentence)) return false;
        if (sentence.field_count < 1) { last_error_ = "short CRCAN"; return false; }

        char message_id[16] = {0};
        nmea_copy_span(message_id, sizeof(message_id), sentence.field(0));
        inmarsat_cr_mark_message_acknowledged(model.notifications.inmarsat.safetynet, message_id, now_us);

        auto& event = model.notifications.messages.event;
        nmea_copy_span(event.event_id, sizeof(event.event_id), sentence.field(0));
        nmea_copy_cstr(event.event_source, sizeof(event.event_source), "inmarsat");
        event.event_state = sentence.field_count > 1 && !sentence.field(1).empty() ? sentence.field(1)[0] : 'C';
        if (sentence.field_count > 2) nmea_copy_span(event.event_text, sizeof(event.event_text), sentence.field(2));
        else nmea_copy_cstr(event.event_text, sizeof(event.event_text), "message_cancelled");
        event.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
        set_source(event.source, source);
        event.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_inmarsat_crq(const NmeaSentence& sentence,
                            Model& model,
                            uint64_t now_us,
                            ship_data_model::SensorSource source) {
        if (!inmarsat_cr_sentence_is_from_terminal(sentence)) return false;
        if (sentence.field_count < 1) { last_error_ = "short CRCRQ"; return false; }

        auto& rec = model.notifications.messages.text;
        if (sentence.field_count > 0) nmea_copy_span(rec.id, sizeof(rec.id), sentence.field(0));
        else nmea_copy_cstr(rec.id, sizeof(rec.id), "CRQ");
        if (sentence.field_count > 1) nmea_copy_span(rec.code, sizeof(rec.code), sentence.field(1));
        else nmea_copy_cstr(rec.code, sizeof(rec.code), "query");
        if (sentence.field_count > 2) nmea_copy_span(rec.value, sizeof(rec.value), sentence.field(2));
        if (sentence.field_count > 3) nmea_copy_span(rec.text, sizeof(rec.text), sentence.field(3));
        else if (sentence.field_count > 2) nmea_copy_span(rec.text, sizeof(rec.text), sentence.field(2));
        rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
        set_source(rec.source, source);
        rec.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_inmarsat_dsm(const NmeaSentence& sentence,
                            Model& model,
                            uint64_t now_us,
                            ship_data_model::SensorSource source) {
        if (!inmarsat_cr_sentence_is_from_terminal(sentence)) return false;
        if (sentence.field_count < 1) { last_error_ = "short CRDSM"; return false; }

        auto& rec = model.notifications.messages.text;
        nmea_copy_cstr(rec.id, sizeof(rec.id), "DSM");
        if (sentence.field_count > 0) nmea_copy_span(rec.code, sizeof(rec.code), sentence.field(0));
        if (sentence.field_count > 1) nmea_copy_span(rec.value, sizeof(rec.value), sentence.field(1));
        if (sentence.field_count > 2) nmea_copy_span(rec.text, sizeof(rec.text), sentence.field(2));
        else if (sentence.field_count > 1) nmea_copy_span(rec.text, sizeof(rec.text), sentence.field(1));
        rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
        set_source(rec.source, source);
        rec.last_update_us = now_us;

        auto& latest = model.notifications.inmarsat.safetynet.latest_message;
        set_source(latest.source, source);
        nmea_copy_span(latest.terminal_id, sizeof(latest.terminal_id), sentence.talker);
        if (sentence.field_count > 0 && !sentence.field(0).empty()) {
            latest.ocean_region_code = sentence.field(0)[0];
            nmea_copy_cstr(latest.ocean_region_label, sizeof(latest.ocean_region_label), inmarsat_ocean_region_label(latest.ocean_region_code));
        }
        if (sentence.field_count > 1 && !sentence.field(1).empty()) {
            latest.priority_code = sentence.field(1)[0];
            nmea_copy_cstr(latest.priority_label, sizeof(latest.priority_label), inmarsat_priority_label(latest.priority_code));
        }
        if (sentence.field_count > 2) {
            nmea_copy_span(latest.service_code, sizeof(latest.service_code), sentence.field(2));
            nmea_copy_cstr(latest.service_label, sizeof(latest.service_label), inmarsat_service_label(latest.service_code));
        }
        latest.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_inmarsat_tmd(const NmeaSentence& sentence,
                            Model& model,
                            uint64_t now_us,
                            ship_data_model::SensorSource source) {
        if (!inmarsat_cr_sentence_is_from_terminal(sentence)) return false;
        if (sentence.field_count < 1) { last_error_ = "short CRTMD"; return false; }

        NmeaSentence multipart = sentence;
        nmea_set_fragment_info(multipart, 0, 1, 2, 3);
        if (multipart.fragment.is_fragmented) {
            update_inmarsat_multipart_message_state(multipart, now_us, source);
            if (!state_.inmarsat_message.complete) return true;
            if (!inmarsat_cstr_printable_ascii(state_.inmarsat_message.text)) {
                auto& safetynet = model.notifications.inmarsat.safetynet;
                safetynet.unsupported_count.set(safetynet.unsupported_count.value + 1, now_us);
                return true;
            }
            return commit_inmarsat_multipart_message(multipart, model, now_us, source);
        }

        auto& safetynet = model.notifications.inmarsat.safetynet;
        auto& dst = safetynet.latest_message;
        dst = ship_data_model::InmarsatSafetyNetMessageData<Real>{};
        set_source(dst.source, source);
        nmea_copy_span(dst.terminal_id, sizeof(dst.terminal_id), sentence.talker);
        if (sentence.field_count > 0) nmea_copy_span(dst.message_id, sizeof(dst.message_id), sentence.field(0));
        else nmea_copy_cstr(dst.message_id, sizeof(dst.message_id), "TMD");
        if (sentence.field_count > 1 && !sentence.field(1).empty()) dst.msi_status = sentence.field(1)[0];
        NmeaSpan payload = inmarsat_best_payload_field(sentence);
        if (!payload.empty()) nmea_copy_span(dst.message_text, sizeof(dst.message_text), payload);
        dst.total_fragments.set(1, now_us);
        dst.fragment_number.set(1, now_us);
        dst.text_length.set(static_cast<int32_t>(strlen(dst.message_text)), now_us);
        dst.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
        dst.body_complete = true;
        dst.complete = true;
        dst.overflow = payload.length >= sizeof(dst.message_text);
        dst.body_has_unknown_char = inmarsat_span_has_unknown_char(payload);
        dst.first_seen_us = now_us;
        dst.last_update_us = now_us;
        commit_inmarsat_notification_message(safetynet, dst, now_us);
        return true;
    }
};

} // namespace nmea0183_connector
