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
        update_multipart_message_state(sentence, now_us, source);

#define NMEA_APPLY(ID, FN) if (sentence_is(sentence, ID)) return FN(sentence, model, now_us, source)
#define NMEA_APPLY_NO_SOURCE(ID, FN) if (sentence_is(sentence, ID)) return FN(sentence, model, now_us)
        if (sentence_is(sentence, "VDM") || sentence_is(sentence, "VDO")) return apply_ais_vdm_vdo_with_own_vessel(sentence, model, now_us, source);
        NMEA_APPLY("AAM", apply_aam);
        NMEA_APPLY("ACK", apply_ack);
        NMEA_APPLY("ADS", apply_ads);
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
        if (sentence.family == NmeaSentenceFamily::Inmarsat) return apply_inmarsat(sentence, model, now_us, source);
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
    bool dsc_pending_commit_;
    uint64_t dsc_pending_started_us_;
    ship_data_model::SensorSource dsc_pending_source_;

#include "nmea_rx_multipart.hpp"
#include "nmea_dsc.hpp"
#include "nmea_inmarsat.hpp"
#include "nmea_ais.hpp"
#include "nmea_ais_control.hpp"
#include "nmea_ais_own_vessel.hpp"
#include "nmea_A_E.hpp"
#include "nmea_F_G.hpp"
#include "nmea_H_N.hpp"
#include "nmea_O_Z.hpp"
};

} // namespace nmea0183_connector
