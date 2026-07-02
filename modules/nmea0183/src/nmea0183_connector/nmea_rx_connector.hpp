#pragma once

#include <stdint.h>
#include <string.h>

#include <ship_data_model.hpp>

#include "sentence_parser.hpp"
#include "nmea_rx_helpers.hpp"

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

        if (sentence_is(sentence, "AAM")) return apply_aam(sentence, model, now_us, source);
        if (sentence_is(sentence, "ALM")) return apply_alm(sentence, model, now_us, source);
        if (sentence_is(sentence, "APA")) return apply_apa(sentence, model, now_us, source);
        if (sentence_is(sentence, "APB")) return apply_apb(sentence, model, now_us, source);
        if (sentence_is(sentence, "BOD")) return apply_bod_bww(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWC")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWR")) return apply_bwc_bwr(sentence, model, now_us, source);
        if (sentence_is(sentence, "BWW")) return apply_bod_bww(sentence, model, now_us, source);
        if (sentence_is(sentence, "DBK")) return apply_depth_below_keel(sentence, model, now_us, source);
        if (sentence_is(sentence, "DBS")) return apply_depth_below_surface(sentence, model, now_us, source);
        if (sentence_is(sentence, "DBT")) return apply_dbt(sentence, model, now_us, source);
        if (sentence_is(sentence, "DCN")) return apply_dcn(sentence, model, now_us, source);
        if (sentence_is(sentence, "DPT")) return apply_dpt(sentence, model, now_us, source);
        if (sentence_is(sentence, "DTM")) return apply_dtm(sentence, model, now_us, source);
        if (sentence_is(sentence, "FSI")) return apply_fsi(sentence, model, now_us, source);
        if (sentence_is(sentence, "GBS")) return apply_gbs(sentence, model, now_us, source);
        if (sentence_is(sentence, "GGA")) return apply_gga(sentence, model, now_us, source);
        if (sentence_is(sentence, "GLC")) return apply_glc(sentence, model, now_us, source);
        if (sentence_is(sentence, "GLL")) return apply_gll(sentence, model, now_us, source);
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
        if (sentence_is(sentence, "MSK")) return apply_msk(sentence, model, now_us, source);
        if (sentence_is(sentence, "MSS")) return apply_mss(sentence, model, now_us, source);
        if (sentence_is(sentence, "MTW")) return apply_mtw(sentence, model, now_us, source);
        if (sentence_is(sentence, "MWD")) return apply_mwd(sentence, model, now_us, source);
        if (sentence_is(sentence, "MWV")) return apply_mwv(sentence, model, now_us, source);
        if (sentence_is(sentence, "OLN")) return apply_oln(sentence, model, now_us, source);
        if (sentence_is(sentence, "OSD")) return apply_osd(sentence, model, now_us, source);
        if (sentence_is(sentence, "R00")) return apply_r00(sentence, model, now_us, source);
        if (sentence_is(sentence, "RMA")) return apply_rma(sentence, model, now_us, source);
        if (sentence_is(sentence, "RMB")) return apply_rmb(sentence, model, now_us, source);
        if (sentence_is(sentence, "RMC")) return apply_rmc(sentence, model, now_us, source);
        if (sentence_is(sentence, "ROT")) return apply_rot(sentence, model, now_us);
        if (sentence_is(sentence, "RSA")) return apply_rsa(sentence, model, now_us, source);
        if (sentence_is(sentence, "VHW")) return apply_vhw(sentence, model, now_us, source);
        if (sentence_is(sentence, "VTG")) return apply_vtg(sentence, model, now_us, source);
        if (sentence_is(sentence, "VWR")) return apply_vwr(sentence, model, now_us, source, false);
        if (sentence_is(sentence, "VWT")) return apply_vwr(sentence, model, now_us, source, true);
        if (sentence_is(sentence, "XDR")) return apply_xdr(sentence, model, now_us);
        if (sentence_is(sentence, "XTE")) return apply_xte(sentence, model, now_us, source);

        last_error_ = "unsupported sentence";
        return false;
    }

private:
    const char* last_error_;
    ship_data_model::AutopilotMode last_apb_mode_;
    char last_apb_sender_id_[3];

    template<typename Setting>
    void set_source(Setting& setting, ship_data_model::SensorSource source) {
        if (source != ship_data_model::SensorSource::none) setting.value = source;
    }

#include "nmea_A_E.hpp"
#include "nmea_F_G.hpp"
#include "nmea_H_N.hpp"
#include "nmea_O_Z.hpp"
};

} // namespace nmea0183_connector
