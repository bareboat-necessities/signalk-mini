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

    template<typename Model>
    bool apply_fir(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        return apply_raw_sentence_record(sentence, model.nmea_extensions.fire_detection, "FIR", now_us, source);
    }

    template<typename Model>
    bool apply_wdc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        return apply_raw_sentence_record(sentence, model.nmea_extensions.waypoint_distance_great_circle, "WDC", now_us, source);
    }

    template<typename Model>
    bool apply_wdr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        return apply_raw_sentence_record(sentence, model.nmea_extensions.waypoint_distance_rhumb, "WDR", now_us, source);
    }

    template<typename Model>
    bool apply_zdl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (sentence.field_count < 3) {
            last_error_ = "short ZDL";
            return false;
        }
        float value = 0.0f;
        if (parse_utc_time_of_day_s(sentence.field(0), value) || parse_real(sentence.field(0), value)) {
            model.nmea_extensions.variable_point.time_to_point_s.set(static_cast<Real>(value), now_us);
        }
        if (parse_distance_nmi(sentence.field(1), sentence.field_count > 2 ? sentence.field(2) : NmeaSpan(), value)) {
            model.nmea_extensions.variable_point.distance_to_point_nmi.set(static_cast<Real>(value), now_us);
        }
        if (sentence.field_count > 3) nmea_copy_span(model.nmea_extensions.variable_point.point_id,
                                                     sizeof(model.nmea_extensions.variable_point.point_id),
                                                     sentence.field(3));
        set_source(model.nmea_extensions.variable_point.source, source);
        model.nmea_extensions.variable_point.last_update_us = now_us;
        return true;
    }
};

} // namespace nmea0183_connector
