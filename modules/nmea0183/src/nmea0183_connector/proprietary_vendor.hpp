#pragma once

#include <stdint.h>

#include "nmea0183_helpers.hpp"

namespace nmea0183_connector {

enum class NmeaProprietaryVendor : uint16_t {
    Unknown = 0,
    Actisense,
    AcrElectronics,
    AdvanSea,
    Airmar,
    Amec,
    Anritsu,
    Ashtech,
    Autohelm,
    AvMap,
    BandG,
    BrookesAndGatehouse,
    Brookhouse,
    CMap,
    Comar,
    ComNav,
    DigitalYacht,
    Furuno,
    Garmin,
    Geonav,
    Hemisphere,
    Icom,
    JapanRadioCompany,
    Javad,
    Kvh,
    Leica,
    Lowrance,
    Maretron,
    McMurdo,
    MediaTek,
    MglAvionics,
    Magellan,
    Navico,
    Navionics,
    Nke,
    Nobeltec,
    Northstar,
    Novatel,
    Onwa,
    OpenCpn,
    Qstarz,
    Quectel,
    QuarkElec,
    Raymarine,
    Raytheon,
    Robertson,
    Rockwell,
    Sailmon,
    SailTimer,
    Samyung,
    Sanav,
    SeaCross,
    SeaPilot,
    ShipModul,
    SiRf,
    Simrad,
    Sitex,
    SkyTraq,
    StandardHorizon,
    Stowe,
    Teledyne,
    Transas,
    Trimble,
    TrueHeading,
    Ublox,
    Unicore,
    Vesper,
    Weatherdock,
    YachtDevices,
    YakBitz,
};

inline bool nmea_vendor_code_equals(NmeaSpan code, const char* text) {
    return code.length == 3 && text && text[0] && text[1] && text[2] && text[3] == '\0' &&
           code.data && code.data[0] == text[0] && code.data[1] == text[1] && code.data[2] == text[2];
}

inline NmeaSpan nmea_proprietary_vendor_code_from_body(NmeaSpan body) {
    if (!body.data || body.length < 4 || body[0] != 'P') return NmeaSpan();
    return NmeaSpan(body.data + 1, 3);
}

inline NmeaSpan nmea_proprietary_vendor_code_from_token(NmeaSpan token) {
    if (!token.data || token.length < 5 || token[0] != '$' || token[1] != 'P') return NmeaSpan();
    return NmeaSpan(token.data + 2, 3);
}

inline NmeaProprietaryVendor nmea_proprietary_vendor_from_code(NmeaSpan code) {
    if (nmea_vendor_code_equals(code, "ATN")) return NmeaProprietaryVendor::Actisense;
    if (nmea_vendor_code_equals(code, "ACR")) return NmeaProprietaryVendor::AcrElectronics;
    if (nmea_vendor_code_equals(code, "ADS")) return NmeaProprietaryVendor::AdvanSea;
    if (nmea_vendor_code_equals(code, "AIR") || nmea_vendor_code_equals(code, "AMR") || nmea_vendor_code_equals(code, "AMT")) return NmeaProprietaryVendor::Airmar;
    if (nmea_vendor_code_equals(code, "AME")) return NmeaProprietaryVendor::Amec;
    if (nmea_vendor_code_equals(code, "ANR")) return NmeaProprietaryVendor::Anritsu;
    if (nmea_vendor_code_equals(code, "ASH")) return NmeaProprietaryVendor::Ashtech;
    if (nmea_vendor_code_equals(code, "AHM") || nmea_vendor_code_equals(code, "ATM")) return NmeaProprietaryVendor::Autohelm;
    if (nmea_vendor_code_equals(code, "AVM")) return NmeaProprietaryVendor::AvMap;
    if (nmea_vendor_code_equals(code, "BGC")) return NmeaProprietaryVendor::BandG;
    if (nmea_vendor_code_equals(code, "BGH")) return NmeaProprietaryVendor::BrookesAndGatehouse;
    if (nmea_vendor_code_equals(code, "BKH")) return NmeaProprietaryVendor::Brookhouse;
    if (nmea_vendor_code_equals(code, "CMP") || nmea_vendor_code_equals(code, "CMA")) return NmeaProprietaryVendor::CMap;
    if (nmea_vendor_code_equals(code, "CMR")) return NmeaProprietaryVendor::Comar;
    if (nmea_vendor_code_equals(code, "CNV") || nmea_vendor_code_equals(code, "CNI")) return NmeaProprietaryVendor::ComNav;
    if (nmea_vendor_code_equals(code, "DGY") || nmea_vendor_code_equals(code, "DYA") || nmea_vendor_code_equals(code, "DYT")) return NmeaProprietaryVendor::DigitalYacht;
    if (nmea_vendor_code_equals(code, "FEC") || nmea_vendor_code_equals(code, "FUR")) return NmeaProprietaryVendor::Furuno;
    if (nmea_vendor_code_equals(code, "GRM")) return NmeaProprietaryVendor::Garmin;
    if (nmea_vendor_code_equals(code, "GEO")) return NmeaProprietaryVendor::Geonav;
    if (nmea_vendor_code_equals(code, "HEM") || nmea_vendor_code_equals(code, "SAT")) return NmeaProprietaryVendor::Hemisphere;
    if (nmea_vendor_code_equals(code, "ICO")) return NmeaProprietaryVendor::Icom;
    if (nmea_vendor_code_equals(code, "JRC")) return NmeaProprietaryVendor::JapanRadioCompany;
    if (nmea_vendor_code_equals(code, "JAV")) return NmeaProprietaryVendor::Javad;
    if (nmea_vendor_code_equals(code, "KVH")) return NmeaProprietaryVendor::Kvh;
    if (nmea_vendor_code_equals(code, "LEI")) return NmeaProprietaryVendor::Leica;
    if (nmea_vendor_code_equals(code, "LWR")) return NmeaProprietaryVendor::Lowrance;
    if (nmea_vendor_code_equals(code, "MAR") || nmea_vendor_code_equals(code, "MRT")) return NmeaProprietaryVendor::Maretron;
    if (nmea_vendor_code_equals(code, "MCM")) return NmeaProprietaryVendor::McMurdo;
    if (nmea_vendor_code_equals(code, "MTK")) return NmeaProprietaryVendor::MediaTek;
    if (nmea_vendor_code_equals(code, "MGL")) return NmeaProprietaryVendor::MglAvionics;
    if (nmea_vendor_code_equals(code, "MGN")) return NmeaProprietaryVendor::Magellan;
    if (nmea_vendor_code_equals(code, "NVC") || nmea_vendor_code_equals(code, "NVO")) return NmeaProprietaryVendor::Navico;
    if (nmea_vendor_code_equals(code, "NAV")) return NmeaProprietaryVendor::Navionics;
    if (nmea_vendor_code_equals(code, "NKE")) return NmeaProprietaryVendor::Nke;
    if (nmea_vendor_code_equals(code, "NOB")) return NmeaProprietaryVendor::Nobeltec;
    if (nmea_vendor_code_equals(code, "NST")) return NmeaProprietaryVendor::Northstar;
    if (nmea_vendor_code_equals(code, "NOV") || nmea_vendor_code_equals(code, "NVS")) return NmeaProprietaryVendor::Novatel;
    if (nmea_vendor_code_equals(code, "ONW")) return NmeaProprietaryVendor::Onwa;
    if (nmea_vendor_code_equals(code, "OPC")) return NmeaProprietaryVendor::OpenCpn;
    if (nmea_vendor_code_equals(code, "QST")) return NmeaProprietaryVendor::Qstarz;
    if (nmea_vendor_code_equals(code, "QTM")) return NmeaProprietaryVendor::Quectel;
    if (nmea_vendor_code_equals(code, "QEK")) return NmeaProprietaryVendor::QuarkElec;
    if (nmea_vendor_code_equals(code, "RAY") || nmea_vendor_code_equals(code, "RYA")) return NmeaProprietaryVendor::Raymarine;
    if (nmea_vendor_code_equals(code, "RTH")) return NmeaProprietaryVendor::Raytheon;
    if (nmea_vendor_code_equals(code, "ROB")) return NmeaProprietaryVendor::Robertson;
    if (nmea_vendor_code_equals(code, "RWI")) return NmeaProprietaryVendor::Rockwell;
    if (nmea_vendor_code_equals(code, "SLM")) return NmeaProprietaryVendor::Sailmon;
    if (nmea_vendor_code_equals(code, "STM")) return NmeaProprietaryVendor::SailTimer;
    if (nmea_vendor_code_equals(code, "SMY")) return NmeaProprietaryVendor::Samyung;
    if (nmea_vendor_code_equals(code, "SAV")) return NmeaProprietaryVendor::Sanav;
    if (nmea_vendor_code_equals(code, "SCX")) return NmeaProprietaryVendor::SeaCross;
    if (nmea_vendor_code_equals(code, "SPL")) return NmeaProprietaryVendor::SeaPilot;
    if (nmea_vendor_code_equals(code, "SMD")) return NmeaProprietaryVendor::ShipModul;
    if (nmea_vendor_code_equals(code, "SRF")) return NmeaProprietaryVendor::SiRf;
    if (nmea_vendor_code_equals(code, "SIM")) return NmeaProprietaryVendor::Simrad;
    if (nmea_vendor_code_equals(code, "SIT")) return NmeaProprietaryVendor::Sitex;
    if (nmea_vendor_code_equals(code, "STI")) return NmeaProprietaryVendor::SkyTraq;
    if (nmea_vendor_code_equals(code, "SHI") || nmea_vendor_code_equals(code, "STH")) return NmeaProprietaryVendor::StandardHorizon;
    if (nmea_vendor_code_equals(code, "STO")) return NmeaProprietaryVendor::Stowe;
    if (nmea_vendor_code_equals(code, "TDM")) return NmeaProprietaryVendor::Teledyne;
    if (nmea_vendor_code_equals(code, "TRN")) return NmeaProprietaryVendor::Transas;
    if (nmea_vendor_code_equals(code, "TNL") || nmea_vendor_code_equals(code, "TRM") || nmea_vendor_code_equals(code, "TCM")) return NmeaProprietaryVendor::Trimble;
    if (nmea_vendor_code_equals(code, "THD")) return NmeaProprietaryVendor::TrueHeading;
    if (nmea_vendor_code_equals(code, "UBX")) return NmeaProprietaryVendor::Ublox;
    if (nmea_vendor_code_equals(code, "UNI")) return NmeaProprietaryVendor::Unicore;
    if (nmea_vendor_code_equals(code, "VES") || nmea_vendor_code_equals(code, "VSP")) return NmeaProprietaryVendor::Vesper;
    if (nmea_vendor_code_equals(code, "WDC")) return NmeaProprietaryVendor::Weatherdock;
    if (nmea_vendor_code_equals(code, "YDH") || nmea_vendor_code_equals(code, "YDE")) return NmeaProprietaryVendor::YachtDevices;
    if (nmea_vendor_code_equals(code, "YAK")) return NmeaProprietaryVendor::YakBitz;
    return NmeaProprietaryVendor::Unknown;
}

inline const char* nmea_proprietary_vendor_name(NmeaProprietaryVendor vendor) {
    switch (vendor) {
    case NmeaProprietaryVendor::Actisense: return "Actisense";
    case NmeaProprietaryVendor::AcrElectronics: return "ACR Electronics";
    case NmeaProprietaryVendor::AdvanSea: return "AdvanSea";
    case NmeaProprietaryVendor::Airmar: return "Airmar";
    case NmeaProprietaryVendor::Amec: return "AMEC";
    case NmeaProprietaryVendor::Anritsu: return "Anritsu";
    case NmeaProprietaryVendor::Ashtech: return "Ashtech";
    case NmeaProprietaryVendor::Autohelm: return "Autohelm";
    case NmeaProprietaryVendor::AvMap: return "AvMap";
    case NmeaProprietaryVendor::BandG: return "B&G";
    case NmeaProprietaryVendor::BrookesAndGatehouse: return "Brookes and Gatehouse";
    case NmeaProprietaryVendor::Brookhouse: return "Brookhouse";
    case NmeaProprietaryVendor::CMap: return "C-MAP";
    case NmeaProprietaryVendor::Comar: return "Comar";
    case NmeaProprietaryVendor::ComNav: return "ComNav";
    case NmeaProprietaryVendor::DigitalYacht: return "Digital Yacht";
    case NmeaProprietaryVendor::Furuno: return "Furuno";
    case NmeaProprietaryVendor::Garmin: return "Garmin";
    case NmeaProprietaryVendor::Geonav: return "Geonav";
    case NmeaProprietaryVendor::Hemisphere: return "Hemisphere GNSS";
    case NmeaProprietaryVendor::Icom: return "Icom";
    case NmeaProprietaryVendor::JapanRadioCompany: return "Japan Radio Company";
    case NmeaProprietaryVendor::Javad: return "Javad";
    case NmeaProprietaryVendor::Kvh: return "KVH";
    case NmeaProprietaryVendor::Leica: return "Leica";
    case NmeaProprietaryVendor::Lowrance: return "Lowrance";
    case NmeaProprietaryVendor::Maretron: return "Maretron";
    case NmeaProprietaryVendor::McMurdo: return "McMurdo";
    case NmeaProprietaryVendor::MediaTek: return "MediaTek";
    case NmeaProprietaryVendor::MglAvionics: return "MGL Avionics";
    case NmeaProprietaryVendor::Magellan: return "Magellan";
    case NmeaProprietaryVendor::Navico: return "Navico";
    case NmeaProprietaryVendor::Navionics: return "Navionics";
    case NmeaProprietaryVendor::Nke: return "NKE";
    case NmeaProprietaryVendor::Nobeltec: return "Nobeltec";
    case NmeaProprietaryVendor::Northstar: return "Northstar";
    case NmeaProprietaryVendor::Novatel: return "NovAtel";
    case NmeaProprietaryVendor::Onwa: return "ONWA";
    case NmeaProprietaryVendor::OpenCpn: return "OpenCPN";
    case NmeaProprietaryVendor::Qstarz: return "Qstarz";
    case NmeaProprietaryVendor::Quectel: return "Quectel";
    case NmeaProprietaryVendor::QuarkElec: return "Quark-elec";
    case NmeaProprietaryVendor::Raymarine: return "Raymarine";
    case NmeaProprietaryVendor::Raytheon: return "Raytheon";
    case NmeaProprietaryVendor::Robertson: return "Robertson";
    case NmeaProprietaryVendor::Rockwell: return "Rockwell";
    case NmeaProprietaryVendor::Sailmon: return "Sailmon";
    case NmeaProprietaryVendor::SailTimer: return "SailTimer";
    case NmeaProprietaryVendor::Samyung: return "Samyung";
    case NmeaProprietaryVendor::Sanav: return "SANAV";
    case NmeaProprietaryVendor::SeaCross: return "SeaCross";
    case NmeaProprietaryVendor::SeaPilot: return "SeaPilot";
    case NmeaProprietaryVendor::ShipModul: return "ShipModul";
    case NmeaProprietaryVendor::SiRf: return "SiRF";
    case NmeaProprietaryVendor::Simrad: return "Simrad";
    case NmeaProprietaryVendor::Sitex: return "SI-TEX";
    case NmeaProprietaryVendor::SkyTraq: return "SkyTraq";
    case NmeaProprietaryVendor::StandardHorizon: return "Standard Horizon";
    case NmeaProprietaryVendor::Stowe: return "Stowe";
    case NmeaProprietaryVendor::Teledyne: return "Teledyne";
    case NmeaProprietaryVendor::Transas: return "Transas";
    case NmeaProprietaryVendor::Trimble: return "Trimble";
    case NmeaProprietaryVendor::TrueHeading: return "True Heading";
    case NmeaProprietaryVendor::Ublox: return "u-blox";
    case NmeaProprietaryVendor::Unicore: return "Unicore";
    case NmeaProprietaryVendor::Vesper: return "Vesper";
    case NmeaProprietaryVendor::Weatherdock: return "Weatherdock";
    case NmeaProprietaryVendor::YachtDevices: return "Yacht Devices";
    case NmeaProprietaryVendor::YakBitz: return "YakBitz";
    case NmeaProprietaryVendor::Unknown: return "Unknown";
    }
    return "Unknown";
}

} // namespace nmea0183_connector
