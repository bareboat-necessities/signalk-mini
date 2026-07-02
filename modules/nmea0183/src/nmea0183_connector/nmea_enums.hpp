#pragma once

#include <stdint.h>
#include "nmea0183_helpers.hpp"

namespace nmea0183_connector {

enum class NmeaTalkerId : uint8_t {
    Unknown = 0,
    IndependentAisBaseStation,
    DependentAisBaseStation,
    AutopilotGeneral,
    MobileAisStation,
    AisAidToNavigation,
    AutopilotMagnetic,
    AisReceivingStation,
    AisTransmittingStation,
    AisSimplexRepeater,
    BeiDou,
    BilgeSystem,
    BridgeNavigationalWatchAlarm,
    CentralAlarm,
    ComputerProgrammedCalculator,
    DigitalSelectiveCalling,
    ComputerMemoryData,
    DataReceiver,
    CommunicationsSatellite,
    RadioTelephoneMfHf,
    RadioTelephoneVhf,
    ScanningReceiver,
    DeccaNavigation,
    DirectionFinder,
    SpeedLogWaterMagnetic,
    DynamicPosition,
    DuplexRepeater,
    Ecdis,
    Epirb,
    EngineRoomMonitoring,
    FireDoor,
    FireSprinkler,
    Galileo,
    NavIC,
    Glonass,
    CombinedGnss,
    Gps,
    Qzss,
    HeadingMagneticCompass,
    HullDoor,
    HeadingNorthSeekingGyro,
    HeadingFluxgate,
    HeadingNonNorthSeekingGyro,
    HullStress,
    IntegratedInstrumentation,
    IntegratedNavigation,
    AlarmAndMonitoring,
    WaterMonitoring,
    PowerManagement,
    PropulsionControl,
    EngineControl,
    PropulsionBoiler,
    AuxiliaryBoiler,
    EngineGovernor,
    LoranA,
    LoranC,
    MicrowavePositioning,
    Multiplexer,
    NavigationLightController,
    OmegaNavigation,
    DistressAlarmSystem,
    Proprietary,
    RadarArpa,
    RecordBook,
    PropulsionMachinery,
    RudderAngleIndicator,
    PhysicalShoreAusStation,
    DepthSounder,
    SteeringGear,
    ElectronicPositioningSystem,
    ScanningSounder,
    SkytraqDebug,
    TrackControl,
    TurnRateIndicator,
    TransitNavigation,
    UserConfigured,
    MicroprocessorController,
    VdesAsm,
    VelocitySensorDoppler,
    SpeedLogWaterMagneticVm,
    VoyageDataRecorder,
    VdesSatellite,
    VdesTerrestrial,
    SpeedLogWaterMechanical,
    WatertightDoor,
    WeatherInstrument,
    WaterLevel,
    TransducerTemperature,
    TransducerDisplacement,
    TransducerFrequency,
    TransducerLevel,
    TransducerPressure,
    TransducerFlowRate,
    TransducerTachometer,
    TransducerVolume,
    Transducer,
    TimekeeperAtomicClock,
    TimekeeperChronometer,
    TimekeeperQuartz,
    TimekeeperRadioUpdate,
};

enum class NmeaFaaModeIndicator : uint8_t {
    Unknown = 0,
    Autonomous,
    Caution,
    Differential,
    EstimatedDeadReckoning,
    RtkFloat,
    ManualInput,
    DataNotValid,
    Precise,
    RtkInteger,
    Simulated,
    Unsafe,
};

enum class NmeaGnssSystem : uint8_t {
    Unknown = 0,
    Gps,
    Glonass,
    Galileo,
    BeiDou,
    Qzss,
    NavIC,
    Combined,
    Sbas,
    Imes,
};

enum class NmeaGnssSystemId : uint8_t {
    Unknown = 0,
    Gps = 1,
    Glonass = 2,
    Galileo = 3,
    BeiDou = 4,
    Qzss = 5,
    NavIC = 6,
};

enum class NmeaGnssSignalId : uint8_t {
    Unknown = 0,
    AllSignals,
    GpsL1Ca,
    GpsL1Py,
    GpsL1M,
    GpsL2Py,
    GpsL2CM,
    GpsL2CL,
    GpsL5I,
    GpsL5Q,
    GlonassL1Ca,
    GlonassL1P,
    GlonassL2Ca,
    GlonassL2P,
    GalileoE5a,
    GalileoE5b,
    GalileoE5ab,
    GalileoE6A,
    GalileoE6BC,
    GalileoL1A,
    GalileoL1BC,
    BeiDouB1I,
    BeiDouB1Q,
    BeiDouB1C,
    BeiDouB1A,
    BeiDouB2a,
    BeiDouB2b,
    BeiDouB2ab,
    BeiDouB3I,
    BeiDouB3Q,
    BeiDouB3A,
    BeiDouB2I,
    BeiDouB2Q,
    QzssL1Ca,
    QzssL1CD,
    QzssL1CP,
    QzssLis,
    QzssL2CM,
    QzssL2CL,
    QzssL5I,
    QzssL5Q,
    QzssL6D,
    QzssL6E,
    NavicL5Sps,
    NavicSSps,
    NavicL5Rs,
    NavicSRs,
    NavicL1Sps,
    Reserved,
};

inline bool nmea_talker_code_equals(NmeaSpan talker, const char* code) {
    return talker.length == 2 && talker.data && code && code[0] && code[1] && code[2] == '\0' &&
           talker[0] == code[0] && talker[1] == code[1];
}

inline NmeaTalkerId nmea_talker_id_from_span(NmeaSpan talker) {
    if (talker.length == 2 && talker[0] == 'U' && talker[1] >= '0' && talker[1] <= '9') return NmeaTalkerId::UserConfigured;
    if (talker.length >= 1 && talker[0] == 'P') return NmeaTalkerId::Proprietary;
    if (nmea_talker_code_equals(talker, "AB")) return NmeaTalkerId::IndependentAisBaseStation;
    if (nmea_talker_code_equals(talker, "AD")) return NmeaTalkerId::DependentAisBaseStation;
    if (nmea_talker_code_equals(talker, "AG")) return NmeaTalkerId::AutopilotGeneral;
    if (nmea_talker_code_equals(talker, "AI")) return NmeaTalkerId::MobileAisStation;
    if (nmea_talker_code_equals(talker, "AN")) return NmeaTalkerId::AisAidToNavigation;
    if (nmea_talker_code_equals(talker, "AP")) return NmeaTalkerId::AutopilotMagnetic;
    if (nmea_talker_code_equals(talker, "AR")) return NmeaTalkerId::AisReceivingStation;
    if (nmea_talker_code_equals(talker, "AT")) return NmeaTalkerId::AisTransmittingStation;
    if (nmea_talker_code_equals(talker, "AX")) return NmeaTalkerId::AisSimplexRepeater;
    if (nmea_talker_code_equals(talker, "BD") || nmea_talker_code_equals(talker, "GB")) return NmeaTalkerId::BeiDou;
    if (nmea_talker_code_equals(talker, "BI")) return NmeaTalkerId::BilgeSystem;
    if (nmea_talker_code_equals(talker, "BN")) return NmeaTalkerId::BridgeNavigationalWatchAlarm;
    if (nmea_talker_code_equals(talker, "CA")) return NmeaTalkerId::CentralAlarm;
    if (nmea_talker_code_equals(talker, "CC")) return NmeaTalkerId::ComputerProgrammedCalculator;
    if (nmea_talker_code_equals(talker, "CD")) return NmeaTalkerId::DigitalSelectiveCalling;
    if (nmea_talker_code_equals(talker, "CM")) return NmeaTalkerId::ComputerMemoryData;
    if (nmea_talker_code_equals(talker, "CR")) return NmeaTalkerId::DataReceiver;
    if (nmea_talker_code_equals(talker, "CS")) return NmeaTalkerId::CommunicationsSatellite;
    if (nmea_talker_code_equals(talker, "CT")) return NmeaTalkerId::RadioTelephoneMfHf;
    if (nmea_talker_code_equals(talker, "CV")) return NmeaTalkerId::RadioTelephoneVhf;
    if (nmea_talker_code_equals(talker, "CX")) return NmeaTalkerId::ScanningReceiver;
    if (nmea_talker_code_equals(talker, "DE")) return NmeaTalkerId::DeccaNavigation;
    if (nmea_talker_code_equals(talker, "DF")) return NmeaTalkerId::DirectionFinder;
    if (nmea_talker_code_equals(talker, "DM")) return NmeaTalkerId::SpeedLogWaterMagnetic;
    if (nmea_talker_code_equals(talker, "DP")) return NmeaTalkerId::DynamicPosition;
    if (nmea_talker_code_equals(talker, "DU")) return NmeaTalkerId::DuplexRepeater;
    if (nmea_talker_code_equals(talker, "EC")) return NmeaTalkerId::Ecdis;
    if (nmea_talker_code_equals(talker, "EP")) return NmeaTalkerId::Epirb;
    if (nmea_talker_code_equals(talker, "ER")) return NmeaTalkerId::EngineRoomMonitoring;
    if (nmea_talker_code_equals(talker, "FD")) return NmeaTalkerId::FireDoor;
    if (nmea_talker_code_equals(talker, "FS")) return NmeaTalkerId::FireSprinkler;
    if (nmea_talker_code_equals(talker, "GA")) return NmeaTalkerId::Galileo;
    if (nmea_talker_code_equals(talker, "GI")) return NmeaTalkerId::NavIC;
    if (nmea_talker_code_equals(talker, "GL")) return NmeaTalkerId::Glonass;
    if (nmea_talker_code_equals(talker, "GN")) return NmeaTalkerId::CombinedGnss;
    if (nmea_talker_code_equals(talker, "GP")) return NmeaTalkerId::Gps;
    if (nmea_talker_code_equals(talker, "GQ") || nmea_talker_code_equals(talker, "QZ") || nmea_talker_code_equals(talker, "PQ")) return NmeaTalkerId::Qzss;
    if (nmea_talker_code_equals(talker, "HC")) return NmeaTalkerId::HeadingMagneticCompass;
    if (nmea_talker_code_equals(talker, "HD")) return NmeaTalkerId::HullDoor;
    if (nmea_talker_code_equals(talker, "HE")) return NmeaTalkerId::HeadingNorthSeekingGyro;
    if (nmea_talker_code_equals(talker, "HF")) return NmeaTalkerId::HeadingFluxgate;
    if (nmea_talker_code_equals(talker, "HN")) return NmeaTalkerId::HeadingNonNorthSeekingGyro;
    if (nmea_talker_code_equals(talker, "HS")) return NmeaTalkerId::HullStress;
    if (nmea_talker_code_equals(talker, "II")) return NmeaTalkerId::IntegratedInstrumentation;
    if (nmea_talker_code_equals(talker, "IN")) return NmeaTalkerId::IntegratedNavigation;
    if (nmea_talker_code_equals(talker, "JA")) return NmeaTalkerId::AlarmAndMonitoring;
    if (nmea_talker_code_equals(talker, "JB")) return NmeaTalkerId::WaterMonitoring;
    if (nmea_talker_code_equals(talker, "JC")) return NmeaTalkerId::PowerManagement;
    if (nmea_talker_code_equals(talker, "JD")) return NmeaTalkerId::PropulsionControl;
    if (nmea_talker_code_equals(talker, "JE")) return NmeaTalkerId::EngineControl;
    if (nmea_talker_code_equals(talker, "JF")) return NmeaTalkerId::PropulsionBoiler;
    if (nmea_talker_code_equals(talker, "JG")) return NmeaTalkerId::AuxiliaryBoiler;
    if (nmea_talker_code_equals(talker, "JH")) return NmeaTalkerId::EngineGovernor;
    if (nmea_talker_code_equals(talker, "LA")) return NmeaTalkerId::LoranA;
    if (nmea_talker_code_equals(talker, "LC")) return NmeaTalkerId::LoranC;
    if (nmea_talker_code_equals(talker, "MP")) return NmeaTalkerId::MicrowavePositioning;
    if (nmea_talker_code_equals(talker, "MX")) return NmeaTalkerId::Multiplexer;
    if (nmea_talker_code_equals(talker, "NL")) return NmeaTalkerId::NavigationLightController;
    if (nmea_talker_code_equals(talker, "OM")) return NmeaTalkerId::OmegaNavigation;
    if (nmea_talker_code_equals(talker, "OS")) return NmeaTalkerId::DistressAlarmSystem;
    if (nmea_talker_code_equals(talker, "RA")) return NmeaTalkerId::RadarArpa;
    if (nmea_talker_code_equals(talker, "RB")) return NmeaTalkerId::RecordBook;
    if (nmea_talker_code_equals(talker, "RC")) return NmeaTalkerId::PropulsionMachinery;
    if (nmea_talker_code_equals(talker, "RI")) return NmeaTalkerId::RudderAngleIndicator;
    if (nmea_talker_code_equals(talker, "SA")) return NmeaTalkerId::PhysicalShoreAusStation;
    if (nmea_talker_code_equals(talker, "SD")) return NmeaTalkerId::DepthSounder;
    if (nmea_talker_code_equals(talker, "SG")) return NmeaTalkerId::SteeringGear;
    if (nmea_talker_code_equals(talker, "SN")) return NmeaTalkerId::ElectronicPositioningSystem;
    if (nmea_talker_code_equals(talker, "SS")) return NmeaTalkerId::ScanningSounder;
    if (nmea_talker_code_equals(talker, "ST")) return NmeaTalkerId::SkytraqDebug;
    if (nmea_talker_code_equals(talker, "TC")) return NmeaTalkerId::TrackControl;
    if (nmea_talker_code_equals(talker, "TI")) return NmeaTalkerId::TurnRateIndicator;
    if (nmea_talker_code_equals(talker, "TR")) return NmeaTalkerId::TransitNavigation;
    if (nmea_talker_code_equals(talker, "UP")) return NmeaTalkerId::MicroprocessorController;
    if (nmea_talker_code_equals(talker, "VA")) return NmeaTalkerId::VdesAsm;
    if (nmea_talker_code_equals(talker, "VD")) return NmeaTalkerId::VelocitySensorDoppler;
    if (nmea_talker_code_equals(talker, "VM")) return NmeaTalkerId::SpeedLogWaterMagneticVm;
    if (nmea_talker_code_equals(talker, "VR")) return NmeaTalkerId::VoyageDataRecorder;
    if (nmea_talker_code_equals(talker, "VS")) return NmeaTalkerId::VdesSatellite;
    if (nmea_talker_code_equals(talker, "VT")) return NmeaTalkerId::VdesTerrestrial;
    if (nmea_talker_code_equals(talker, "VW")) return NmeaTalkerId::SpeedLogWaterMechanical;
    if (nmea_talker_code_equals(talker, "WD")) return NmeaTalkerId::WatertightDoor;
    if (nmea_talker_code_equals(talker, "WI")) return NmeaTalkerId::WeatherInstrument;
    if (nmea_talker_code_equals(talker, "WL")) return NmeaTalkerId::WaterLevel;
    if (nmea_talker_code_equals(talker, "YC")) return NmeaTalkerId::TransducerTemperature;
    if (nmea_talker_code_equals(talker, "YD")) return NmeaTalkerId::TransducerDisplacement;
    if (nmea_talker_code_equals(talker, "YF")) return NmeaTalkerId::TransducerFrequency;
    if (nmea_talker_code_equals(talker, "YL")) return NmeaTalkerId::TransducerLevel;
    if (nmea_talker_code_equals(talker, "YP")) return NmeaTalkerId::TransducerPressure;
    if (nmea_talker_code_equals(talker, "YR")) return NmeaTalkerId::TransducerFlowRate;
    if (nmea_talker_code_equals(talker, "YT")) return NmeaTalkerId::TransducerTachometer;
    if (nmea_talker_code_equals(talker, "YV")) return NmeaTalkerId::TransducerVolume;
    if (nmea_talker_code_equals(talker, "YX")) return NmeaTalkerId::Transducer;
    if (nmea_talker_code_equals(talker, "ZA")) return NmeaTalkerId::TimekeeperAtomicClock;
    if (nmea_talker_code_equals(talker, "ZC")) return NmeaTalkerId::TimekeeperChronometer;
    if (nmea_talker_code_equals(talker, "ZQ")) return NmeaTalkerId::TimekeeperQuartz;
    if (nmea_talker_code_equals(talker, "ZV")) return NmeaTalkerId::TimekeeperRadioUpdate;
    return NmeaTalkerId::Unknown;
}

inline NmeaTalkerId nmea_talker_id_from_token(NmeaSpan token) {
    if (!token.data || token.length < 3 || (token[0] != '$' && token[0] != '!')) return NmeaTalkerId::Unknown;
    return nmea_talker_id_from_span(NmeaSpan(token.data + 1, 2));
}

inline const char* nmea_talker_id_name(NmeaTalkerId id) {
    switch (id) {
    case NmeaTalkerId::IndependentAisBaseStation: return "Independent AIS Base Station";
    case NmeaTalkerId::DependentAisBaseStation: return "Dependent AIS Base Station";
    case NmeaTalkerId::AutopilotGeneral: return "Autopilot - General";
    case NmeaTalkerId::MobileAisStation: return "Mobile AIS Station";
    case NmeaTalkerId::AisAidToNavigation: return "AIS Aid to Navigation";
    case NmeaTalkerId::AutopilotMagnetic: return "Autopilot - Magnetic";
    case NmeaTalkerId::DigitalSelectiveCalling: return "Digital Selective Calling";
    case NmeaTalkerId::BeiDou: return "BeiDou";
    case NmeaTalkerId::Galileo: return "Galileo";
    case NmeaTalkerId::Glonass: return "GLONASS";
    case NmeaTalkerId::CombinedGnss: return "Combined GNSS";
    case NmeaTalkerId::Gps: return "GPS";
    case NmeaTalkerId::Qzss: return "QZSS";
    case NmeaTalkerId::NavIC: return "NavIC";
    case NmeaTalkerId::Ecdis: return "ECDIS";
    case NmeaTalkerId::IntegratedInstrumentation: return "Integrated Instrumentation";
    case NmeaTalkerId::IntegratedNavigation: return "Integrated Navigation";
    case NmeaTalkerId::DepthSounder: return "Depth Sounder";
    case NmeaTalkerId::WeatherInstrument: return "Weather Instrument";
    case NmeaTalkerId::Proprietary: return "Proprietary";
    default: return "unknown";
    }
}

inline NmeaGnssSystem nmea_gnss_system_from_talker(NmeaTalkerId id) {
    switch (id) {
    case NmeaTalkerId::Gps: return NmeaGnssSystem::Gps;
    case NmeaTalkerId::Glonass: return NmeaGnssSystem::Glonass;
    case NmeaTalkerId::Galileo: return NmeaGnssSystem::Galileo;
    case NmeaTalkerId::BeiDou: return NmeaGnssSystem::BeiDou;
    case NmeaTalkerId::Qzss: return NmeaGnssSystem::Qzss;
    case NmeaTalkerId::NavIC: return NmeaGnssSystem::NavIC;
    case NmeaTalkerId::CombinedGnss: return NmeaGnssSystem::Combined;
    default: return NmeaGnssSystem::Unknown;
    }
}

inline NmeaGnssSystem nmea_gnss_system_from_system_id(uint8_t system_id) {
    switch (system_id) {
    case 1: return NmeaGnssSystem::Gps;
    case 2: return NmeaGnssSystem::Glonass;
    case 3: return NmeaGnssSystem::Galileo;
    case 4: return NmeaGnssSystem::BeiDou;
    case 5: return NmeaGnssSystem::Qzss;
    case 6: return NmeaGnssSystem::NavIC;
    default: return NmeaGnssSystem::Unknown;
    }
}

inline NmeaFaaModeIndicator nmea_faa_mode_from_char(char mode) {
    switch (mode) {
    case 'A': return NmeaFaaModeIndicator::Autonomous;
    case 'C': return NmeaFaaModeIndicator::Caution;
    case 'D': return NmeaFaaModeIndicator::Differential;
    case 'E': return NmeaFaaModeIndicator::EstimatedDeadReckoning;
    case 'F': return NmeaFaaModeIndicator::RtkFloat;
    case 'M': return NmeaFaaModeIndicator::ManualInput;
    case 'N': return NmeaFaaModeIndicator::DataNotValid;
    case 'P': return NmeaFaaModeIndicator::Precise;
    case 'R': return NmeaFaaModeIndicator::RtkInteger;
    case 'S': return NmeaFaaModeIndicator::Simulated;
    case 'U': return NmeaFaaModeIndicator::Unsafe;
    default: return NmeaFaaModeIndicator::Unknown;
    }
}

inline const char* nmea_faa_mode_name(NmeaFaaModeIndicator mode) {
    switch (mode) {
    case NmeaFaaModeIndicator::Autonomous: return "autonomous";
    case NmeaFaaModeIndicator::Caution: return "caution";
    case NmeaFaaModeIndicator::Differential: return "differential";
    case NmeaFaaModeIndicator::EstimatedDeadReckoning: return "estimated_dead_reckoning";
    case NmeaFaaModeIndicator::RtkFloat: return "rtk_float";
    case NmeaFaaModeIndicator::ManualInput: return "manual_input";
    case NmeaFaaModeIndicator::DataNotValid: return "data_not_valid";
    case NmeaFaaModeIndicator::Precise: return "precise";
    case NmeaFaaModeIndicator::RtkInteger: return "rtk_integer";
    case NmeaFaaModeIndicator::Simulated: return "simulated";
    case NmeaFaaModeIndicator::Unsafe: return "unsafe";
    default: return "unknown";
    }
}

inline NmeaGnssSystem nmea_satellite_system_from_nmea_id(uint16_t id) {
    if (id >= 1 && id <= 32) return NmeaGnssSystem::Gps;
    if ((id >= 33 && id <= 54) || (id >= 152 && id <= 158)) return NmeaGnssSystem::Sbas;
    if (id >= 65 && id <= 96) return NmeaGnssSystem::Glonass;
    if (id >= 173 && id <= 182) return NmeaGnssSystem::Imes;
    if (id >= 193 && id <= 200) return NmeaGnssSystem::Qzss;
    if (id >= 301 && id <= 336) return NmeaGnssSystem::Galileo;
    if (id >= 401 && id <= 437) return NmeaGnssSystem::BeiDou;
    return NmeaGnssSystem::Unknown;
}

inline uint16_t nmea_global_satellite_id(NmeaTalkerId talker, uint16_t emitted_id) {
    if (talker == NmeaTalkerId::Glonass && emitted_id >= 1 && emitted_id <= 32) return static_cast<uint16_t>(emitted_id + 64u);
    return emitted_id;
}

inline NmeaGnssSignalId nmea_gnss_signal_from_system_and_signal(uint8_t system_id, uint8_t signal_id) {
    if (signal_id == 0) return NmeaGnssSignalId::AllSignals;
    switch (system_id) {
    case 1:
        switch (signal_id) {
        case 1: return NmeaGnssSignalId::GpsL1Ca;
        case 2: return NmeaGnssSignalId::GpsL1Py;
        case 3: return NmeaGnssSignalId::GpsL1M;
        case 4: return NmeaGnssSignalId::GpsL2Py;
        case 5: return NmeaGnssSignalId::GpsL2CM;
        case 6: return NmeaGnssSignalId::GpsL2CL;
        case 7: return NmeaGnssSignalId::GpsL5I;
        case 8: return NmeaGnssSignalId::GpsL5Q;
        default: return signal_id <= 16 ? NmeaGnssSignalId::Reserved : NmeaGnssSignalId::Unknown;
        }
    case 2:
        switch (signal_id) {
        case 1: return NmeaGnssSignalId::GlonassL1Ca;
        case 2: return NmeaGnssSignalId::GlonassL1P;
        case 3: return NmeaGnssSignalId::GlonassL2Ca;
        case 4: return NmeaGnssSignalId::GlonassL2P;
        default: return signal_id <= 16 ? NmeaGnssSignalId::Reserved : NmeaGnssSignalId::Unknown;
        }
    case 3:
        switch (signal_id) {
        case 1: return NmeaGnssSignalId::GalileoE5a;
        case 2: return NmeaGnssSignalId::GalileoE5b;
        case 3: return NmeaGnssSignalId::GalileoE5ab;
        case 4: return NmeaGnssSignalId::GalileoE6A;
        case 5: return NmeaGnssSignalId::GalileoE6BC;
        case 6: return NmeaGnssSignalId::GalileoL1A;
        case 7: return NmeaGnssSignalId::GalileoL1BC;
        default: return signal_id <= 16 ? NmeaGnssSignalId::Reserved : NmeaGnssSignalId::Unknown;
        }
    case 4:
        switch (signal_id) {
        case 1: return NmeaGnssSignalId::BeiDouB1I;
        case 2: return NmeaGnssSignalId::BeiDouB1Q;
        case 3: return NmeaGnssSignalId::BeiDouB1C;
        case 4: return NmeaGnssSignalId::BeiDouB1A;
        case 5: return NmeaGnssSignalId::BeiDouB2a;
        case 6: return NmeaGnssSignalId::BeiDouB2b;
        case 7: return NmeaGnssSignalId::BeiDouB2ab;
        case 8: return NmeaGnssSignalId::BeiDouB3I;
        case 9: return NmeaGnssSignalId::BeiDouB3Q;
        case 10: return NmeaGnssSignalId::BeiDouB3A;
        case 11: return NmeaGnssSignalId::BeiDouB2I;
        case 12: return NmeaGnssSignalId::BeiDouB2Q;
        default: return signal_id <= 16 ? NmeaGnssSignalId::Reserved : NmeaGnssSignalId::Unknown;
        }
    case 5:
        switch (signal_id) {
        case 1: return NmeaGnssSignalId::QzssL1Ca;
        case 2: return NmeaGnssSignalId::QzssL1CD;
        case 3: return NmeaGnssSignalId::QzssL1CP;
        case 4: return NmeaGnssSignalId::QzssLis;
        case 5: return NmeaGnssSignalId::QzssL2CM;
        case 6: return NmeaGnssSignalId::QzssL2CL;
        case 7: return NmeaGnssSignalId::QzssL5I;
        case 8: return NmeaGnssSignalId::QzssL5Q;
        case 9: return NmeaGnssSignalId::QzssL6D;
        case 10: return NmeaGnssSignalId::QzssL6E;
        default: return signal_id <= 16 ? NmeaGnssSignalId::Reserved : NmeaGnssSignalId::Unknown;
        }
    case 6:
        switch (signal_id) {
        case 1: return NmeaGnssSignalId::NavicL5Sps;
        case 2: return NmeaGnssSignalId::NavicSSps;
        case 3: return NmeaGnssSignalId::NavicL5Rs;
        case 4: return NmeaGnssSignalId::NavicSRs;
        case 5: return NmeaGnssSignalId::NavicL1Sps;
        default: return signal_id <= 16 ? NmeaGnssSignalId::Reserved : NmeaGnssSignalId::Unknown;
        }
    default:
        return NmeaGnssSignalId::Unknown;
    }
}

} // namespace nmea0183_connector
