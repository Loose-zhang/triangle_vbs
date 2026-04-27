// 定义枚举类型 E_NavRecType
#pragma once
#include"rtklib.h"

//rinex4 包含的数据类型
typedef enum {
    E_NavRec_NONE,			///< NONE for unknown */
    EPH,			///< Ephemerides data including orbit, clock, biases, accuracy and status parameters */
    STO,			///< System Time and UTC proxy offset parameters */
    EOP,			///< Earth Orientation Parameters */
    ION			///< Global/Regional ionospheric model parameters */
} E_NavRecType;

//星历类型
typedef enum {
    E_NavMsg_NONE,			///< NONE for unknown
    LNAV,			///< GPS/QZSS/NavIC Legacy Navigation Messages
    FDMA,			///< GLONASS Legacy FDMA Navigation Message
    FNAV,			///< Galileo Free Navigation Message
    INAV,			///< Galileo Integrity Navigation Message
    IFNV,			///< Galileo INAV or FNAV Navigation Message
    D1,				///< BeiDou-2/3 MEO/IGSO Navigation Message
    D2,				///< BeiDou-2/3 GEO Navigation Message
    D1D2,			///< BeiDou-2/3 MEO/IGSO and GEO Navigation Message
    SBAS,			///< SBAS Navigation Message
    CNAV,			///< GPS/QZSS CNAV Navigation Message
    CNV1,			///< BeiDou-3 CNAV-1 Navigation Message
    CNV2,			///< GPS/QZSS CNAV-2 Navigation Message	  BeiDou-3 CNAV-2 Navigation Message
    CNV3,			///< BeiDou-3 CNAV-3 Navigation Message
    CNVX			///< GPS/QZSS CNAV or CNAV-2 Navigation Message  BeiDou-3 CNAV-1, CNAV-2 or CNAV-3 Navigation Message
} E_NavMsgType;

//星历种类
typedef enum {
    E_Eph_NONE,			///< NONE for unknown
    Eph_EPH,			///< GPS/QZS LNAV, GAL IFNV, BDS D1D2 Ephemeris
    Eph_GEPH,			///< GLO Ephemeris
    Eph_SEPH,			///< SBAS Ephemeris
    Eph_CEPH,			///< GPS/QZS/BDS CNVX Ephemeris
    // Eph_STO,			///< STO message
    // Eph_EOP,			///< EOP message
    // Eph_ION			///< ION message
}E_EphType;

// 将 E_NavRecType 枚举值转换为字符串
const char* NavRecToString(E_NavRecType type) {
    switch (type) {
        case E_NavRec_NONE: return "NONE";
        case EPH: return "EPH";
        case STO: return "STO";
        case EOP: return "EOP";
        case ION: return "ION";
        default: return "Unknown";
    }
}

// 辅助函数：将 E_NavMsgType 枚举值转换为字符串
const char* NavMsgToString(E_NavMsgType type) {
    switch (type) {
        case E_NavMsg_NONE: return "NONE";
        case LNAV: return "LNAV";
        case FDMA: return "FDMA";
        case FNAV: return "FNAV";
        case INAV: return "INAV";
        case IFNV: return "IFNV";
        case D1: return "D1";
        case D2: return "D2";
        case D1D2: return "D1D2";
        case SBAS: return "SBAS";
        case CNAV: return "CNAV";
        case CNV1: return "CNV1";
        case CNV2: return "CNV2";
        case CNV3: return "CNV3";
        case CNVX: return "CNVX";
        default: return "Unknown";
    }
}
// 将字符串转换为 E_NavRecType 枚举值
E_NavRecType StringToNavRec(const char* str) {
    if (strcmp(str, "NONE") == 0) return E_NavRec_NONE;
    if (strcmp(str, "EPH") == 0) return EPH;
    if (strcmp(str, "STO") == 0) return STO;
    if (strcmp(str, "EOP") == 0) return EOP;
    if (strcmp(str, "ION") == 0) return ION;
    return E_NavRec_NONE;  // 默认为 E_NavRec_NONE
}


// 将字符串转换为 E_NavMsgType 枚举值
E_NavMsgType StringToNavMsg(const char* str) {
    if (strcmp(str, "NONE") == 0) return E_NavMsg_NONE;
    if (strcmp(str, "LNAV") == 0) return LNAV;
    if (strcmp(str, "FDMA") == 0) return FDMA;
    if (strcmp(str, "FNAV") == 0) return FNAV;
    if (strcmp(str, "INAV") == 0) return INAV;
    if (strcmp(str, "IFNV") == 0) return IFNV;
    if (strcmp(str, "D1  ") == 0) return D1;
    if (strcmp(str, "D2  ") == 0) return D2;
    if (strcmp(str, "D1D2") == 0) return D1D2;
    if (strcmp(str, "SBAS") == 0) return SBAS;
    if (strcmp(str, "CNAV") == 0) return CNAV;
    if (strcmp(str, "CNV1") == 0) return CNV1;
    if (strcmp(str, "CNV2") == 0) return CNV2;
    if (strcmp(str, "CNV3") == 0) return CNV3;
    if (strcmp(str, "CNVX") == 0) return CNVX;
    return E_NavMsg_NONE;  // 默认为 E_NavMsg_NONE
}
//Default navigation massage type for RINEX 3 and 2
const E_NavMsgType defNavMsgType[] = {
    [SYS_GPS] = LNAV,
    [SYS_GLO] = FDMA,
    [SYS_GAL] = IFNV,
    [SYS_CMP] = D1D2,
    [SYS_QZS] = LNAV,
    [SYS_IRN] = LNAV,
    [SYS_SBS] = SBAS
};