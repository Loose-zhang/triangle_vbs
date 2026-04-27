/*------------------------------------------------------------------------------
* unicore.c : Decoder for Unicore custom receiver format
*
*           Developed by Xu@CUGB and Liu@APM
*
* description : This file contains the implementation for parsing Unicore
*               company receiver's custom binary format. The decoder
*               includes strategies for data reading and partial decoding.
* options : None
*
* version : $Revision:$ $Date:$
* history : 2024/3/17    Initial version by Xu@CUGB
*           2024/11/20   Improved reading strategies and decoding methods,
*                        changed cpp to c by Liu@APM
*
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include "B2b.h"

#define UNICORE_SYNC1       0xAA    /* message start sync code 1 */
#define UNICORE_SYNC2       0x44    /* message start sync code 2 */
#define UNICORE_SYNC3       0xB5    /* message start sync code 3 */

#define UNICORE_LEN         24      /* message header length (bytes) */
#define MAX_UNICORE_LEN     16384   /* 16384 is default max length by rtklib, use it because L_D_PPPB2BINFO3 is unkonw */
                                    /* max message length:H(24)+D(??)+CRC(4) (bytes) 
                                       D_PPPB2BINFO1:40
                                       D_PPPB2BINFO2:80
                                       D_PPPB2BINFO3:8+64*Satnum(?)
                                       D_PPPB2BINFO4:104 
                                    */

#define PPPPB2BINFO1        2302
#define PPPPB2BINFO2        2304   
#define PPPPB2BINFO3        2306
#define PPPPB2BINFO4        2308
#define GPSEPH              106
#define BD3EPH             2999

#define U1(p) (*((uint8_t *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}


// Explicitly initialize using .field to avoid compiler warnings about missing initializer types
B2bmask_t unicore_mask = {
    .MASK_BD = {0},
    .MASK_GPS = {0},
    .MASK_GALILEO = {0},
    .MASK_GLONASS = {0},
    .IOD_SSR = -1,
    .IODP = -1,
    .satnum = -1,
    .satno = {0}
};

// Function to check for Unicore synchronization bytes
static int sync_unicore(uint8_t *buff, uint8_t data)
{
    buff[0] = buff[1]; 
    buff[1] = buff[2]; 
    buff[2] = data;
    return buff[0] == UNICORE_SYNC1 && buff[1] == UNICORE_SYNC2 && buff[2] == UNICORE_SYNC3;
}

// Convert mask bytes to a binary array
static void MaskToBinary(unsigned char Mask[32], int binaryArray[256]) {
    for (int i = 0; i < 32; ++i) {
        unsigned char mask = Mask[i];
        for (int j = 0; j < 8; ++j) {
            // Shift mask to the right by j bits and check the least significant bit
            // Parse from the most significant bit?
            binaryArray[i * 8 + j] = (mask >> (7 - j)) & 1;
        }
    }
}



static int decode_PPPPB2BINFO1(raw_t* raw)
{
    int MASK[256] = {0};
    int udi;
    gtime_t temp_time;

    // Switch from reading file to reading memory
    PACKED_PPPB2BINF01 data1 = {0};
    memcpy(&data1, raw->buff + UNICORE_LEN, sizeof(data1));

    temp_time = B2btod2time(raw->time, data1.Sow);

    // Calculate update interval (udi)
    if (unicore_mask.time.time == 0 && unicore_mask.time.sec == 0.0) {
        udi = 0;
    } else {
        udi = timediff(temp_time, unicore_mask.time);
    }

    // Clear unicore_mask
    memset(&unicore_mask.m_time, 0, sizeof(unicore_mask.m_time));
    memset(&unicore_mask.time, 0, sizeof(unicore_mask.time));
    memset(unicore_mask.MASK_BD, 0, sizeof(unicore_mask.MASK_BD));
    memset(unicore_mask.MASK_GPS, 0, sizeof(unicore_mask.MASK_GPS));
    memset(unicore_mask.MASK_GALILEO, 0, sizeof(unicore_mask.MASK_GALILEO));
    memset(unicore_mask.MASK_GLONASS, 0, sizeof(unicore_mask.MASK_GLONASS));
    unicore_mask.IOD_SSR = unicore_mask.IODP = 0;

    raw->geoprn = data1.Prn - 161 + 1;

    if (raw->geoprn == 62) {
        char time_str[128];
        time2str(raw->time, time_str, 3);
        trace(22, "Skipping PRN 62 at time %s: mes_type = 1 \n", time_str);
        return 0;
    }

    unicore_mask.m_time = raw->time;
    unicore_mask.IOD_SSR = data1.Iodssr;
    unicore_mask.IODP = data1.Iodp;
    unicore_mask.time = B2btod2time(raw->time, data1.Sow);

    MaskToBinary(data1.Mask, MASK);

    // BDS system (0-62)
    for (int i = 0; i < 63; i++) {
        unicore_mask.MASK_BD[i] = MASK[i];
    }
    // GPS system (63-99)
    for (int i = 63; i < 100; i++) {
        unicore_mask.MASK_GPS[i - 63] = MASK[i];
    }
    // GALILEO system (100-136)
    for (int i = 100; i < 137; i++) {
        unicore_mask.MASK_GALILEO[i - 100] = MASK[i];
    }
    // GLONASS system (137-173)
    for (int i = 137; i < 174; i++) {
        unicore_mask.MASK_GLONASS[i - 137] = MASK[i];
    }

    unicore_mask.satnum = mask2satno(&unicore_mask);

    // Call output function
    output_B2bInfo1(raw, &unicore_mask, udi);

    return 20;
}

static int decode_PPPPB2BINFO2(raw_t* raw)
{
    gtime_t m_time;
    double m_time_ep[8];
    char satstr[8];
    int satno, udi;

    PACKED_PPPB2BINF02 data2 = {0};
    memcpy(&data2, raw->buff + UNICORE_LEN, sizeof(data2));
    
    raw->geoprn = data2.Prn - 161 + 1;

    if (raw->geoprn == 62) {
        raw->num_PPPB2BINF02--;
        char time_str[128];
        time2str(raw->time, time_str, 3);
        trace(22, "Skipping PRN 62 at time %s: mes_type = 2 \n", time_str);
        return 0;
    }

    if (data2.Iodssr != unicore_mask.IOD_SSR) {
        trace(22, "error: msg2(eph), new iodssr=%d, msg->iodssr=%d\n", 
              data2.Iodssr, unicore_mask.IOD_SSR);
        return 0;
    }

    uint32_t sow = data2.Sow;
    m_time = B2btod2time(raw->time, sow);
    time2epoch(m_time, m_time_ep);
    int verify_sow = (int)(m_time_ep[3] * 60 * 60 + m_time_ep[4] * 60 + m_time_ep[5]);

    for (int i = 0; i < 6; ++i) {
        satno = slot2satno(data2.StOrbitCorr[i].usPrn);
        if (satno <= 0) {
            continue;
        }
        raw->nav.B2bssr[satno].iodssr[0] = data2.Iodssr;
        raw->nav.B2bssr[satno].iodn = data2.StOrbitCorr[i].usIodn;
        raw->nav.B2bssr[satno].iodcorr[0] = data2.StOrbitCorr[i].ucIODCorr;

        raw->nav.B2bssr[satno].t0[0] = m_time;
        raw->nav.B2bssr[satno].sow = sow;
        raw->nav.B2bssr[satno].verify_sow = verify_sow;

        if (abs(data2.StOrbitCorr[i].sRadial) >= 16383 ||
            abs(data2.StOrbitCorr[i].sInTrack) >= 4095 ||
            abs(data2.StOrbitCorr[i].sCross) >= 4095) {
            satno2id(satno, satstr);
            trace(22, "error: bad-orb, sat=%s, deph=%8.3f,%8.3f,%8.3f\n", 
                  satstr, raw->nav.B2bssr[satno].deph[0], raw->nav.B2bssr[satno].deph[1], raw->nav.B2bssr[satno].deph[2]);
            continue;
        }

        raw->nav.B2bssr[satno].deph[0] = (double)data2.StOrbitCorr[i].sRadial * 0.0016;
        raw->nav.B2bssr[satno].deph[1] = (double)data2.StOrbitCorr[i].sInTrack * 0.0064;
        raw->nav.B2bssr[satno].deph[2] = (double)data2.StOrbitCorr[i].sCross * 0.0064;

        raw->nav.B2bssr[satno].ura = data2.StOrbitCorr[i].ucURAI;

        raw->nav.B2bssr[satno].update = 1;
    }
    output_B2bInfo2(raw, &raw->nav);
    return 20;
}

static int decode_PPPPB2BINFO3(raw_t* raw)
{
    int satno, sys, mode, type, *cods = NULL;
    char satstr[10];
    gtime_t m_time;
    double m_time_ep[8];

    PACKED_PPPB2BINF03 data3 = {0};

    memcpy(&data3, raw->buff + UNICORE_LEN, 8);
    
    int satnum = data3.SatNum;
    data3.StCodeBias_t = (PACKED_StCodeBias_t *)malloc(satnum * sizeof(PACKED_StCodeBias_t));
    if (!data3.StCodeBias_t) {
        printf("Error: Memory allocation failed for StCodeBias_t.\n");
        return -1;
    }
    // Read StCodeBias_t data
    int bias_start = UNICORE_LEN + 8; // Offset: UNICORE_LEN + 8 bytes
    memcpy(data3.StCodeBias_t, raw->buff + bias_start, satnum * sizeof(PACKED_StCodeBias_t));

    // Read CRC data
    int xxx_start = bias_start + satnum * sizeof(PACKED_StCodeBias_t); // Offset: after StCodeBias_t
    memcpy(&data3.Xxxx, raw->buff + xxx_start, 4);

    raw->geoprn = data3.Prn - 161 + 1;

    if (raw->geoprn == 62) {
        raw->num_PPPB2BINF03--;
        char time_str[128];
        time2str(raw->time, time_str, 3);
        trace(22, "Skipping PRN 62 at time %s: mes_type = 3 \n", time_str);
        return 0;
    }

    if (data3.Iodssr != unicore_mask.IOD_SSR) {
        trace(22, "error: msg3(cbia), new iodssr=%d, msg->iodssr=%d\n", 
              data3.Iodssr, unicore_mask.IOD_SSR);
        return 0;
    }

    uint32_t sow = data3.Sow;
    m_time = B2btod2time(raw->time, sow);
    time2epoch(m_time, m_time_ep);
    int verify_sow = (int)(m_time_ep[3] * 60 * 60 + m_time_ep[4] * 60 + m_time_ep[5]);
    
    for (int i = 0; i < satnum; ++i) {
        satno = slot2satno(data3.StCodeBias_t[i].usSatSlot);
        if (satno <= 0) {
            continue;
        }
        satno2id(satno, satstr); 
        sys = satsys(satno, NULL);
        if (sys == SYS_GPS) cods = b2b_gps_codebias_mode;
        else if (sys == SYS_GLO) cods = b2b_glo_codebias_mode;
        else if (sys == SYS_GAL) cods = b2b_gal_codebias_mode;
        else if (sys == SYS_CMP) cods = b2b_bds_codebias_mode;
        else continue;

        raw->nav.B2bssr[satno].iodssr[1] = data3.Iodssr;
        raw->nav.B2bssr[satno].t0[1] = m_time;
        raw->nav.B2bssr[satno].sow = sow;
        raw->nav.B2bssr[satno].verify_sow = verify_sow;

        int signum = data3.StCodeBias_t[i].usBiasNum;

        for (int j = 0; j < signum; ++j) {
            mode = data3.StCodeBias_t[i].stCodeCorr[j].usMode;
            if (abs(data3.StCodeBias_t[i].stCodeCorr[j].sCodeCorr) >= 2103) {
                continue;
            }
            if (mode < 0 || mode >= B2B_CodeBiasModeNum) {
                continue;
            }
            type = cods[mode];
            if (type == CODE_NONE) continue;
            raw->nav.B2bssr[satno].cbias[type] = data3.StCodeBias_t[i].stCodeCorr[j].sCodeCorr * 0.017;
            raw->nav.B2bssr[satno].update = 1;
        }
    }
    output_B2bInfo3(raw, &raw->nav);
    return 20;
}

static int decode_PPPPB2BINFO4(raw_t* raw)
{
    int satno, i;
    double m_time_ep[8];
    gtime_t m_time;
    char satstr[10];
    double c0;

    PACKED_PPPB2BINF04 data4 = {0};
    memcpy(&data4, raw->buff + UNICORE_LEN, sizeof(data4));

    raw->geoprn = data4.Prn - 161 + 1;

    if (raw->geoprn == 62) {
        raw->num_PPPB2BINF04--;
        char time_str[128];
        time2str(raw->time, time_str, 3);
        trace(22, "Skipping PRN 62 at time %s: mes_type = 4 \n", time_str);
        return 0;
    }

    uint32_t sow = data4.Sow;
    m_time = B2btod2time(raw->time, sow);
    time2epoch(m_time, m_time_ep);
    int verify_sow = (int)(m_time_ep[3] * 60 * 60 + m_time_ep[4] * 60 + m_time_ep[5]);

    if (data4.Iodssr != unicore_mask.IOD_SSR || data4.Iodp != unicore_mask.IODP) {
        trace(22, "error: msg4(clk), new iodssr=%d, msg->iodssr=%d, iodp=%d, msg->iodp=%d\n", 
              data4.Iodssr, unicore_mask.IOD_SSR, data4.Iodp, unicore_mask.IODP);
        return 0;
    }

    /*
    * Each cycle starts with message type 1,
    * followed by continuous broadcast of message type 4, interspersed with message types 3 and 2,
    * after broadcasting types 2 and 3, continue updating type 4.
    * Additionally, the epoch time of PPP-B2b information will have a certain delay compared to the reception time.
    */

    if (data4.SubType > 31) {
        trace(4, "error: msg4(clk), bad subtype1=%d\n", data4.SubType);
        return 0;
    }

    int begin = data4.SubType * 23;

    for (i = 0; i < 23; i++) {
        if (unicore_mask.satno[begin + i] != 0) {
            satno = unicore_mask.satno[begin + i];
            raw->nav.B2bssr[satno].t0[2] = m_time;
            raw->nav.B2bssr[satno].sow = sow;
            raw->nav.B2bssr[satno].verify_sow = verify_sow;
            raw->nav.B2bssr[satno].iodssr[2] = unicore_mask.IOD_SSR;
            raw->nav.B2bssr[satno].iodp[0] = unicore_mask.IODP;
            raw->nav.B2bssr[satno].iodcorr[1] = data4.ClkCorr[i].usIodCorr;
            if (abs(data4.ClkCorr[i].sC0) >= 16383 || data4.ClkCorr[i].usIodCorr > 7) {
                satno2id(satno, satstr);
                trace(22, "error: msg4(clk) bad-clk, sat=%s, iodcor=%d, c0=%d \n", 
                      satstr, data4.ClkCorr[i].usIodCorr, data4.ClkCorr[i].sC0);
                continue;
            }
            raw->nav.B2bssr[satno].dclk[0] = (double)data4.ClkCorr[i].sC0 * 0.0016;
            raw->nav.B2bssr[satno].update = 1;
        }
    }
    output_B2bInfo4(raw, &raw->nav);

    return 20;
}

/* URA value (m) to URA index ------------------------------------------------*/
static int uraindex(double value)
{
    static const double ura_eph[] = {
        2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
        3072.0,6144.0,0.0
    };
    int i;
    for (i = 0; i < 15; i++) if (ura_eph[i] >= value) break;
    return i;
}

static int decode_GPSEPH(raw_t* raw) {
    PACKED_UNICORE_GPSEPH data_GPSEPH = {0};
    memcpy(&data_GPSEPH, raw->buff + UNICORE_LEN, sizeof(data_GPSEPH));

    eph_t eph = {0};
    double tow;
    int sat, week;
    char satid[8];

    if (!(sat = satno(SYS_GPS, data_GPSEPH.Prn))) {
        trace(2, "unicore gpsephb satellite error: prn=%d\n", data_GPSEPH.Prn);
        return -1;
    }
    satno2id(sat, satid);

    eph.sat = sat;
    eph.code = 0;
    
    // Assign ephemeris parameters
    eph.f0 = data_GPSEPH.af0;
    eph.f1 = data_GPSEPH.af1;
    eph.f2 = data_GPSEPH.af2;

    // Orbital parameters
    eph.A = data_GPSEPH.A; 
    eph.e = data_GPSEPH.Ecc; 
    eph.i0 = data_GPSEPH.I0; 
    eph.OMG0 = data_GPSEPH.omega0; 
    eph.omg = data_GPSEPH.omega; 
    eph.M0 = data_GPSEPH.M0; 
    eph.deln = data_GPSEPH.delta_N; 
    eph.OMGd = data_GPSEPH.omegaDot; 
    eph.idot = data_GPSEPH.IDOT; 
    eph.crc = data_GPSEPH.crc; 
    eph.crs = data_GPSEPH.crs; 
    eph.cuc = data_GPSEPH.cuc;
    eph.cus = data_GPSEPH.cus; 
    eph.cic = data_GPSEPH.cic; 
    eph.cis = data_GPSEPH.cis;

    // Timing and accuracy parameters
    eph.tgd[0] = data_GPSEPH.tgd; 
    eph.sva = uraindex(data_GPSEPH.URA);  /* Convert URA value to index */

    // Ephemeris data consistency checks
    eph.iode = data_GPSEPH.IODE1;      /* Issue of Data, Ephemeris (IODE) */
    if (data_GPSEPH.IODE1 != data_GPSEPH.IODE2) {
        trace(22, "Warning: IODE1(%d) ≠ IODE2(%d)\n", data_GPSEPH.IODE1, data_GPSEPH.IODE2);
    }
    eph.iodc = data_GPSEPH.iodc;      /* Issue of Data, Clock (IODC) */
    if (eph.iode != eph.iodc) {
        trace(22, "Warning: IODE(%d) ≠ IODC(%d)\n", eph.iode, eph.iodc);
    }

    // Time parameter calculations
    eph.toes = data_GPSEPH.Toe;      /* Time of Ephemeris (seconds in GPS week) */
    eph.week = data_GPSEPH.Week;      /* GPS week number */
    
    /* Handle week number rollover scenarios:
     * GPS ephemeris is received 2 hours ahead. A message received at 23:59:00 
     * on week 2023 might contain data for week 2024 starting at 00:00:00.
     * This logic adjusts the week number if the time difference exceeds ±3.5 days.
     */
    tow = time2gpst(raw->time, &week);  /* Current GPS time from receiver timestamp */
    
    eph.toe = gpst2time(eph.week, eph.toes);
    double tt = timediff(eph.toe, raw->time);
    
    // Week number adjustment logic
    if (tt < -302400.0) {  // -3.5 days
        trace(22, "GPS(%s) WEEK++ (tt=%.1fs < -302400) at %s\n", 
              satid, tt, time_str(raw->time, 3));
        eph.week++;
    } 
    else if (tt > 302400.0) {  // +3.5 days
        trace(22, "GPS(%s) WEEK-- (tt=%.1fs > 302400) at %s\n", 
              satid, tt, time_str(raw->time, 3));
        eph.week--;
    } 

    // Final time calculations
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, data_GPSEPH.toc);  /* Clock reference time */
    eph.ttr = raw->time;  /* Time of transmission */
    
    // Health status check
    eph.svh = data_GPSEPH.health;  /* SV health status */
    if (eph.svh != 0) {
        trace(22, "Warning: SV health status=%d for %s\n", eph.svh, satid);
    }
    
    // Ephemeris update check
    if (!strstr(raw->opt, "-EPHALL")) {
        if (fabs(timediff(raw->nav.eph[sat - 1].toe, eph.toe)) < 1e-9 &&
            fabs(timediff(raw->nav.eph[sat - 1].toc, eph.toc)) < 1e-9) return 0;
    }

    // Update navigation data
    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}

static int decode_BDSEPH(raw_t* raw) {
    PACKED_UNICORE_BD3EPH data_BDSEPH = {0};
    memcpy(&data_BDSEPH, raw->buff + UNICORE_LEN, sizeof(data_BDSEPH));

    if (data_BDSEPH.FreqType != 0){ // FreqType=0 is B1cNav,FreqType=1 is B2aNav,FreqType=2 is B2bNav
        trace(22,"Only decode CNAV1(FreqType = 0),the message is %d \n",data_BDSEPH.FreqType);
        return 0;
    }

    eph_t eph = { 0 };
    gtime_t gps_toe = {0};
    gtime_t raw_bds_toe = {0};
    char time_str0[64],time_str1[64];
    int sat, gps_week,ref_A;
    char satid[8];

    if (!(sat = satno(SYS_CMP, data_BDSEPH.Prn))) {
        trace(2, "unicore gpsephb satellite error: prn=%d\n", data_BDSEPH.Prn);
        return -1;
    }
    satno2id(sat,satid);

    eph.svh = data_BDSEPH.health;
    if(eph.svh!=0){
        trace(2, "unhealthy sat: prn=%d\n", data_BDSEPH.Prn);
        return 0;
    }
    int sat_type = data_BDSEPH.SatType;
    eph.sat = sat;
    eph.iode = data_BDSEPH.IODE;
    eph.iodc = data_BDSEPH.IODC;

    gps_week = data_BDSEPH.Week;

    eph.week = gps_week - 1356; // GPS WEEK --->BDS WEEK
    int zweek = data_BDSEPH.Zweek - 1356; // GPS WEEK --->BDS WEEK
    double tow = data_BDSEPH.Tow;


    eph.toes = data_BDSEPH.Toe;
    eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */

    double tt = timediff(eph.toe,raw->time);
    if (tt < -302400.0){
        trace(22,"BDS(%s) WEEK JUMP(tt < -302400) at %s ! \n",satid,time_str(raw->time,3));
        eph.week++;
    } 
    else if (tt > 302400.0){
        trace(22,"BDS(%s) WEEK JUMP(tt >  302400) at %s ! \n",satid,time_str(raw->time,3));
        eph.week--;
    }
    eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */

    eph.toc = bdt2gpst(bdt2time(eph.week, data_BDSEPH.toc));      /* bdt -> gpst */
    eph.ttr = raw->time;
    time2str(eph.ttr,time_str1,3);

    trace(22,"eph.ttr = %s \n",time_str1);


    // ref_A + delt_A = sqrt(A) * sqrt(A)
    if (sat_type ==1) ref_A  = 42162200; //(GEO)
    else if (sat_type ==2) ref_A  = 42162200; //(IGSO)
    else if (sat_type ==3) ref_A  = 27906100; //(MEO)

    eph.A = ref_A + data_BDSEPH.DeltaA;

    eph.Adot = data_BDSEPH.dDeltaA;
    eph.deln = data_BDSEPH.DeltaN;
    eph.ndot = data_BDSEPH.dDeltaN;
    eph.M0   = data_BDSEPH.M0;
    eph.e    = data_BDSEPH.Ecc;
    eph.omg  = data_BDSEPH.omega;
    eph.cuc  = data_BDSEPH.Cuc;
    eph.cus  = data_BDSEPH.Cus;
    eph.crc  = data_BDSEPH.crc;
    eph.crs  = data_BDSEPH.crs;
    eph.cic  = data_BDSEPH.cic;
    eph.cis  = data_BDSEPH.cis;
    eph.i0   = data_BDSEPH.I0;
    eph.idot = data_BDSEPH.IDOT;
    eph.OMG0 = data_BDSEPH.omega0;
    eph.OMGd = data_BDSEPH.omega_dot;
    eph.tgd[2]  = data_BDSEPH.Tgdb1cp;
    eph.tgd[4]  = data_BDSEPH.ISCb1cd;
    eph.f0   = data_BDSEPH.af0;
    eph.f1   = data_BDSEPH.af1;
    eph.f2   = data_BDSEPH.af2;
    eph.sisa[0] = data_BDSEPH.SISAIoe;
    eph.sisa[1] = data_BDSEPH.SISAIocb;
    eph.sisa[2] = data_BDSEPH.SISAIoc1;
    eph.sisa[3] = data_BDSEPH.SISAIoc2;

    if (!strstr(raw->opt, "-EPHALL")) {
        if (fabs(timediff(raw->nav.eph[sat - 1].toe, eph.toe)) < 1e-9 &&
            fabs(timediff(raw->nav.eph[sat - 1].toc, eph.toc)) < 1e-9) return 0;
    }

    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    PPP_Glo.BDS_CNV1_flag = 1;

    return 2;
}




/* decode UM980/982 message -----------------------------------------*/
/*H24*8---data???---CRC32*/
static int decode_unicore(raw_t* raw)
{
    double tow;
    char tstr[32];
    int stat, week, type = U2(raw->buff + 4);
    int ret = 0;

    trace(3, "decode_unicore: type=%3d len=%d\n", type, raw->len);

    /* check crc32 */
    if (rtk_crc32(raw->buff, raw->len) != U4(raw->buff + raw->len)) {
        trace(2, "unicore crc error: type=%3d len=%d\n", type, raw->len);
        return -1;
    }
    stat = U1(raw->buff + 9);
    week = U2(raw->buff + 10);

    if (stat == 201 || week == 0) {
        trace(3, "unicore time error: type=%3d stat=%d week=%d\n", type,
            stat, week);
        return 0;
    }
    // week = adjgpsweek(week);
    tow = U4(raw->buff + 12) * 0.001;
    raw->time = gpst2time(week, tow);
    double ep[6];
    time2epoch(raw->time, ep);

    if (raw->outtype) {
        time2str(gpst2time(week, tow), tstr, 2);
        sprintf(raw->msgtype, "UNICORE %4d (%4d): %s", type, raw->len, tstr);
    }

    int prev_inf01 = raw->num_PPPB2BINF01;
    int prev_inf02 = raw->num_PPPB2BINF02;
    int prev_inf03 = raw->num_PPPB2BINF03;
    int prev_inf04 = raw->num_PPPB2BINF04;

    switch (type) {
        case PPPPB2BINFO1: {
            raw->num_PPPB2BINF01++; 
            ret =  decode_PPPPB2BINFO1(raw);break;
            }
        case PPPPB2BINFO2: {
            raw->num_PPPB2BINF02++; 
            ret =  decode_PPPPB2BINFO2(raw);break;}
        case PPPPB2BINFO3: {
            raw->num_PPPB2BINF03++; 
            ret =  decode_PPPPB2BINFO3(raw);break;}
        case PPPPB2BINFO4: {
            raw->num_PPPB2BINF04++; 
            ret =  decode_PPPPB2BINFO4(raw);break;}
        case GPSEPH:{
            ret =  decode_GPSEPH(raw);break;}
        case BD3EPH:{
            ret =  decode_BDSEPH(raw);break;}
        
    }

    if (ret >= 0) {
        if (raw->num_PPPB2BINF01>prev_inf01) {
            raw->raw_nmsg[0]++;  /* INFO1 */
        } 
        else if (raw->num_PPPB2BINF02>prev_inf02) {
            raw->raw_nmsg[1]++;  /* INFO2 */
        } 
        else if (raw->num_PPPB2BINF03>prev_inf03) {
            raw->raw_nmsg[2]++;  /* INFO3 */
        } 
        else if (raw->num_PPPB2BINF04>prev_inf04) {
            raw->raw_nmsg[3]++;  /* INFO4 */
        } 
        else if(type == GPSEPH){
            raw->raw_nmsg[8]++;
        }
        else if(type == BD3EPH){
            raw->raw_nmsg[9]++;
        }
        else {
            raw->raw_nmsg[7]++;  /* B2b_Unknown */
        }
        
    }
    return ret;
}


/* input UM980/982 message -----------------------------------------*/
/*H24*8---data???---CRC32*/
extern int input_unicore(raw_t *raw,uint8_t data){

    // trace(5,"input_unicore_B2b: data=%02x\n",data);
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        if (sync_unicore(raw->buff,data)) raw->nbyte=3;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;

    // Get Message length
    if (raw->nbyte==8&&(raw->len=U2(raw->buff+6)+UNICORE_LEN)>MAX_UNICORE_LEN-4) {
        trace(2,"sinan length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (raw->nbyte<8||raw->nbyte<raw->len+4) return 0;
    raw->nbyte=0;
    
    /* decode sinan message */
    return decode_unicore(raw);
}

extern int input_unicoref(raw_t *raw, FILE *fp)
{
    int i,data=0,ret;

    // trace(5,"input_unicore_B2bf");

    for (i=0;i<4096;i++) {
        if ((data=fgetc(fp))==EOF) return -2;
        if ((ret=input_unicore(raw,(uint8_t)data))) return ret;
    }
    return 0;
}

