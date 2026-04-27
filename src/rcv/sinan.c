/*------------------------------------------------------------------------------
* sinan.c : Decoder for Shanghai Sinan custom receiver format
*
*           Developed by Xu@CUGB and Liu@APM
*
* description : This file contains the implementation for parsing Shanghai
*               Sinan company receiver's custom binary format. The decoder
*               includes strategies for data reading and partial decoding.
* options : None
*
* version : $Revision:$ $Date:$
* history : 2024/10/30   Initial version by Xu@CUGB
*           2024/11/19   Improved reading strategies and decoding methods,
*                        changed cpp to c by Liu@APM
*
*-----------------------------------------------------------------------------*/


#include "rtklib.h"
#include "B2b.h"


#define SINAN_SYNC1       0xAA    /* message start sync code 1 */
#define SINAN_SYNC2       0x44    /* message start sync code 2 */
#define SINAN_SYNC3       0x12    /* message start sync code 3 */

#define SINAN_LEN         28      /* message header length (bytes) */
#define MAX_SINAN_LEN     16384     /* max message length:H(28)+D(127)+CRC(4) (bytes) */
// Liu@APM:B2b length is too short for another message


#define B2BRAWNAVSUBFRAMEb 1697   /*sinan B2b type */
#define GPSEPHEM 71
#define BD3EPHEM 72

static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}

B2bmask_t sinan_mask ={
    .MASK_BD = {0},
    .MASK_GPS = {0},
    .MASK_GALILEO = {0},
    .MASK_GLONASS = {0},
    .IOD_SSR = -1,
    .IODP = -1,
    .satnum = -1,
    .satno = {0}
};

/* sync header ---------------------------------------------------------------*/
static int sync_sinan(uint8_t *buff, uint8_t data)
{
    buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=data;
    return buff[0]==SINAN_SYNC1&&buff[1]==SINAN_SYNC2&&buff[2]==SINAN_SYNC3;
}

static int process_message_type_1(raw_t *raw, int* pose)
{
    const uint8_t* buffer = raw->buff;
    int num,udi = 0;
    gtime_t temp_time = {0};

    temp_time = sinan_mask.time;
    
    memset(&sinan_mask.time, 0, sizeof(sinan_mask.time));
    memset(&sinan_mask.m_time, 0, sizeof(sinan_mask.m_time));
    memset(sinan_mask.MASK_BD, 0, sizeof(sinan_mask.MASK_BD));
    memset(sinan_mask.MASK_GPS, 0, sizeof(sinan_mask.MASK_GPS));
    memset(sinan_mask.MASK_GALILEO, 0, sizeof(sinan_mask.MASK_GALILEO));
    memset(sinan_mask.MASK_GLONASS, 0, sizeof(sinan_mask.MASK_GLONASS));
    sinan_mask.IOD_SSR = sinan_mask.IODP = 0;

    uint32_t sow = getbitu(buffer, *pose, 17); *pose += 17;
    sinan_mask.time = B2btod2time(raw->time,sow);
    sinan_mask.m_time = raw->time;
    
    if (temp_time.time == 0 && temp_time.sec == 0.0) {
        udi = 0;
    } else {
        udi = timediff(sinan_mask.time,temp_time);
    }

    uint32_t reserve2 = getbitu(buffer, *pose, 4); *pose += 4;
    sinan_mask.IOD_SSR = getbitu(buffer, *pose, 2); *pose += 2;
    sinan_mask.IODP = getbitu(buffer, *pose, 4); *pose += 4;

    for (int i = 0; i < 63; i++) {
        sinan_mask.MASK_BD[i] = getbitu(buffer, *pose, 1);
        *pose += 1;
    }

    for (int i = 0; i < 37; i++) {
        sinan_mask.MASK_GPS[i] = getbitu(buffer, *pose, 1);
        *pose += 1;
    }

    for (int i = 0; i < 37; i++) {
        sinan_mask.MASK_GALILEO[i] = getbitu(buffer, *pose, 1);
        *pose += 1;
    }

    for (int i = 0; i < 37; i++) {
        sinan_mask.MASK_GLONASS[i] = getbitu(buffer, *pose, 1);
        *pose += 1;
    }

    num = mask2satno(&sinan_mask);

    uint64_t MASK_reserve = getbitu(buffer, *pose, 81); *pose += 81;
    uint64_t reserve_type1 = getbitu(buffer, *pose, 174); *pose += 174;

    output_B2bInfo1(raw, &sinan_mask,udi);
    return 20;

}

static int process_message_type_2(raw_t *raw, int* pose)
{
    const uint8_t* buffer = raw->buff;
    int satno,satslot;
    char satid[8];
    gtime_t m_time;
    double m_time_ep[8];
    

	uint32_t sow = getbitu(buffer, *pose, 17); *pose += 17;
    m_time = B2btod2time(raw->time,sow);
    time2epoch(m_time,m_time_ep);
    int verify_sow = (int)(m_time_ep[3]*60*60 + m_time_ep[4]*60 + m_time_ep[5]);

	uint32_t reserve2 = getbitu(buffer, *pose, 4); *pose += 4;

	uint32_t IOD_SSR = getbitu(buffer, *pose, 2); *pose += 2;

    if (IOD_SSR != sinan_mask.IOD_SSR) {
        // printf("error: msg2(eph), new iodssr=%d,msg->iodssr=%d\n", 
        // IOD_SSR, sinan_mask.IOD_SSR);
        trace(22,"error: msg2(eph), new iodssr=%d,msg->iodssr=%d\n", 
        IOD_SSR, sinan_mask.IOD_SSR);
        return 0;
    }

	for (int i = 0; i < 6; i++) {
		satslot = getbitu(buffer, *pose, 9); *pose += 9;
        satno = slot2satno(satslot);

        raw->nav.B2bssr[satno].iodssr[0] = IOD_SSR;
        raw->nav.B2bssr[satno].t0[0] = m_time;
        raw->nav.B2bssr[satno].sow = sow;
        raw->nav.B2bssr[satno].verify_sow = verify_sow;

        raw->nav.B2bssr[satno].iodn = getbitu(buffer, *pose, 10); *pose += 10;

        raw->nav.B2bssr[satno].iodcorr[0] = getbitu(buffer, *pose, 3); *pose += 3;
        
        int sRadial = getbits(buffer, *pose, 15); *pose += 15;
        
        int sInTrack = getbits(buffer, *pose, 13); *pose += 13;
        
        int sCross = getbits(buffer, *pose, 13); *pose += 13;
        
        if (abs(sRadial) >= 16383 ||abs(sInTrack) >= 4095 ||abs(sCross) >= 4095) {
                satno2id(satno,satid);
                trace(22,"error: bad-orb, sat=%s,deph=%8.3f,%8.3f,%8.3f\n", 
                satid, raw->nav.B2bssr[satno].deph[0], raw->nav.B2bssr[satno].deph[1], raw->nav.B2bssr[satno].deph[2]);
                // printf("error: bad-orb, sat=%s,deph=%8.3f,%8.3f,%8.3f\n", 
                // satid, raw->nav.B2bssr[satno].deph[0], raw->nav.B2bssr[satno].deph[1], raw->nav.B2bssr[satno].deph[2]);
                continue;
        }

        raw->nav.B2bssr[satno].deph[0] = sRadial * 0.0016; 
        raw->nav.B2bssr[satno].deph[1] = sInTrack* 0.0064;
        raw->nav.B2bssr[satno].deph[2] = sCross* 0.0064;

        raw->nav.B2bssr[satno].ura = getbitu(buffer, *pose, 6); *pose += 6;

        raw->nav.B2bssr[satno].update = 1;
	}
	uint32_t reserve_info2 = getbitu(buffer, *pose, 6); *pose += 6;
    output_B2bInfo2(raw,&raw->nav);
    return 20;
}

static int process_message_type_3(raw_t *raw, int* pose)
{
    const uint8_t* buffer = raw->buff;
    int satno,satslot,sys,type,*cods = NULL;
    double m_time_ep[8];
    char satid[8];
    gtime_t m_time;

    uint32_t sow = getbitu(buffer, *pose, 17); *pose += 17;
    m_time = B2btod2time(raw->time,sow);
    time2epoch(m_time,m_time_ep);
    int verify_sow = (int)(m_time_ep[3]*60*60 + m_time_ep[4]*60 + m_time_ep[5]);


    uint32_t reserve2 = getbitu(buffer, *pose, 4); *pose += 4;
    uint32_t IOD_SSR = getbitu(buffer, *pose, 2); *pose += 2;
    if (IOD_SSR != sinan_mask.IOD_SSR) {
        // printf("error: msg3(cbia), new iodssr=%d,msg->iodssr=%d\n", 
        // IOD_SSR, sinan_mask.IOD_SSR);
        trace(22,"error: msg3(cbia), new iodssr=%d,msg->iodssr=%d\n", 
        IOD_SSR, sinan_mask.IOD_SSR);
        return 0;
    }

    uint32_t SatNum = getbitu(buffer, *pose, 5); *pose += 5;

    for (int i = 0; i < SatNum; i++) {
        satslot = getbitu(buffer, *pose, 9); *pose += 9;
        satno = slot2satno(satslot);
        satno2id(satno,satid);
        sys = satsys(satno, NULL);
        if (sys == SYS_GPS) cods = b2b_gps_codebias_mode;
        else if (sys == SYS_GLO) cods = b2b_glo_codebias_mode;
        else if (sys == SYS_GAL) cods = b2b_gal_codebias_mode;
        else if (sys == SYS_CMP) cods = b2b_bds_codebias_mode;
        else continue;

        raw->nav.B2bssr[satno].iodssr[1] = IOD_SSR;
        raw->nav.B2bssr[satno].t0[1] = m_time;
        raw->nav.B2bssr[satno].sow = sow;
        raw->nav.B2bssr[satno].verify_sow = verify_sow;

        uint32_t SigNum = getbitu(buffer, *pose, 4); *pose += 4;


        for (int j = 0; j < SigNum; j++) {
            uint32_t mode = getbitu(buffer, *pose, 4); *pose += 4;
            int32_t DCB = getbits(buffer, *pose, 12); *pose += 12;

            if (abs(DCB) >= 2103) {
                continue;
            }
            if (mode < 0 || mode >= B2B_CodeBiasModeNum) {
                continue;
            }
            type = cods[mode];
            if (type == CODE_NONE) continue;
            raw->nav.B2bssr[satno].cbias[type] = DCB*0.017;
            raw->nav.B2bssr[satno].update = 1;
        }
    }

    uint32_t reserve3 = getbitu(buffer, *pose, 10); *pose += 10;
    output_B2bInfo3(raw,&raw->nav);
    return 20;
}



static int process_message_type_4(raw_t *raw, int* pose)
{
    const uint8_t* buffer = raw->buff;
    int satno,satslot,sys,type,flag, i, *cods = NULL;
    double m_time_ep[8];
    char satid[8],time_str[64];
    gtime_t m_time;

    uint32_t sow = getbitu(buffer, *pose, 17); *pose += 17;
    m_time = B2btod2time(raw->time,sow);

    uint32_t reserve2 = getbitu(buffer, *pose, 4); *pose += 4;
    uint32_t IOD_SSR = getbitu(buffer, *pose, 2); *pose += 2;
    uint32_t IODP = getbitu(buffer, *pose, 4); *pose += 4;
    uint32_t SubTypel = getbitu(buffer, *pose, 5); *pose += 5;

     if (IOD_SSR != sinan_mask.IOD_SSR || IODP != sinan_mask.IODP) {
        // printf("error: msg4(clk), new iodssr=%d,msg->iodssr=%d,iodp=%d,msg->iodp=%d\n", 
        // IOD_SSR, sinan_mask.IOD_SSR, IODP, sinan_mask.IODP);
        trace(22,"error: msg4(clk), new iodssr=%d,msg->iodssr=%d,iodp=%d,msg->iodp=%d\n", 
        IOD_SSR, sinan_mask.IOD_SSR, IODP, sinan_mask.IODP);
        return 0;
    }


    if (SubTypel > 31) {
        trace(4, "error: msg4(clk), bad subtype1=%d\n", SubTypel);
    }

    time2str(m_time,time_str,3);

    int begin = SubTypel * 23;

    for (i = 0; i < 23; i++){
        if(sinan_mask.satno[begin+i] != 0){
            satno = sinan_mask.satno[begin+i];
            raw->nav.B2bssr[satno].t0[2] = m_time;
            raw->nav.B2bssr[satno].sow = sow;
            time2epoch(m_time,m_time_ep);
            raw->nav.B2bssr[satno].verify_sow = (int)(m_time_ep[3]*60*60 + m_time_ep[4]*60 + m_time_ep[5]);
            raw->nav.B2bssr[satno].iodssr[2] = IOD_SSR;
            raw->nav.B2bssr[satno].iodp[0] = IODP;
            raw->nav.B2bssr[satno].iodcorr[1] = getbitu(buffer, *pose, 3); *pose += 3;

            int sC0 = getbits(buffer, *pose, 15); *pose += 15;

            if (abs(sC0) >= 16383 || raw->nav.B2bssr[satno].iodcorr[1] > 7) {
                satno2id(satno,satid);
					trace(22, "error: msg4(clk) bad-clk, sat=%s,iodcor=%d,c0=%d \n", satid, 
                    raw->nav.B2bssr[satno].iodcorr[1], sC0);
					continue;
				}
            
            raw->nav.B2bssr[satno].dclk[0] = 0.0016 * sC0;
            raw->nav.B2bssr[satno].update = 1;
        }
    }
    uint32_t reserve3 = getbitu(buffer, *pose, 10); *pose += 10;
    output_B2bInfo4(raw,&raw->nav);
    return 20;
}

/*H28*8---prn32---prn6---reserve6---mes_type6---data966---CRC32*/
static int decode_B2b(raw_t *raw)
{

    int pose = SINAN_LEN*8;
    // "PRN_32 and PRN_6 in the message should be consistent"? See "Sinan B2b Navigation Message Description - V1.0.pdf"
    // Liu@APM: Actual testing shows inconsistency, so this check is canceled.
    uint32_t prn_32 = getbitu(raw->buff, pose, 32); pose = pose + 32;
    uint32_t prn_6 = getbitu(raw->buff, pose, 6); pose = pose + 6;
    uint32_t reserve = getbitu(raw->buff, pose, 6); pose = pose + 6;
    uint32_t mes_type = getbitu(raw->buff, pose, 6); pose = pose + 6;

    raw->geoprn = prn_6;

    if (raw->geoprn == 62) {
        char time_str[128];
        time2str(raw->time, time_str, 3);
        trace(22, "Skipping PRN 62 at time %s: prn_6 = %u, mes_type = %u \n",
              time_str, prn_6, mes_type);
        return 0;
    }

    switch (mes_type) {
		case 1:
            raw->num_PPPB2BINF01++;
			return process_message_type_1(raw, &pose);
		case 2:
            raw->num_PPPB2BINF02++;
			return process_message_type_2(raw, &pose);
		case 3:
            raw->num_PPPB2BINF03++;
			return process_message_type_3(raw, &pose);
		case 4:
            raw->num_PPPB2BINF04++;
            return process_message_type_4(raw, &pose);
    }
    return 1;
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

static int decode_GPSEPHEM(raw_t *raw) {
    PACKED_SINO_GPSEPHEM data_GPSEPH = {0};
    memcpy(&data_GPSEPH, raw->buff + SINAN_LEN, sizeof(data_GPSEPH));

    eph_t eph={0};
    double tow;
    int sat, week;

    if(!data_GPSEPH.blFlag){
        trace(2, "unicore gpsephb satellite error: prn=%d\n", data_GPSEPH.ID);
        return 0;
    }

    if (!(sat = satno(SYS_GPS, data_GPSEPH.ID))) {
        trace(2, "unicore gpsephb satellite error: prn=%d\n", data_GPSEPH.ID);
        return 0;
    }

    eph.sva = data_GPSEPH.bHealth;
    eph.sat = sat;
    eph.iodc = data_GPSEPH.iodc;
    eph.sva  = uraindex(data_GPSEPH.accuracy);  /* URA index (m->index) */

    eph.week=data_GPSEPH.week + 2048;      /* GPS week */
    eph.iode=data_GPSEPH.iode;
    eph.toes = data_GPSEPH.toe;

    eph.toe = gpst2time( eph.week,  eph.toes);
    double tt = timediff(eph.toe, raw->time);
    if (tt < -302400.0) eph.week++;
    else if (tt > 302400.0) eph.week--;

    eph.toe = gpst2time( eph.week,  eph.toes);
    eph.toc = gpst2time( eph.week,  data_GPSEPH.toc);
    eph.ttr = raw->time;

    eph.f0  = data_GPSEPH.af0;
    eph.f1  = data_GPSEPH.af1;
    eph.f2  = data_GPSEPH.af2;
    eph.M0  = data_GPSEPH.Ms0;
    eph.deln= data_GPSEPH.deltan;
    eph.e   = data_GPSEPH.es;
    eph.A   = data_GPSEPH.roora * data_GPSEPH.roora;
    eph.OMG0= data_GPSEPH.Omega0;
    eph.i0  = data_GPSEPH.i0;
    eph.omg = data_GPSEPH.ws;
    eph.OMGd= data_GPSEPH.omegaot;
    eph.idot= data_GPSEPH.itoet;
    eph.cuc = data_GPSEPH.cuc;
    eph.cus = data_GPSEPH.cus;
    eph.crc = data_GPSEPH.crc;
    eph.crs = data_GPSEPH.crs;
    eph.cic = data_GPSEPH.cic;
    eph.cis = data_GPSEPH.cis;
    eph.tgd[0] = data_GPSEPH.tgd;

    if (!strstr(raw->opt, "-EPHALL")) {
        if (fabs(timediff(raw->nav.eph[sat - 1].toe, eph.toe)) < 1e-9 &&
            fabs(timediff(raw->nav.eph[sat - 1].toc, eph.toc)) < 1e-9) return 0;
    }

    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}


static int decode_BD3EPHEM(raw_t *raw) {
    PACKED_SINO_BD3EPHEM data_BDSEPH = {0};
    memcpy(&data_BDSEPH, raw->buff + SINAN_LEN, sizeof(data_BDSEPH));

    eph_t eph = { 0 };
    gtime_t gps_toe = {0};
    int sat, sat_type, gps_week, gps_tow, ref_A;

    if (!(sat = satno(SYS_CMP, data_BDSEPH.Prn))) {
        trace(22, "unicore gpsephb satellite error: prn=%d\n", data_BDSEPH.Prn);
        return 0;
    }
    eph.sat = sat;
    sat_type = data_BDSEPH.sattype;
    eph.svh = data_BDSEPH.health;
    eph.sva = data_BDSEPH.URAI;
    eph.iode= data_BDSEPH.IODE;
    eph.iodc= data_BDSEPH.IODC;

    int sif = data_BDSEPH.SIF;
    if (sif != 0){ // sif is narmal
        trace(22,"The signal is incomplete! SIF = %d \n",sif);
        return 0;
    }

    int aif = data_BDSEPH.AIF;
    if (aif != 0){ // sif is narmal
        trace(22,"The SISMAI is effective! AIF = %d \n",aif);
        return 0;
    }
    gps_tow  = time2gpst(raw->time,&gps_week); // week con by ttr
    gps_toe  = gpst2time(gps_week,data_BDSEPH.toe); // TOE in gpst

    double tt = timediff(gps_toe, raw->time);
    if (tt < -302400.0) gps_week++;
    else if (tt > 302400.0) gps_week--;

    eph.week = gps_week - 1356; // GPS WEEK --->BDS WEEK
    eph.toes = data_BDSEPH.toe;
    eph.toe = bdt2gpst(bdt2time(eph.week,eph.toes));
    eph.toc = bdt2gpst(bdt2time(eph.week,data_BDSEPH.toc));
    eph.ttr = raw->time;

    // ref_A + delt_A = sqrt(A) * sqrt(A)
    if (sat_type ==1) ref_A  = 42162200; //(GEO)
    else if (sat_type ==2) ref_A  = 42162200; //(IGSO)
    else if (sat_type ==3) ref_A  = 27906100; //(MEO)

    eph.A = ref_A + data_BDSEPH.Delt_A;
    eph.Adot = data_BDSEPH.Dot_A;
    eph.deln = data_BDSEPH.Delt_n0;
    eph.ndot = data_BDSEPH.Dot_n0;
    eph.M0   = data_BDSEPH.M0;
    eph.e    = data_BDSEPH.e;
    eph.omg  = data_BDSEPH.w;
    eph.OMG0 = data_BDSEPH.Omega0;
    eph.i0   = data_BDSEPH.i0;
    eph.OMGd = data_BDSEPH.Omega_dot;
    eph.idot = data_BDSEPH.i_dot;

    eph.cuc = data_BDSEPH.Cuc;
    eph.cus = data_BDSEPH.Cus;
    eph.crc = data_BDSEPH.Crc;
    eph.crs = data_BDSEPH.Crs;
    eph.cic = data_BDSEPH.Cic;
    eph.cis = data_BDSEPH.Cis;

    eph.f0  = data_BDSEPH.a0;
    eph.f1  = data_BDSEPH.a1;
    eph.f2  = data_BDSEPH.a2;

    eph.tgd[2] = data_BDSEPH.tgdB1Cp;
    eph.tgd[4] = data_BDSEPH.tgdB1Cd;

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



/* decode SINAN K803 LITE/W/S message -----------------------------------------*/
/*H28*8---prn32---prn6---reserve6---mes_type6---data966---CRC32*/
static int decode_sino(raw_t *raw)
{
    double tow;
    char tstr[32];
    int msg,stat,week,type=U2(raw->buff+4);
    int ret = 0;
    
    trace(3,"decode_sino: type=%3d len=%d\n",type,raw->len);
    
    /* check crc32 */
    if (rtk_crc32(raw->buff,raw->len)!=U4(raw->buff+raw->len)) {
        trace(2,"SINO crc error: type=%3d len=%d\n",type,raw->len);
        return -1;
    }
    // msg =(U1(raw->buff+6)>>4)&0x3; /* message type: 0=binary,1=ascii */
    // stat=U1(raw->buff+13);
    week=U2(raw->buff+14);
    
    if (stat==20||week==0) {
        trace(3,"SINO time error: type=%3d msg=%d stat=%d week=%d\n",type,msg,
              stat,week);
        return 0;
    }
    // week=adjgpsweek(week); 
    tow =U4(raw->buff+16)*0.001;
    raw->time=gpst2time(week,tow);
    char time_str[64];
    time2str(raw->time,time_str,5);
    // if (msg!=0) return 0;
    
    if (raw->outtype) {
        time2str(gpst2time(week,tow),tstr,2);
        sprintf(raw->msgtype,"SINO %4d (%4d): %s",type,raw->len,tstr);
    }


    int prev_inf01 = raw->num_PPPB2BINF01;
    int prev_inf02 = raw->num_PPPB2BINF02;
    int prev_inf03 = raw->num_PPPB2BINF03;
    int prev_inf04 = raw->num_PPPB2BINF04;

    switch (type) {
        case B2BRAWNAVSUBFRAMEb: ret = decode_B2b(raw); break;
        case GPSEPHEM:           ret = decode_GPSEPHEM(raw); break;
        case BD3EPHEM:           ret = decode_BD3EPHEM(raw); break;

    }

    if (ret >= 0) {
        if (type == B2BRAWNAVSUBFRAMEb) {

            if (raw->num_PPPB2BINF01 > prev_inf01) {
                raw->raw_nmsg[0]++;  /* INFO1 */
            } else if (raw->num_PPPB2BINF02 > prev_inf02) {
                raw->raw_nmsg[1]++;  /* INFO2 */
            } else if (raw->num_PPPB2BINF03 > prev_inf03) {
                raw->raw_nmsg[2]++;  /* INFO3 */
            } else if (raw->num_PPPB2BINF04 > prev_inf04) {
                raw->raw_nmsg[3]++;  /* INFO4 */
            } else {
                raw->raw_nmsg[7]++;  /* B2b_Unknown */
            }
        }
        else if(type == GPSEPHEM){
            raw->raw_nmsg[8]++;
        }
        else if(type == BD3EPHEM){
            raw->raw_nmsg[9]++;
        }
        
    }
    return ret;
}


/*H28*8---prn32---prn6---reserve6---mes_type6---data966---CRC32*/
extern int input_sino(raw_t *raw, uint8_t data)
{
    // trace(5,"input_sinan_B2b: data=%02x\n",data);
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        if (sync_sinan(raw->buff,data)) raw->nbyte=3;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;
    
    // Get Message length
    if (raw->nbyte==10&&(raw->len=U2(raw->buff+8)+SINAN_LEN)>MAX_SINAN_LEN-4) {
        trace(2,"sinan length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (raw->nbyte<10||raw->nbyte<raw->len+4) return 0;
    raw->nbyte=0;
    
    /* decode sinan message */
    return decode_sino(raw);
}


extern int input_sinof(raw_t *raw, FILE *fp)
{
    int i, data = 0, ret;

    if (raw == NULL) {
        fprintf(stderr, "Error: raw is NULL\n");
        return -1;
    }
    if (fp == NULL) {
        fprintf(stderr, "Error: File pointer is NULL\n");
        return -1;
    }

    // trace(5,"input_sinof");

    for (i = 0; i < 4096; i++) {
        data = fgetc(fp);
        if (data == EOF) return -2;
        ret = input_sino(raw, (uint8_t)data);
        if (ret) return ret;
    }

    return 0;
}
