/*------------------------------------------------------------------------------
* ephemeris.c : satellite ephemeris and clock functions
*
*          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
*
* references :
*     [1] IS-GPS-200K, Navstar GPS Space Segment/Navigation User Interfaces,
*         May 6, 2019
*     [2] Global Navigation Satellite System GLONASS, Interface Control Document
*         Navigational radiosignal In bands L1, L2, (Version 5.1), 2008
*     [3] RTCA/DO-229C, Minimum operational performance standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
*     [4] RTCM Paper, April 12, 2010, Proposed SSR Messages for SV Orbit Clock,
*         Code Biases, URA
*     [5] RTCM Paper 012-2009-SC104-528, January 28, 2009 (previous ver of [4])
*     [6] RTCM Paper 012-2009-SC104-582, February 2, 2010 (previous ver of [4])
*     [7] European GNSS (Galileo) Open Service Signal In Space Interface Control
*         Document, Issue 1.3, December, 2016
*     [8] Quasi-Zenith Satellite System Interface Specification Satellite
*         Positioning, Navigation and Timing Service (IS-QZSS-PNT-003), Cabinet
*         Office, November 5, 2018
*     [9] BeiDou navigation satellite system signal in space interface control
*         document open service signal B1I (version 3.0), China Satellite
*         Navigation office, February, 2019
*     [10] RTCM Standard 10403.3, Differential GNSS (Global Navigation
*         Satellite Systems) Services - version 3, October 7, 2016
*
* version : $Revision:$ $Date:$
* history : 2010/07/28 1.1  moved from rtkcmn.c
*                           added api:
*                               eph2clk(),geph2clk(),seph2clk(),satantoff()
*                               satposs()
*                           changed api:
*                               eph2pos(),geph2pos(),satpos()
*                           deleted api:
*                               satposv(),satposiode()
*           2010/08/26 1.2  add ephemeris option EPHOPT_LEX
*           2010/09/09 1.3  fix problem when precise clock outage
*           2011/01/12 1.4  add api alm2pos()
*                           change api satpos(),satposs()
*                           enable valid unhealthy satellites and output status
*                           fix bug on exception by glonass ephem computation
*           2013/01/10 1.5  support beidou (compass)
*                           use newton's method to solve kepler eq.
*                           update ssr correction algorithm
*           2013/03/20 1.6  fix problem on ssr clock relativistic correction
*           2013/09/01 1.7  support negative pseudorange
*                           fix bug on variance in case of ura ssr = 63
*           2013/11/11 1.8  change constant MAXAGESSR 70.0 -> 90.0
*           2014/10/24 1.9  fix bug on return of var_uraeph() if ura<0||15<ura
*           2014/12/07 1.10 modify MAXDTOE for qzss,gal and bds
*                           test max number of iteration for Kepler
*           2015/08/26 1.11 update RTOL_ELPLER 1E-14 -> 1E-13
*                           set MAX_ITER_KEPLER for alm2pos()
*           2017/04/11 1.12 fix bug on max number of obs data in satposs()
*           2018/10/10 1.13 update reference [7]
*                           support ura value in var_uraeph() for galileo
*                           test eph->flag to recognize beidou geo
*                           add api satseleph() for ephemeris selection
*           2020/11/30 1.14 update references [1],[2],[8],[9] and [10]
*                           add API getseleph()
*                           rename API satseleph() as setseleph()
*                           support NavIC/IRNSS by API satpos() and satposs()
*                           support BDS C59-63 as GEO satellites in eph2pos()
*                           default selection of I/NAV for Galileo ephemeris
*                           no support EPHOPT_LEX by API satpos() and satposs()
*                           unselect Galileo ephemeris with AOD<=0 in seleph()
*                           fix bug on clock iteration in eph2clk(), geph2clk()
*                           fix bug on clock reference time in satpos_ssr()
*                           fix bug on wrong value with ura=15 in var_ura()
*                           use integer types in stdint.h
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include <sys/stat.h>
/* constants and macros ------------------------------------------------------*/

#define SQR(x)   ((x)*(x))

#define RE_GLO   6378136.0        /* radius of earth (m)            ref [2] */
#define MU_GPS   3.9860050E14     /* gravitational constant         ref [1] */
#define MU_GLO   3.9860044E14     /* gravitational constant         ref [2] */
#define MU_GAL   3.986004418E14   /* earth gravitational constant   ref [7] */
#define MU_CMP   3.986004418E14   /* earth gravitational constant   ref [9] */
#define J2_GLO   1.0826257E-3     /* 2nd zonal harmonic of geopot   ref [2] */

#define OMGE_GLO 7.292115E-5      /* earth angular velocity (rad/s) ref [2] */
#define OMGE_GAL 7.2921151467E-5  /* earth angular velocity (rad/s) ref [7] */
#define OMGE_CMP 7.292115E-5      /* earth angular velocity (rad/s) ref [9] */

#define MEO_A_ref  27906100
#define IGSOGEO_A_ref  42162200

#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */

#define ERREPH_GLO 5.0            /* error of glonass ephemeris (m) */
#define TSTEP    60.0             /* integration step glonass ephemeris (s) */
#define RTOL_KEPLER 1E-13         /* relative tolerance for Kepler equation */

#define DEFURASSR 0.15            /* default accuracy of ssr corr (m) */
#define MAXECORSSR 10.0          /* max orbit correction of ssr (m) */
#define MAXCCORSSR (1E-6*CLIGHT)  /* max clock correction of ssr (m) */
#define MAXAGESSR 100.0            /* max age of ssr orbit and clock (s) */
#define B2beph_MAXAGESSR  96+30           /* max age of B2b orbit and eph (s) */
#define B2bclk_MAXAGESSR  12+30            /* max age of B2b clock (s) */
#define MAXAGESSR_HRCLK 10.0      /* max age of ssr high-rate clock (s) */
#define STD_BRDCCLK 30.0          /* error of broadcast clock (m) */
#define STD_GAL_NAPA 500.0        /* error of galileo ephemeris for NAPA (m) */

#define MAX_ITER_KEPLER 30        /* max number of iteration of Kepler */

char sp3file[128];
int sp3init_flag = 0;
int sp3continue_flag = 0;

/* ephemeris selections ------------------------------------------------------*/
static int eph_sel[]={ /* GPS,GLO,GAL,QZS,BDS,IRN,SBS */
    0,0,0,0,0,0,0
};

/* variance by ura ephemeris -------------------------------------------------*/
static double var_uraeph(int sys, int ura)
{
    const double ura_value[]={
        2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
        3072.0,6144.0
    };
    if (sys==SYS_GAL) { /* galileo sisa (ref [7] 5.1.11) */
        if (ura<= 49) return SQR(ura*0.01);
        if (ura<= 74) return SQR(0.5+(ura- 50)*0.02);
        if (ura<= 99) return SQR(1.0+(ura- 75)*0.04);
        if (ura<=125) return SQR(2.0+(ura-100)*0.16);
        return SQR(STD_GAL_NAPA);
    }
    else { /* gps ura (ref [1] 20.3.3.3.1.1) */
        return ura<0||14<ura?SQR(6144.0):SQR(ura_value[ura]);
    }
}
/* variance by ura ssr (ref [10] table 3.3-1 DF389) --------------------------*/
static double var_urassr(int ura)
{
    double std;
    if (ura<= 0) return SQR(DEFURASSR);
    if (ura>=63) return SQR(5.4665);
    std=(pow(3.0,(ura>>3)&7)*(1.0+(ura&7)/4.0)-1.0)*1E-3;
    return SQR(std);
}
/* variance by SISA ephemeris -------------------------------------------------*/
static double var_uraephSISA(int sys, const int *sisa)
{
    return 0;
}
static double var_uraB2b(int ura)
{
    // double std;
    // if (ura<= 0) return SQR(DEFURASSR);
    // if (ura>=63) return SQR(5.4665);
    // std=(pow(3.0,(ura>>3)&7)*(1.0+(ura&7)/4.0)-1.0)*1E-3;
    // return SQR(std);
    return 0;
}
/* almanac to satellite position and clock bias --------------------------------
* compute satellite position and clock bias with almanac (gps, galileo, qzss)
* args   : gtime_t time     I   time (gpst)
*          alm_t *alm       I   almanac
*          double *rs       O   satellite position (ecef) {x,y,z} (m)
*          double *dts      O   satellite clock bias (s)
* return : none
* notes  : see ref [1],[7],[8]
*-----------------------------------------------------------------------------*/
extern void alm2pos(gtime_t time, const alm_t *alm, double *rs, double *dts)
{
    double tk,M,E,Ek,sinE,cosE,u,r,i,O,x,y,sinO,cosO,cosi,mu;
    int n;

    trace(4,"alm2pos : time=%s sat=%2d\n",time_str(time,3),alm->sat);

    tk=timediff(time,alm->toa);

    if (alm->A<=0.0) {
        rs[0]=rs[1]=rs[2]=*dts=0.0;
        return;
    }
    mu=satsys(alm->sat,NULL)==SYS_GAL?MU_GAL:MU_GPS;

    M=alm->M0+sqrt(mu/(alm->A*alm->A*alm->A))*tk;
    for (n=0,E=M,Ek=0.0;fabs(E-Ek)>RTOL_KEPLER&&n<MAX_ITER_KEPLER;n++) {
        Ek=E; E-=(E-alm->e*sin(E)-M)/(1.0-alm->e*cos(E));
    }
    if (n>=MAX_ITER_KEPLER) {
        trace(2,"alm2pos: kepler iteration overflow sat=%2d\n",alm->sat);
    }
    sinE=sin(E); cosE=cos(E);
    u=atan2(sqrt(1.0-alm->e*alm->e)*sinE,cosE-alm->e)+alm->omg;
    r=alm->A*(1.0-alm->e*cosE);
    i=alm->i0;
    O=alm->OMG0+(alm->OMGd-OMGE)*tk-OMGE*alm->toas;
    x=r*cos(u); y=r*sin(u); sinO=sin(O); cosO=cos(O); cosi=cos(i);
    rs[0]=x*cosO-y*cosi*sinO;
    rs[1]=x*sinO+y*cosi*cosO;
    rs[2]=y*sin(i);
    *dts=alm->f0+alm->f1*tk;
}
/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias with broadcast ephemeris (gps, galileo, qzss)
* args   : gtime_t time     I   time by satellite clock (gpst)
*          eph_t *eph       I   broadcast ephemeris
* return : satellite clock bias (s) without relativity correction
* notes  : see ref [1],[7],[8]
*          satellite clock does not include relativity correction and tdg
*-----------------------------------------------------------------------------*/
extern double eph2clk(gtime_t time, const eph_t *eph)
{
    double t,ts;
    int i;

    trace(4,"eph2clk : time=%s sat=%2d\n",time_str(time,3),eph->sat);

    t=ts=timediff(time,eph->toc);

    for (i=0;i<2;i++) {
        t=ts-(eph->f0+eph->f1*t+eph->f2*t*t);
    }
    trace(4,"ephclk: t=%.12f ts=%.12f dts=%.12f f0=%.12f f1=%.9f f2=%.9f\n",t,ts,
        eph->f0+eph->f1*t+eph->f2*t*t,eph->f0,eph->f1,eph->f2);

    return eph->f0+eph->f1*t+eph->f2*t*t;
}
/* broadcast ephemeris to satellite position and clock bias --------------------
* compute satellite position and clock bias with broadcast ephemeris (gps,
* galileo, qzss)
* args   : gtime_t time     I   time (gpst)
*          eph_t *eph       I   broadcast ephemeris
*          double *rs       O   satellite position (ecef) {x,y,z} (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [1],[7],[8]
*          satellite clock includes relativity correction without code bias
*          (tgd or bgd)
*-----------------------------------------------------------------------------*/
extern void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts,
                    double *var)
{
    double tk,M,E,Ek,sinE,cosE,u,r,i,O,sin2u,cos2u,x,y,sinO,cosO,cosi,mu,omge;
    double xg,yg,zg,sino,coso;
    int n,sys,prn;

    trace(4,"eph2pos : time=%s sat=%2d\n",time_str(time,3),eph->sat);

    if (eph->A<=0.0) {
        rs[0]=rs[1]=rs[2]=*dts=*var=0.0;
        return;
    }
    tk=timediff(time,eph->toe);

    switch ((sys=satsys(eph->sat,&prn))) {
        case SYS_GAL: mu=MU_GAL; omge=OMGE_GAL; break;
        case SYS_CMP: mu=MU_CMP; omge=OMGE_CMP; break;
        default:      mu=MU_GPS; omge=OMGE;     break;
    }
    M=eph->M0+(sqrt(mu/(eph->A*eph->A*eph->A))+eph->deln)*tk;

    for (n=0,E=M,Ek=0.0;fabs(E-Ek)>RTOL_KEPLER&&n<MAX_ITER_KEPLER;n++) {
        Ek=E; E-=(E-eph->e*sin(E)-M)/(1.0-eph->e*cos(E));
    }
    if (n>=MAX_ITER_KEPLER) {
        trace(2,"eph2pos: kepler iteration overflow sat=%2d\n",eph->sat);
    }
    sinE=sin(E); cosE=cos(E);

    trace(4,"kepler: sat=%2d e=%8.5f n=%2d del=%10.3e\n",eph->sat,eph->e,n,E-Ek);

    u=atan2(sqrt(1.0-eph->e*eph->e)*sinE,cosE-eph->e)+eph->omg;
    r=eph->A*(1.0-eph->e*cosE);
    i=eph->i0+eph->idot*tk;
    sin2u=sin(2.0*u); cos2u=cos(2.0*u);
    u+=eph->cus*sin2u+eph->cuc*cos2u;
    r+=eph->crs*sin2u+eph->crc*cos2u;
    i+=eph->cis*sin2u+eph->cic*cos2u;
    x=r*cos(u); y=r*sin(u); cosi=cos(i);

    /* beidou geo satellite */
    if (sys==SYS_CMP&&(prn<=5||prn>=59)) { /* ref [9] table 4-1 */
        O=eph->OMG0+eph->OMGd*tk-omge*eph->toes;
        sinO=sin(O); cosO=cos(O);
        xg=x*cosO-y*cosi*sinO;
        yg=x*sinO+y*cosi*cosO;
        zg=y*sin(i);
        sino=sin(omge*tk); coso=cos(omge*tk);
        rs[0]= xg*coso+yg*sino*COS_5+zg*sino*SIN_5;
        rs[1]=-xg*sino+yg*coso*COS_5+zg*coso*SIN_5;
        rs[2]=-yg*SIN_5+zg*COS_5;
    }
    else {
        O=eph->OMG0+(eph->OMGd-omge)*tk-omge*eph->toes;
        sinO=sin(O); cosO=cos(O);
        rs[0]=x*cosO-y*cosi*sinO;
        rs[1]=x*sinO+y*cosi*cosO;
        rs[2]=y*sin(i);
    }
    tk=timediff(time,eph->toc);
    *dts=eph->f0+eph->f1*tk+eph->f2*tk*tk;

    /* relativity correction */
    *dts-=2.0*sqrt(mu*eph->A)*eph->e*sinE/SQR(CLIGHT);

    /* position and clock error variance */
    *var=var_uraeph(sys,eph->sva);
    trace(4,"eph2pos: sat=%d, dts=%.10f rs=%.4f %.4f %.4f var=%.3f\n",eph->sat,
        *dts,rs[0],rs[1],rs[2],*var);
}
/* glonass orbit differential equations --------------------------------------*/
static void deq(const double *x, double *xdot, const double *acc)
{
    double a,b,c,r2=dot3(x,x),r3=r2*sqrt(r2),omg2=SQR(OMGE_GLO);

    if (r2<=0.0) {
        xdot[0]=xdot[1]=xdot[2]=xdot[3]=xdot[4]=xdot[5]=0.0;
        return;
    }
    /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
    a=1.5*J2_GLO*MU_GLO*SQR(RE_GLO)/r2/r3; /* 3/2*J2*mu*Ae^2/r^5 */
    b=5.0*x[2]*x[2]/r2;                    /* 5*z^2/r^2 */
    c=-MU_GLO/r3-a*(1.0-b);                /* -mu/r^3-a(1-b) */
    xdot[0]=x[3]; xdot[1]=x[4]; xdot[2]=x[5];
    xdot[3]=(c+omg2)*x[0]+2.0*OMGE_GLO*x[4]+acc[0];
    xdot[4]=(c+omg2)*x[1]-2.0*OMGE_GLO*x[3]+acc[1];
    xdot[5]=(c-2.0*a)*x[2]+acc[2];
}
/* glonass position and velocity by numerical integration --------------------*/
static void glorbit(double t, double *x, const double *acc)
{
    double k1[6],k2[6],k3[6],k4[6],w[6];
    int i;

    deq(x,k1,acc); for (i=0;i<6;i++) w[i]=x[i]+k1[i]*t/2.0;
    deq(w,k2,acc); for (i=0;i<6;i++) w[i]=x[i]+k2[i]*t/2.0;
    deq(w,k3,acc); for (i=0;i<6;i++) w[i]=x[i]+k3[i]*t;
    deq(w,k4,acc);
    for (i=0;i<6;i++) x[i]+=(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i])*t/6.0;
}
/* glonass ephemeris to satellite clock bias -----------------------------------
* compute satellite clock bias with glonass ephemeris
* args   : gtime_t time     I   time by satellite clock (gpst)
*          geph_t *geph     I   glonass ephemeris
* return : satellite clock bias (s)
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
extern double geph2clk(gtime_t time, const geph_t *geph)
{
    double t,ts;
    int i;

    trace(4,"geph2clk: time=%s sat=%2d\n",time_str(time,3),geph->sat);

    t=ts=timediff(time,geph->toe);

    for (i=0;i<2;i++) {
        t=ts-(-geph->taun+geph->gamn*t);
    }
    trace(4,"geph2clk: t=%.12f ts=%.12f taun=%.12f gamn=%.12f\n",t,ts,geph->taun,
        geph->gamn);
    return -geph->taun+geph->gamn*t;
}
/* glonass ephemeris to satellite position and clock bias ----------------------
* compute satellite position and clock bias with glonass ephemeris
* args   : gtime_t time     I   time (gpst)
*          geph_t *geph     I   glonass ephemeris
*          double *rs       O   satellite position {x,y,z} (ecef) (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
extern void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts,
                     double *var)
{
    double t,tt,x[6];
    int i;

    trace(4,"geph2pos: time=%s sat=%2d\n",time_str(time,3),geph->sat);

    t=timediff(time,geph->toe);

    *dts=-geph->taun+geph->gamn*t;
    trace(4,"geph2pos: sat=%d\n",geph->sat);

    for (i=0;i<3;i++) {
        x[i  ]=geph->pos[i];
        x[i+3]=geph->vel[i];
    }
    for (tt=t<0.0?-TSTEP:TSTEP;fabs(t)>1E-9;t-=tt) {
        if (fabs(t)<TSTEP) tt=t;
        glorbit(tt,x,geph->acc);
    }
    for (i=0;i<3;i++) rs[i]=x[i];

    *var=SQR(ERREPH_GLO);
}

extern void geph2pos_otp(gtime_t time, const geph_t *geph, double *rs, double *dts,
                     double *var)
{
    double t,tt,x[6];
    int i;
    
    trace(4,"geph2pos: time=%s sat=%2d\n",time_str(time,3),geph->sat);
    
    t=timediff(time,geph->toe);
    
    *dts=-geph->taun;  //+geph->gamn*t;
    
    for (i=0;i<3;i++) {
        x[i  ]=geph->pos[i];
        x[i+3]=geph->vel[i];
    }
    for (tt=t<0.0?-TSTEP:TSTEP;fabs(t)>1E-9;t-=tt) {
        if (fabs(t)<TSTEP) tt=t;
        glorbit(tt,x,geph->acc);
    }
    for (i=0;i<3;i++) rs[i]=x[i];
    
    *var=SQR(ERREPH_GLO);
}



/* sbas ephemeris to satellite clock bias --------------------------------------
* compute satellite clock bias with sbas ephemeris
* args   : gtime_t time     I   time by satellite clock (gpst)
*          seph_t *seph     I   sbas ephemeris
* return : satellite clock bias (s)
* notes  : see ref [3]
*-----------------------------------------------------------------------------*/
extern double seph2clk(gtime_t time, const seph_t *seph)
{
    double t;
    int i;

    trace(4,"seph2clk: time=%s sat=%2d\n",time_str(time,3),seph->sat);

    t=timediff(time,seph->t0);

    for (i=0;i<2;i++) {
        t-=seph->af0+seph->af1*t;
    }
    return seph->af0+seph->af1*t;
}
/* sbas ephemeris to satellite position and clock bias -------------------------
* compute satellite position and clock bias with sbas ephemeris
* args   : gtime_t time     I   time (gpst)
*          seph_t  *seph    I   sbas ephemeris
*          double  *rs      O   satellite position {x,y,z} (ecef) (m)
*          double  *dts     O   satellite clock bias (s)
*          double  *var     O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [3]
*-----------------------------------------------------------------------------*/
extern void seph2pos(gtime_t time, const seph_t *seph, double *rs, double *dts,
                     double *var)
{
    double t;
    int i;

    trace(4,"seph2pos: time=%s sat=%2d\n",time_str(time,3),seph->sat);

    t=timediff(time,seph->t0);

    for (i=0;i<3;i++) {
        rs[i]=seph->pos[i]+seph->vel[i]*t+seph->acc[i]*t*t/2.0;
    }
    *dts=seph->af0+seph->af1*t;

    *var=var_uraeph(SYS_SBS,seph->sva);
}
/* select ephemeris --------------------------------------------------------*/
static eph_t *seleph(gtime_t time, int sat, int iode, const nav_t *nav)
{
	double t, tmax, tmin;
	int i, j = -1, sys, sel = 0;
    int n;
    int brdc_iod;  // Holds ephemeris IOD value for SSR matching

	trace(4, "seleph  : time=%s sat=%2d iode=%d\n", time_str(time, 3), sat, iode);

	sys = satsys(sat, NULL);
	switch (sys) {
	case SYS_GPS: tmax = MAXDTOE + 1.0; sel = eph_sel[0]; break;
	case SYS_GAL: tmax = MAXDTOE_GAL; sel = eph_sel[2]; break;
	case SYS_QZS: tmax = MAXDTOE_QZS + 1.0; sel = eph_sel[3]; break;
	case SYS_CMP: tmax = MAXDTOE_CMP + 1.0; sel = eph_sel[4]; break;
	case SYS_IRN: tmax = MAXDTOE_IRN + 1.0; sel = eph_sel[5]; break;
	default: tmax = MAXDTOE + 1.0; break;
	}
	tmin = tmax + 1.0;

    if (PPP_Glo.RT_flag == 0) n = nav->n;
    if (PPP_Glo.RT_flag == 1) n = 2*nav->n;
    

	for (i = 0; i<n; i++) {
        // B2b-SSR scenario: IODC from ephemeris for B2bSSR IODN comparison
        if (PPP_Glo.BDS_CNV1_flag == 1) brdc_iod = nav->eph[i].iodc;  
        // IGS-SSR scenario: IODE from ephemeris for IGS-SSR IODE comparison  
        if (PPP_Glo.BDS_CNV1_flag == 0) brdc_iod = nav->eph[i].iode;
        
		if (nav->eph[i].sat != sat) continue;
		if (iode >= 0 && brdc_iod != iode) continue;
		if (sys == SYS_GAL) {
			sel = getseleph(SYS_GAL);
			/**
             * Liu@APM:In demo5 version, Galileo ephemeris is arbitrarily selected (the newer one between FNAV and INAV is used).
             * However, according to the description in "igs_ssr_v1.pdf":
             * "Clock corrections in RTCM-SSR are related to a broadcast reference clock. 
             * The I/NAV clock has been chosen as the reference clock for RTCM Galileo SSR correction."
             * Therefore, the program should choose the I/NAV ephemeris.
             * So sel will be return 0,nav->eph[i].code&(1<<9) is INAV.
             */
			if (sel==0&&!(nav->eph[i].code&(1<<9))) continue; /* I/NAV */
			if (sel==1&&!(nav->eph[i].code&(1<<8))) continue; /* F/NAV */
			// if (sel == 1 && !(nav->eph[i].code&(1 << 9))) continue; /* I/NAV */
			// if (sel == 2 && !(nav->eph[i].code&(1 << 8))) continue; /* F/NAV */
			if (timediff(nav->eph[i].toe, time) >= 0.0) continue; /* AOD<=0 */
		}
		if ((t = fabs(timediff(nav->eph[i].toe, time)))>tmax) continue;
		if (iode >= 0) return nav->eph + i;
		if (t <= tmin) { j = i; tmin = t; } /* toe closest to time */
	}
	if (iode >= 0 || j<0) {
		trace(2, "no broadcast ephemeris: %s sat=%2d iode=%3d\n", time_str(time, 0),
			sat, iode);
		return NULL;
	}
	trace(4, "seleph: sat=%d dt=%.0f\n", sat, tmin);
	return nav->eph + j;
}
/* select glonass ephemeris --------------------------------------------------*/
static geph_t *selgeph(gtime_t time, int sat, int iode, const nav_t *nav)
{
    double t,tmax=MAXDTOE_GLO,tmin=tmax+1.0;
    int i,j=-1;

    trace(4,"selgeph : time=%s sat=%2d iode=%2d\n",time_str(time,3),sat,iode);

    for (i=0;i<nav->ng;i++) {
        if (nav->geph[i].sat!=sat) continue;
        if (iode>=0&&nav->geph[i].iode!=iode) continue;
        if ((t=fabs(timediff(nav->geph[i].toe,time)))>tmax) continue;
        if (iode>=0) return nav->geph+i;
        if (t<=tmin) {j=i; tmin=t;} /* toe closest to time */
    }
    if (iode>=0||j<0) {
        trace(3,"no glonass ephemeris  : %s sat=%2d iode=%2d\n",time_str(time,0),
              sat,iode);
        return NULL;
    }
    trace(4,"selgeph: sat=%d dt=%.0f\n",sat,tmin);
    return nav->geph+j;
}
/* select sbas ephemeris -----------------------------------------------------*/
static seph_t *selseph(gtime_t time, int sat, const nav_t *nav)
{
    double t,tmax=MAXDTOE_SBS,tmin=tmax+1.0;
    int i,j=-1;

    trace(4,"selseph : time=%s sat=%2d\n",time_str(time,3),sat);

    for (i=0;i<nav->ns;i++) {
        if (nav->seph[i].sat!=sat) continue;
        if ((t=fabs(timediff(nav->seph[i].t0,time)))>tmax) continue;
        if (t<=tmin) {j=i; tmin=t;} /* toe closest to time */
    }
    if (j<0) {
        trace(3,"no sbas ephemeris     : %s sat=%2d\n",time_str(time,0),sat);
        return NULL;
    }
    return nav->seph+j;
}

// /* select B2b ephememeris --------------------------------------------------------*/
// static eph_t *selB2beph(gtime_t time, int sat, int iodn, const nav_t *nav)
// {
//     double t,tmax,tmin;
//     int i,j=-1,sys,sel;
//     int n;
//     char toe_str[64],time_str0[64],satid[8];
//     satno2id(sat,satid);
    
//     trace(4,"seleph  : time=%s sat=%2d iodn=%d\n",time_str(time,3),sat,iodn);
    
//     sys=satsys(sat,NULL);
//     switch (sys) {
//         case SYS_GPS: tmax=MAXDTOE+1.0    ; break;
//         case SYS_CMP: tmax=MAXDTOE_CMP+1.0; break;
//         default: tmax=MAXDTOE+1.0; break;
//     }
//     tmin=tmax+1.0;
    
//     if (PPP_Glo.RT_flag == 0) n = nav->n;
//     if (PPP_Glo.RT_flag == 1) n = 2*nav->n;

//     for (i=0;i<n;i++) {   //2*nav->n for ssr with eph

//         time2str(time,time_str0,3);
//         time2str(nav->eph[i].toe,toe_str,3);

//         if (nav->eph[i].sat!=sat) continue;
//         if (iodn>=0&&nav->eph[i].iodc!=iodn) {
//             trace(22,"time: %s sat(%s)toe: %s nav->eph[i].iode(%d)!=iodn(%d)\n",
//             time_str0,satid,toe_str,nav->eph[i].iodc,iodn);
//             continue;
//         }
        

//         if ((t=fabs(timediff(nav->eph[i].toe,time)))>tmax) continue;
//         if (iodn>=0) return nav->eph+i;  //BRDC:-1 to choose closest EPH
//         if (t<=tmin) {j=i; tmin=t;} /* toe closest to time */
//     }
//     if (iodn>=0||j<0) {
//         trace(3,"no broadcast ephemeris: %s sat=%2d iode=%3d\n",
//               time_str(time,0),sat,iodn);
//         return NULL;
//     }
//     return nav->eph+j;
// }

/* satellite clock with broadcast ephemeris ----------------------------------*/
static int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  double *dts)
{
    eph_t  *eph;
    geph_t *geph;
    seph_t *seph;
    int sys;

    trace(4,"ephclk  : time=%s sat=%2d\n",time_str(time,3),sat);

    sys=satsys(sat,NULL);

    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP||sys==SYS_IRN) {
        if (!(eph=seleph(teph,sat,-1,nav))) return 0;
        *dts=eph2clk(time,eph);
    }
    else if (sys==SYS_GLO) {
        if (!(geph=selgeph(teph,sat,-1,nav))) return 0;
        if (fabs(geph->taun) > 1) return 0; /* reject invalid data to prevent fp error */
        *dts=geph2clk(time,geph);
    }
    else if (sys==SYS_SBS) {
        if (!(seph=selseph(teph,sat,nav))) return 0;
        *dts=seph2clk(time,seph);
    }
    else return 0;

    return 1;
}

static void CNAV1eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts, double *var)
{
    // Reference: "BeiDou System Space Signal Interface Control Document B1C (Version 1.0)"
    // GEO: PRN1-5, 59, 60, 61, 62
    // IGSO: PRN6-10, 13, 16, 31, 38, 39, 40, 56
    double xg, yg, zg, sino, coso;
    double tk, mu, omge, A_ref, O, sinO, cosO, cosi;
    int sys, prn;

    sys = satsys(eph->sat, &prn);
    trace(4, "CNAV1eph2pos : time=%s PRN=%d\n", time_str(time, 3), prn);
    
    if (eph->A <= 0.0) {
        rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
        return;
    }
    tk = timediff(time, eph->toe);  // t_k = t - t_oe
    // Liu@APM: if (satnum > SAT_NUM_SYS && satnum <= 2 * SAT_NUM_SYS) tk -= 14;
    // This line is not needed in this program, as toe is already converted to GPST during reading
    if (tk > 302400.0) tk -= 604800.0;
    if (tk < -302400.0) tk += 604800.0; // See "B1C Interface Control Document" P.37: Consider week crossover

    mu = MU_CMP;
    omge = OMGE_CMP;

    // See "B1C Interface Control Document" P.37: Reference semi-major axis for MEO, IGSO, GEO
    if (prn <= 10 || prn == 13 || prn == 16 || prn == 31 || (prn >= 38 && prn <= 40) || prn == 56 || prn >= 59) {
        A_ref = IGSOGEO_A_ref;
    } else {
        A_ref = MEO_A_ref;
    }

    // The difference between RINEX3 and RINEX4 is in A: one is deltaA, the other is sqrt A
    // double A0 = A_ref + eph->A;    // A_0 = A_ref + Delta_A 
    double A0 = eph->A;    // A_0 = A_ref + Delta_A = sqrt_A * sqrt_A
    // double A0 = 27906100 + NAV[i].A; A0 = 42162200 + NAV[i].A;

    // n_0 = sqrt(mu / (A_0)^3)
    // Delta_n_A = Delta_n_0 + 0.5 * Delta_ndot * t_k
    // n_A = n_0 + Delta_n_A
    double n = sqrt(mu / pow(A0, 3)) + eph->deln + 0.5 * eph->ndot * tk;

    double Mk = eph->M0 + n * tk; // M_k = M_0 + n_A * t_k
    double Ek1 = Mk, Ek = 0, dek = 1;
    // Iteratively calculate the eccentric anomaly: M_k = E_k - e * sin(E_k)
    while (fabs(dek) > 1.0e-10) {
        Ek = Mk + eph->e * sin(Ek1);
        dek = Ek - Ek1;
        Ek1 = Ek;
    }

    // tan(v_k) = sin(v_k) / cos(v_k) = [sqrt(1 - e^2) * sin(E_k)] / [cos(E_k) - e]
    // v_k = atan2(tan(v_k))
    // phi_k = v_k + w
    double uk0 = eph->omg + atan2(sqrt(1 - pow(eph->e, 2)) * sin(Ek), cos(Ek) - eph->e);

    // D_u_k = C_us * sin(2 * phi_k) + C_uc * cos(2 * phi_k)
    // D_r_k = C_rs * sin(2 * phi_k) + C_rc * cos(2 * phi_k)
    // D_i_k = C_is * sin(2 * phi_k) + C_ic * cos(2 * phi_k)
    double Duk = eph->cuc * cos(2 * uk0) + eph->cus * sin(2 * uk0);
    double Drk = eph->crc * cos(2 * uk0) + eph->crs * sin(2 * uk0);
    double Dik = eph->cic * cos(2 * uk0) + eph->cis * sin(2 * uk0);

    double uk = uk0 + Duk; // u_k = phi_k + D_u_k

    // A_k = A_0 + A_dot * t_k
    // r_k = A_k * (1 - e * cos(E_k)) + D_r_k
    double rk = (A0 + tk * eph->Adot) * (1 - eph->e * cos(Ek)) + Drk; 
    double ik = eph->i0 + eph->idot * tk + Dik; // i_k = i_0 + i_dot * t_k + D_i_k

    double xk = rk * cos(uk); // x_k = r_k * cos(u_k)
    double yk = rk * sin(uk); // y_k = r_k * sin(u_k)

    double OMGk = 0.0;
    // OMG_k = OMG_0 + (Delta_OMG - OMG_e) * t_k - OMG_e * t_oe
    if (prn <= 5 || prn >= 59) {
        OMGk = eph->OMG0 + (eph->OMGd - 0.0) * tk - omge * eph->toes;
    } else {
        OMGk = eph->OMG0 + (eph->OMGd - omge) * tk - omge * eph->toes;
    }

    /* BeiDou GEO satellite */
    if (sys == SYS_CMP && (prn <= 5 || prn >= 59)) {
        // OMG_k = OMG_0 + (Delta_OMG - 0.0) * t_k - OMG_e * t_oe
        O = OMGk;
        sinO = sin(O);
        cosO = cos(O);
        xg = xk * cosO - yk * cos(ik) * sinO;
        yg = xk * sinO + yk * cos(ik) * cosO;
        zg = yk * sin(ik);
        sino = sin(omge * tk);
        coso = cos(omge * tk);
        rs[0] = xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
        rs[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
        rs[2] = -yg * SIN_5 + zg * COS_5;
    } else {
        // OMG_k = OMG_0 + (Delta_OMG - OMG_e) * t_k - OMG_e * t_oe
        O = eph->OMG0 + (eph->OMGd - omge) * tk - omge * eph->toes;
        sinO = sin(O);
        cosO = cos(O);
        cosi = cos(ik);
        rs[0] = xk * cosO - yk * cosi * sinO;
        rs[1] = xk * sinO + yk * cosi * cosO;
        rs[2] = yk * sin(ik);
    }
    tk = timediff(time, eph->toc);
    *dts = eph->f0 + eph->f1 * tk + eph->f2 * tk * tk;
    
    /* relativity correction */
    *dts -= 2.0 * sqrt(mu * eph->A) * eph->e * sin(Ek) / SQR(CLIGHT);
    
    /* position and clock error variance */
    *var = var_uraephSISA(sys, eph->sisa);
}
/* satellite position and clock by broadcast ephemeris -----------------------*/
// static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
//                   int iode, double *rs, double *dts, double *var, int *svh)
// {
//     eph_t  *eph;
//     geph_t *geph;
//     seph_t *seph;
//     double rst[3],dtst[1],tt=1E-3;
//     int i,sys;

//     trace(4,"ephpos  : time=%s sat=%2d iode=%d\n",time_str(time,3),sat,iode);

//     sys=satsys(sat,NULL);

//     *svh=-1;

//     if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP||sys==SYS_IRN) {
//         if (!(eph=seleph(teph,sat,iode,nav))) return 0;
//         eph2pos(time,eph,rs,dts,var);
//         time=timeadd(time,tt);
//         eph2pos(time,eph,rst,dtst,var);
//         *svh=eph->svh;
//     }
//     else if (sys==SYS_GLO) {
//         if (!(geph=selgeph(teph,sat,iode,nav))) return 0;
//         geph2pos(time,geph,rs,dts,var);
//         time=timeadd(time,tt);
//         geph2pos(time,geph,rst,dtst,var);
//         *svh=geph->svh;
//     }
//     else if (sys==SYS_SBS) {
//         if (!(seph=selseph(teph,sat,nav))) return 0;
//         seph2pos(time,seph,rs,dts,var);
//         time=timeadd(time,tt);
//         seph2pos(time,seph,rst,dtst,var);
//         *svh=seph->svh;
//     }
//     else return 0;

//     /* satellite velocity and clock drift by differential approx */
//     for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;
//     dts[1]=(dtst[0]-dts[0])/tt;

//     return 1;
// }

/* satellite position and clock by broadcast ephemeris -----------------------*/
static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  int iode, double *rs, double *dts, double *var, int *svh)
{
    eph_t  *eph;
    geph_t *geph;
    seph_t *seph;
    double rst[3],dtst[1],tt=1E-3;
    int i,sys;
    
    trace(4,"ephpos  : time=%s sat=%2d iode=%d\n",time_str(time,3),sat,iode);
    
    sys=satsys(sat,NULL);
    
    *svh=-1;
    if(sys==SYS_CMP && PPP_Glo.BDS_CNV1_flag == 0){
        int a = 0;
    }
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP && PPP_Glo.BDS_CNV1_flag == 0||sys==SYS_IRN) {
        if (!(eph=seleph(teph,sat,iode,nav))) return 0;
        eph2pos(time,eph,rs,dts,var);
        time=timeadd(time,tt);
        eph2pos(time,eph,rst,dtst,var);
        *svh=eph->svh;
    }
    else if (sys == SYS_CMP && PPP_Glo.BDS_CNV1_flag == 1) {
        if (!(eph=seleph(teph,sat,iode,nav))) return 0;
        CNAV1eph2pos(time,eph,rs,dts,var);
        // CSDNCNAV1eph2pos(time,eph,rs,dts,var);
        time=timeadd(time,tt);
        CNAV1eph2pos(time,eph,rst,dtst,var);
        // CSDNCNAV1eph2pos(time,eph,rs,dts,var);
        *svh=eph->svh;
    }
    
    else if (sys==SYS_GLO) {
        if (!(geph=selgeph(teph,sat,iode,nav))) return 0;
        geph2pos(time,geph,rs,dts,var);
        time=timeadd(time,tt);
        geph2pos(time,geph,rst,dtst,var);
        *svh=geph->svh;
    }
    else if (sys==SYS_SBS) {
        if (!(seph=selseph(teph,sat,nav))) return 0;
        seph2pos(time,seph,rs,dts,var);
        time=timeadd(time,tt);
        seph2pos(time,seph,rst,dtst,var);
        *svh=seph->svh;
    }
    else return 0;
    
    /* satellite velocity and clock drift by differential approx */
    for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;
    dts[1]=(dtst[0]-dts[0])/tt;
    
    return 1;
}

/* satellite position and clock by broadcast ephemeris -----------------------*/
static int ephpos_otp(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  int iode, double *rs, double *dts, double *var, int *svh)
{
    eph_t  *eph;
    geph_t *geph;
    seph_t *seph;
    double rst[3],dtst[1],tt=1E-3;
    int i,sys;

    trace(4,"ephpos  : time=%s sat=%2d iode=%d\n",time_str(time,3),sat,iode);

    sys=satsys(sat,NULL);

    *svh=-1;

    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP && PPP_Glo.BDS_CNV1_flag == 0||sys==SYS_IRN) {
        if (!(eph=seleph(teph,sat,iode,nav))) return 0;
        eph2pos(time,eph,rs,dts,var);
        time=timeadd(time,tt);
        eph2pos(time,eph,rst,dtst,var);
        *svh=eph->svh;
    }
    // /* Liu@APM:BDS CNAV1星历计算入口 */
    else if (sys == SYS_CMP && PPP_Glo.BDS_CNV1_flag == 1) {
        //这里的iode在B2b中调用实际上是用的iodn
        if (!(eph=seleph(teph,sat,iode,nav))) return 0;
        CNAV1eph2pos(time,eph,rs,dts,var);
        // CSDNCNAV1eph2pos(time,eph,rs,dts,var);
        time=timeadd(time,tt);
        CNAV1eph2pos(time,eph,rst,dtst,var);
        // CSDNCNAV1eph2pos(time,eph,rs,dts,var);
        *svh=eph->svh;
    }
    else if (sys==SYS_GLO) {
        if (!(geph=selgeph(teph,sat,iode,nav))) return 0;
        geph2pos_otp(time,geph,rs,dts,var);
        time=timeadd(time,tt);
        geph2pos_otp(time,geph,rst,dtst,var);
        *svh=geph->svh;
    }
    else return 0;

    /* satellite velocity and clock drift by differential approx */
    for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;
    dts[1]=(dtst[0]-dts[0])/tt;

    return 1;
}

/* satellite position and clock with sbas correction -------------------------*/
static int satpos_sbas(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                        double *rs, double *dts, double *var, int *svh)
{
    const sbssatp_t *sbs=NULL;
    int i;

    trace(4,"satpos_sbas: time=%s sat=%2d\n",time_str(time,3),sat);

    /* search sbas satellite correction */
    for (i=0;i<nav->sbssat.nsat;i++) {
        sbs=nav->sbssat.sat+i;
        if (sbs->sat==sat) break;
    }
    if (i>=nav->sbssat.nsat) {
        trace(2,"no sbas, use brdcast: %s sat=%2d\n",time_str(time,0),sat);
        if (!ephpos(time,teph,sat,nav,-1,rs,dts,var,svh)) return 0;
        /* *svh=-1; */ /* use broadcast if no sbas */
        return 1;
    }
    /* satellite position and clock by broadcast ephemeris */
    if (!ephpos(time,teph,sat,nav,sbs->lcorr.iode,rs,dts,var,svh)) return 0;

    /* sbas satellite correction (long term and fast) */
    if (sbssatcorr(time,sat,nav,rs,dts,var)) return 1;
    *svh=-1;
    return 0;
}
/* satellite position and clock with ssr correction --------------------------*/
static int satpos_ssr(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                      int opt, double *rs, double *dts, double *var, int *svh)
{
    const ssr_t *ssr;
    eph_t *eph;
    double t1,t2,t3,t5,er[3],ea[3],ec[3],rc[3],deph[3],dclk,dant[3]={0},tk;
    int i,sys;
    char satid[8];

    trace(4,"satpos_ssr: time=%s sat=%2d\n",time_str(time,3),sat);

    ssr=nav->ssr+sat-1;

    if (!ssr->t0[0].time) {
        trace(2,"no ssr orbit correction: %s sat=%2d\n",time_str(time,0),sat);
        return 0;
    }
    if (!ssr->t0[1].time) {
        trace(2,"no ssr clock correction: %s sat=%2d\n",time_str(time,0),sat);
        return 0;
    }
    /* inconsistency between orbit and clock correction */
    if (ssr->iod[0]!=ssr->iod[1]) {
        trace(2,"inconsist ssr correction: %s sat=%2d iod=%d %d\n",
              time_str(time,0),sat,ssr->iod[0],ssr->iod[1]);
        *svh=-1;
        return 0;
    }
    t1=timediff(time,ssr->t0[0]);
    t2=timediff(time,ssr->t0[1]);
    t3=timediff(time,ssr->t0[2]);
    t5=timediff(time,ssr->t0[5]);

    satno2id(sat,satid);
    trace(2,"age of ssr: %s sat=%2d(%s) t=%.0f %.0f %.0f\n",time_str(time,0),
              sat,satid,t1,t2,t5);

    if (fabs(t1 - t2) > 5) {
        trace(2, "SSR time inconsistency: %s sat=%2d t1-t2=%.3f\n",
              time_str(time,0), sat, fabs(t1 - t2));
    }
    /* ssr orbit and clock correction (ref [4]) */
    if (fabs(t1)>MAXAGESSR||fabs(t2)>MAXAGESSR) {
        trace(2,"age of ssr error: %s sat=%2d t=%.0f %.0f\n",time_str(time,0),
              sat,satid,t1,t2);
        *svh=-1;
        return 0;
    }
    if (ssr->udi[0]>=1.0) t1-=ssr->udi[0]/2.0;
    if (ssr->udi[1]>=1.0) t2-=ssr->udi[1]/2.0;

    for (i=0;i<3;i++) deph[i]=ssr->deph[i]+ssr->ddeph[i]*t1;
    dclk=ssr->dclk[0]+ssr->dclk[1]*t2+ssr->dclk[2]*t2*t2;

    /* ssr highrate clock correction (ref [4]) */
    if (ssr->iod[0]==ssr->iod[2]&&ssr->t0[2].time&&fabs(t3)<MAXAGESSR_HRCLK) {
        dclk+=ssr->hrclk;
    }
    if (norm(deph,3)>MAXECORSSR||fabs(dclk)>MAXCCORSSR) {
        trace(3,"invalid ssr correction: %s deph=%.1f dclk=%.1f\n",
              time_str(time,0),norm(deph,3),dclk);
        *svh=-1;
        return 0;
    }
    /* satellite position and clock by broadcast ephemeris */
    if (!ephpos(time,teph,sat,nav,ssr->iode,rs,dts,var,svh)) return 0;

    /* satellite clock for gps, galileo and qzss */
    sys=satsys(sat,NULL);
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
        if (!(eph=seleph(teph,sat,ssr->iode,nav))) return 0;

        /* satellite clock by clock parameters */
        tk=timediff(time,eph->toc);
        dts[0]=eph->f0+eph->f1*tk+eph->f2*tk*tk;
        dts[1]=eph->f1+2.0*eph->f2*tk;

        /* relativity correction */
        dts[0]-=2.0*dot3(rs,rs+3)/CLIGHT/CLIGHT;
    }
    /* radial-along-cross directions in ecef */
    if (!normv3(rs+3,ea)) return 0;
    cross3(rs,rs+3,rc);
    if (!normv3(rc,ec)) {
        *svh=-1;
        return 0;
    }
    cross3(ea,ec,er);

    /* satellite antenna offset correction */
//    if (opt) {
//        satantoff(time,rs,sat,nav,dant);
//    }
    for (i=0;i<3;i++) {
        rs[i]+=-(er[i]*deph[0]+ea[i]*deph[1]+ec[i]*deph[2])+dant[i];
    }
    /* t_corr = t_sv - (dts(brdc) + dclk(ssr) / CLIGHT) (ref [10] eq.3.12-7) */
    dts[0]+=dclk/CLIGHT;

    /* variance by ssr ura */
    *var=var_urassr(ssr->ura);

    trace(5,"satpos_ssr: %s sat=%2d deph=%6.3f %6.3f %6.3f er=%6.3f %6.3f %6.3f dclk=%6.3f var=%6.3f\n",
          time_str(time,2),sat,deph[0],deph[1],deph[2],er[0],er[1],er[2],dclk,*var);

    return 1;
}
static int satpos_B2b(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                      int opt, double *rs, double *dts, double *var, int *svh)
{
    // const ssr_t *ssr;
    const B2bssr_t *B2bssr;
    eph_t *eph;
    double t1,t2,er[3],ea[3],ec[3],rc[3],deph[3],dclk,dant[3]={0},tk;
    int i,sys;
    char satid[8];
    satno2id(sat,satid);
    
    trace(2,"satpos_B2b: time=%s sat=%2d(%s)\n",time_str(time,3),sat,satid);
    
    // B2bssr=nav->B2bssr+sat-1;
    B2bssr=nav->B2bssr+sat;
    
    if (!B2bssr->t0[0].time) {
        trace(2,"no ssr orbit correction: %s sat=%2d\n",time_str(time,0),sat);
        return 0;
    }
    if (!B2bssr->t0[2].time) {
        trace(2,"no ssr clock correction: %s sat=%2d\n",time_str(time,0),sat);
        return 0;
    }
    char time_str1[64];
    char time_str2[64];
    strcpy(time_str1, time_str(time, 0));
    strcpy(time_str2, time_str(B2bssr->t0[2], 3));
    trace(2, "satpos_B2b: %s sat=%2d t0=%s \n", time_str1, sat, time_str2);

    
    /* inconsistency between orbit and clock correction */
    if (B2bssr->iodcorr[0]!=B2bssr->iodcorr[1]) {
        trace(2,"inconsist ssr correction: %s sat=%2d iod=%d %d\n",
              time_str(time,0),sat,B2bssr->iodcorr[0],B2bssr->iodcorr[1]);
        // *svh=-1;
        return 0;
    }
    t1=timediff(time,B2bssr->t0[0]);  // ORB
    t2=timediff(time,B2bssr->t0[2]);  // CLK
    trace(2,"out age of ssr: %s sat=%2d t=%.0f %.0f\n",time_str(time,0),
              sat,t1,t2);
    
    /* ssr orbit and clock correction (ref [4]) */
    if (fabs(t1)>B2beph_MAXAGESSR||fabs(t2)>B2bclk_MAXAGESSR) {
        trace(2,"age of ssr error: %s sat=%2d t=%.0f %.0f\n",time_str(time,0),
              sat,t1,t2);
        // *svh=-1;
        return 0;
    }

    if (B2bssr->udi[0]>=1.0) t1-=B2bssr->udi[0]/2.0;
    if (B2bssr->udi[2]>=1.0) t2-=B2bssr->udi[2]/2.0;
    
    for (i=0;i<3;i++) deph[i]=B2bssr->deph[i]+B2bssr->ddeph[i]*t1;
    dclk=B2bssr->dclk[0]+B2bssr->dclk[1]*t2+B2bssr->dclk[2]*t2*t2;
    
    // /* ssr highrate clock correction (ref [4]) */
    // if (ssr->iod[0]==ssr->iod[2]&&ssr->t0[2].time&&fabs(t3)<MAXAGESSR_HRCLK) {
    //     dclk+=ssr->hrclk;
    // }

    if (norm(deph,3)>MAXECORSSR||fabs(dclk)>MAXCCORSSR) {
        trace(3,"invalid ssr correction: %s deph=%.1f dclk=%.1f\n",
              time_str(time,0),norm(deph,3),dclk);
        *svh=-1;
        return 0;
    }
    /* satellite postion and clock by broadcast ephemeris */
    if (!ephpos(time,teph,sat,nav,B2bssr->iodn,rs,dts,var,svh)) return 0;

    trace(2, "satpos_B2b: %s sat=%2d(%s) position=%.3f %.3f %.3f\n", 
          time_str(time, 2), sat, satid, rs[0], rs[1], rs[2]);
    
    /* satellite clock for gps, galileo and qzss */
    sys=satsys(sat,NULL);
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
        // if (!(eph=selB2beph(teph,sat,B2bssr->iodn,nav))) return 0;
        if (!(eph=seleph(teph,sat,B2bssr->iodn,nav))) return 0;
        
        /* satellite clock by clock parameters */
        tk=timediff(time,eph->toc);
        dts[0]=eph->f0+eph->f1*tk+eph->f2*tk*tk;
        trace(2,"satpos_B2b: %s sat=%2d dts=%f \n",
          time_str(time,2),sat,dts[0]*1e9);
        
        dts[1]=eph->f1+2.0*eph->f2*tk;
        
        /* relativity correction */
        dts[0]-=2.0*dot(rs,rs+3,3)/CLIGHT/CLIGHT;
    }
    /* radial-along-cross directions in ecef */
    if (!normv3(rs+3,ea)) return 0;
    cross3(rs,rs+3,rc);
    if (!normv3(rc,ec)) {
        *svh=-1;
        return 0;
    }
    cross3(ea,ec,er);
    
    /* satellite antenna offset correction */
    if (opt) {
        satantoff2(time,rs,sat,nav,dant,1);
    }
    trace(2,"satantoff2 dant = %f %f %f \n",dant[0],dant[1],dant[2]);
    
    for (i=0;i<3;i++) {
        rs[i]+=-(er[i]*deph[0]+ea[i]*deph[1]+ec[i]*deph[2])-dant[i];  //+dant[i]
    }
    /* t_corr = t_sv - (dts(brdc) + dclk(ssr) / CLIGHT) (ref [10] eq.3.12-7) */
    dts[0]-=dclk/CLIGHT;
    
    /* variance by ssr ura */
    *var=var_uraB2b(B2bssr->ura);
    
    trace(2,"satpos_B2b: %s sat=%2d deph=%6.3f %6.3f %6.3f er=%6.3f %6.3f %6.3f dclk=%6.3f var=%6.3f\n",
          time_str(time,2),sat,deph[0],deph[1],deph[2],er[0],er[1],er[2],dclk,*var);
    
    return 1;
}


static int satpos_ssr_otp(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                      int opt, double *rs, double *dts, double *var, int *svh)
{
    const ssr_t *ssr;
    eph_t *eph;
    double t1,t2,t3,er[3],ea[3],ec[3],rc[3],deph[3],dclk,dant[3]={0},tk;
    int i,sys;

    trace(4,"satpos_ssr: time=%s sat=%2d\n",time_str(time,3),sat);

    ssr=nav->ssr+sat-1;

    if (!ssr->t0[0].time) {
        trace(2,"no ssr orbit correction: %s sat=%2d\n",time_str(time,0),sat);
        return 0;
    }
    if (!ssr->t0[1].time) {
        trace(2,"no ssr clock correction: %s sat=%2d\n",time_str(time,0),sat);
        return 0;
    }
    /* inconsistency between orbit and clock correction */
    if (ssr->iod[0]!=ssr->iod[1]) {
        trace(2,"inconsist ssr correction: %s sat=%2d iod=%d %d\n",
              time_str(time,0),sat,ssr->iod[0],ssr->iod[1]);
        *svh=-1;
        return 0;
    }
    t1=timediff(time,ssr->t0[0]);
    t2=timediff(time,ssr->t0[1]);
    t3=timediff(time,ssr->t0[2]);

    /* ssr orbit and clock correction (ref [4]) */
    if (fabs(t1)>MAXAGESSR||fabs(t2)>MAXAGESSR) {
        trace(7,"age of ssr error: %s sat=%2d t=%.0f %.0f\n",time_str(time,0),
              sat,t1,t2);
        *svh=-1;
        return 0;
    }
    if (ssr->udi[0]>=1.0) t1-=ssr->udi[0]/2.0;
    if (ssr->udi[1]>=1.0) t2-=ssr->udi[1]/2.0;

    for (i=0;i<3;i++) deph[i]=ssr->deph[i]+ssr->ddeph[i]*t1;
    dclk=ssr->dclk[0]+ssr->dclk[1]*t2+ssr->dclk[2]*t2*t2;

    /* ssr highrate clock correction (ref [4]) */
    if (ssr->iod[0]==ssr->iod[2]&&ssr->t0[2].time&&fabs(t3)<MAXAGESSR_HRCLK) {
        dclk+=ssr->hrclk;
    }
    if (norm(deph,3)>MAXECORSSR||fabs(dclk)>MAXCCORSSR) {
        trace(3,"invalid ssr correction: %s deph=%.1f dclk=%.1f\n",
              time_str(time,0),norm(deph,3),dclk);
        *svh=-1;
        return 0;
    }
    /* satellite postion and clock by broadcast ephemeris */
    if (!ephpos_otp(time,teph,sat,nav,ssr->iode,rs,dts,var,svh)) return 0;

    /* satellite clock for gps, galileo and qzss */
    sys=satsys(sat,NULL);
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
        if (!(eph=seleph(teph,sat,ssr->iode,nav))) return 0;

        /* satellite clock by clock parameters */
        tk=timediff(time,eph->toc);
        dts[0]=eph->f0+eph->f1*tk+eph->f2*tk*tk;
        dts[1]=eph->f1+2.0*eph->f2*tk;

        /* relativity correction */
        // Relativistic effect is not needed to calculate the satellite position at launch time.
        // dts[0]-=2.0*dot(rs,rs+3,3)/CLIGHT/CLIGHT;
    }
    /* radial-along-cross directions in ecef */
    if (!normv3(rs+3,ea)) return 0;
    cross3(rs,rs+3,rc);
    if (!normv3(rc,ec)) {
        *svh=-1;
        return 0;
    }
    cross3(ea,ec,er);

    /* satellite antenna offset correction */
    // if (opt) {
    //     satantoff(time,rs,sat,nav,dant);
    // }
    for (i=0;i<3;i++) {
        rs[i]+=-(er[i]*deph[0]+ea[i]*deph[1]+ec[i]*deph[2]);   //+dant[i];
    }
    /* t_corr = t_sv - (dts(brdc) + dclk(ssr) / CLIGHT) (ref [10] eq.3.12-7) */
    dts[0]+=dclk/CLIGHT;

    /* variance by ssr ura */
    *var=var_urassr(ssr->ura);

    // trace(7,"satpos_ssr_otp: %s sat=%2d deph=%6.3f %6.3f %6.3f er=%6.3f %6.3f %6.3f dclk=%6.3f var=%6.3f\n",
    //       time_str(time,2),sat,deph[0],deph[1],deph[2],er[0],er[1],er[2],dclk,*var);

    return 1;
}

static int satpos_B2b_otp(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                      int opt, double *rs, double *dts, double *var, int *svh)
{
    // const ssr_t *ssr;
    const B2bssr_t *B2bssr;
    eph_t *eph;
    double t1,t2,er[3],ea[3],ec[3],rc[3],deph[3],dclk,dant[3]={0},tk;
    int i,sys;
    int prn;

    trace(4,"satpos_ssr: time=%s sat=%2d\n",time_str(time,3),sat);

    // B2bssr=nav->B2bssr+sat-1;
    B2bssr=nav->B2bssr+sat;

    if (!B2bssr->t0[0].time) {
        trace(2,"no ssr orbit correction: %s sat=%2d\n",time_str(time,0),sat);
        return 0;
    }
    if (!B2bssr->t0[2].time) {
        trace(2,"no ssr clock correction: %s sat=%2d\n",time_str(time,0),sat);
        return 0;
    }
    /* inconsistency between orbit and clock correction */
    if (B2bssr->iodcorr[0]!=B2bssr->iodcorr[1]) {
        trace(2,"inconsist ssr correction: %s sat=%2d iod=%d %d\n",
              time_str(time,0),sat,B2bssr->iodcorr[0],B2bssr->iodcorr[1]);
        *svh=-1;
        return 0;
    }
    t1=timediff(time,B2bssr->t0[0]);
    t2=timediff(time,B2bssr->t0[2]);
    // t3=timediff(time,B2bssr->t0[2]);

    // trace(7,"!!!age of ssr: %s sat=%2d t=%.0f %.0f\n",time_str(time,0),
    //           sat,t1,t2);

    /* ssr orbit and clock correction (ref [4]) */
    if (fabs(t1)>B2beph_MAXAGESSR||fabs(t2)>B2bclk_MAXAGESSR) {
        trace(2,"age of ssr error: %s sat=%2d t=%.0f %.0f\n",time_str(time,0),
              sat,t1,t2);
        // *svh=-1;
        return 0;
    }
    // 这一步需要吗？
    if (B2bssr->udi[0]>=1.0) t1-=B2bssr->udi[0]/2.0;
    if (B2bssr->udi[2]>=1.0) t2-=B2bssr->udi[2]/2.0;

    for (i=0;i<3;i++) deph[i]=B2bssr->deph[i]+B2bssr->ddeph[i]*t1;
    dclk=B2bssr->dclk[0]+B2bssr->dclk[1]*t2+B2bssr->dclk[2]*t2*t2;

    double freq[2];
    sys=satsys(sat,&prn);
                        
    // /* ssr highrate clock correction (ref [4]) */
    // if (ssr->iod[0]==ssr->iod[2]&&ssr->t0[2].time&&fabs(t3)<MAXAGESSR_HRCLK) {
    //     dclk+=ssr->hrclk;
    // }

    if (norm(deph,3)>MAXECORSSR||fabs(dclk)>MAXCCORSSR) {
        trace(3,"invalid ssr correction: %s deph=%.1f dclk=%.1f\n",
              time_str(time,0),norm(deph,3),dclk);
        *svh=-1;
        return 0;
    }
    /* satellite postion and clock by broadcast ephemeris */
    if (!ephpos_otp(time,teph,sat,nav,B2bssr->iodn,rs,dts,var,svh)) return 0;

    /* satellite clock for gps, galileo and qzss */
    sys=satsys(sat,NULL);
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
        if (!(eph=seleph(teph,sat,B2bssr->iodn,nav))) return 0;

        /* satellite clock by clock parameters */
        tk=timediff(time,eph->toc);
        dts[0]=eph->f0+eph->f1*tk+eph->f2*tk*tk;
        dts[1]=eph->f1+2.0*eph->f2*tk;

        /* relativity correction */
        // dts[0]-=2.0*dot(rs,rs+3,3)/CLIGHT/CLIGHT;
    }
    /* radial-along-cross directions in ecef */
    if (!normv3(rs+3,ea)) return 0;
    cross3(rs,rs+3,rc);
    if (!normv3(rc,ec)) {
        *svh=-1;
        return 0;
    }
    cross3(ea,ec,er);

    /* satellite antenna offset correction */
    if (opt) {
        satantoff2(time,rs,sat,nav,dant,1);
    }
    for (i=0;i<3;i++) {
        rs[i]+=-(er[i]*deph[0]+ea[i]*deph[1]+ec[i]*deph[2])-dant[i];
    }
    /* t_corr = t_sv - (dts(brdc) + dclk(ssr) / CLIGHT) (ref [10] eq.3.12-7) */
    dts[0]-=dclk/CLIGHT;

    if (sys==SYS_CMP) {
        freq[0]=FREQ1_CMP;
        freq[1]=FREQ3_CMP;
        double C= SQR(freq[0])/(SQR(freq[0])-SQR(freq[1]));
        double DCB = B2bssr->cbias[40];
        if (DCB == 0) return 0;
        dts[0]=dts[0]-C*DCB/CLIGHT;
    }

    /* variance by ssr ura */
    *var=var_uraB2b(B2bssr->ura);

    trace(5,"satpos_B2b: %s sat=%2d deph=%6.3f %6.3f %6.3f er=%6.3f %6.3f %6.3f dclk=%6.3f var=%6.3f\n",
          time_str(time,2),sat,deph[0],deph[1],deph[2],er[0],er[1],er[2],dclk,*var);

    return 1;
}

/* satellite position and clock ------------------------------------------------
* compute satellite position, velocity and clock
* args   : gtime_t time     I   time (gpst)
*          gtime_t teph     I   time to select ephemeris (gpst)
*          int    sat       I   satellite number
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          nav_t  *nav      I   navigation data
*          double *rs       O   sat position and velocity (ecef)
*                               {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts      O   sat clock {bias,drift} (s|s/s)
*          double *var      O   sat position and clock error variance (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : status (1:ok,0:error)
* notes  : satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*-----------------------------------------------------------------------------*/
extern int satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
                  const nav_t *nav, double *rs, double *dts, double *var,
                  int *svh)
{
    trace(4,"satpos  : time=%s sat=%2d ephopt=%d\n",time_str(time,3),sat,ephopt);

    *svh=0;

    switch (ephopt) {
        case EPHOPT_BRDC  : return ephpos     (time,teph,sat,nav,-1,rs,dts,var,svh);
        case EPHOPT_SBAS  : return satpos_sbas(time,teph,sat,nav,   rs,dts,var,svh);
        case EPHOPT_SSRAPC: return satpos_ssr (time,teph,sat,nav, 0,rs,dts,var,svh);
        case EPHOPT_SSRCOM: return satpos_ssr (time,teph,sat,nav, 1,rs,dts,var,svh);
        case EPHOPT_B2b:    return satpos_B2b (time,teph,sat,nav, 1,rs,dts,var,svh);
        case EPHOPT_PREC  :
            if (!peph2pos(time,sat,nav,1,rs,dts,var)) break; else return 1;
    }
    *svh=-1;
    return 0;
}


extern int satpos_otp(gtime_t time, gtime_t teph, int sat, int ephopt,
                  const nav_t *nav, double *rs, double *dts, double *var,
                  int *svh)
{
    trace(4,"satpos  : time=%s sat=%2d ephopt=%d\n",time_str(time,3),sat,ephopt);

    *svh=0;

    switch (ephopt) {
        case EPHOPT_SSRCOM: return satpos_ssr_otp (time,teph,sat,nav, 1,rs,dts,var,svh);
        case EPHOPT_B2b: return satpos_B2b_otp (time,teph,sat,nav, 1,rs,dts,var,svh);
        case EPHOPT_BRDC  : return ephpos_otp     (time,teph,sat,nav,-1,rs,dts,var,svh);
        case EPHOPT_PREC: if (!peph2pos_otp(time,sat,nav,1,rs,dts,var)) break; else return 1;
    }
    *svh=-1;
    return 0;
}


/* satellite positions and clocks ----------------------------------------------
* compute satellite positions, velocities and clocks
* args   : gtime_t teph     I   time to select ephemeris (gpst)
*          obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   satellite positions and velocities (ecef)
*          double *dts      O   satellite clocks
*          double *var      O   sat position and clock error variances (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : none
* notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
*          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
*          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
*          var[i]        = obs[i] sat position and clock error variance (m^2)
*          svh[i]        = obs[i] sat health flag
*          if no navigation data, set 0 to rs[], dts[], var[] and svh[]
*          satellite position and clock are values at signal transmission time
*          satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*          any pseudorange and broadcast ephemeris are always needed to get
*          signal transmission time
*-----------------------------------------------------------------------------*/
extern void satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
                    int ephopt, double *rs, double *dts, double *var, int *svh)
{
    gtime_t time[2*MAXOBS]={{0}};
    double dt,pr;
    int i,j;
    char satid[8];

    trace(3,"satposs : teph=%s n=%d ephopt=%d\n",time_str(teph,3),n,ephopt);

    for (i=0;i<n&&i<2*MAXOBS;i++) {
        for (j=0;j<6;j++) rs [j+i*6]=0.0;
        for (j=0;j<2;j++) dts[j+i*2]=0.0;
        var[i]=0.0; svh[i]=0;
        PPP_Glo.brdc[i] = 0;

        /* search any pseudorange */
        for (j=0,pr=0.0;j<NFREQ;j++) if ((pr=obs[i].P[j])!=0.0) break;

        if (j>=NFREQ) {
            trace(2,"no pseudorange %s sat=%2d\n",time_str(obs[i].time,3),obs[i].sat);
            continue;
        }
        /* transmission time by satellite clock */
        time[i]=timeadd(obs[i].time,-pr/CLIGHT);

        /* satellite clock bias by broadcast ephemeris */
        if (!ephclk(time[i],teph,obs[i].sat,nav,&dt)) {
            trace(3,"no broadcast clock %s sat=%2d\n",time_str(time[i],3),obs[i].sat);
            continue;
        }
        time[i]=timeadd(time[i],-dt);

        /* satellite position and clock at transmission time */
        if (!satpos(time[i],teph,obs[i].sat,ephopt,nav,rs+i*6,dts+i*2,var+i,svh+i)) {
                // if (!satpos(time[i],teph,obs[i].sat,0,nav,rs+i*6,dts+i*2,var+i,svh+i)) {
                //     trace(3,"no ephemeris %s sat=%2d\n",time_str(time[i],3),obs[i].sat);
                //     continue;
                // }
                // PPP_Glo.brdc[i] = 1;
                continue;
                trace(22,"PPP_Glo.brdc[i] = 1; %s sat=%2d\n",time_str(time[i],3),obs[i].sat);
        }
        /* if no precise clock available, use broadcast clock instead */
        if (dts[i*2]==0.0) {
            if (!ephclk(time[i],teph,obs[i].sat,nav,dts+i*2)) continue;
            dts[1+i*2]=0.0;
            *var=SQR(STD_BRDCCLK);
        }
        trace(4,"satposs: %d,time=%.9f dt=%.9f pr=%.3f rs=%13.3f %13.3f %13.3f dts=%12.3f var=%7.3f\n",
            obs[i].sat,time[i].sec,dt,pr,rs[i*6],rs[1+i*6],rs[2+i*6],dts[i*2]*1E9,
            var[i]);

    }
    for (i=0;i<n&&i<2*MAXOBS;i++) {
        satno2id(obs[i].sat,satid);
        trace(22,"%s sat=%2d(%s) rs=%13.3f %13.3f %13.3f dts=%12.3f var=%7.3f svh=%02X\n",
              time_str(time[i],9),obs[i].sat,satid,rs[i*6],rs[1+i*6],rs[2+i*6],
              dts[i*2]*1E9,var[i],svh[i]);
    }
}

/* satellite positions output to SP3 file ------------------------------------
 * compute and store satellite positions and clock data into a specified SP3 file
 * args   : gtime_t teph     I   time for selecting ephemeris (gpst)
 *          const nav_t *nav  I   navigation data structure
 *          int    ephopt    I   ephemeris option (e.g., EPHOPT_???)
 *          int    sampling   I   sampling interval for output
 *          double *rs       O   satellite positions and velocities (ECEF)
 *          double *dts      O   satellite clocks (bias and drift)
 *          double *var      O   variance of satellite positions and clocks (m^2)
 *          int    *svh      O   satellite health flags (-1: correction unavailable)
 * return : none
 * notes  : rs [(0:2)+i*6] = satellite position {x,y,z} (m)
 *          rs [(3:5)+i*6] = satellite velocity {vx,vy,vz} (m/s)
 *          dts[(0:1)+i*2]  = satellite clock {bias,drift} (s|s/s)
 *          var[i]          = variance for satellite position and clock (m^2)
 *          svh[i]          = health status of satellite
 *          if no ephemeris data is available, outputs will be initialized to zero
 *          satellite data is logged only at specified sampling intervals
 *-----------------------------------------------------------------------------*/
extern void satposs_otp(char *stationname, gtime_t teph, const nav_t *nav,
                    int ephopt, int sampling, double *rs, double *dts, double *var, int *svh) {
    FILE *fp_sp3;
    char id[32];
    char doy_str[8];

    if (!sp3init_flag) {
    int doy = time2doy(teph);
    // snprintf(sp3file, sizeof(sp3file), "%s%d.sp3", stationname, doy);
    snprintf(sp3file, sizeof(sp3file), "%s.sp3", stationname);
    sp3init_flag = 1;
    }
    
    struct stat buffer; // 用于存储文件状态信息
    // int exist;
    double dt,pr,gps_second;
    int i,j,gps_week;
    double current_time[6];
    time2epoch(teph,current_time);
    gps_second = time2gpst(teph,&gps_week);

    trace(3,"satposs : teph=%s n=%d ephopt=%d\n",time_str(teph,3),ephopt);

    // 检查文件是否存在
    // exist = stat(sp3file, &buffer);

    // 根据文件是否存在选择打开模式
    if (sp3continue_flag == 1) {
        fp_sp3 = fopen(sp3file, "a"); // 打开文件追加数据
    } else {
        // 文件不存在
        fp_sp3 = fopen(sp3file, "w"); // 创建并写入新文件
        sp3continue_flag = 1;
        if (!fp_sp3) {
            perror("Failed to open file");
            return;
        }
        // 写入SP3文件头部
        fprintf(fp_sp3, "#dP");
        fprintf(fp_sp3, "%4d%3d%3d%3d%3d%12.8f\n", (int)current_time[0], (int)current_time[1], (int)current_time[2],
		(int)current_time[3], (int)current_time[4], current_time[5]);
        fprintf(fp_sp3, "##");
	    fprintf(fp_sp3, "%5d%16.8f%15.8f\n", gps_week, gps_second, (float)sampling);
    }

    if((int)current_time[5]%sampling == 0)
    {
    fprintf(fp_sp3, "*  ");
	fprintf(fp_sp3, "%4d%3d%3d%3d%3d%12.8f\n", (int)current_time[0], (int)current_time[1], (int)current_time[2], 
    (int)current_time[3], (int)current_time[4], current_time[5]);

    for (i = 0; i < MAXSAT; i++) {
        for (j = 0; j < 6; j++) rs[j+i*6] = 0.0;
        for (j = 0; j < 2; j++) dts[j+i*2] = 0.0;
        var[i] = 0.0; svh[i] = 0;

        if (!satpos_otp(teph, teph, i, ephopt, nav, rs + i * 6, dts + i * 2, var + i, svh + i)) {
            trace(3, "no ephemeris %s sat=%2d\n", time_str(teph,3), i);
            continue;
        }

        if (dts[i*2] == 0.0) {
            if (!ephclk(teph, teph, i, nav, dts + i * 2)) continue;
            dts[1+i*2] = 0.0;
            var[i] = SQR(STD_BRDCCLK);
        }
        satno2id(i,id);

        // 格式化并写入每颗卫星的数据
        fprintf(fp_sp3, "P%s %14.6f %14.6f %14.6f %14.6f\n",
        id,
        rs[i*6] / 1000.0,        // 将米转换为公里
        rs[1+i*6] / 1000.0,      // 将米转换为公里
        rs[2+i*6] / 1000.0,      // 将米转换为公里
        dts[i*2] * 1000000.0);   // 将秒转换为微秒
    }
    }

    // fprintf(fp, "EOF\n");
    fflush(fp_sp3);
    fclose(fp_sp3); // 关闭文件
}

/* set selected satellite ephemeris --------------------------------------------
* Set selected satellite ephemeris for multiple ones like LNAV - CNAV, I/NAV -
* F/NAV. Call it before calling satpos(),satposs() to use unselected one.
* args   : int    sys       I   satellite system (SYS_???)
*          int    sel       I   selection of ephemeris
*                                 GPS,QZS : 0:LNAV ,1:CNAV  (default: LNAV)
*  b33 and demo5 b34:             GAL: 0:any,1:I/NAV,2:F/NAV
*  2.4.3 b34 but not functional?  GAL     : 0:I/NAV,1:F/NAV (default: I/NAV)
*                                 others : undefined
* return : none
*-----------------------------------------------------------------------------*/
extern void setseleph(int sys, int sel)
{
    switch (sys) {
        case SYS_GPS: eph_sel[0]=sel; break;
        case SYS_GLO: eph_sel[1]=sel; break;
        case SYS_GAL: eph_sel[2]=sel; break;
        case SYS_QZS: eph_sel[3]=sel; break;
        case SYS_CMP: eph_sel[4]=sel; break;
        case SYS_IRN: eph_sel[5]=sel; break;
        case SYS_SBS: eph_sel[6]=sel; break;
    }
}
/* get selected satellite ephemeris -------------------------------------------
* Get the selected satellite ephemeris.
* args   : int    sys       I   satellite system (SYS_???)
* return : selected ephemeris
*            refer setseleph()
*-----------------------------------------------------------------------------*/
extern int getseleph(int sys)
{
    switch (sys) {
        case SYS_GPS: return eph_sel[0];
        case SYS_GLO: return eph_sel[1];
        case SYS_GAL: return eph_sel[2];
        case SYS_QZS: return eph_sel[3];
        case SYS_CMP: return eph_sel[4];
        case SYS_IRN: return eph_sel[5];
        case SYS_SBS: return eph_sel[6];
    }
    return 0;
}
