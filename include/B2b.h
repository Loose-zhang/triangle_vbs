#pragma once


/* type definitions ----------------------------------------------------------*/
typedef struct {
    int mjd;                      /* modified Julian date (MJD) */
    double sod;                   /* seconds of the day */
} mjd_gtime_t;

mjd_gtime_t yrdoy_to_mjd_time(int year, int doy, double sod);

mjd_gtime_t gpst_to_bdst(mjd_gtime_t tt_gps);
mjd_gtime_t bdst_to_gpst(mjd_gtime_t tt_bds);

int mjd_time_to_gpst(mjd_gtime_t tt, int *week, double *sow);

EXPORT int mask2satno(B2bmask_t *B2bmask);
EXPORT int slot2satno(int slot);
EXPORT int init_B2b(B2b_t *b2b);

EXPORT int checkout_B2beph(B2bssr_t *B2bssr0, B2bssr_t *B2bssr1);
EXPORT int checkout_B2bcbia(B2bssr_t *B2bssr0, B2bssr_t *B2bssr1);
EXPORT int checkout_B2bclk(B2bssr_t *B2bssr0, B2bssr_t *B2bssr1);
EXPORT void B2b_urai2ura(gtime_t time, const char *satid, int URAI, FILE *UraFile);

EXPORT void output_B2bInfo1(raw_t *raw, const B2bmask_t *mask, int udi);
EXPORT void output_B2bInfo2(raw_t *raw, nav_t *nav);
EXPORT void output_B2bInfo3(raw_t *raw, nav_t *nav);
EXPORT void output_B2bInfo4(raw_t *raw, nav_t *nav);