/* debug trace functions -----------------------------------------------------*/
#include "rtklib.h"

#ifdef TRACE

static FILE *fp_trace = NULL;    /* file pointer of trace */
static char file_trace[1024];    /* trace file */
static int level_trace = 0;      /* level of trace */
static uint32_t tick_trace = 0;  /* tick time at traceopen (ms) */
static gtime_t time_trace = {0}; /* time at traceopen */
static rtklib_lock_t lock_trace; /* lock for trace */

static FILE *fp_B2b_trace = NULL; 
static int B2b_level_trace = 0;
static char file_B2b_trace[1024]; 
static rtklib_lock_t lock_B2b_trace; 
static gtime_t B2btime_trace = {0}; 
static uint32_t B2btick_trace = 0;

static void traceswap(void)
{
    gtime_t time = utc2gpst(timeget());
    char path[1024];

    rtklib_lock(&lock_trace);

    if ((int)(time2gpst(time,       NULL) / INT_SWAP_TRAC) ==
        (int)(time2gpst(time_trace, NULL) / INT_SWAP_TRAC)) {
        rtklib_unlock(&lock_trace);
        return;
    }
    time_trace = time;

    if (!reppath(file_trace, path, time, "", "")) {
        rtklib_unlock(&lock_trace);
        return;
    }
    if (fp_trace) fclose(fp_trace);

    if (!(fp_trace = fopen(path, "w"))) {
        fp_trace = stderr;
    }
    rtklib_unlock(&lock_trace);
}
extern void traceopen(const char *file)
{
    gtime_t time = utc2gpst(timeget());
    char path[1024];

    reppath(file, path, time, "", "");
    if (!*path || !(fp_trace = fopen(path, "w"))) fp_trace = stderr;
    strcpy(file_trace, file);
    tick_trace = tickget();
    time_trace = time;
    rtklib_initlock(&lock_trace);
}
extern void traceclose(void)
{
    if (fp_trace && fp_trace != stderr) fclose(fp_trace);
    fp_trace = NULL;
    file_trace[0] = '\0';
}
extern void tracelevel(int level) { level_trace = level; }
extern void B2b_tracelevel(int level) { B2b_level_trace = level; }

extern int gettracelevel(void) { return level_trace; }
extern int B2b_gettracelevel(void) { return B2b_level_trace; }

extern void trace_impl(int level, const char *format, ...)
{
    va_list ap;

    /* print error message to stderr */
    if (level <= 1) {
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
    }
    if (!fp_trace || level > level_trace) return;
    // if (level != 22) return;
    traceswap();
    fprintf(fp_trace, "%d ", level);
    va_start(ap, format);
    vfprintf(fp_trace, format, ap);
    va_end(ap);
    fflush(fp_trace);
}
extern void tracet_impl(int level, const char *format, ...)
{
    va_list ap;

    if (!fp_trace || level > level_trace) return;
    traceswap();
    fprintf(fp_trace, "%d %9.3f: ", level, (tickget() - tick_trace) / 1000.0);
    va_start(ap, format);
    vfprintf(fp_trace, format, ap);
    va_end(ap);
    fflush(fp_trace);
}
extern void tracemat_impl(int level, const double *A, int n, int m, int p,
                          int q)
{
    if (!fp_trace || level > level_trace) return;
    matfprint(A, n, m, p, q, fp_trace);
    fflush(fp_trace);
}
extern void traceobs_impl(int level, const obsd_t *obs, int n)
{
    char str[64], id[16];
    int i;

    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < n; i++) {
        time2str(obs[i].time, str, 3);
        satno2id(obs[i].sat, id);
        fprintf(fp_trace,
                " (%2d) %s %-3s rcv%d %13.3f %13.3f %13.3f %13.3f %d %d %d %d "
                "%x %x %3.1f %3.1f\n",
                i + 1, str, id, obs[i].rcv, obs[i].L[0], obs[i].L[1],
                obs[i].P[0], obs[i].P[1], obs[i].LLI[0], obs[i].LLI[1],
                obs[i].code[0], obs[i].code[1], obs[i].Lstd[0], obs[i].Pstd[0],
                obs[i].SNR[0] * SNR_UNIT, obs[i].SNR[1] * SNR_UNIT);
    }
    fflush(fp_trace);
}
extern void tracenav_impl(int level, const nav_t *nav)
{
    char s1[64], s2[64], id[16];
    int i,n;
    int temp_n = MAXSAT;
    if (PPP_Glo.RT_flag == 0) n = nav->n;
    if (PPP_Glo.RT_flag == 1) n = 2*nav->n;
    // n = nav->n;

    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < n; i++) {
        time2str(nav->eph[i].toe, s1, 0);
        time2str(nav->eph[i].ttr, s2, 0);
        satno2id(nav->eph[i].sat, id);
        fprintf(fp_trace, "(%3d) %-3s : %s %s %3d %3d %02x\n", i + 1, id, s1,
                s2, nav->eph[i].iode, nav->eph[i].iodc, nav->eph[i].svh);
    }
    fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav->ion_gps[0],
            nav->ion_gps[1], nav->ion_gps[2], nav->ion_gps[3]);
    fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav->ion_gps[4],
            nav->ion_gps[5], nav->ion_gps[6], nav->ion_gps[7]);
    fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav->ion_gal[0],
            nav->ion_gal[1], nav->ion_gal[2], nav->ion_gal[3]);
}
extern void tracegnav_impl(int level, const nav_t *nav)
{
    char s1[64], s2[64], id[16];
    int i;

    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < nav->ng; i++) {
        time2str(nav->geph[i].toe, s1, 0);
        time2str(nav->geph[i].tof, s2, 0);
        satno2id(nav->geph[i].sat, id);
        fprintf(fp_trace, "(%3d) %-3s : %s %s %2d %2d %8.3f\n", i + 1, id, s1,
                s2, nav->geph[i].frq, nav->geph[i].svh,
                nav->geph[i].taun * 1E6);
    }
}
extern void tracehnav_impl(int level, const nav_t *nav)
{
    char s1[64], s2[64], id[16];
    int i;

    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < nav->ns; i++) {
        time2str(nav->seph[i].t0, s1, 0);
        time2str(nav->seph[i].tof, s2, 0);
        satno2id(nav->seph[i].sat, id);
        fprintf(fp_trace, "(%3d) %-3s : %s %s %2d %2d\n", i + 1, id, s1, s2,
                nav->seph[i].svh, nav->seph[i].sva);
    }
}
extern void tracepeph_impl(int level, const nav_t *nav)
{
    char s[64], id[16];
    int i, j;

    if (!fp_trace || level > level_trace) return;

    for (i = 0; i < nav->ne; i++) {
        time2str(nav->peph[i].time, s, 0);
        for (j = 0; j < MAXSAT; j++) {
            satno2id(j + 1, id);
            fprintf(fp_trace,
                    "%-3s %d %-3s %13.3f %13.3f %13.3f %13.3f %6.3f %6.3f "
                    "%6.3f %6.3f\n",
                    s, nav->peph[i].index, id, nav->peph[i].pos[j][0],
                    nav->peph[i].pos[j][1], nav->peph[i].pos[j][2],
                    nav->peph[i].pos[j][3] * 1E9, nav->peph[i].std[j][0],
                    nav->peph[i].std[j][1], nav->peph[i].std[j][2],
                    nav->peph[i].std[j][3] * 1E9);
        }
    }
}
extern void tracepclk_impl(int level, const nav_t *nav)
{
    char s[64], id[16];
    int i, j;

    if (!fp_trace || level > level_trace) return;

    for (i = 0; i < nav->nc; i++) {
        time2str(nav->pclk[i].time, s, 0);
        for (j = 0; j < MAXSAT; j++) {
            satno2id(j + 1, id);
            fprintf(fp_trace, "%-3s %d %-3s %13.3f %6.3f\n", s,
                    nav->pclk[i].index, id, nav->pclk[i].clk[j][0] * 1E9,
                    nav->pclk[i].std[j][0] * 1E9);
        }
    }
}
extern void traceb_impl(int level, const uint8_t *p, int n)
{
    int i;
    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < n; i++)
        fprintf(fp_trace, "%02X%s", *p++, i % 8 == 7 ? " " : "");
    fprintf(fp_trace, "\n");
}

static void B2btraceswap(void)
{
    gtime_t time = utc2gpst(timeget());
    char path[1024];

    rtklib_lock(&lock_B2b_trace);

    if ((int)(time2gpst(time,       NULL) / INT_SWAP_TRAC) ==
        (int)(time2gpst(B2btime_trace, NULL) / INT_SWAP_TRAC)) {
        rtklib_unlock(&lock_B2b_trace);
        return;
    }
    B2btime_trace = time;

    if (!reppath(file_B2b_trace, path, time, "", "")) {
        rtklib_unlock(&lock_B2b_trace);
        return;
    }
    if (fp_B2b_trace) fclose(fp_B2b_trace);

    if (!(fp_B2b_trace = fopen(path, "w"))) {
        fp_B2b_trace = stderr;
    }
    rtklib_unlock(&lock_B2b_trace);
}

extern void traceB2b_impl(int level, const nav_t *nav)
{
    char t0_str[6][64], satid[16];
    int i, code;
    const B2bssr_t *b2b;

    if (!fp_trace || level > B2b_level_trace) return;
    
    fprintf(fp_trace, "\n============== Trace B2b SSR ==============\n");
    
    for (i = 0; i < MAXSAT; i++) {
        b2b = &nav->B2bssr[i];
        // if (!b2b->update) continue;

        satno2id(i, satid);
        
        for (int j = 0; j < 6; j++) {
            time2str(b2b->t0[j], t0_str[j], 2);
        }
        fprintf(fp_trace, "(SATID) %-4s\n", satid);
        fprintf(fp_trace, "(B2b IODN) %d \n",b2b->iodn);

        fprintf(fp_trace, "(B2b ORBIT) %-4s | toe: %s\n", satid, t0_str[0]);
        fprintf(fp_trace, "  deph: %8.3f %8.3f %8.3f m\n", 
                b2b->deph[0], b2b->deph[1], b2b->deph[2]);
        fprintf(fp_trace, "  ddeph: %6.3f %6.3f %6.3f m/s\n",
                b2b->ddeph[0], b2b->ddeph[1], b2b->ddeph[2]);
        
        fprintf(fp_trace, "(B2b CLOCK) %-4s | toe: %s\n", satid, t0_str[2]);
        fprintf(fp_trace, "  dclk: %10.3f %8.3f %8.3f m\n",
                b2b->dclk[0], b2b->dclk[1], b2b->dclk[2]);

        
        fprintf(fp_trace, "(B2b CODEBIAS) %-4s | toe: %s\n", satid, t0_str[1]);
        for (code = 0; code < MAXCODE; code++) {
            if (fabs(b2b->cbias[code]) > 1e-4) { 
                fprintf(fp_trace, "  CODE%d: %6.3f m\n", code+1, b2b->cbias[code]);
            }
        }
        
        fprintf(fp_trace, "  iodssr: %3d(eph) %3d(cbia) %3d(clk)\n", 
                b2b->iodssr[0], b2b->iodssr[1], b2b->iodssr[2]);
        fprintf(fp_trace, "  iodcorr: %4d(eph) %4d(clk)\n",
                b2b->iodcorr[0], b2b->iodcorr[1]);
        
        fprintf(fp_trace, "------------------------------------------------\n");
    }
}

extern void B2b_traceopen(const char *file) {
    gtime_t time = utc2gpst(timeget());
    char path[1024];

    reppath(file, path, time, "", "");
    if (!*path || !(fp_B2b_trace = fopen(path, "w"))) fp_B2b_trace = stderr;
    strcpy(file_B2b_trace, file);
    B2btick_trace = tickget();
    B2btime_trace = time;
    rtklib_initlock(&lock_B2b_trace);
}

extern void B2b_traceclose(void) {
    if (fp_B2b_trace && fp_B2b_trace != stderr) fclose(fp_B2b_trace);
    fp_B2b_trace = NULL;
    file_B2b_trace[0] = '\0';
}

extern void B2b_trace_impl(int level, const char *format, ...) {
    va_list ap;
    if (!fp_B2b_trace || level > B2b_level_trace) return;
    // rtklib_lock(&lock_B2b_trace);
    // fprintf(fp_B2b_trace, "%d ", level);
    B2btraceswap();
    va_start(ap, format);
    vfprintf(fp_B2b_trace, format, ap);
    va_end(ap);
    fflush(fp_B2b_trace);
    rtklib_unlock(&lock_B2b_trace);
}



#endif /* TRACE */
