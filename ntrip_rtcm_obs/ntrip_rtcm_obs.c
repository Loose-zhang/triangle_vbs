/*------------------------------------------------------------------------------
 * ntrip_rtcm_obs.c : TCP listener for RTCM3 (e.g. NTRIP via STRSVR), print P/L
 *
 * Usage:
 *   Server (wait for STRSVR to connect in):
 *     ntrip_rtcm_obs [listen_port] [label]
 *       listen_port — default 50001
 *   Client — one or more streams in ONE terminal (each has own decoder + thread):
 *     ntrip_rtcm_obs -c <host> <port> [label] [-c <host> <port> [label] ...]
 *       Example (two STRSVR servers on 51000 / 51001):
 *         ntrip_rtcm_obs -c 127.0.0.1 51000 BASE_A -c 127.0.0.1 51001 BASE_B
 *       Omit label to auto-use host:port as tag.
 *
 * Parses bytes with RTKLIB input_rtcm3():
 *   ret==1 — observation epoch (MSM / 1004 …): pseudorange P, carrier L
 *   ret==5 — station / antenna: includes RTCM 1005 (ARP ECEF), 1006 (+height),
 *            1007/1008/1033 (antenna/rec), etc.
 *
 * Same STATION ID (e.g. 1660) on two bases:
 *   RTCM staid is only 12 bits — collisions are normal. Use separate -c streams,
 *   different [label], and/or 1005/1006 ECEF to tell stations apart.
 *
 * Two streams (-c x2), aligned epochs (|Δt|≤0.5 s), no ephemeris / no sat position:
 *   Apollonius (triangle median length) with baseline b = ||ARP2-ARP1|| from 1005/1006:
 *     P_mid = sqrt( max(0, (P_1^2+P_2^2)/2 - b^2/4) )
 *   where P_1,P_2 are the two stations' pseudoranges (slant-range proxies;
 *   clock/iono residuals small vs. geometry on short baselines).
 *   Carrier uses the same Apollonius construction per signal directly in cycles:
 *   convert baseline b to cycles for that signal, keep L_i in cycles, and output
 *   L_mid in cycles.
 *   If either station lacks valid L for that signal, L_mid = 0.
 *----------------------------------------------------------------------------*/
#include "rtklib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* RTCM MSM7 signal masks (rtcm3.c); used to map obs codes MSM encoders accept */
extern const char *msm_sig_gps[32];
extern const char *msm_sig_glo[32];
extern const char *msm_sig_gal[32];
extern const char *msm_sig_qzs[32];
extern const char *msm_sig_sbs[32];
extern const char *msm_sig_cmp[32];
extern const char *msm_sig_irn[32];

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
typedef SOCKET sock_t;
#define SOCK_INVALID INVALID_SOCKET
#define sock_close closesocket
#define sock_errno WSAGetLastError()
#else
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
typedef int sock_t;
#define SOCK_INVALID (-1)
#define sock_close close
#define sock_errno errno
#endif

#ifndef DEFAULT_PORT
#define DEFAULT_PORT 50001
#endif

#define RTCM_OUT_PORT   52000
#define OUT_VIRT_STAID  2999
#define MAX_CLIENT_STREAMS 16
#define VIRT_STATION_INTERVAL 10.0
#define STREAM_THREAD_STACK_SIZE (8u * 1024u * 1024u)

typedef struct {
    const char *host;
    unsigned short port;
    const char *label; /* NULL or argv; if NULL, use label_storage */
    int eph_only;      /* -e <host> <port> [label]: feed broadcast eph only */
} cli_stream_t;

typedef struct {
    cli_stream_t s;
    char label_storage[64];
    int stream_index; /* 0..n-1 worker order (any role) */
    int base_index;   /* 0..g_base_n-1 for base streams; -1 for eph-only */
    rtcm_t rtcm;
} stream_worker_t;

/* B2bLib declares PPP_Glo in rtklib.h; full apps (e.g. rtkrcv) define it elsewhere. */
PPPGlobal_t PPP_Glo;

static rtklib_lock_t g_print_lock;

/* Multi-stream: latest ARP from RTCM 1005/1006 per BASE stream; midpoint when all valid.
 * Indexed by base_index, NOT stream_index. Eph-only streams skip these arrays. */
static int g_multi_n;       /* total streams (base + eph) */
static int g_base_n;        /* count of base streams (-c) only */
static double g_arp_ecef[MAX_CLIENT_STREAMS][3];
static int g_arp_valid[MAX_CLIENT_STREAMS];
static char g_stream_tag[MAX_CLIENT_STREAMS][64];
static double g_last_mid_ecef[3];
static int g_have_mid_print;

/* Global broadcast ephemeris pool, shared by all worker threads.
 * Updated whenever any stream (base or eph-only) returns ret==2 from input_rtcm3().
 * Read by synth_virt_obs_with_eph(). Protected by g_nav_lock. */
static nav_t g_nav;
static int g_nav_inited;
static rtklib_lock_t g_nav_lock;

/* Latest epoch snapshot per stream (for dual-base virtual midpoint obs) */
typedef struct {
    gtime_t time;
    int n;
    obsd_t data[MAXOBS];
} epoch_snap_t;

static epoch_snap_t g_epoch_snap[MAX_CLIENT_STREAMS];
static gtime_t g_last_synth_epoch;
static int g_have_synth_epoch;
static gtime_t g_last_station_tx_epoch;
static int g_have_station_tx_epoch;

static int g_suppress_input_obs;
static rtcm_t g_enc_rtcm;
static rtcm_t g_dec_rtcm;
static int g_enc_inited;
static rtklib_lock_t g_tx_lock;
static sock_t g_tx_listen = SOCK_INVALID;
static sock_t g_tx_conn = SOCK_INVALID;

static void print_enter(void);
static void print_leave(void);

static void print_init(void)
{
    rtklib_initlock(&g_print_lock);
}

static void tx_lock_init(void)
{
    rtklib_initlock(&g_tx_lock);
}

static void tx_send_buf(const uint8_t *p, int n)
{
    if (!p || n <= 0) return;
    rtklib_lock(&g_tx_lock);
    if (g_tx_conn != SOCK_INVALID)
        (void)send(g_tx_conn, (const char *)p, n, 0);
    rtklib_unlock(&g_tx_lock);
}

#ifdef _WIN32
static DWORD WINAPI rtcm_out_server_thread(LPVOID arg)
{
    sock_t ls, c;
    struct sockaddr_in a;
    (void)arg;

    ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ls == SOCK_INVALID)
        return 0;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(RTCM_OUT_PORT);
    if (bind(ls, (struct sockaddr *)&a, sizeof(a)) != 0 ||
        listen(ls, 2) != 0) {
        print_enter();
        fprintf(stderr, "RTCM TCP out: bind/listen port %u failed (%d)\n",
                (unsigned)RTCM_OUT_PORT, sock_errno);
        fflush(stderr);
        print_leave();
        sock_close(ls);
        return 0;
    }
    g_tx_listen = ls;
    print_enter();
    fprintf(stderr, "RTCM TCP out: listening on port %u (connect to receive stream)\n",
            (unsigned)RTCM_OUT_PORT);
    fflush(stderr);
    print_leave();
    for (;;) {
        c = accept(ls, NULL, NULL);
        if (c == SOCK_INVALID) continue;
        rtklib_lock(&g_tx_lock);
        if (g_tx_conn != SOCK_INVALID)
            sock_close(g_tx_conn);
        g_tx_conn = c;
        rtklib_unlock(&g_tx_lock);
    }
}
#else
static void *rtcm_out_server_thread(void *arg)
{
    sock_t ls, c;
    struct sockaddr_in a;
    (void)arg;

    ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ls == SOCK_INVALID)
        return NULL;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(RTCM_OUT_PORT);
    if (bind(ls, (struct sockaddr *)&a, sizeof(a)) != 0 ||
        listen(ls, 2) != 0) {
        print_enter();
        fprintf(stderr, "RTCM TCP out: bind/listen port %u failed (%d)\n",
                (unsigned)RTCM_OUT_PORT, sock_errno);
        fflush(stderr);
        print_leave();
        sock_close(ls);
        return NULL;
    }
    g_tx_listen = ls;
    print_enter();
    fprintf(stderr, "RTCM TCP out: listening on port %u (connect to receive stream)\n",
            (unsigned)RTCM_OUT_PORT);
    fflush(stderr);
    print_leave();
    for (;;) {
        c = accept(ls, NULL, NULL);
        if (c == SOCK_INVALID) continue;
        rtklib_lock(&g_tx_lock);
        if (g_tx_conn != SOCK_INVALID)
            sock_close(g_tx_conn);
        g_tx_conn = c;
        rtklib_unlock(&g_tx_lock);
    }
}
#endif

static void print_enter(void)
{
    rtklib_lock(&g_print_lock);
}

static void print_leave(void)
{
    rtklib_unlock(&g_print_lock);
}

static const char *stream_tag(const char *label)
{
    return (label && label[0]) ? label : "stream";
}

static const char *worker_label(const stream_worker_t *w)
{
    return (w->s.label && w->s.label[0]) ? w->s.label : w->label_storage;
}

/* RTCM3 message number in current frame (after successful decode). */
static int last_rtcm3_type(const rtcm_t *rtcm)
{
    return (int)getbitu(rtcm->buff, 24, 12);
}

static double norm3diff(const double a[3], const double b[3])
{
    double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* ---- Global broadcast ephemeris pool -----------------------------------*/

static int init_global_nav(void)
{
    static const eph_t  eph0  = {0};
    static const geph_t geph0 = {0};
    int i;

    if (g_nav_inited) return 1;
    memset(&g_nav, 0, sizeof(g_nav));
    g_nav.eph  = (eph_t  *)malloc(sizeof(eph_t)  * MAXSAT * 2);
    g_nav.geph = (geph_t *)malloc(sizeof(geph_t) * MAXPRNGLO);
    if (!g_nav.eph || !g_nav.geph) {
        free(g_nav.eph);  g_nav.eph  = NULL;
        free(g_nav.geph); g_nav.geph = NULL;
        return 0;
    }
    g_nav.n  = g_nav.nmax  = MAXSAT * 2;
    g_nav.ng = g_nav.ngmax = MAXPRNGLO;
    for (i = 0; i < MAXSAT * 2; i++)  g_nav.eph[i]  = eph0;
    for (i = 0; i < MAXPRNGLO; i++)   g_nav.geph[i] = geph0;
    rtklib_initlock(&g_nav_lock);
    g_nav_inited = 1;
    return 1;
}

/* Copy the most-recently-decoded ephemeris from a worker's rtcm_t into g_nav.
 * Called whenever input_rtcm3() returns 2 (eph updated) or 9 (special msgs that
 * may carry GLONASS FCN tables). */
static void merge_eph_from_rtcm(const rtcm_t *src)
{
    int sat, set, prn, sys, slot;

    if (!g_nav_inited || !src) return;
    sat = src->ephsat;
    set = src->ephset;
    if (sat <= 0 || sat > MAXSAT) return;

    sys = satsys(sat, &prn);
    rtklib_lock(&g_nav_lock);
    if (sys == SYS_GLO) {
        if (prn >= 1 && prn <= MAXPRNGLO)
            g_nav.geph[prn - 1] = src->nav.geph[prn - 1];
    }
    else {
        slot = (set == 1) ? sat - 1 + MAXSAT : sat - 1;
        if (slot >= 0 && slot < MAXSAT * 2)
            g_nav.eph[slot] = src->nav.eph[slot];
    }
    /* Iono parameters may arrive in 1019/1042/1044/1045/1046; copy lazily. */
    memcpy(g_nav.ion_gps, src->nav.ion_gps, sizeof(g_nav.ion_gps));
    memcpy(g_nav.ion_gal, src->nav.ion_gal, sizeof(g_nav.ion_gal));
    memcpy(g_nav.ion_qzs, src->nav.ion_qzs, sizeof(g_nav.ion_qzs));
    memcpy(g_nav.ion_cmp, src->nav.ion_cmp, sizeof(g_nav.ion_cmp));
    memcpy(g_nav.glo_fcn, src->nav.glo_fcn, sizeof(g_nav.glo_fcn));
    rtklib_unlock(&g_nav_lock);
}

/* True when at least one ephemeris of `sat` is in the pool. Cheap, no lock. */
static int g_nav_has_sat(int sat)
{
    int slot;
    int sys, prn;

    if (!g_nav_inited || sat <= 0 || sat > MAXSAT) return 0;
    sys = satsys(sat, &prn);
    if (sys == SYS_GLO)
        return (prn >= 1 && prn <= MAXPRNGLO) ? (g_nav.geph[prn - 1].sat != 0) : 0;
    slot = sat - 1;
    if (g_nav.eph[slot].sat) return 1;
    if (slot + MAXSAT < MAXSAT * 2 && g_nav.eph[slot + MAXSAT].sat) return 1;
    return 0;
}

/* Total ephemeris count for go/no-go on the new VRS path. */
static int g_nav_n_total(void)
{
    int i, c = 0;
    if (!g_nav_inited) return 0;
    for (i = 0; i < MAXSAT * 2; i++)
        if (g_nav.eph[i].sat) c++;
    for (i = 0; i < MAXPRNGLO; i++)
        if (g_nav.geph[i].sat) c++;
    return c;
}

/* After each 1005/1006: store ARP; when every BASE stream has ARP, print ECEF midpoint (mean).
 * Eph-only streams (base_index < 0) never feed the midpoint even if they carry 1005/1006. */
static void update_arp_midpoint_from_1005(const rtcm_t *rtcm, const stream_worker_t *w)
{
    int t = last_rtcm3_type(rtcm);
    int i, k, all;
    double mid[3], llh[3];

    if (g_base_n < 2)
        return;
    if (t != 1005 && t != 1006)
        return;
    if (w->base_index < 0)
        return;

    k = w->base_index;
    if (k >= g_base_n)
        return;

    print_enter();
    g_arp_ecef[k][0] = rtcm->sta.pos[0];
    g_arp_ecef[k][1] = rtcm->sta.pos[1];
    g_arp_ecef[k][2] = rtcm->sta.pos[2];
    g_arp_valid[k] = 1;

    all = 1;
    for (i = 0; i < g_base_n; i++) {
        if (!g_arp_valid[i]) {
            all = 0;
            break;
        }
    }
    if (!all) {
        print_leave();
        return;
    }

    mid[0] = mid[1] = mid[2] = 0.0;
    for (i = 0; i < g_base_n; i++) {
        mid[0] += g_arp_ecef[i][0];
        mid[1] += g_arp_ecef[i][1];
        mid[2] += g_arp_ecef[i][2];
    }
    mid[0] /= (double)g_base_n;
    mid[1] /= (double)g_base_n;
    mid[2] /= (double)g_base_n;

    if (g_have_mid_print && norm3diff(mid, g_last_mid_ecef) < 0.02) {
        print_leave();
        return;
    }
    memcpy(g_last_mid_ecef, mid, sizeof(mid));
    g_have_mid_print = 1;
    g_have_station_tx_epoch = 0;

    ecef2pos(mid, llh);
    printf("\n=== Midpoint (mean ECEF of %d base RTCM 1005 ARPs) ===\n", g_base_n);
    printf("  ECEF X=%.4f  Y=%.4f  Z=%.4f (m)\n", mid[0], mid[1], mid[2]);
    printf("  LLH  lat=%.8f deg  lon=%.8f deg  h_ellip=%.4f (m)\n",
           llh[0] * R2D, llh[1] * R2D, llh[2]);
    fflush(stdout);
    print_leave();
}

static void print_station_msg(const rtcm_t *rtcm, const char *label)
{
    int type = last_rtcm3_type(rtcm);
    double pos[3];
    const char *tag = stream_tag(label);

    if (type != 1005)
        return;

    print_enter();
    printf("\n[%s] RTCM %d - station / antenna (ARP in 1005/1006)\n", tag, type);
    if (rtcm->msgtype[0]) printf("  %s\n", rtcm->msgtype);

    printf("  staid=%d  sta.name=%s  ITRF year=%d\n",
           rtcm->staid, rtcm->sta.name, rtcm->sta.itrf);

    ecef2pos(rtcm->sta.pos, pos);
    printf("  ARP ECEF  X=%.4f  Y=%.4f  Z=%.4f (m)\n",
           rtcm->sta.pos[0], rtcm->sta.pos[1], rtcm->sta.pos[2]);
    printf("  ARP LLH   lat=%.8f deg  lon=%.8f deg  h_ellip=%.4f m\n",
           pos[0] * R2D, pos[1] * R2D, pos[2]);

    fflush(stdout);
    print_leave();
}

static void print_obs_epoch(const rtcm_t *rtcm, const char *label)
{
    char tstr[32], satid[16];
    int i, j;
    const char *tag = stream_tag(label);

    if (rtcm->obs.n <= 0) return;
    if (g_suppress_input_obs) {
        (void)label;
        return;
    }

    print_enter();
    time2str(rtcm->obs.data[0].time, tstr, 3);
    printf("\n[%s] --- epoch %s  RTCM staid=%d  n=%d ---\n",
           tag, tstr, rtcm->staid, rtcm->obs.n);
    if (rtcm->msgtype[0]) printf("last msg: %s\n", rtcm->msgtype);

    for (i = 0; i < rtcm->obs.n; i++) {
        const obsd_t *d = &rtcm->obs.data[i];
        satno2id(d->sat, satid);
        for (j = 0; j < NFREQ + NEXOBS; j++) {
            int hasp = d->P[j] != 0.0;
            int hasl = d->L[j] != 0.0;
            if (!hasp && !hasl) continue;
            printf("  %-4s  sig=%-3s", satid, code2obs(d->code[j]));
            if (hasp) printf("  P=%.3f m", d->P[j]);
            else printf("  P=---");
            if (hasl) printf("  L=%.6f cyc", d->L[j]);
            else printf("  L=---");
            if (d->SNR[j]) printf("  SNR=%.1f dBHz", d->SNR[j] * 0.001);
            printf("\n");
        }
    }
    fflush(stdout);
    print_leave();
}

static int find_sat_in_snap(const epoch_snap_t *sn, int sat)
{
    int i;
    for (i = 0; i < sn->n; i++)
        if (sn->data[i].sat == sat) return i;
    return -1;
}

static int find_same_code_idx(const obsd_t *d, uint8_t code)
{
    int j;
    for (j = 0; j < NFREQ + NEXOBS; j++)
        if (d->code[j] == code && (d->P[j] != 0.0 || d->L[j] != 0.0)) return j;
    return -1;
}

static void store_epoch_snapshot(int sidx, const obs_t *obs)
{
    int k;
    if (sidx < 0 || sidx >= MAX_CLIENT_STREAMS || obs->n <= 0) return;
    print_enter();
    g_epoch_snap[sidx].time = obs->data[0].time;
    g_epoch_snap[sidx].n = obs->n;
    for (k = 0; k < obs->n && k < MAXOBS; k++)
        g_epoch_snap[sidx].data[k] = obs->data[k];
    print_leave();
}

/* RTCM 3 MSM7 decode requires nsat*nsig <= 64; keep constellations small enough. */
#define MSM_MAX_SATS_PER_SYS 18

static const char **msm_sig_table(int sys)
{
    switch (sys) {
    case SYS_GPS: return msm_sig_gps;
    case SYS_GLO: return msm_sig_glo;
    case SYS_GAL: return msm_sig_gal;
    case SYS_QZS: return msm_sig_qzs;
    case SYS_SBS: return msm_sig_sbs;
    case SYS_CMP: return msm_sig_cmp;
    case SYS_IRN: return msm_sig_irn;
    default: return NULL;
    }
}

static int obs_code_in_msm_table(int sys, uint8_t code)
{
    const char **tbl = msm_sig_table(sys);
    char *s;
    int i;

    if (!tbl || code == CODE_NONE) return 0;
    s = code2obs(code);
    if (!s || !*s) return 0;
    for (i = 0; i < 32; i++) {
        if (!tbl[i] || !tbl[i][0]) continue;
        if (!strcmp(s, tbl[i])) return 1;
    }
    return 0;
}

static uint8_t msm_remap_obs_code(int sys, uint8_t code)
{
    if (code == CODE_NONE) return code;
    if (obs_code_in_msm_table(sys, code)) return code;

    switch (sys) {
    case SYS_CMP:
        /* msm_sig_cmp has no "1C" (B1C); map to codes in RTCM 1127 mask */
        if (code == CODE_L1C) return CODE_L1X;
        if (code == CODE_L1I) return CODE_L2I;
        if (code == CODE_L1Q) return CODE_L2Q;
        if (code == CODE_L1A) return CODE_L1X;
        if (code == CODE_L6A) return CODE_L6I;
        if (code == CODE_L8D || code == CODE_L8P) return CODE_L5X;
        if (code == CODE_L8X) return CODE_L7X;
        break;
    case SYS_GAL:
        if (code == CODE_L1C) return CODE_L1X;
        break;
    case SYS_GLO:
        return CODE_L1C;
    case SYS_IRN:
        return CODE_L5A;
    default:
        break;
    }
    if (!obs_code_in_msm_table(sys, code)) return CODE_NONE;
    return code;
}

static int msm_remap_preserves_phase(int sys, uint8_t from, uint8_t to, int fcn)
{
    double f_from, f_to;

    if (from == to) return 1;

    /* RTKLIB has historical BDS B1I codes as 1I/1Q; RTCM MSM uses 2I/2Q.
     * Treat these as label-only remaps even though code2freq() maps 1*
     * through the modern B1C branch. */
    if (sys == SYS_CMP) {
        if (from == CODE_L1I && to == CODE_L2I) return 1;
        if (from == CODE_L1Q && to == CODE_L2Q) return 1;
    }

    f_from = code2freq(sys, from, fcn);
    f_to   = code2freq(sys, to,   fcn);

    return f_from > 0.0 && f_to > 0.0 && fabs(f_from - f_to) < 1.0;
}

static void remap_obs_codes_for_msm7(obsd_t *data, int n)
{
    int i, j, prn, sys;
    uint8_t c;

    for (i = 0; i < n; i++) {
        sys = satsys(data[i].sat, &prn);
        for (j = 0; j < NFREQ + NEXOBS; j++) {
            uint8_t code_was;
            double L_was;

            if (data[i].code[j] == CODE_NONE) continue;
            if (data[i].P[j] == 0.0 && data[i].L[j] == 0.0) continue;
            code_was = data[i].code[j];
            L_was = data[i].L[j];
            c = msm_remap_obs_code(sys, code_was);
            if (c == CODE_NONE || !obs_code_in_msm_table(sys, c)) {
                data[i].code[j] = CODE_NONE;
                data[i].P[j] = data[i].L[j] = 0.0;
                continue;
            }
            if (c != code_was) {
                data[i].code[j] = c;
                if (L_was != 0.0 &&
                    !msm_remap_preserves_phase(sys, code_was, c, data[i].freq)) {
                    data[i].L[j] = 0.0;
                    data[i].LLI[j] = 0;
                }
            }
        }
    }
}

static int sys_bit_index(int sys)
{
    switch (sys) {
    case SYS_GPS: return 0;
    case SYS_GLO: return 1;
    case SYS_GAL: return 2;
    case SYS_QZS: return 3;
    case SYS_SBS: return 4;
    case SYS_CMP: return 5;
    case SYS_IRN: return 6;
    default: return -1;
    }
}

static void cap_rows_per_gnss_sys(obsd_t *data, int n, int max_per_sys)
{
    int i, prn, sys, bi;
    int cnt[7];

    memset(cnt, 0, sizeof(cnt));
    for (i = 0; i < n; i++) {
        sys = satsys(data[i].sat, &prn);
        bi = sys_bit_index(sys);
        if (bi < 0) continue;
        cnt[bi]++;
        if (cnt[bi] > max_per_sys)
            memset(&data[i], 0, sizeof(data[i]));
    }
}

static int compress_obs_rows(obsd_t *data, int n)
{
    int w, r, j, has;

    for (w = 0, r = 0; r < n; r++) {
        has = 0;
        for (j = 0; j < NFREQ + NEXOBS; j++) {
            if (data[r].code[j] != CODE_NONE &&
                (data[r].P[j] != 0.0 || data[r].L[j] != 0.0)) {
                has = 1;
                break;
            }
        }
        if (!has) continue;
        if (w != r) data[w] = data[r];
        w++;
    }
    return w;
}

static void prepare_virt_for_msm7(obsd_t *virt, int *pnv)
{
    remap_obs_codes_for_msm7(virt, *pnv);
    cap_rows_per_gnss_sys(virt, *pnv, MSM_MAX_SATS_PER_SYS);
    /* Keep all per-frequency slots that passed remap (e.g. dual-frequency for
     * RTK/PPP). build_msm_chunks() splits by nsat*nsig<=64. */
    *pnv = compress_obs_rows(virt, *pnv);
}

static int obs_has_any_for_sys(const obs_t *obs, int sys)
{
    int i, j, prn;

    for (i = 0; i < obs->n; i++) {
        if (satsys(obs->data[i].sat, &prn) != sys)
            continue;
        for (j = 0; j < NFREQ + NEXOBS; j++) {
            if (obs->data[i].code[j] == CODE_NONE)
                continue;
            if (obs->data[i].P[j] != 0.0 || obs->data[i].L[j] != 0.0)
                return 1;
        }
    }
    return 0;
}

static const struct { int sys; int type; } k_msm7[] = {
    { SYS_GPS, 1077 }, { SYS_GLO, 1087 }, { SYS_GAL, 1097 },
    { SYS_QZS, 1117 }, { SYS_SBS, 1107 }, { SYS_CMP, 1127 }, { SYS_IRN, 1137 },
};

#define N_MSM7 (int)(sizeof(k_msm7) / sizeof(k_msm7[0]))
/* Chunk count bound: worst case many small MSM pieces when nsig is large;
 * N_MSM7*MAXOBS alone would allocate tens of MB on the worker thread stack. */
#define MSM_MAX_CHUNKS (N_MSM7 * ((MAXOBS + 63) / 64))

typedef struct {
    int type;
    int n;
    obsd_t data[MAXOBS];
} msm_chunk_t;

static int build_msm_chunks(const obs_t *obs, msm_chunk_t *chunks, int max_chunks)
{
    int k, i, j, n, ns, nobs, nsig, code, nchunk = 0;
    int mask[MAXCODE];
    const obsd_t *data = obs->data;

    for (k = 0; k < N_MSM7; k++) {
        memset(mask, 0, sizeof(mask));
        nobs = nsig = 0;

        for (i = 0; i < obs->n && i < MAXOBS; i++) {
            if (satsys(data[i].sat, NULL) != k_msm7[k].sys) continue;
            nobs++;
            for (j = 0; j < NFREQ + NEXOBS; j++) {
                code = data[i].code[j];
                if (!code || code > MAXCODE || mask[code - 1]) continue;
                mask[code - 1] = 1;
                nsig++;
            }
        }
        if (nobs <= 0 || nsig <= 0) continue;

        ns = 64 / nsig;
        if (ns <= 0) continue;

        for (i = 0; i < obs->n && i < MAXOBS;) {
            msm_chunk_t *chunk;

            while (i < obs->n && i < MAXOBS &&
                   satsys(data[i].sat, NULL) != k_msm7[k].sys) {
                i++;
            }
            if (i >= obs->n || i >= MAXOBS) break;
            if (nchunk >= max_chunks) return nchunk;

            chunk = &chunks[nchunk++];
            chunk->type = k_msm7[k].type;
            chunk->n = 0;

            for (n = 0; n < ns && i < obs->n && i < MAXOBS; i++) {
                if (satsys(data[i].sat, NULL) != k_msm7[k].sys) continue;
                chunk->data[chunk->n++] = data[i];
                n++;
            }
        }
    }
    return nchunk;
}

/* Forward declarations for SNR helpers (bodies live further down near
 * median_of, but Builder A below already needs snr_combine_x1000). */
static double   snr_to_weight(uint16_t snr_x1000);
static uint16_t snr_combine_x1000(uint16_t snr_a_x1000, uint16_t snr_b_x1000);
static int      snr_gate_pass(uint16_t snr_a_x1000, uint16_t snr_b_x1000);

/* ------------------------------------------------------------------------- */
/* Builder A: legacy Apollonius median formula on P and L (cycles).           */
/* Used as fallback when broadcast ephemeris is not yet available.            */
/* ------------------------------------------------------------------------- */
static int build_virt_obs_apollonius(
    const epoch_snap_t *s0, const epoch_snap_t *s1,
    double b, obsd_t *virt, int *pnv)
{
    int i0, i1, j, j1, sat, prn, sys, nv = 0;
    double P0, P1, Pm_sq, Pm, freq, lam, bcyc, L0, L1, Lm_sq;
    const obsd_t *d0, *d1;

    for (i0 = 0; i0 < s0->n && nv < MAXOBS; i0++) {
        obsd_t vd;
        int any = 0;
        d0 = &s0->data[i0];
        sat = d0->sat;
        i1 = find_sat_in_snap(s1, sat);
        if (i1 < 0) continue;
        d1 = &s1->data[i1];

        memset(&vd, 0, sizeof(vd));
        vd.time = d0->time;
        vd.sat = sat;
        vd.freq = d0->freq;
        sys = satsys(sat, &prn); (void)prn;

        for (j = 0; j < NFREQ + NEXOBS; j++) {
            if (d0->code[j] == CODE_NONE) continue;
            if (d0->P[j] == 0.0 && d0->L[j] == 0.0) continue;
            j1 = find_same_code_idx(d1, d0->code[j]);
            if (j1 < 0) continue;

            P0 = d0->P[j]; P1 = d1->P[j1];
            if (P0 <= 0.0 || P1 <= 0.0) continue;

            Pm_sq = 0.5 * (P0 * P0 + P1 * P1) - 0.25 * b * b;
            if (Pm_sq < 0.0) continue;
            Pm = sqrt(Pm_sq);

            vd.code[j] = d0->code[j];
            vd.P[j] = Pm;
            vd.L[j] = 0.0;
            freq = code2freq(sys, d0->code[j], d0->freq);
            lam = (freq > 0.0) ? CLIGHT / freq : 0.0;
            if (lam > 0.0 && d0->L[j] != 0.0 && d1->L[j1] != 0.0) {
                L0 = d0->L[j]; L1 = d1->L[j1];
                bcyc = b / lam;
                Lm_sq = 0.5 * (L0 * L0 + L1 * L1) - 0.25 * bcyc * bcyc;
                if (Lm_sq >= 0.0) vd.L[j] = sqrt(Lm_sq);
            }
            /* Apollonius is a cold-start fallback: keep its observation
             * combination unchanged (geometric mean), but use the dB-domain
             * SNR sum so the output value is physically meaningful and
             * cannot overflow uint16_t. Apollonius does not gate by SNR. */
            vd.SNR[j] = snr_combine_x1000(d0->SNR[j], d1->SNR[j1]);
            any = 1;
        }
        if (any) virt[nv++] = vd;
    }
    *pnv = nv;
    return nv > 0 ? 1 : 0;
}

/* Per-satellite cached geometry for the ephemeris-aware builder.
 * Static under print_lock (try_synth_virtual_obs holds print_lock entire run).
 * Indexed by SAT NUMBER directly (sat range 1..MAXSAT) so that union-mode
 * processing of sats present on only one base works without sat<->index
 * lookup tables. valid_A / valid_B are independent flags so a single base
 * outage doesn't invalidate the geometry of the surviving base. */
typedef struct {
    int    valid_A, valid_B;     /* per-base geometry validity (independent) */
    double dts0;                 /* satellite clock bias (s) at transmit time */
    double rho_A, rho_B, rho_V;  /* Sagnac-aware geometric ranges (m) */
    double ion_A, ion_B, ion_V;  /* Klobuchar L1 vertical-mapped iono delay (m) */
    double trp_A, trp_B, trp_V;  /* Saastamoinen tropo delay (m) */
    double elev_A, elev_B;       /* per-base elevation (rad) */
} sat_geom_t;

static sat_geom_t g_geom_by_sat[MAXSAT + 1];

/* Sat → snapshot-index lookup tables, refilled each epoch.
 * -1 means the sat is absent from that base's snapshot. */
static int g_sat_to_iA[MAXSAT + 1];
static int g_sat_to_iB[MAXSAT + 1];

/* ------------------------------------------------------------------------- */
/* Phase-side: A-B double-difference integer-ambiguity cache.                 */
/*                                                                            */
/* For each (system, code) we maintain a single reference satellite j*; for   */
/* every (sat k, code) with valid L on both bases we cache the fixed integer  */
/*   DD_n^k = (N_A^k - N_B^k) - (N_A^j* - N_B^j*)                             */
/* in cycles. The clock-difference term cancels in DD, so DD_n is recoverable */
/* by simple round-to-nearest with mm-level noise on short baselines.         */
/*                                                                            */
/* On reference-sat switch j_old -> j_new we transform every cached entry     */
/*   DD_n_new^k = DD_n_old^k - DD_n_old^{j_new}                               */
/* so previously-fixed sats keep their fix without re-search.                 */
/* ------------------------------------------------------------------------- */

#define VRS_AMB_FIX_THRES 0.20  /* cycle: round-to-int residual must be below this */
#define VRS_AMB_KEEP_THRES 0.40 /* cycle: cached fix re-validates if within this */

/* SNR gate: a signal whose either base reports SNR below this is dropped from
 * BOTH pseudorange and carrier output (and not used to fix DD ambiguities or
 * to be picked as a reference sat). 25 dBHz is the conventional "below this is
 * mostly multipath/edge of antenna pattern" floor. obsd_t.SNR is in 0.001 dBHz. */
#define MIN_SNR_THRES_X1000 25000

typedef struct {
    int    valid;       /* 1 = dd_n is current, 0 = unfixed/invalidated */
    double dd_n;        /* fixed DD integer (cycles), see formula above */
    int    ref_sat;     /* the reference sat this dd_n is wrt (must match g_ref_sat) */
    gtime_t t_last;     /* last epoch this entry was confirmed */
} amb_cache_t;

/* Reference sat per (sys_bit_index, code-1).  0 = none. */
static int          g_ref_sat[7][MAXCODE];
static amb_cache_t  g_amb[MAXSAT][MAXCODE];

/* Per-epoch QC counters (set by build_virt_obs_with_eph). */
static int g_qc_n_fix, g_qc_n_float, g_qc_n_drop, g_qc_ref_switch;
static int g_qc_n_lowsnr;     /* signals dropped because either base SNR < gate */
static int g_qc_n_p_only_A;   /* sats only on base A: P-only output (DUAL mode) */
static int g_qc_n_p_only_B;   /* sats only on base B: P-only output (DUAL mode) */

/* ------------------------------------------------------------------------- */
/* VRS operating mode (epoch-level state machine).                            */
/*                                                                            */
/* DUAL  : both bases healthy → existing 2-base ML blend (best precision).    */
/* A_ONLY: base B is dead → all output synthesized from A only, geometrically */
/*         shifted to the midpoint V using broadcast ephemeris. Carrier amb.  */
/*         convention is N_A^k for ALL sats (rover DD stays integer).         */
/* B_ONLY: symmetric.                                                          */
/* NONE  : neither base usable → no VRS output (Apollonius fallback also      */
/*         fails because it needs both bases).                                */
/*                                                                            */
/* Mode transitions are gated by hysteresis to prevent RTCM packet-loss-      */
/* induced thrashing from injecting spurious LLI=1 cycle slips on the rover. */
/*   DUAL → SINGLE_X: triggered by MODE_FAIL_TO_SINGLE consecutive epochs    */
/*                    of "other-side has < 4 valid clock sats".              */
/*   SINGLE_X → DUAL: triggered by MODE_RECOVER_TO_DUAL consecutive good     */
/*                    epochs (much shorter to react to recoveries).          */
/*                                                                            */
/* On each transition (DUAL ↔ SINGLE), the carrier ambiguity convention      */
/* changes by 0.5*(N_B^j* − N_A^j*) per sat, which the rover sees as a       */
/* whole-network cycle slip. We force LLI=1 on every output sat in the FIRST */
/* epoch after the switch, prompting the rover to drop and refresh its      */
/* ambiguity state cleanly.                                                   */
/* ------------------------------------------------------------------------- */
typedef enum {
    VRS_MODE_NONE = 0,
    VRS_MODE_DUAL,
    VRS_MODE_A_ONLY,
    VRS_MODE_B_ONLY
} vrs_mode_t;

#define MODE_FAIL_TO_SINGLE  3   /* epochs of side-X-bad before leaving DUAL */
#define MODE_RECOVER_TO_DUAL 5   /* epochs of side-X-good before re-entering DUAL */
#define MIN_CLOCK_SATS       4   /* per base, for clock estimation viability */

static vrs_mode_t g_vrs_mode      = VRS_MODE_NONE;
static int        g_mode_bad_A    = 0;   /* consecutive epochs of "A < MIN_CLOCK_SATS" */
static int        g_mode_bad_B    = 0;
static int        g_mode_good_A   = 0;   /* consecutive epochs of "A >= MIN_CLOCK_SATS" */
static int        g_mode_good_B   = 0;
static int        g_force_lli_next = 0;  /* set by mode transition; consumed by next synth */

static void init_amb_cache(void)
{
    memset(g_ref_sat, 0, sizeof(g_ref_sat));
    memset(g_amb,     0, sizeof(g_amb));
}

static double median_of(double *a, int n)
{
    int i, j;
    double t;
    /* insertion sort (n is small, typically <= 40) */
    for (j = 1; j < n; j++) {
        t = a[j]; i = j - 1;
        while (i >= 0 && a[i] > t) { a[i + 1] = a[i]; i--; }
        a[i + 1] = t;
    }
    return (n & 1) ? a[n / 2] : 0.5 * (a[n / 2 - 1] + a[n / 2]);
}

/* SNR -> linear-power weight (10^(SNR_dB/10)).
 * SNR is in 0.001 dBHz (RTKLIB convention). Returns 0 for unknown SNR (=0). */
static double snr_to_weight(uint16_t snr_x1000)
{
    if (!snr_x1000) return 0.0;
    return pow(10.0, ((double)snr_x1000 * 0.001) * 0.1);
}

/* Combine two SNRs (both in 0.001 dBHz) into the output SNR of a coherent
 * sum of two independent signals: 10*log10(P_a + P_b).
 *   - If only one input has SNR (the other is 0), pass it through.
 *   - The result is capped at 65.5 dBHz so it always fits uint16_t safely
 *     (raw addition of two uint16_t SNRs would already be near the limit;
 *     the dB-domain version makes the cap explicit and physically correct). */
static uint16_t snr_combine_x1000(uint16_t snr_a_x1000, uint16_t snr_b_x1000)
{
    double a_db, b_db, comb_db;
    if (!snr_a_x1000) return snr_b_x1000;
    if (!snr_b_x1000) return snr_a_x1000;
    a_db = (double)snr_a_x1000 * 0.001;
    b_db = (double)snr_b_x1000 * 0.001;
    comb_db = 10.0 * log10(pow(10.0, a_db * 0.1) + pow(10.0, b_db * 0.1));
    if (comb_db > 65.5) comb_db = 65.5;
    if (comb_db < 0.0) comb_db = 0.0;
    return (uint16_t)(comb_db * 1000.0 + 0.5);
}

/* Quick gate: returns 1 iff BOTH SNRs are known and >= MIN_SNR_THRES_X1000.
 * Returns 1 (permissive) if either SNR is unknown (=0), so streams without
 * SNR reporting still flow through unchanged. */
static int snr_gate_pass(uint16_t snr_a_x1000, uint16_t snr_b_x1000)
{
    if (snr_a_x1000 > 0 && snr_a_x1000 < MIN_SNR_THRES_X1000) return 0;
    if (snr_b_x1000 > 0 && snr_b_x1000 < MIN_SNR_THRES_X1000) return 0;
    return 1;
}

/* Locate row index in a snapshot for the (sat, code) pair, with valid L on both
 * (P doesn't matter). Returns slot j on s0 and *j1 on s1, or -1 on either if not
 * found. We require *both* L != 0 because the DD step needs both phases. */
static int find_obs_idx_with_L(const epoch_snap_t *s0, int i0,
                               const epoch_snap_t *s1, int *out_i1, int *out_j1,
                               uint8_t code)
{
    int j, j1, i1;
    if (s0->data[i0].sat == 0) return -1;
    for (j = 0; j < NFREQ + NEXOBS; j++) {
        if (s0->data[i0].code[j] == code &&
            s0->data[i0].L[j] != 0.0) break;
    }
    if (j >= NFREQ + NEXOBS) return -1;
    i1 = find_sat_in_snap(s1, s0->data[i0].sat);
    if (i1 < 0) return -1;
    j1 = find_same_code_idx(&s1->data[i1], code);
    if (j1 < 0 || s1->data[i1].L[j1] == 0.0) return -1;
    *out_i1 = i1;
    *out_j1 = j1;
    return j;
}

/* Pick the highest-elevation satellite tracked on both bases with a valid L
 * for the given (sys, code). Returns sat number or 0 if none qualifies. */
/* Pick a reference sat for (sys, code) DD.
 * Requires: sat present on BOTH s0 AND s1, geometry valid on both bases,
 *           BOTH bases have valid L on the requested code, and BOTH SNRs
 *           pass the 25 dBHz gate. Among candidates, takes the one with
 *           highest elevation (averaged over A/B; on short baselines they
 *           are within ~0.1°, so this is well-defined).
 * Used only in DUAL mode; SINGLE_X modes don't need DD ambiguity fixing. */
static int pick_ref_sat(int sys, uint8_t code,
                        const epoch_snap_t *s0, const epoch_snap_t *s1)
{
    int best_sat = 0;
    double best_elev = 0.0;
    int sat, prn, iA, iB, jB, j;

    for (sat = 1; sat <= MAXSAT; sat++) {
        const sat_geom_t *g = &g_geom_by_sat[sat];
        double elev;
        if (!g->valid_A || !g->valid_B) continue;
        if (satsys(sat, &prn) != sys) continue;
        iA = g_sat_to_iA[sat];
        if (iA < 0) continue;
        j = find_obs_idx_with_L(s0, iA, s1, &iB, &jB, code);
        if (j < 0) continue;
        /* Reject as ref any sat whose carrier on either base is below the
         * SNR gate. A bad ref injects noise into ALL non-ref DD residuals,
         * so its quality matters disproportionately. Permissive when SNR=0
         * (unknown) so streams without SNR reporting still produce a ref. */
        if (!snr_gate_pass(s0->data[iA].SNR[j], s1->data[iB].SNR[jB])) continue;
        elev = 0.5 * (g->elev_A + g->elev_B);
        if (elev > best_elev) {
            best_elev = elev;
            best_sat  = sat;
        }
    }
    return best_sat;
}

/* On reference-sat switch j_old -> j_new for (sys, code):
 *   DD_n_new^k = DD_n_old^k - DD_n_old^{j_new}
 * (Old j_new entry has valid DD_n_old^{j_new} cached vs j_old; after swap we
 *  set j_new's slot to dd_n=0, and j_old's slot to -DD_n_old^{j_new}.)
 * Sats that were not fixed under the old ref stay invalid. */
static void transform_amb_on_ref_change(int sys, uint8_t code,
                                        int j_old, int j_new)
{
    double dd_jnew = 0.0;
    int i, prn, sat;
    int code_idx = (int)code - 1;

    if (code_idx < 0 || code_idx >= MAXCODE) return;
    if (j_new <= 0 || j_new > MAXSAT) return;

    if (j_old > 0 && j_old <= MAXSAT &&
        g_amb[j_new - 1][code_idx].valid &&
        g_amb[j_new - 1][code_idx].ref_sat == j_old) {
        dd_jnew = g_amb[j_new - 1][code_idx].dd_n;
    }
    else {
        /* No bridge — invalidate everything for this (sys, code) and re-fix
         * from scratch on the next iteration. */
        for (i = 0; i < MAXSAT; i++) {
            sat = i + 1;
            if (satsys(sat, &prn) != sys) continue;
            g_amb[i][code_idx].valid = 0;
        }
        if (j_new > 0)
            g_amb[j_new - 1][code_idx].dd_n = 0.0;
        return;
    }

    for (i = 0; i < MAXSAT; i++) {
        amb_cache_t *a = &g_amb[i][code_idx];
        sat = i + 1;
        if (satsys(sat, &prn) != sys) continue;
        if (!a->valid || a->ref_sat != j_old) {
            a->valid = 0;
            continue;
        }
        a->dd_n   -= dd_jnew;
        a->ref_sat = j_new;
    }
    /* j_new becomes the new origin: dd_n=0 by definition. */
    g_amb[j_new - 1][code_idx].valid   = 1;
    g_amb[j_new - 1][code_idx].dd_n    = 0.0;
    g_amb[j_new - 1][code_idx].ref_sat = j_new;
    /* j_old is now a regular satellite at -dd_jnew. */
    if (j_old > 0 && j_old <= MAXSAT) {
        g_amb[j_old - 1][code_idx].valid   = 1;
        g_amb[j_old - 1][code_idx].dd_n    = -dd_jnew;
        g_amb[j_old - 1][code_idx].ref_sat = j_new;
    }
}

/* ------------------------------------------------------------------------- */
/* Builder B: broadcast-ephemeris-aware VRS pseudorange.                      */
/*                                                                            */
/* For each satellite k present in BOTH base snapshots:                       */
/*   1) sat pos r_s and clock dt_s via satposs() with broadcast eph           */
/*   2) sagnac-aware geometric ranges rho_A, rho_B, rho_V via geodist()       */
/*   3) Klobuchar L1 iono and Saastamoinen tropo at A, B, V                   */
/*                                                                            */
/* Per-base receiver clock (m) is estimated as:                               */
/*   c*dt_rcv_i = median over sats of (P_i - rho_i + c*dt_s - I_i - T_i)      */
/* using the first valid pseudorange of each satellite (single-frequency for  */
/* clock, robust enough for code-level VRS).                                  */
/*                                                                            */
/* For each (sat, signal):                                                    */
/*   eps_A = P_A - rho_A + c*dt_s - I_A - T_A - c*dt_rcv_A   [code residual]  */
/*   eps_B = P_B - rho_B + c*dt_s - I_B - T_B - c*dt_rcv_B                    */
/*   c*dt_rcv_V = (c*dt_rcv_A + c*dt_rcv_B) / 2                               */
/*   P_V = rho_V - c*dt_s + I_V + T_V + c*dt_rcv_V + 0.5*(eps_A + eps_B)      */
/*                                                                            */
/* Carrier still uses Apollonius (placeholder until phase upgrade).           */
/* Returns 1 on success (nv populated), 0 if not enough data/ephemeris.       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* Helper: compute per-base geometry into g_geom_by_sat for ONE base.         */
/*                                                                            */
/* For each sat present in `snap`:                                            */
/*   - calls satposs (under g_nav_lock) to get sat ECEF and dts               */
/*   - computes Sagnac-aware geo range, iono (Klobuchar), tropo (Saast.)      */
/*     for the base                                                           */
/*   - on the FIRST base call (caller controls), also fills V's geometry      */
/*     (rho_V, ion_V, trp_V) using V position, since V geom depends only on   */
/*     sat position (~1 ppm difference between A's and B's sat-pos estimate, */
/*     negligible). Subsequent base calls will not overwrite V geometry.     */
/*                                                                            */
/* `which` = 'A' or 'B' selects which side to fill. `init_v_geom` should be  */
/* set on the first call only (idempotent across multiple sats). Returns the */
/* number of sats with valid geometry on this base.                          */
/* ------------------------------------------------------------------------- */
static int compute_geom_for_base(
    const epoch_snap_t *snap, char which,
    const double rBase[3], const double rV[3],
    int init_v_geom)
{
    static double rs[6 * MAXOBS];
    static double dts[2 * MAXOBS];
    static double var[MAXOBS];
    static int    svh[MAXOBS];
    double posBase[3], posV[3], azel[2], e[3];
    int i, n_valid = 0;

    if (snap->n <= 0) return 0;
    if (snap->n > MAXOBS) return 0;

    ecef2pos(rBase, posBase);
    ecef2pos(rV,    posV);

    rtklib_lock(&g_nav_lock);
    satposs(snap->time, snap->data, snap->n, &g_nav, EPHOPT_BRDC,
            rs, dts, var, svh);

    for (i = 0; i < snap->n; i++) {
        const double *r_s = rs + 6 * i;
        const double *ion_p;
        int sat = snap->data[i].sat;
        sat_geom_t *g;

        if (sat <= 0 || sat > MAXSAT) continue;
        g = &g_geom_by_sat[sat];

        if (svh[i]) continue;
        if (r_s[0] == 0.0 && r_s[1] == 0.0 && r_s[2] == 0.0) continue;

        ion_p = (satsys(sat, NULL) == SYS_CMP) ? g_nav.ion_cmp : g_nav.ion_gps;

        {
            double rho = geodist(r_s, rBase, e);
            satazel(posBase, e, azel);
            if (rho <= 0.0 || azel[1] < 5.0 * D2R) continue;
            if (which == 'A') {
                g->rho_A  = rho;
                g->elev_A = azel[1];
                g->ion_A  = ionmodel(snap->time, ion_p, posBase, azel);
                g->trp_A  = tropmodel(snap->time, posBase, azel, 0.7);
                g->valid_A = 1;
            } else {
                g->rho_B  = rho;
                g->elev_B = azel[1];
                g->ion_B  = ionmodel(snap->time, ion_p, posBase, azel);
                g->trp_B  = tropmodel(snap->time, posBase, azel, 0.7);
                g->valid_B = 1;
            }
        }

        /* Fill V's geometry on first base that sees this sat. Using A's or B's
         * sat-pos estimate gives sub-cm difference for V, negligible. */
        if (init_v_geom &&
            !((g->rho_V > 0.0))) {
            double rho_V = geodist(r_s, rV, e);
            satazel(posV, e, azel);
            if (rho_V > 0.0) {
                g->rho_V = rho_V;
                g->ion_V = ionmodel(snap->time, ion_p, posV, azel);
                g->trp_V = tropmodel(snap->time, posV, azel, 0.7);
                g->dts0  = dts[2 * i];
            }
        }
        n_valid++;
    }
    rtklib_unlock(&g_nav_lock);
    return n_valid;
}

/* Estimate single-base receiver clock (m) via median of per-sat residuals.
 * Uses the FIRST valid pseudorange of each sat as a robust 1-sample/sat input;
 * median copes with multipath outliers without per-sat weighting.
 * `which` selects which base's geometry slot to read.
 * Returns the number of sats that contributed (n); clock value via *clk_out.
 * If n < MIN_CLOCK_SATS, *clk_out is set to 0 and the caller should treat
 * this base as "down" for mode-decision purposes. */
static int estimate_base_clock(
    const epoch_snap_t *snap, char which, double *clk_out)
{
    static double res[MAXOBS], tmp[MAXOBS];
    int i, j, n = 0;

    *clk_out = 0.0;
    if (!snap || snap->n <= 0) return 0;

    for (i = 0; i < snap->n; i++) {
        const obsd_t *d = &snap->data[i];
        const sat_geom_t *g;
        double P = 0.0;

        if (d->sat <= 0 || d->sat > MAXSAT) continue;
        g = &g_geom_by_sat[d->sat];
        if (which == 'A' ? !g->valid_A : !g->valid_B) continue;

        for (j = 0; j < NFREQ + NEXOBS; j++) {
            if (d->P[j] > 0.0) { P = d->P[j]; break; }
        }
        if (P <= 0.0) continue;

        if (which == 'A')
            res[n++] = P - g->rho_A + CLIGHT * g->dts0 - g->ion_A - g->trp_A;
        else
            res[n++] = P - g->rho_B + CLIGHT * g->dts0 - g->ion_B - g->trp_B;
    }

    if (n < MIN_CLOCK_SATS) return n;
    memcpy(tmp, res, (size_t)n * sizeof(double));
    *clk_out = median_of(tmp, n);
    return n;
}

/* Mode state machine — call once per epoch with current health flags.
 * Updates g_vrs_mode and sets g_force_lli_next on a transition.
 * `A_healthy`/`B_healthy` should be (n_clock_X >= MIN_CLOCK_SATS). */
static void update_vrs_mode(int A_healthy, int B_healthy)
{
    vrs_mode_t prev = g_vrs_mode;
    vrs_mode_t target;

    /* Streak counters (one per side, mutually exclusive). */
    if (A_healthy) { g_mode_good_A++; g_mode_bad_A = 0; }
    else           { g_mode_bad_A++;  g_mode_good_A = 0; }
    if (B_healthy) { g_mode_good_B++; g_mode_bad_B = 0; }
    else           { g_mode_bad_B++;  g_mode_good_B = 0; }

    /* Decide target mode WITHOUT hysteresis (instantaneous best). */
    if (A_healthy && B_healthy)  target = VRS_MODE_DUAL;
    else if (A_healthy)          target = VRS_MODE_A_ONLY;
    else if (B_healthy)          target = VRS_MODE_B_ONLY;
    else                         target = VRS_MODE_NONE;

    /* Apply hysteresis: only switch AWAY from current mode if the trigger
     * has held for K consecutive epochs. Switching TO DUAL needs more
     * confirmation (RECOVER) than switching AWAY from DUAL (FAIL). */
    if (prev == VRS_MODE_NONE) {
        /* Cold start: accept any non-NONE mode immediately. */
        g_vrs_mode = target;
    }
    else if (prev == VRS_MODE_DUAL) {
        if (target == VRS_MODE_A_ONLY && g_mode_bad_B >= MODE_FAIL_TO_SINGLE)
            g_vrs_mode = VRS_MODE_A_ONLY;
        else if (target == VRS_MODE_B_ONLY && g_mode_bad_A >= MODE_FAIL_TO_SINGLE)
            g_vrs_mode = VRS_MODE_B_ONLY;
        else if (target == VRS_MODE_NONE &&
                 g_mode_bad_A >= MODE_FAIL_TO_SINGLE &&
                 g_mode_bad_B >= MODE_FAIL_TO_SINGLE)
            g_vrs_mode = VRS_MODE_NONE;
        /* otherwise: stay in DUAL even though one side is briefly down */
    }
    else if (prev == VRS_MODE_A_ONLY) {
        if (target == VRS_MODE_DUAL && g_mode_good_B >= MODE_RECOVER_TO_DUAL)
            g_vrs_mode = VRS_MODE_DUAL;
        else if (target == VRS_MODE_NONE && g_mode_bad_A >= MODE_FAIL_TO_SINGLE)
            g_vrs_mode = VRS_MODE_NONE;
        else if (target == VRS_MODE_B_ONLY) {
            /* A died and B alive — direct hand-over (rare, both transitions). */
            g_vrs_mode = VRS_MODE_B_ONLY;
        }
    }
    else if (prev == VRS_MODE_B_ONLY) {
        if (target == VRS_MODE_DUAL && g_mode_good_A >= MODE_RECOVER_TO_DUAL)
            g_vrs_mode = VRS_MODE_DUAL;
        else if (target == VRS_MODE_NONE && g_mode_bad_B >= MODE_FAIL_TO_SINGLE)
            g_vrs_mode = VRS_MODE_NONE;
        else if (target == VRS_MODE_A_ONLY)
            g_vrs_mode = VRS_MODE_A_ONLY;
    }

    if (prev != g_vrs_mode && g_vrs_mode != VRS_MODE_NONE)
        g_force_lli_next = 1;   /* signal cycle-slip on the next emitted epoch */
}

/* Single-base synthesis: shift `snap`'s observations from base position to V
 * using broadcast-ephemeris geometry. ALL sats (with valid geometry on this
 * base) emit BOTH P and L; carrier ambiguity is single-base N_X^k for every
 * sat, so the rover DD remains integer (different convention from DUAL mode,
 * which is why a mode transition forces LLI=1 to refix the rover).
 *
 *   P_V = P_X + (rho_V - rho_X) + (I_V - I_X) + (T_V - T_X)
 *   L_V = L_X + (rho_V - rho_X) / lambda          // iono diff < mm on
 *                                                 // short baselines, ignored
 *
 * `clk_X` is unused here (it cancels exactly in the residual form), but is
 * forwarded to the QC log via the caller. */
static int build_virt_obs_single(
    const epoch_snap_t *snap, char which,
    obsd_t *virt, int *pnv, int force_lli)
{
    int i, j, nv = 0, sys, prn, sat;
    int p_only_counter_inc;

    if (!snap || snap->n <= 0) return 0;

    p_only_counter_inc = 0;  /* SINGLE mode: no "P-only" downgrade — every
                              * eligible signal outputs P+L; this is just
                              * to silence unused-var pedantic warnings. */
    (void)p_only_counter_inc;

    for (i = 0; i < snap->n && nv < MAXOBS; i++) {
        const obsd_t *d = &snap->data[i];
        const sat_geom_t *g;
        obsd_t vd;
        int any = 0;

        sat = d->sat;
        if (sat <= 0 || sat > MAXSAT) continue;
        g = &g_geom_by_sat[sat];
        if (which == 'A' ? !g->valid_A : !g->valid_B) continue;
        if (g->rho_V <= 0.0) continue;
        sys = satsys(sat, &prn); (void)sys; (void)prn;

        memset(&vd, 0, sizeof(vd));
        vd.time = d->time;
        vd.sat  = sat;
        vd.freq = d->freq;

        for (j = 0; j < NFREQ + NEXOBS; j++) {
            double rho_X, ion_X, trp_X;

            if (d->code[j] == CODE_NONE) continue;

            /* SNR floor: same gate as DUAL mode but only on the surviving base. */
            if (d->SNR[j] > 0 && d->SNR[j] < MIN_SNR_THRES_X1000) {
                g_qc_n_lowsnr++;
                continue;
            }

            if (which == 'A') {
                rho_X = g->rho_A; ion_X = g->ion_A; trp_X = g->trp_A;
            } else {
                rho_X = g->rho_B; ion_X = g->ion_B; trp_X = g->trp_B;
            }

            /* Pseudorange */
            if (d->P[j] > 0.0) {
                vd.code[j] = d->code[j];
                vd.P[j] = d->P[j] + (g->rho_V - rho_X)
                                  + (g->ion_V - ion_X)
                                  + (g->trp_V - trp_X);
                vd.SNR[j] = d->SNR[j];
                /* In SINGLE mode "snr_combine" with a single source equals
                 * the source itself. No need to call snr_combine_x1000. */
                any = 1;
            }

            /* Carrier — pure geometric shift. Iono difference on phase has
             * opposite sign vs code; on short baselines (≤ a few km) the
             * I_V−I_X term is < 1 mm, well below carrier noise — we omit it
             * for the same reason the DUAL path does. */
            if (d->L[j] != 0.0) {
                double freq = code2freq(sys, d->code[j], d->freq);
                double lam  = (freq > 0.0) ? CLIGHT / freq : 0.0;
                if (lam > 0.0) {
                    vd.L[j]   = d->L[j] + (g->rho_V - rho_X) / lam;
                    if (vd.code[j] == 0) vd.code[j] = d->code[j];
                    /* Inherit cycle-slip flag from source AND inject LLI=1
                     * on the first epoch of a mode transition. */
                    vd.LLI[j] = (uint8_t)((d->LLI[j] | (force_lli ? 1 : 0)) & 0xFF);
                    any = 1;
                    g_qc_n_fix++;  /* "single-base fix" — count as fix in QC */
                }
            }
        }
        if (any) virt[nv++] = vd;
    }

    *pnv = nv;
    return nv > 0 ? 1 : 0;
}

static int build_virt_obs_with_eph(
    const epoch_snap_t *s0, const epoch_snap_t *s1,
    const double rA[3], const double rB[3], const double rV[3],
    obsd_t *virt, int *pnv,
    double *clk_A_out, double *clk_B_out,
    int *nclk_A_out, int *nclk_B_out,
    double *rms_eps_out)
{
    double clk_A = 0.0, clk_B = 0.0, clk_V;
    double sum_dd2 = 0.0;
    int n_dd = 0;
    int n_res_A = 0, n_res_B = 0;
    int i, i0, i1, j, j1, sat, sys, prn, nv = 0;
    int A_healthy, B_healthy;
    int force_lli;

    if (!g_nav_inited) return 0;
    if (s0->n <= 0 && s1->n <= 0) return 0;     /* both bases empty */
    if (s0->n > MAXOBS || s1->n > MAXOBS) return 0;

    /* ----- Reset sat-keyed geometry & sat→snap-index lookup tables ----- */
    for (i = 0; i <= MAXSAT; i++) {
        g_geom_by_sat[i].valid_A = 0;
        g_geom_by_sat[i].valid_B = 0;
        g_geom_by_sat[i].rho_V   = 0.0;
        g_sat_to_iA[i] = -1;
        g_sat_to_iB[i] = -1;
    }
    for (i = 0; i < s0->n; i++) {
        int s = s0->data[i].sat;
        if (s > 0 && s <= MAXSAT) g_sat_to_iA[s] = i;
    }
    for (i = 0; i < s1->n; i++) {
        int s = s1->data[i].sat;
        if (s > 0 && s <= MAXSAT) g_sat_to_iB[s] = i;
    }

    /* ----- Per-base geometry passes (independent; either may be empty) ----- */
    if (s0->n > 0) compute_geom_for_base(s0, 'A', rA, rV, /*init_v_geom=*/1);
    if (s1->n > 0) compute_geom_for_base(s1, 'B', rB, rV, /*init_v_geom=*/1);

    /* ----- Independent clock estimation per base ----- */
    n_res_A = estimate_base_clock(s0, 'A', &clk_A);
    n_res_B = estimate_base_clock(s1, 'B', &clk_B);

    A_healthy = (n_res_A >= MIN_CLOCK_SATS);
    B_healthy = (n_res_B >= MIN_CLOCK_SATS);

    /* ----- Mode state machine ----- */
    update_vrs_mode(A_healthy, B_healthy);
    force_lli = g_force_lli_next;
    g_force_lli_next = 0;

    /* ----- Out-parameter prep (so QC log shows clock+counts even if NONE) -- */
    if (clk_A_out)   *clk_A_out   = clk_A;
    if (clk_B_out)   *clk_B_out   = clk_B;
    if (nclk_A_out)  *nclk_A_out  = n_res_A;
    if (nclk_B_out)  *nclk_B_out  = n_res_B;
    if (rms_eps_out) *rms_eps_out = 0.0;
    *pnv = 0;

    /* ----- Dispatch by mode ----- */
    if (g_vrs_mode == VRS_MODE_NONE) return 0;

    if (g_vrs_mode == VRS_MODE_A_ONLY) {
        return build_virt_obs_single(s0, 'A', virt, pnv, force_lli);
    }
    if (g_vrs_mode == VRS_MODE_B_ONLY) {
        return build_virt_obs_single(s1, 'B', virt, pnv, force_lli);
    }

    /* ===== DUAL mode below: union iteration with full DD ambiguity fixing == */
    clk_V = 0.5 * (clk_A + clk_B);

    /* ----- Pass 2a: pre-scan codes seen this epoch and refresh ref sats ----
     * Only sats present on BOTH bases (with valid_A && valid_B) can act as
     * a DD reference, so we scan exactly those. */
    {
        int seen_sys_code[7][MAXCODE];   /* track which (sys, code) appeared */
        int sysmap[7] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_SBS, SYS_CMP, SYS_IRN};
        int si, ci, s_iter;
        memset(seen_sys_code, 0, sizeof(seen_sys_code));
        for (s_iter = 1; s_iter <= MAXSAT; s_iter++) {
            const sat_geom_t *g_chk = &g_geom_by_sat[s_iter];
            const obsd_t *d0;
            int iA, sb;
            if (!g_chk->valid_A || !g_chk->valid_B) continue;
            iA = g_sat_to_iA[s_iter];
            if (iA < 0) continue;
            d0 = &s0->data[iA];
            sb = sys_bit_index(satsys(s_iter, &prn));
            if (sb < 0) continue;
            for (j = 0; j < NFREQ + NEXOBS; j++) {
                if (d0->code[j] == CODE_NONE) continue;
                if (d0->L[j] == 0.0) continue;
                seen_sys_code[sb][d0->code[j] - 1] = 1;
            }
        }
        for (si = 0; si < 7; si++) {
            for (ci = 0; ci < MAXCODE; ci++) {
                int old_ref, cand_ref;
                int old_still_visible;
                int dummy_i1, dummy_j1, iA;
                if (!seen_sys_code[si][ci]) continue;
                old_ref = g_ref_sat[si][ci];
                old_still_visible = 0;
                if (old_ref > 0 && old_ref <= MAXSAT &&
                    g_geom_by_sat[old_ref].valid_A &&
                    g_geom_by_sat[old_ref].valid_B) {
                    iA = g_sat_to_iA[old_ref];
                    if (iA >= 0 &&
                        find_obs_idx_with_L(s0, iA, s1, &dummy_i1, &dummy_j1,
                                            (uint8_t)(ci + 1)) >= 0)
                        old_still_visible = 1;
                }
                if (old_still_visible) continue;   /* keep current ref */

                cand_ref = pick_ref_sat(sysmap[si], (uint8_t)(ci + 1), s0, s1);
                if (cand_ref == 0) {
                    /* No valid sat at all: invalidate this (sys, code) family */
                    int kk;
                    for (kk = 0; kk < MAXSAT; kk++) {
                        if (satsys(kk + 1, &prn) != sysmap[si]) continue;
                        g_amb[kk][ci].valid = 0;
                    }
                    g_ref_sat[si][ci] = 0;
                    continue;
                }
                if (old_ref != cand_ref) {
                    transform_amb_on_ref_change(sysmap[si], (uint8_t)(ci + 1),
                                                old_ref, cand_ref);
                    g_ref_sat[si][ci] = cand_ref;
                    g_qc_ref_switch++;
                }
            }
        }
    }

    /* ----- Pass 2b: union-mode synthesis ------------------------------------
     * Iterate over every sat that is present on AT LEAST one base. Three
     * branches per sat:
     *   (a) sat on both bases    → 2-base ML blend (existing logic)
     *   (b) sat on A only         → P-only, geometric shift to V (no L, to
     *                               keep all output L on the same DUAL ambig
     *                               convention so rover DD stays integer)
     *   (c) sat on B only         → P-only, symmetric
     * For (a) we follow the per-signal SNR gate / ML-weighted blend / DD-fix
     * pipeline. For (b)/(c) we just compute P_V via residual-form on the
     * surviving base (ε_X mathematically the same as in single-base mode).
     * --------------------------------------------------------------------- */
    {
        int sat_iter;
        for (sat_iter = 1; sat_iter <= MAXSAT && nv < MAXOBS; sat_iter++) {
            const sat_geom_t *g = &g_geom_by_sat[sat_iter];
            const obsd_t *d0 = NULL, *d1 = NULL;
            obsd_t vd;
            int any = 0, sb;
            double freq, lam;
            int branch;     /* 0 = both, 1 = A-only, 2 = B-only */

            if (!g->valid_A && !g->valid_B) continue;
            if (g->rho_V <= 0.0) continue;

            i0 = g_sat_to_iA[sat_iter];
            i1 = g_sat_to_iB[sat_iter];

            if (g->valid_A && g->valid_B && i0 >= 0 && i1 >= 0) {
                branch = 0;
                d0 = &s0->data[i0];
                d1 = &s1->data[i1];
            } else if (g->valid_A && i0 >= 0) {
                branch = 1;
                d0 = &s0->data[i0];
            } else if (g->valid_B && i1 >= 0) {
                branch = 2;
                d1 = &s1->data[i1];
            } else continue;

            sat = sat_iter;
            sys = satsys(sat, &prn); (void)prn;
            sb = sys_bit_index(sys);
            if (sb < 0) continue;

            memset(&vd, 0, sizeof(vd));
            vd.time = (branch == 2) ? d1->time : d0->time;
            vd.sat  = sat;
            vd.freq = (branch == 2) ? d1->freq : d0->freq;

            /* ===== Branch (b)/(c): single-base P-only, then continue ====== */
            if (branch != 0) {
                const obsd_t *d   = (branch == 1) ? d0 : d1;
                double rho_X = (branch == 1) ? g->rho_A : g->rho_B;
                double ion_X = (branch == 1) ? g->ion_A : g->ion_B;
                double trp_X = (branch == 1) ? g->trp_A : g->trp_B;
                for (j = 0; j < NFREQ + NEXOBS; j++) {
                    if (d->code[j] == CODE_NONE) continue;
                    if (d->P[j] <= 0.0) continue;
                    if (d->SNR[j] > 0 && d->SNR[j] < MIN_SNR_THRES_X1000) {
                        g_qc_n_lowsnr++;
                        continue;
                    }
                    vd.code[j] = d->code[j];
                    vd.P[j]    = d->P[j] + (g->rho_V - rho_X)
                                          + (g->ion_V - ion_X)
                                          + (g->trp_V - trp_X);
                    vd.SNR[j]  = d->SNR[j];
                    /* Carrier intentionally not emitted on single-base sats
                     * in DUAL mode — see the long comment in the carrier
                     * branch below. */
                    if (branch == 1) g_qc_n_p_only_A++;
                    else             g_qc_n_p_only_B++;
                    any = 1;
                }
                if (any) virt[nv++] = vd;
                continue;
            }
            /* ===== Branch (a): both bases present, full 2-base pipeline === */

            memset(&vd, 0, sizeof(vd));
            vd.time = d0->time;
            vd.sat = sat;
            vd.freq = d0->freq;

            for (j = 0; j < NFREQ + NEXOBS; j++) {
                double eps_A, eps_B, Pv;
                double w_A, w_B, w_sum;
                if (d0->code[j] == CODE_NONE) continue;
                j1 = find_same_code_idx(d1, d0->code[j]);
                if (j1 < 0) continue;

                /* ----- SNR gate (applies to BOTH P and L outputs) -----
                 * If either base reports SNR below MIN_SNR_THRES_X1000, this
                 * signal is dropped from the VRS entirely. Permissive when
                 * SNR is unknown (=0) so streams without SNR reporting still
                 * flow through. */
                if (!snr_gate_pass(d0->SNR[j], d1->SNR[j1])) {
                    g_qc_n_lowsnr++;
                    continue;
                }

                /* SNR-derived linear-power weights for ε combination. */
                w_A = snr_to_weight(d0->SNR[j]);
                w_B = snr_to_weight(d1->SNR[j1]);
                w_sum = w_A + w_B;

                /* ----- Pseudorange path: SNR-weighted residual blend -----
                 * clk_V is the *epoch-level* receiver clock = mean of clk_A,
                 * clk_B (set in pass 1) and is the same across signals; only
                 * the per-signal residuals (ε_A, ε_B) are SNR-blended. The
                 * blend asymptotically becomes max-likelihood when σ ∝
                 * 1/sqrt(C/N0). */
                if (d0->P[j] > 0.0 && d1->P[j1] > 0.0) {
                    double w_eps;
                    eps_A = d0->P[j]   - g->rho_A + CLIGHT * g->dts0
                          - g->ion_A - g->trp_A - clk_A;
                    eps_B = d1->P[j1]  - g->rho_B + CLIGHT * g->dts0
                          - g->ion_B - g->trp_B - clk_B;
                    if (fabs(eps_A - eps_B) > 30.0) {
                        /* multipath/slip on this signal — skip whole signal */
                        continue;
                    }
                    sum_dd2 += (eps_A - eps_B) * (eps_A - eps_B);
                    n_dd++;
                    /* Equal-weight fallback if both SNRs are unknown (=0). */
                    w_eps = (w_sum > 0.0) ? (w_A * eps_A + w_B * eps_B) / w_sum
                                          : 0.5 * (eps_A + eps_B);
                    Pv = g->rho_V - CLIGHT * g->dts0 + g->ion_V + g->trp_V
                       + clk_V + w_eps;
                    vd.code[j] = d0->code[j];
                    vd.P[j] = Pv;
                    /* Output SNR: physical-power sum, not arithmetic average.
                     * Capped inside snr_combine_x1000 to fit uint16_t. */
                    vd.SNR[j] = snr_combine_x1000(d0->SNR[j], d1->SNR[j1]);
                    any = 1;
                }
                else {
                    /* No P on at least one base: skip both P and L for safety. */
                    continue;
                }

                /* ----- Carrier path: DD-fix between A and B, then average ----- */
                vd.L[j] = 0.0;
                if (d0->L[j] == 0.0 || d1->L[j1] == 0.0) {
                    /* Mixed L availability; drop carrier (avoid ambig mixing) */
                    g_qc_n_drop++;
                    continue;
                }
                freq = code2freq(sys, d0->code[j], d0->freq);
                lam = (freq > 0.0) ? CLIGHT / freq : 0.0;
                if (lam <= 0.0) continue;

                {
                    int ci = (int)d0->code[j] - 1;
                    int ref_sat = g_ref_sat[sb][ci];
                    int ref_iA;
                    int ref_j0_idx, ref_j1_idx;
                    int ref_iB;
                    amb_cache_t *a = &g_amb[sat - 1][ci];
                    int slip = ((d0->LLI[j] | d1->LLI[j1]) & 1);

                    if (ref_sat <= 0) {
                        /* No ref sat: cannot DD; skip carrier */
                        g_qc_n_drop++;
                        continue;
                    }
                    ref_iA = g_sat_to_iA[ref_sat];
                    if (ref_iA < 0 ||
                        !g_geom_by_sat[ref_sat].valid_A ||
                        !g_geom_by_sat[ref_sat].valid_B) {
                        g_qc_n_drop++;
                        continue;
                    }
                    /* Locate ref sat's L on both bases for THIS code */
                    if (find_obs_idx_with_L(s0, ref_iA, s1, &ref_iB, &ref_j1_idx,
                                            d0->code[j]) < 0) {
                        g_qc_n_drop++;
                        continue;
                    }
                    /* Get the L slot index on s0 for ref */
                    for (ref_j0_idx = 0; ref_j0_idx < NFREQ + NEXOBS; ref_j0_idx++) {
                        if (s0->data[ref_iA].code[ref_j0_idx] == d0->code[j] &&
                            s0->data[ref_iA].L[ref_j0_idx] != 0.0) break;
                    }
                    if (ref_j0_idx >= NFREQ + NEXOBS) { g_qc_n_drop++; continue; }

                    {
                        const obsd_t *r0 = &s0->data[ref_iA];
                        const obsd_t *r1 = &s1->data[ref_iB];
                        const sat_geom_t *gr = &g_geom_by_sat[ref_sat];
                        double dd_geom_cyc, dd_float, dd_n_round, dd_resid;
                        double L_B_aligned, L_A_at_V, L_B_at_V, L_V_avg;
                        int slip_ref = ((r0->LLI[ref_j0_idx] | r1->LLI[ref_j1_idx]) & 1);

                        if (sat == ref_sat) {
                            /* Ref sat itself: dd_n = 0 by definition. */
                            a->valid   = 1;
                            a->dd_n    = 0.0;
                            a->ref_sat = ref_sat;
                            a->t_last  = d0->time;
                        }
                        else {
                            /* DD float ambiguity (cycles).
                             * (clk_A - clk_B)/lam cancels exactly across SD-SD. */
                            dd_geom_cyc = ((g->rho_A  - g->rho_B)
                                         - (gr->rho_A - gr->rho_B)) / lam;
                            dd_float = (d0->L[j]  - d1->L[j1]
                                      - r0->L[ref_j0_idx] + r1->L[ref_j1_idx])
                                     - dd_geom_cyc;

                            if (slip || slip_ref) a->valid = 0;

                            if (a->valid && a->ref_sat == ref_sat &&
                                fabs(dd_float - a->dd_n) < VRS_AMB_KEEP_THRES) {
                                /* Cached fix still consistent — reuse. */
                                a->t_last = d0->time;
                            }
                            else {
                                dd_n_round = floor(dd_float + 0.5);
                                dd_resid = dd_float - dd_n_round;
                                if (fabs(dd_resid) <= VRS_AMB_FIX_THRES) {
                                    a->valid   = 1;
                                    a->dd_n    = dd_n_round;
                                    a->ref_sat = ref_sat;
                                    a->t_last  = d0->time;
                                }
                                else {
                                    a->valid = 0;
                                }
                            }
                        }

                        if (!a->valid) {
                            /* Float / unfixable — drop carrier for this (sat, code).
                             * Keeps ALL output sats on the same averaged-formula
                             * convention so DD at the rover stays integer. */
                            g_qc_n_float++;
                            continue;
                        }

                        /* Build VRS carrier via clock-cancelling averaging.
                         * L_B is realigned to base-A's reference frame using DD_n;
                         * (clk_A - clk_B) does NOT need to be subtracted because
                         * its noise (~1.5 cyc on B1I) would dominate carrier noise.
                         *
                         * IMPORTANT — equal weights here are deliberate:
                         * the resulting L_V has effective ambig
                         *   N_A^k + 0.5 * (N_B^ref - N_A^ref)
                         * which is the SAME half-integer offset for all sats
                         * (and all epochs until ref switch), so the rover DD
                         * stays integer. If we used SNR-weights w_A, w_B the
                         * offset would become w_B/(w_A+w_B)*(N_B^ref - N_A^ref)
                         * — a per-signal value — and DD would no longer be
                         * integer. Carrier noise on signals that pass the
                         * 25 dBHz gate is ≤ a few mm anyway, so the noise
                         * loss from equal-weighting is negligible compared
                         * to losing integer ambiguities at the rover. */
                        L_B_aligned = d1->L[j1] + a->dd_n;
                        L_A_at_V    = d0->L[j]  + (g->rho_V - g->rho_A) / lam;
                        L_B_at_V    = L_B_aligned + (g->rho_V - g->rho_B) / lam;
                        L_V_avg     = 0.5 * (L_A_at_V + L_B_at_V);
                        vd.L[j] = L_V_avg;
                        /* Inherit cycle-slip flags AND inject LLI=1 on the
                         * first epoch after a SINGLE↔DUAL mode transition. */
                        vd.LLI[j] = (uint8_t)((d0->LLI[j] | d1->LLI[j1]
                                              | (force_lli ? 1 : 0)) & 0xFF);
                        g_qc_n_fix++;
                    }
                }
            }
            if (any) virt[nv++] = vd;
        }
    }

    *pnv = nv;
    /* clk_X_out / nclk_X_out / *pnv already populated above (kept for callers
     * that read these even when we early-returned). Just refresh rms_eps. */
    if (rms_eps_out)  *rms_eps_out  = (n_dd > 0) ? sqrt(sum_dd2 / n_dd) * 0.5 : 0.0;
    return nv > 0 ? 1 : 0;
}

/* Two bases: ephemeris path (clock-corrected average) preferred, Apollonius fallback. */
static void try_synth_virtual_obs(void)
{
    const epoch_snap_t *s0 = &g_epoch_snap[0], *s1 = &g_epoch_snap[1];
    double b, mid[3];
    int nv = 0, k, msm_count, send_station = 0;
    msm_chunk_t msm_chunks[MSM_MAX_CHUNKS];
    uint8_t tx_agg[16384];
    int tx_agg_len = 0;
    int dec_obs = 0, dec_sta = 0, dec_err = 0;
    obsd_t virt[MAXOBS];
    int used_eph = 0;
    double clk_A = 0.0, clk_B = 0.0, rms_eps = 0.0;
    int nclk_A = 0, nclk_B = 0;

    if (g_base_n != 2) return;
    if (!g_enc_inited) return;
    /* Allow single-base operation: at least ONE base must have data. The
     * mode state machine inside build_virt_obs_with_eph decides whether to
     * stay in DUAL or fall to SINGLE_X based on hysteresis-protected health
     * of each base. The "both ARPs known" requirement remains because we
     * still want to anchor the VRS at the geometric midpoint regardless of
     * which base is currently producing observations. */
    if (s0->n <= 0 && s1->n <= 0) return;
    if (s0->n > 0 && s1->n > 0 &&
        fabs(timediff(s0->time, s1->time)) > 0.5) return;
    /* ARP gate. If we never saw one base's 1005 (typical: it died before its
     * first heartbeat), we cannot place V at the midpoint. Emit a one-shot
     * diagnostic so the user understands why VRS is silent — otherwise it
     * looks like the program just hung. */
    if (!g_arp_valid[0] || !g_arp_valid[1]) {
        static int warned = 0;
        if (!warned) {
            print_enter();
            printf("VRS [waiting]: base %s ARP not yet received "
                   "(have_A=%d have_B=%d). Cannot compute midpoint V; will "
                   "stay silent until both 1005/1006 messages arrive.\n",
                   (!g_arp_valid[0] && !g_arp_valid[1]) ? "A and B" :
                   (!g_arp_valid[0] ? "A" : "B"),
                   g_arp_valid[0], g_arp_valid[1]);
            fflush(stdout);
            print_leave();
            warned = 1;
        }
        return;
    }

    b = norm3diff(g_arp_ecef[1], g_arp_ecef[0]);
    if (b <= 0.0) return;

    print_enter();

    /* Choose the active epoch timestamp. In SINGLE_X mode one snap may be
     * empty; pick the surviving side. */
    {
        gtime_t t_epoch = (s0->n > 0) ? s0->time : s1->time;

        if (g_have_synth_epoch &&
            fabs(timediff(t_epoch, g_last_synth_epoch)) < 0.95) {
            print_leave();
            return;
        }
        g_last_synth_epoch = t_epoch;
        g_have_synth_epoch = 1;
    }

    mid[0] = 0.5 * (g_arp_ecef[0][0] + g_arp_ecef[1][0]);
    mid[1] = 0.5 * (g_arp_ecef[0][1] + g_arp_ecef[1][1]);
    mid[2] = 0.5 * (g_arp_ecef[0][2] + g_arp_ecef[1][2]);

    /* Reset per-epoch carrier-side QC counters (build_virt_obs_with_eph fills them). */
    g_qc_n_fix = g_qc_n_float = g_qc_n_drop = g_qc_ref_switch = 0;
    g_qc_n_lowsnr = 0;
    g_qc_n_p_only_A = g_qc_n_p_only_B = 0;

    /* Prefer ephemeris-aware path (clean clock removal + true geometry). */
    if (g_nav_inited && g_nav_n_total() >= 4) {
        if (build_virt_obs_with_eph(s0, s1, g_arp_ecef[0], g_arp_ecef[1], mid,
                                    virt, &nv, &clk_A, &clk_B,
                                    &nclk_A, &nclk_B, &rms_eps)) {
            used_eph = 1;
        }
    }
    if (!used_eph) {
        build_virt_obs_apollonius(s0, s1, b, virt, &nv);
    }

    prepare_virt_for_msm7(virt, &nv);

    if (nv <= 0) {
        printf("VRS [%s]: no valid sats this epoch (eph_n=%d)\n",
               used_eph ? "eph" : "apollonius", g_nav_n_total());
        fflush(stdout);
        print_leave();
        return;
    }

    {
    gtime_t t_epoch = (s0->n > 0) ? s0->time : s1->time;

    memset(g_enc_rtcm.cp, 0, sizeof(g_enc_rtcm.cp));
    g_enc_rtcm.time = t_epoch;
    g_enc_rtcm.staid = OUT_VIRT_STAID;
    g_enc_rtcm.seqno = (g_enc_rtcm.seqno + 1) & 7;
    g_enc_rtcm.sta.pos[0] = mid[0];
    g_enc_rtcm.sta.pos[1] = mid[1];
    g_enc_rtcm.sta.pos[2] = mid[2];

    g_enc_rtcm.obs.n = nv;
    memcpy(g_enc_rtcm.obs.data, virt, (size_t)nv * sizeof(obsd_t));

    msm_count = build_msm_chunks(&g_enc_rtcm.obs, msm_chunks, MSM_MAX_CHUNKS);

    if (!g_have_station_tx_epoch) {
        send_station = 1;
    }
    else {
        double dt = timediff(t_epoch, g_last_station_tx_epoch);
        if (dt >= VIRT_STATION_INTERVAL - 0.5 || dt < -0.5)
            send_station = 1;
    }

    if (send_station && gen_rtcm3(&g_enc_rtcm, 1005, 0, 0)) {
        if (tx_agg_len + g_enc_rtcm.nbyte <= (int)sizeof(tx_agg)) {
            memcpy(tx_agg + tx_agg_len, g_enc_rtcm.buff, (size_t)g_enc_rtcm.nbyte);
            tx_agg_len += g_enc_rtcm.nbyte;
        }
        tx_send_buf(g_enc_rtcm.buff, g_enc_rtcm.nbyte);
        g_last_station_tx_epoch = t_epoch;
        g_have_station_tx_epoch = 1;
    }
    }

    for (k = 0; k < msm_count; k++) {
        g_enc_rtcm.obs.n = msm_chunks[k].n;
        memcpy(g_enc_rtcm.obs.data, msm_chunks[k].data,
               (size_t)msm_chunks[k].n * sizeof(obsd_t));

        if (!gen_rtcm3(&g_enc_rtcm, msm_chunks[k].type, 0,
                       k != msm_count - 1 ? 1 : 0)) {
            continue;
        }
        if (tx_agg_len + g_enc_rtcm.nbyte <= (int)sizeof(tx_agg)) {
            memcpy(tx_agg + tx_agg_len, g_enc_rtcm.buff, (size_t)g_enc_rtcm.nbyte);
            tx_agg_len += g_enc_rtcm.nbyte;
        }
        tx_send_buf(g_enc_rtcm.buff, g_enc_rtcm.nbyte);
    }

    /* Fresh frame sync for verify pass (avoids stray 0xD3 in decoder state). */
    g_dec_rtcm.nbyte = 0;
    g_dec_rtcm.staid = 0;

    for (k = 0; k < tx_agg_len; k++) {
        int r = input_rtcm3(&g_dec_rtcm, tx_agg[k]);
        if (r < 0)
            dec_err++;
        else if (r == 1)
            dec_obs++;
        else if (r == 5)
            dec_sta++;
    }

    if (used_eph) {
        const char *mode_tag =
            (g_vrs_mode == VRS_MODE_DUAL)   ? "DUAL"   :
            (g_vrs_mode == VRS_MODE_A_ONLY) ? "A-only" :
            (g_vrs_mode == VRS_MODE_B_ONLY) ? "B-only" : "NONE";
        printf("VRS [eph %s] :%u TX=%dB nv=%d | clk_A=%+.3fm(%dsv) "
               "clk_B=%+.3fm(%dsv) dclk=%+.3fm | rms((epsA-epsB)/2)=%.3fm | "
               "L: fix=%d float=%d drop=%d refSwap=%d lowSNR=%d | "
               "Aonly=%d Bonly=%d | obs=%d sta=%d err=%d\n",
               mode_tag, (unsigned)RTCM_OUT_PORT, tx_agg_len, nv,
               clk_A, nclk_A, clk_B, nclk_B, clk_A - clk_B,
               rms_eps,
               g_qc_n_fix, g_qc_n_float, g_qc_n_drop, g_qc_ref_switch,
               g_qc_n_lowsnr,
               g_qc_n_p_only_A, g_qc_n_p_only_B,
               dec_obs, dec_sta, dec_err);
    }
    else {
        printf("VRS [apollonius fallback] :%u TX=%dB nv=%d eph_n=%d | obs=%d sta=%d err=%d\n",
               (unsigned)RTCM_OUT_PORT, tx_agg_len, nv, g_nav_n_total(),
               dec_obs, dec_sta, dec_err);
    }
    fflush(stdout);
    print_leave();
}

static int winsock_startup(void)
{
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        fprintf(stderr, "WSAStartup failed\n");
        return -1;
    }
#endif
    return 0;
}

static sock_t tcp_connect_host(const char *host, unsigned short port, int *err)
{
    char serv[16];
    struct addrinfo hints, *res = NULL, *p;
    sock_t s = SOCK_INVALID;
    int e;

    snprintf(serv, sizeof(serv), "%u", (unsigned)port);
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    if (err) *err = 0;

    if ((e = getaddrinfo(host, serv, &hints, &res)) != 0) {
        if (err) *err = e;
        return SOCK_INVALID;
    }

    for (p = res; p != NULL; p = p->ai_next) {
        s = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (s == SOCK_INVALID)
            continue;
        if (connect(s, p->ai_addr, (int)p->ai_addrlen) == 0)
            break;
        if (err) *err = sock_errno;
        sock_close(s);
        s = SOCK_INVALID;
    }
    freeaddrinfo(res);
    return s;
}

/* w_mid non-NULL only in multi-client mode (for 1005/1006 midpoint and eph merge). */
static int run_client(sock_t cfd, rtcm_t *rtcm, const char *label, stream_worker_t *w_mid)
{
    uint8_t buf[4096];
    int n, i, ret;
    const char *tag = stream_tag(label);
    int eph_only = (w_mid != NULL) ? w_mid->s.eph_only : 0;

    while ((n = (int)recv(cfd, (char *)buf, (int)sizeof(buf), 0)) > 0) {
        for (i = 0; i < n; i++) {
            ret = input_rtcm3(rtcm, buf[i]);
            if (ret == 1) {
                /* Observation epoch */
                if (eph_only) continue;
                print_obs_epoch(rtcm, label);
                if (w_mid != NULL && g_base_n == 2 && w_mid->base_index >= 0 &&
                    w_mid->base_index < MAX_CLIENT_STREAMS) {
                    store_epoch_snapshot(w_mid->base_index, &rtcm->obs);
                    try_synth_virtual_obs();
                }
            }
            else if (ret == 2) {
                /* Ephemeris updated -> merge into global pool (any role can feed) */
                merge_eph_from_rtcm(rtcm);
            }
            else if (ret == 5) {
                /* Station / antenna info */
                if (eph_only) continue;  /* eph-only streams must not bias midpoint */
                print_station_msg(rtcm, label);
                if (w_mid != NULL)
                    update_arp_midpoint_from_1005(rtcm, w_mid);
            }
            else if (ret < 0) {
                print_enter();
                fprintf(stderr, "[%s] RTCM decode error (ret=%d)\n", tag, ret);
                fflush(stderr);
                print_leave();
            }
        }
    }
    return n < 0 ? -1 : 0;
}

static void print_identity_hint(void)
{
    fputs(
        "Hint: RTCM Station ID is 12-bit - duplicate staid (e.g. 1660) is common.\n"
        "  Use one -c per station, optional labels, and/or 1005 ECEF to distinguish.\n\n",
        stdout);
}

static void worker_prepare(stream_worker_t *w, const cli_stream_t *spec)
{
    memset(w, 0, sizeof(*w));
    w->s = *spec;
    if (!spec->label || !spec->label[0]) {
        snprintf(w->label_storage, sizeof(w->label_storage), "%s:%u",
                 spec->host, (unsigned)spec->port);
    }
}

/* One TCP client: reconnect forever (runs in worker thread or main). */
static void stream_connect_forever(stream_worker_t *w)
{
    const char *host = w->s.host;
    unsigned short port = w->s.port;
    const char *lab = worker_label(w);
    int err;

    for (;;) {
        sock_t s = tcp_connect_host(host, port, &err);
        if (s == SOCK_INVALID) {
            print_enter();
            fprintf(stderr, "[%s] connect %s:%u failed (%d), retry in 2s...\n",
                    lab, host, (unsigned)port, err);
            fflush(stderr);
            print_leave();
#ifdef _WIN32
            Sleep(2000);
#else
            sleep(2);
#endif
            continue;
        }
        print_enter();
        printf("[%s] connected to %s:%u\n", lab, host, (unsigned)port);
        fflush(stdout);
        print_leave();
        run_client(s, &w->rtcm, lab, w);
        print_enter();
        if (g_base_n > 0 && w->base_index >= 0 && w->base_index < g_base_n) {
            /* DO NOT clear g_arp_valid[] on disconnect: the antenna doesn't
             * physically move when the network blip ends, and we need both
             * ARPs (even from a dead stream) to keep computing the midpoint
             * V in SINGLE_X mode. A new 1005 from a reconnected stream will
             * overwrite the cached value anyway. */
            g_have_mid_print = 0;
            g_have_station_tx_epoch = 0;
            /* Wipe the snapshot so the next VRS attempt sees s_X.n = 0 and
             * the mode state machine can switch to SINGLE_other after the
             * MODE_FAIL_TO_SINGLE hysteresis window. */
            g_epoch_snap[w->base_index].n = 0;
        }
        printf("[%s] disconnected from %s:%u, reconnecting...\n",
               lab, host, (unsigned)port);
        fflush(stdout);
        print_leave();
        sock_close(s);
    }
}

#ifdef _WIN32
static DWORD WINAPI stream_thread_proc(LPVOID param)
{
    stream_connect_forever((stream_worker_t *)param);
    return 0;
}
#else
static void *stream_thread_proc(void *param)
{
    stream_connect_forever((stream_worker_t *)param);
    return NULL;
}
#endif

/* Parse: argv[1] must be -c or -e, then repeating ((-c|-e) host port [label])+
 *   -c <host> <port> [label]   base station (obs + ARP, plus eph if present)
 *   -e <host> <port> [label]   broadcast-ephemeris-only stream (no obs/ARP) */
static int parse_client_streams(int argc, char **argv, cli_stream_t *out, int max_n)
{
    int i = 1, n = 0;

    if (i >= argc || (strcmp(argv[i], "-c") != 0 && strcmp(argv[i], "-e") != 0))
        return -1;

    while (i < argc) {
        int eph_only;
        if (strcmp(argv[i], "-c") == 0)      eph_only = 0;
        else if (strcmp(argv[i], "-e") == 0) eph_only = 1;
        else                                  return -1;
        i++;
        if (i >= argc)
            return -1;
        out[n].host = argv[i++];
        if (i >= argc)
            return -1;
        {
            int p = atoi(argv[i++]);
            if (p <= 0 || p >= 65536)
                return -1;
            out[n].port = (unsigned short)p;
        }
        out[n].label = NULL;
        out[n].eph_only = eph_only;
        if (i < argc && strcmp(argv[i], "-c") != 0 && strcmp(argv[i], "-e") != 0)
            out[n].label = argv[i++];
        n++;
        if (n > max_n)
            return -1;
    }
    return n;
}

static int run_multi_client(int argc, char **argv)
{
    cli_stream_t specs[MAX_CLIENT_STREAMS];
    stream_worker_t *workers = NULL;
    int n, k, j, nbase = 0, neph = 0;

    n = parse_client_streams(argc, argv, specs, MAX_CLIENT_STREAMS);
    if (n <= 0) {
        fprintf(stderr, "invalid -c/-e arguments\n");
        return -1;
    }
    for (k = 0; k < n; k++) {
        if (specs[k].eph_only) neph++;
        else                   nbase++;
    }

    /* stream_worker_t embeds rtcm_t (very large). N copies on stack overflows
     * the default ~1MB thread stack — use heap (fixes silent crash / 0xC00000FD). */
    workers = (stream_worker_t *)calloc((size_t)n, sizeof(stream_worker_t));
    if (!workers) {
        fprintf(stderr, "out of memory for %d stream(s)\n", n);
        return -1;
    }

    print_init();
    tx_lock_init();
    if (winsock_startup() != 0) {
        free(workers);
        return -1;
    }

    print_identity_hint();
    printf("Client mode: %d TCP stream(s) in this process (base=%d, eph-only=%d).\n",
           n, nbase, neph);
    if (nbase >= 2) {
        printf("RTCM 1005 ARP from base streams -> midpoint ECEF; input MSM epochs are not printed.\n");
        printf("Synthetic midpoint RTCM (1005 + MSM7) is sent on TCP port %u (one client at a time).\n",
               (unsigned)RTCM_OUT_PORT);
    }
    if (neph > 0)
        printf("Eph-only stream(s) feed the global broadcast ephemeris pool (no obs/ARP used).\n");
    fflush(stdout);

    tracelevel(0);

    g_multi_n = n;
    g_base_n  = nbase;
    g_suppress_input_obs = (nbase >= 2) ? 1 : 0;
    memset(g_arp_valid, 0, sizeof(g_arp_valid));
    g_have_mid_print = 0;
    memset(g_epoch_snap, 0, sizeof(g_epoch_snap));
    g_have_synth_epoch = 0;
    g_have_station_tx_epoch = 0;

    if (!init_global_nav()) {
        fprintf(stderr, "init_global_nav failed (out of memory)\n");
        free(workers);
        return -1;
    }

    if (!g_enc_inited) {
        if (!init_rtcm(&g_enc_rtcm) || !init_rtcm(&g_dec_rtcm)) {
            fprintf(stderr, "init_rtcm encoder/decoder failed\n");
            free(workers);
            return -1;
        }
        g_enc_rtcm.outtype = 0;
        g_dec_rtcm.outtype = 1;
        g_enc_inited = 1;
    }
    init_amb_cache();

#ifdef _WIN32
    if (!CreateThread(NULL, 0, rtcm_out_server_thread, NULL, 0, NULL))
        fprintf(stderr, "warning: RTCM TCP out thread not started (port %u)\n",
                (unsigned)RTCM_OUT_PORT);
#else
    {
        pthread_t txth;
        if (pthread_create(&txth, NULL, rtcm_out_server_thread, NULL) != 0)
            fprintf(stderr, "warning: RTCM TCP out thread not started (port %u)\n",
                    (unsigned)RTCM_OUT_PORT);
        else
            pthread_detach(txth);
    }
#endif

    {
        int bi = 0;
        for (k = 0; k < n; k++) {
            worker_prepare(&workers[k], &specs[k]);
            workers[k].stream_index = k;
            workers[k].base_index = specs[k].eph_only ? -1 : bi++;
            strncpy(g_stream_tag[k], worker_label(&workers[k]), sizeof(g_stream_tag[k]) - 1);
            g_stream_tag[k][sizeof(g_stream_tag[k]) - 1] = '\0';
            if (!init_rtcm(&workers[k].rtcm)) {
                fprintf(stderr, "init_rtcm failed for stream %d\n", k);
                for (j = 0; j < k; j++)
                    free_rtcm(&workers[j].rtcm);
                free(workers);
                return -1;
            }
            workers[k].rtcm.outtype = 1;
        }
    }

#ifdef _WIN32
    for (k = 0; k < n; k++) {
        HANDLE th = CreateThread(NULL, STREAM_THREAD_STACK_SIZE, stream_thread_proc,
                                 &workers[k], 0, NULL);
        if (!th) {
            fprintf(stderr, "CreateThread failed for stream %d\n", k);
            /* threads 0..k-1 may already be running; only safe to release if none started */
            if (k == 0) {
                for (j = 0; j < n; j++)
                    free_rtcm(&workers[j].rtcm);
                free(workers);
            }
            return -1;
        }
        CloseHandle(th);
    }
    for (;;)
        Sleep(86400000);
#else
    {
        pthread_attr_t attr;
        pthread_t th[MAX_CLIENT_STREAMS];
        int attr_ok = pthread_attr_init(&attr) == 0;

        if (attr_ok)
            pthread_attr_setstacksize(&attr, STREAM_THREAD_STACK_SIZE);

        for (k = 0; k < n; k++) {
            if (pthread_create(&th[k], attr_ok ? &attr : NULL,
                               stream_thread_proc, &workers[k]) != 0) {
                fprintf(stderr, "pthread_create failed for stream %d\n", k);
                if (attr_ok)
                    pthread_attr_destroy(&attr);
                return -1;
            }
        }
        if (attr_ok)
            pthread_attr_destroy(&attr);
        for (k = 0; k < n; k++)
            pthread_join(th[k], NULL);
    }
#endif
    return 0;
}

static int listen_loop(unsigned short port, const char *label)
{
    print_init();

    if (winsock_startup() != 0)
        return -1;

    sock_t lfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in addr;
    rtcm_t *rtcm = NULL;

    if (lfd == SOCK_INVALID) {
        fprintf(stderr, "socket: failed (%d)\n", sock_errno);
        return -1;
    }
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(lfd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "bind port %u: failed (%d)\n", (unsigned)port, sock_errno);
        sock_close(lfd);
        return -1;
    }
    if (listen(lfd, 1) != 0) {
        fprintf(stderr, "listen: failed (%d)\n", sock_errno);
        sock_close(lfd);
        return -1;
    }

    print_identity_hint();
    printf("Listening on TCP port %u - connect STRSVR (or caster) here.\n",
           (unsigned)port);
    if (label && label[0]) printf("Stream label: [%s]\n", label);
    fflush(stdout);

    rtcm = (rtcm_t *)calloc(1, sizeof(*rtcm));
    if (!rtcm) {
        fprintf(stderr, "init_rtcm: out of memory\n");
        sock_close(lfd);
        return -1;
    }

    if (!init_rtcm(rtcm)) {
        fprintf(stderr, "init_rtcm: out of memory\n");
        free(rtcm);
        sock_close(lfd);
        return -1;
    }
    rtcm->outtype = 1;
    tracelevel(0);

    for (;;) {
        sock_t cfd = accept(lfd, NULL, NULL);
        if (cfd == SOCK_INVALID) {
            fprintf(stderr, "accept: failed (%d)\n", sock_errno);
            continue;
        }
        print_enter();
        printf("client connected\n");
        fflush(stdout);
        print_leave();
        run_client(cfd, rtcm, label, NULL);
        print_enter();
        printf("client disconnected\n");
        fflush(stdout);
        print_leave();
        sock_close(cfd);
    }
}

static void usage(void)
{
    fprintf(stderr,
            "usage:\n"
            "  ntrip_rtcm_obs [listen_port] [label]     TCP server (default port %d)\n"
            "  ntrip_rtcm_obs (-c|-e) <host> <port> [label] [(-c|-e) <host> <port> [label] ...]\n"
            "      -c   base station stream  (obs + ARP, eph also accepted)\n"
            "      -e   broadcast-ephemeris-only stream (no obs/ARP used)\n"
            "      Multiple streams run in one process (one thread each).\n"
            "      Need exactly 2 base streams to enable virtual midpoint synthesis.\n",
            DEFAULT_PORT);
}

int main(int argc, char **argv)
{
    unsigned short port = DEFAULT_PORT;
    const char *label = NULL;

    if (argc >= 2 && (strcmp(argv[1], "-c") == 0 || strcmp(argv[1], "-e") == 0))
        return run_multi_client(argc, argv) ? 1 : 0;

    if (argc >= 2) {
        int p = atoi(argv[1]);
        if (p > 0 && p < 65536) port = (unsigned short)p;
        else {
            fprintf(stderr, "bad port: %s\n", argv[1]);
            usage();
            return 1;
        }
    }
    if (argc >= 3) label = argv[2];

    return listen_loop(port, label) ? 1 : 0;
}
