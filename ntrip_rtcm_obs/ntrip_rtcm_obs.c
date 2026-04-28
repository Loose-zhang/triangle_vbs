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
 *   where P_1,P_2 are the two stations' pseudoranges (used as slant-range proxies;
 *   clock/iono residuals should be small vs. geometry for short baselines).
 *   Carrier (cycles): mean of both stations only when both have valid phase
 *   (L≠0); otherwise L=0 even if pseudorange is synthesized.
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
} cli_stream_t;

typedef struct {
    cli_stream_t s;
    char label_storage[64];
    int stream_index; /* 0..n-1 in multi-client mode; for ARP midpoint */
    rtcm_t rtcm;
} stream_worker_t;

/* B2bLib declares PPP_Glo in rtklib.h; full apps (e.g. rtkrcv) define it elsewhere. */
PPPGlobal_t PPP_Glo;

static rtklib_lock_t g_print_lock;

/* Multi-stream: latest ARP from RTCM 1005/1006 per stream; midpoint when all valid */
static int g_multi_n;
static double g_arp_ecef[MAX_CLIENT_STREAMS][3];
static int g_arp_valid[MAX_CLIENT_STREAMS];
static char g_stream_tag[MAX_CLIENT_STREAMS][64];
static double g_last_mid_ecef[3];
static int g_have_mid_print;

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

/* After each 1005/1006: store ARP; when every stream has ARP, print ECEF midpoint (mean). */
static void update_arp_midpoint_from_1005(const rtcm_t *rtcm, const stream_worker_t *w)
{
    int t = last_rtcm3_type(rtcm);
    int i, k, all;
    double mid[3], llh[3];

    if (g_multi_n < 2)
        return;
    if (t != 1005 && t != 1006)
        return;

    k = w->stream_index;
    if (k < 0 || k >= g_multi_n)
        return;

    print_enter();
    g_arp_ecef[k][0] = rtcm->sta.pos[0];
    g_arp_ecef[k][1] = rtcm->sta.pos[1];
    g_arp_ecef[k][2] = rtcm->sta.pos[2];
    g_arp_valid[k] = 1;

    all = 1;
    for (i = 0; i < g_multi_n; i++) {
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
    for (i = 0; i < g_multi_n; i++) {
        mid[0] += g_arp_ecef[i][0];
        mid[1] += g_arp_ecef[i][1];
        mid[2] += g_arp_ecef[i][2];
    }
    mid[0] /= (double)g_multi_n;
    mid[1] /= (double)g_multi_n;
    mid[2] /= (double)g_multi_n;

    if (g_have_mid_print && norm3diff(mid, g_last_mid_ecef) < 0.02) {
        print_leave();
        return;
    }
    memcpy(g_last_mid_ecef, mid, sizeof(mid));
    g_have_mid_print = 1;
    g_have_station_tx_epoch = 0;

    ecef2pos(mid, llh);
    printf("\n=== Midpoint (mean ECEF of %d RTCM 1005 ARPs) ===\n", g_multi_n);
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

static void remap_obs_codes_for_msm7(obsd_t *data, int n)
{
    int i, j, prn, sys;
    uint8_t c;
    double fq;

    for (i = 0; i < n; i++) {
        sys = satsys(data[i].sat, &prn);
        for (j = 0; j < NFREQ + NEXOBS; j++) {
            double L_was;

            if (data[i].code[j] == CODE_NONE) continue;
            if (data[i].P[j] == 0.0 && data[i].L[j] == 0.0) continue;
            L_was = data[i].L[j];
            c = msm_remap_obs_code(sys, data[i].code[j]);
            if (c == CODE_NONE || !obs_code_in_msm_table(sys, c)) {
                data[i].code[j] = CODE_NONE;
                data[i].P[j] = data[i].L[j] = 0.0;
                continue;
            }
            if (c != data[i].code[j]) {
                data[i].code[j] = c;
                fq = code2freq(sys, c, data[i].freq);
                if (fq > 0.0 && data[i].P[j] != 0.0 && L_was != 0.0)
                    data[i].L[j] = data[i].P[j] / (CLIGHT / fq);
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

/* Two bases: Apollonius on (P1,P2,b); L = mean(L1,L2) only when both phases valid else 0.
 * Encode RTCM 1005 + MSM7, TCP :52000. */
static void try_synth_virtual_obs(void)
{
    const epoch_snap_t *s0 = &g_epoch_snap[0], *s1 = &g_epoch_snap[1];
    double P0, P1, Pm_sq, Pm, b;
    double mid[3];
    int i0, i1, j, j1, sat, prn, sys;
    int nv, k, msm_count, send_station = 0;
    msm_chunk_t msm_chunks[MSM_MAX_CHUNKS];
    uint8_t tx_agg[16384];
    int tx_agg_len = 0;
    int dec_obs = 0, dec_sta = 0, dec_err = 0;
    obsd_t virt[MAXOBS];
    const obsd_t *d0, *d1;

    if (g_multi_n != 2) return;
    if (!g_enc_inited) return;
    if (s0->n <= 0 || s1->n <= 0) return;
    if (fabs(timediff(s0->time, s1->time)) > 0.5) return;
    if (!g_arp_valid[0] || !g_arp_valid[1]) return;

    b = norm3diff(g_arp_ecef[1], g_arp_ecef[0]);
    if (b <= 0.0) return;

    print_enter();

    if (g_have_synth_epoch && fabs(timediff(s0->time, g_last_synth_epoch)) < 0.95) {
        print_leave();
        return;
    }
    g_last_synth_epoch = s0->time;
    g_have_synth_epoch = 1;

    mid[0] = 0.5 * (g_arp_ecef[0][0] + g_arp_ecef[1][0]);
    mid[1] = 0.5 * (g_arp_ecef[0][1] + g_arp_ecef[1][1]);
    mid[2] = 0.5 * (g_arp_ecef[0][2] + g_arp_ecef[1][2]);

    nv = 0;
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

        sys = satsys(sat, &prn);
        (void)prn;

        for (j = 0; j < NFREQ + NEXOBS; j++) {
            if (d0->code[j] == CODE_NONE) continue;
            if (d0->P[j] == 0.0 && d0->L[j] == 0.0) continue;
            j1 = find_same_code_idx(d1, d0->code[j]);
            if (j1 < 0) continue;

            P0 = d0->P[j];
            P1 = d1->P[j];
            if (P0 <= 0.0 || P1 <= 0.0) continue;

            Pm_sq = 0.5 * (P0 * P0 + P1 * P1) - 0.25 * b * b;
            if (Pm_sq < 0.0) continue;
            Pm = sqrt(Pm_sq);

            vd.code[j] = d0->code[j];
            vd.P[j] = Pm;
            vd.L[j] = (d0->L[j] != 0.0 && d1->L[j1] != 0.0)
                ? 0.5 * (d0->L[j] + d1->L[j1]) : 0.0;
            if (d0->SNR[j] && d1->SNR[j1])
                vd.SNR[j] = (d0->SNR[j] + d1->SNR[j1]) / 2;
            else
                vd.SNR[j] = d0->SNR[j] ? d0->SNR[j] : d1->SNR[j1];
            any = 1;
        }
        if (any)
            virt[nv++] = vd;
    }

    prepare_virt_for_msm7(virt, &nv);

    if (nv <= 0) {
        print_leave();
        return;
    }

    memset(g_enc_rtcm.cp, 0, sizeof(g_enc_rtcm.cp));
    g_enc_rtcm.time = s0->time;
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
        double dt = timediff(s0->time, g_last_station_tx_epoch);
        if (dt >= VIRT_STATION_INTERVAL - 0.5 || dt < -0.5)
            send_station = 1;
    }

    if (send_station && gen_rtcm3(&g_enc_rtcm, 1005, 0, 0)) {
        if (tx_agg_len + g_enc_rtcm.nbyte <= (int)sizeof(tx_agg)) {
            memcpy(tx_agg + tx_agg_len, g_enc_rtcm.buff, (size_t)g_enc_rtcm.nbyte);
            tx_agg_len += g_enc_rtcm.nbyte;
        }
        tx_send_buf(g_enc_rtcm.buff, g_enc_rtcm.nbyte);
        g_last_station_tx_epoch = s0->time;
        g_have_station_tx_epoch = 1;
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

    printf("Virtual RTCM -> port %u: TX %d B; parse check obs_epoch=%d station=%d err=%d\n",
           (unsigned)RTCM_OUT_PORT, tx_agg_len, dec_obs, dec_sta, dec_err);
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

/* w_mid non-NULL only in multi-client mode (for 1005/1006 midpoint). */
static int run_client(sock_t cfd, rtcm_t *rtcm, const char *label, stream_worker_t *w_mid)
{
    uint8_t buf[4096];
    int n, i, ret;
    const char *tag = stream_tag(label);

    while ((n = (int)recv(cfd, (char *)buf, (int)sizeof(buf), 0)) > 0) {
        for (i = 0; i < n; i++) {
            ret = input_rtcm3(rtcm, buf[i]);
            if (ret == 1) {
                print_obs_epoch(rtcm, label);
                if (w_mid != NULL && g_multi_n == 2 && w_mid->stream_index >= 0 &&
                    w_mid->stream_index < MAX_CLIENT_STREAMS) {
                    store_epoch_snapshot(w_mid->stream_index, &rtcm->obs);
                    try_synth_virtual_obs();
                }
            }
            else if (ret == 5) {
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
        if (g_multi_n > 0 && w->stream_index >= 0 && w->stream_index < g_multi_n) {
            g_arp_valid[w->stream_index] = 0;
            g_have_mid_print = 0;
            g_have_station_tx_epoch = 0;
            g_epoch_snap[w->stream_index].n = 0;
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

/* Parse: argv[1] must be -c, then repeating (-c host port [label])+ */
static int parse_client_streams(int argc, char **argv, cli_stream_t *out, int max_n)
{
    int i = 1, n = 0;

    if (i >= argc || strcmp(argv[i], "-c") != 0)
        return -1;

    while (i < argc) {
        if (strcmp(argv[i], "-c") != 0)
            return -1;
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
        if (i < argc && strcmp(argv[i], "-c") != 0)
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
    int n, k, j;

    n = parse_client_streams(argc, argv, specs, MAX_CLIENT_STREAMS);
    if (n <= 0) {
        fprintf(stderr, "invalid -c arguments\n");
        return -1;
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
    printf("Client mode: %d TCP stream(s) in this process.\n", n);
    if (n >= 2) {
        printf("RTCM 1005 ARP from each stream -> midpoint ECEF; input MSM epochs are not printed.\n");
        printf("Synthetic midpoint RTCM (1005 + MSM7) is sent on TCP port %u (one client at a time).\n",
               (unsigned)RTCM_OUT_PORT);
    }
    fflush(stdout);

    tracelevel(0);

    g_multi_n = n;
    g_suppress_input_obs = (n >= 2) ? 1 : 0;
    memset(g_arp_valid, 0, sizeof(g_arp_valid));
    g_have_mid_print = 0;
    memset(g_epoch_snap, 0, sizeof(g_epoch_snap));
    g_have_synth_epoch = 0;
    g_have_station_tx_epoch = 0;

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

    for (k = 0; k < n; k++) {
        worker_prepare(&workers[k], &specs[k]);
        workers[k].stream_index = k;
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
            "  ntrip_rtcm_obs -c <host> <port> [label] [-c <host> <port> [label] ...]\n"
            "      TCP client(s); multiple -c run in one process (one thread each).\n",
            DEFAULT_PORT);
}

int main(int argc, char **argv)
{
    unsigned short port = DEFAULT_PORT;
    const char *label = NULL;

    if (argc >= 2 && strcmp(argv[1], "-c") == 0)
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
