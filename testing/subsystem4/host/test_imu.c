/**
 * @file    test_imu.c
 * @brief   Host tests for Buddy 4 (IMU Terrain): statechart 6.4, hump
 *          height, shock, motion classes, data health and the telemetry
 *          frame. Compiles app_program/subsystem4/imu_proc.c and b4_pack.c
 *          unchanged; no kernel, no Pico.
 *
 * The car is simulated from geometry: the front contact follows a ground
 * profile h(s); the rear contact follows the same profile one wheelbase
 * behind; the body pitch is asin((h_front - h_rear) / L); the
 * accelerometer sees gravity plus the car's own acceleration (including
 * the vertical acceleration over the hump), plus noise, at 400 Hz.
 *
 *   make -C testing/subsystem4/host
 *
 * Group IS28, Buddy 4.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imu_proc.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond, ...)                                            \
    do {                                                                \
        if (cond) { g_pass++; printf("PASS  %-8s ", id); }              \
        else      { g_fail++; printf("FAIL  %-8s ", id); }              \
        printf(__VA_ARGS__);                                            \
        printf("\n");                                                   \
    } while (0)

/* ---------------------------------------------------------------------
 * Ground profiles: height (mm) of the ground at path distance s (mm).
 * ------------------------------------------------------------------ */
typedef enum { P_FLAT, P_TRAP, P_COS, P_RAMP } prof_kind_t;

typedef struct
{
    prof_kind_t kind;
    double      start;      /* where it begins                        */
    double      h;          /* height                                 */
    double      up;         /* ramp length (trapezoid / ramp)         */
    double      top;        /* top length (trapezoid); length (cos)   */
} prof_t;

static double
ground (prof_t const * p, double s)
{
    double x = s - p->start;

    switch (p->kind)
    {
        case P_TRAP:
            if (x <= 0.0) return 0.0;
            if (x < p->up) return p->h * x / p->up;
            if (x < p->up + p->top) return p->h;
            if (x < 2.0 * p->up + p->top)
                return p->h * (2.0 * p->up + p->top - x) / p->up;
            return 0.0;

        case P_COS:
            if ((x <= 0.0) || (x >= p->top)) return 0.0;
            return p->h * 0.5 * (1.0 - cos(2.0 * M_PI * x / p->top));

        case P_RAMP:                    /* up and stays up            */
            if (x <= 0.0) return 0.0;
            if (x < p->up) return p->h * x / p->up;
            return p->h;

        default:
            return 0.0;
    }
}

/* ---------------------------------------------------------------------
 * Simulated car + sensor
 * ------------------------------------------------------------------ */
typedef struct
{
    b4_ctx_t  c;
    b4_out_t  o;            /* results of the last period              */
    uint32_t  t_us;
    double    s;            /* path distance of the front contact, mm  */
    double    v;            /* mm/s                                    */
    double    a;            /* mm/s^2 (speed change)                   */
    double    head;         /* heading, dd                             */
    double    turn;         /* dd/s                                    */
    double    noise;        /* mg, uniform +-                          */
    bool      b_motion;     /* Buddy 2 estimate available              */
    prof_t    prof;
    double    hc_prev[2];   /* body centre height history (vert. acc.) */
    int       spike_at;     /* FIFO index of a shock spike, -1 none    */
    double    spike_mg;
    uint8_t   n_override;   /* force FIFO count (0 = normal 8)         */
    bool      b_read_fail;
    bool      b_stuck;
    uint32_t  gap_extra_us;
    /* collected events */
    int       n_ev;
    imu_event_t ev[256];
    bool      b_shock_seen;
    uint32_t  shock_age_us;
    uint8_t   fault_reported;
    bool      b_cleared;
    bool      b_init_req;
} sim_t;

/* Own random numbers, so every PC (glibc, macOS) runs the same cases. */
static uint32_t g_seed = 1U;

static void
seed (uint32_t s)
{
    g_seed = (s == 0U) ? 1U : s;
}

static double
urand (void)
{
    g_seed = (g_seed * 1664525U) + 1013904223U;
    return ((double)(g_seed >> 8) / 8388608.0) - 1.0;      /* -1 .. 1 */
}

static void
collect (sim_t * p)
{
    uint32_t i;

    for (i = 0U; i < p->o.n_events; i++)
    {
        if (p->n_ev < 256)
        {
            p->ev[p->n_ev++] = p->o.events[i];
        }
    }

    if (p->o.b_shock)
    {
        p->b_shock_seen = true;
        p->shock_age_us = p->o.shock_age_us;
    }

    if (p->o.fault != 0U)
    {
        p->fault_reported = p->o.fault;
    }

    if (p->o.b_clear_fault)
    {
        p->b_cleared = true;
    }

    if (p->o.b_init)
    {
        p->b_init_req = true;
    }
}

static void
sim_init (sim_t * p)
{
    memset(p, 0, sizeof(*p));
    b4_reset(&p->c);
    p->b_motion = true;
    p->noise    = 2.0;                  /* a real sensor is never silent */
    p->spike_at = -1;
    p->prof.kind = P_FLAT;
}

/* One 20 ms period. */
static void
sim_period (sim_t * p)
{
    b4_in_t  in;
    uint32_t i;
    uint32_t n = ((p->n_override != 0U) && (p->n_override <= 32U))
                 ? p->n_override : 8U;
    double   dt = 0.0025;
    double   L = (double)B4_WHEELBASE_MM;

    memset(&in, 0, sizeof(in));
    memset(&p->o, 0, sizeof(p->o));

    for (i = 0U; i < n; i++)
    {
        double hf, hr, sn, cs, hc, az, fwd, up;

        p->v    += p->a * dt;
        if (p->v < 0.0) { p->v = 0.0; }
        p->s    += p->v * dt;
        p->head += p->turn * dt;

        hf = ground(&p->prof, p->s);
        hr = ground(&p->prof, p->s - L);
        sn = (hf - hr) / L;
        if (sn > 1.0) { sn = 1.0; }
        if (sn < -1.0) { sn = -1.0; }
        cs = sqrt(1.0 - sn * sn);

        /* vertical acceleration of the body centre (mm/s^2). Tyres and
         * wheel radius round off a sharp edge: about 12 ms. */
        hc = 0.5 * (hf + hr);
        hc = p->hc_prev[0] + 0.2 * (hc - p->hc_prev[0]);
        az = (hc - 2.0 * p->hc_prev[0] + p->hc_prev[1]) / (dt * dt);
        p->hc_prev[1] = p->hc_prev[0];
        p->hc_prev[0] = hc;

        /* specific force in the body frame, mg */
        fwd = 1000.0 * sn + (p->a / 9.807) + (az / 9.807) * sn;
        up  = 1000.0 * cs + (az / 9.807) * cs;

        if (p->b_stuck)
        {
            fwd = 12.0;
            up  = 998.0;
        }
        else
        {
            fwd += p->noise * urand();
            up  += p->noise * urand();
        }

        if ((int)i == p->spike_at)
        {
            up += p->spike_mg;
        }

        in.s[i].fwd_mg = (int16_t)lround(fwd);
        in.s[i].lat_mg = (int16_t)lround(p->b_stuck ? 3.0
                                                    : p->noise * urand());
        in.s[i].up_mg  = (int16_t)lround(up);
    }

    p->t_us     += 20000U + p->gap_extra_us;
    in.t_us      = p->t_us;
    in.gap_us    = 20000U + p->gap_extra_us;
    in.b_read_ok = !p->b_read_fail;
    in.n         = p->b_read_fail ? 0U : (uint8_t)n;
    if (p->n_override == 255U) { in.n = 0U; }
    in.b_motion  = p->b_motion;
    in.distance_mm = (int16_t)(uint16_t)((int32_t)lround(p->s) & 0xFFFF);
    in.heading_dd  = (int16_t)(uint16_t)((int32_t)lround(p->head) & 0xFFFF);

    b4_tick(&p->c, &in, &p->o);
    collect(p);
    p->spike_at = -1;
}

static void
sim_run_ms (sim_t * p, uint32_t ms)
{
    uint32_t k;

    for (k = 0U; k < ms / 20U; k++)
    {
        sim_period(p);
    }
}

static void
sim_mission (sim_t * p, b4_mission_t m)
{
    memset(&p->o, 0, sizeof(p->o));
    b4_mission(&p->c, m, p->t_us, &p->o);
    collect(p);
}

/* Power up, sensor OK, calibrate still for 2.2 s. */
static void
sim_ready (sim_t * p)
{
    memset(&p->o, 0, sizeof(p->o));
    b4_init_done(&p->c, true, p->t_us, &p->o);
    collect(p);
    sim_run_ms(p, 2200U);
}

static int
count_ev (sim_t const * p, imu_evt_type_t type)
{
    int i;
    int n = 0;

    for (i = 0; i < p->n_ev; i++)
    {
        if (p->ev[i].type == (uint32_t)type)
        {
            n++;
        }
    }

    return n;
}

static imu_event_t const *
last_ev (sim_t const * p, imu_evt_type_t type)
{
    int i;

    for (i = p->n_ev - 1; i >= 0; i--)
    {
        if (p->ev[i].type == (uint32_t)type)
        {
            return &p->ev[i];
        }
    }

    return NULL;
}

/* Drive over a profile at speed v; returns the measured peak or -1 and
 * the reject reason in *p_rej. */
static int
drive_hump (prof_t prof, double v, double noise, int * p_rej)
{
    sim_t               sim;
    imu_event_t const * e;

    sim_init(&sim);
    sim.noise = noise;
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim.prof = prof;
    sim.v    = v;
    sim_run_ms(&sim, (uint32_t)(1000.0 * (prof.start + 2.0 * prof.up +
                                           prof.top + 600.0) / v));
    *p_rej = -1;
    e = last_ev(&sim, IMU_EVT_HUMP_PEAK);

    if (e != NULL)
    {
        return (int)e->height_mm;
    }

    e = last_ev(&sim, IMU_EVT_HUMP_REJECTED);
    *p_rej = (e != NULL) ? (int)e->detail : 99;
    return -1;
}

/* =====================================================================
 * Tests
 * ================================================================== */

static void
test_helpers (void)
{
    int  i;
    int  worst = 0;

    for (i = 0; i <= 600; i += 5)
    {
        double s = sin((double)i / 10.0 * M_PI / 180.0);
        int    got = b4_asin_dd((int32_t)lround(s * 32767.0));
        int    err = abs(got - i);

        worst = (err > worst) ? err : worst;
    }

    CHECK("H-01", worst <= 2, "asin table within 0.2 deg up to 60 deg "
          "(worst %d dd)", worst);
    CHECK("H-02", (b4_asin_dd(-16384) == -300) && (b4_asin_dd(40000) == 900),
          "asin sign and saturation");
    CHECK("H-03", (b4_isqrt(0U) == 0U) && (b4_isqrt(1000000U) == 1000U) &&
          (b4_isqrt(999999U) == 999U) && (b4_isqrt(0xFFFFFFFFU) == 65535U),
          "integer square root");
    CHECK("H-04", b4_nfr02_ok(25, 20) && !b4_nfr02_ok(26, 20) &&
          b4_nfr02_ok(88, 80) && !b4_nfr02_ok(89, 80) &&
          b4_nfr02_ok(55, 50) && !b4_nfr02_ok(56, 50),
          "NFR-IMU-02 bound = max(5 mm, 10 %%), threshold 50 mm");
}

static void
test_pack (void)
{
    imu_tlm_t in;
    imu_tlm_t out;
    uint8_t   f[IMU_TLM_FRAME_BYTES];

    memset(&in, 0, sizeof(in));
    in.seq = 7U; in.state = IMU_ST_HUMP; in.motion = IMU_MOTION_CLIMBING;
    in.evt = IMU_EVT_HUMP_PEAK; in.detail = 3U; in.b_valid = true;
    in.b_odo = true; in.pitch_dd = 123; in.fault = 4U; in.height_mm = 21U;
    in.max_height_mm = 33U; in.humps = 2U; in.rejected = 1U;
    in.shock_mg = 1500U; in.shock_ms = 7U; in.shocks = 3U;
    in.i2c_errors = 5U; in.missed = 9U; in.t_ms = 54321U;
    imu_tlm_pack(&in, f);
    CHECK("P-01", (sizeof(f) == 19U) && ((f[0] & 0xFU) == TLM_VERSION) &&
          ((f[0] >> 4) == TLM_SRC_IMU) && (f[1] == 16U) && (f[2] == 7U),
          "frame = 3-byte header (version 2, source 4, length 16) + 16");
    memset(&out, 0xA5, sizeof(out));
    CHECK("P-02", imu_tlm_unpack(f, sizeof(f), &out) &&
          (out.state == in.state) && (out.motion == in.motion) &&
          (out.evt == in.evt) && (out.detail == 3U) && out.b_valid &&
          out.b_odo && (out.pitch_dd == 123) && (out.fault == 4U) &&
          (out.height_mm == 21U) && (out.max_height_mm == 33U) &&
          (out.humps == 2U) && (out.rejected == 1U) &&
          (out.shock_mg == 1500U) && (out.shock_ms == 7U) &&
          (out.shocks == 3U) && (out.i2c_errors == 5U) &&
          (out.missed == 9U) && (out.t_ms == 54321U) && (out.seq == 7U),
          "normal values round trip");

    memset(&in, 0, sizeof(in));
    imu_tlm_pack(&in, f);
    CHECK("P-03", imu_tlm_unpack(f, sizeof(f), &out) && (out.pitch_dd == 0) &&
          (out.height_mm == 0U) && !out.b_valid && (out.t_ms == 0U),
          "minimum (all zero) round trip");

    in.pitch_dd = -900; in.height_mm = 255U; in.max_height_mm = 255U;
    in.humps = 255U; in.rejected = 255U; in.shock_mg = 4095U;
    in.shock_ms = 63U; in.shocks = 255U; in.i2c_errors = 63U;
    in.missed = 65535U; in.t_ms = 65535U; in.state = IMU_ST_FAULT;
    in.motion = IMU_MOTION_IMPACT; in.evt = IMU_EVT_RUN_SUMMARY;
    in.detail = 15U; in.fault = 15U;
    imu_tlm_pack(&in, f);
    CHECK("P-04", imu_tlm_unpack(f, sizeof(f), &out) &&
          (out.pitch_dd == -900) && (out.height_mm == 255U) &&
          (out.shock_mg == 4095U) && (out.shock_ms == 63U) &&
          (out.i2c_errors == 63U) && (out.missed == 65535U) &&
          (out.t_ms == 65535U) && (out.detail == 15U) && (out.fault == 15U),
          "maximum and negative values round trip");

    in.pitch_dd = 2000; in.shock_mg = 9999U; in.shock_ms = 200U;
    in.state = 9U; in.fault = 99U;
    imu_tlm_pack(&in, f);
    CHECK("P-05", imu_tlm_unpack(f, sizeof(f), &out) &&
          (out.pitch_dd == 900) && (out.shock_mg == 4095U) &&
          (out.shock_ms == 63U) && (out.state == 7U) && (out.fault == 15U),
          "out-of-range values are clamped, never wrapped (R4/R5)");

    in.i2c_errors = 64U;                /* 6-bit counter wraps        */
    imu_tlm_pack(&in, f);
    CHECK("P-06", imu_tlm_unpack(f, sizeof(f), &out) &&
          (out.i2c_errors == 0U), "wrapping counter wraps (6 bits)");

    f[0] = (uint8_t)((f[0] & 0x0FU) | (5U << 4));
    CHECK("P-07", !imu_tlm_unpack(f, sizeof(f), &out) &&
          !imu_tlm_unpack(f, sizeof(f) - 1U, &out),
          "wrong source or length rejected");
    CHECK("P-08", sizeof(imu_event_t) <= 8U, "imu_event_t is %u bytes "
          "(40 bits of data)", (unsigned)sizeof(imu_event_t));
}

static void
test_init_and_cal (void)
{
    sim_t sim;

    /* Initialisation failure -> Fault -> authorised reset -> again */
    sim_init(&sim);
    memset(&sim.o, 0, sizeof(sim.o));
    b4_init_done(&sim.c, false, 0U, &sim.o);
    collect(&sim);
    CHECK("C-01", (sim.c.state == IMU_ST_FAULT) &&
          (sim.fault_reported == IMU_FAULT_INIT) &&
          (count_ev(&sim, IMU_EVT_FAULT) == 1),
          "sensor not answering: Fault, reported to Safety");
    sim_mission(&sim, B4_MISSION_START);
    CHECK("C-02", sim.c.state == IMU_ST_FAULT, "no self-recovery: mission "
          "start leaves it in Fault");
    sim_mission(&sim, B4_MISSION_RESET);
    CHECK("C-03", (sim.c.state == IMU_ST_INIT) && sim.b_init_req,
          "authorised reset: back to Initialising");
    memset(&sim.o, 0, sizeof(sim.o));
    b4_init_done(&sim.c, true, sim.t_us, &sim.o);
    collect(&sim);
    CHECK("C-04", (sim.c.state == IMU_ST_CALIBRATING) && sim.b_cleared,
          "sensor answers: Calibrating, fault bit cleared");
    sim_run_ms(&sim, 1980U);
    CHECK("C-05", sim.c.state == IMU_ST_CALIBRATING, "still calibrating "
          "before 2.0 s");
    sim_run_ms(&sim, 60U);
    CHECK("C-06", (sim.c.state == IMU_ST_IDLE) && (sim.c.g_mg > 990) &&
          (sim.c.g_mg < 1010), "2.0 s still: baseline taken, Idle "
          "(|g| = %d mg)", (int)sim.c.g_mg);

    /* Moving: the window keeps restarting, deadline -> Fault */
    sim_init(&sim);
    sim.noise = 60.0;
    memset(&sim.o, 0, sizeof(sim.o));
    b4_init_done(&sim.c, true, sim.t_us, &sim.o);
    sim_run_ms(&sim, 9900U);
    CHECK("C-07", sim.c.state == IMU_ST_CALIBRATING, "not still: no "
          "baseline yet");
    sim_run_ms(&sim, 200U);
    CHECK("C-08", (sim.c.state == IMU_ST_FAULT) &&
          (sim.fault_reported == IMU_FAULT_CAL_TIMEOUT),
          "calibration deadline (10 s) expired: Fault");

    /* Car rolling on the wheels during calibration (Buddy 2 says so) */
    sim_init(&sim);
    memset(&sim.o, 0, sizeof(sim.o));
    b4_init_done(&sim.c, true, sim.t_us, &sim.o);
    sim.v = 50.0;
    sim_run_ms(&sim, 3000U);
    CHECK("C-09", sim.c.state == IMU_ST_CALIBRATING,
          "[stationary] guard: no baseline while Buddy 2 reports motion");
    sim.v = 0.0;
    sim_run_ms(&sim, 2400U);
    CHECK("C-10", sim.c.state == IMU_ST_IDLE, "stopped: calibrates");

    /* Wrong mounting: gravity on the forward axis */
    sim_init(&sim);
    sim.prof.kind = P_FLAT;
    memset(&sim.o, 0, sizeof(sim.o));
    b4_init_done(&sim.c, true, sim.t_us, &sim.o);
    {
        int k;

        for (k = 0; k < 110; k++)
        {
            b4_in_t in;
            int     i;

            memset(&in, 0, sizeof(in));
            memset(&sim.o, 0, sizeof(sim.o));
            sim.t_us += 20000U;
            in.t_us = sim.t_us; in.gap_us = 20000U; in.b_read_ok = true;
            in.n = 8U;
            for (i = 0; i < 8; i++) { in.s[i].fwd_mg = 1000; in.s[i].up_mg = (int16_t)(k % 3); }
            b4_tick(&sim.c, &in, &sim.o);
            collect(&sim);
        }
    }
    CHECK("C-11", (sim.c.state == IMU_ST_FAULT) &&
          (sim.fault_reported == IMU_FAULT_CAL),
          "gravity not on the UP axis (mounting settings wrong): Fault");
}

static void
test_humps (void)
{
    static struct { prof_kind_t k; double h; double up; double top;
                    double v; double noise; } const cases[] =
    {
        { P_TRAP, 10.0,  30.0,  60.0, 150.0,  5.0 },
        { P_TRAP, 20.0,  40.0,  80.0, 150.0,  5.0 },
        { P_TRAP, 20.0,  40.0,  80.0, 100.0,  5.0 },
        { P_TRAP, 20.0,  40.0,  80.0, 250.0,  5.0 },
        { P_TRAP, 30.0,  60.0, 100.0, 150.0, 10.0 },
        { P_TRAP, 50.0, 100.0, 150.0, 150.0, 10.0 },
        { P_TRAP, 80.0, 160.0, 150.0, 150.0, 10.0 },
        { P_TRAP, 20.0,  30.0,  20.0, 150.0,  5.0 },   /* narrow      */
        { P_COS,  15.0,   0.0, 150.0, 150.0,  5.0 },
        { P_COS,  25.0,   0.0, 200.0, 150.0, 10.0 },
        { P_COS,  40.0,   0.0, 300.0, 200.0, 10.0 },
    };
    unsigned i;
    char     id[16];

    printf("\n  NFR-IMU-02 sweep (allowed error max(5 mm, 10 %%)):\n");

    for (i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        prof_t p = { cases[i].k, 300.0, cases[i].h, cases[i].up,
                     cases[i].top };
        int    rej;
        int    got;

        seed(100U + i);
        got = drive_hump(p, cases[i].v, cases[i].noise, &rej);
        snprintf(id, sizeof(id), "U-%02u", i + 1U);
        CHECK(id, (got >= 0) && b4_nfr02_ok(got, (int)cases[i].h),
              "%s %2.0f mm hump (ramp %3.0f, top %3.0f) at %3.0f mm/s, "
              "noise %2.0f mg: measured %d mm%s%s",
              (cases[i].k == P_COS) ? "cosine" : "trapez",
              cases[i].h, cases[i].up, cases[i].top, cases[i].v,
              cases[i].noise, got, (got < 0) ? " " : "",
              (got < 0) ? imu_reject_name((imu_reject_t)rej) : "");
    }
}

/* Many random humps: heights 8..80 mm, lengths, speeds 80..300 mm/s,
 * noise up to 20 mg. Every one must be inside NFR-IMU-02. */
static void
test_hump_sweep (void)
{
    int n = 0;
    int ok = 0;
    int rejected = 0;
    int worst = 0;
    int sum_err = 0;
    int k;

    seed(4242U);

    for (k = 0; k < 200; k++)
    {
        prof_t p;
        double h = 10.0 + 70.0 * (0.5 + 0.5 * urand());
        double v = 100.0 + 200.0 * (0.5 + 0.5 * urand());
        double noise = 2.0 + 18.0 * (0.5 + 0.5 * urand());
        int    rej;
        int    got;
        int    err;

        p.kind  = (k % 2 == 0) ? P_TRAP : P_COS;
        p.start = 300.0;
        p.h     = h;
        p.up    = h * (1.5 + 2.0 * (0.5 + 0.5 * urand()));  /* <= 34 deg */
        p.top   = (p.kind == P_COS) ? (4.0 * p.up)
                                    : (20.0 + 150.0 * (0.5 + 0.5 * urand()));
        if ((p.kind == P_TRAP) && ((2.0 * p.up + p.top) > 600.0))
        {
            p.top = 20.0;               /* keep it a hump (< 600 mm)   */
            p.up  = 290.0;
            p.h   = (p.h > 0.5 * p.up) ? (0.5 * p.up) : p.h;
            h     = p.h;
        }
        if (p.kind == P_COS)            /* keep it a hump (< 600 mm) */
        {
            p.top = (p.top > 600.0) ? 600.0 : p.top;
            p.h   = (p.h > p.top / 6.0) ? (p.top / 6.0) : p.h;
            h     = p.h;
        }
        got = drive_hump(p, v, noise, &rej);
        n++;

        if (got < 0)
        {
            rejected++;
            continue;
        }

        err = abs(got - (int)lround(h));
        sum_err += got - (int)lround(h);
        worst = (err > worst) ? err : worst;
        ok += b4_nfr02_ok(got, (int)lround(h)) ? 1 : 0;
    }

    CHECK("U-12", (ok == n), "%d random humps (10-80 mm, 100-300 mm/s, "
          "noise 2-20 mg): %d within NFR-IMU-02, %d rejected, worst error %d mm,"
          " mean %+.1f mm", n, ok, rejected, worst,
          (double)sum_err / (double)((n - rejected) > 0 ? (n - rejected) : 1));
}

/* Long flat run with noise and motor vibration: nothing false. */
static void
test_false_events (void)
{
    sim_t sim;
    int   k;

    sim_init(&sim);
    sim.noise = 20.0;
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim.v = 200.0;

    for (k = 0; k < 1500; k++)          /* 30 s */
    {
        sim.spike_at = (k % 7);         /* vibration: +-400 mg samples */
        sim.spike_mg = ((k % 2) == 0) ? 400.0 : -400.0;
        sim_period(&sim);
    }

    CHECK("R-11", (count_ev(&sim, IMU_EVT_HUMP_START) == 0) &&
          (count_ev(&sim, IMU_EVT_IMPACT) == 0),
          "30 s flat with 20 mg noise and 0.4 g vibration spikes: no hump, "
          "no impact");
}

static void
test_hump_rules (void)
{
    sim_t               sim;
    imu_event_t const * e;
    int                 rej;
    prof_t              p;

    /* A ramp up onto a raised floor is not a hump */
    p.kind = P_RAMP; p.start = 300.0; p.h = 30.0; p.up = 100.0; p.top = 0.0;
    (void)drive_hump(p, 150.0, 5.0, &rej);
    sim_init(&sim);
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim.prof = p;
    sim.v = 150.0;
    sim_run_ms(&sim, 9000U);
    e = last_ev(&sim, IMU_EVT_HUMP_REJECTED);
    CHECK("R-01", (count_ev(&sim, IMU_EVT_HUMP_START) == 1) && (e != NULL) &&
          (e->detail == IMU_REJ_NO_DESCENT) &&
          (count_ev(&sim, IMU_EVT_HUMP_PEAK) == 0),
          "ramp that stays up: rejected (no descent), nothing published");

    /* Flat ground, speeding up hard: no false hump (acceleration is
     * taken out using Buddy 2's speed) */
    sim_init(&sim);
    sim.noise = 10.0;
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim.a = 800.0;                      /* 0.08 g: 4.7 deg if not removed */
    sim_run_ms(&sim, 400U);
    sim.a = -800.0;
    sim_run_ms(&sim, 400U);
    sim.a = 0.0;
    sim_run_ms(&sim, 600U);
    CHECK("R-02", count_ev(&sim, IMU_EVT_HUMP_START) == 0,
          "hard acceleration and braking on the flat: no hump");

    /* Same with the estimate missing: the IMU alone cannot tell */
    sim_init(&sim);
    sim_ready(&sim);
    sim.b_motion = false;
    sim_mission(&sim, B4_MISSION_START);
    p.kind = P_TRAP; p.start = 100.0; p.h = 20.0; p.up = 40.0; p.top = 80.0;
    sim.prof = p;
    sim.v = 150.0;
    sim_run_ms(&sim, 5000U);
    e = last_ev(&sim, IMU_EVT_HUMP_REJECTED);
    CHECK("R-03", (count_ev(&sim, IMU_EVT_HUMP_START) == 1) && (e != NULL) &&
          (e->detail == IMU_REJ_NO_ODOMETRY),
          "no Buddy 2 distance: hump detected (FR-IMU-01) but its height "
          "is rejected, not guessed");

    /* Shock in the middle of a hump: hump invalidated, Impact, cleared */
    sim_init(&sim);
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim.prof = p;
    sim.v = 150.0;
    while ((sim.c.state != IMU_ST_HUMP) && (sim.t_us < 10000000U))
    {
        sim_period(&sim);
    }
    sim_run_ms(&sim, 100U);
    sim.spike_at = 2;
    sim.spike_mg = 2500.0;
    sim_period(&sim);
    e = last_ev(&sim, IMU_EVT_HUMP_REJECTED);
    CHECK("R-04", (sim.c.state == IMU_ST_IMPACT) && sim.b_shock_seen &&
          (e != NULL) && (e->detail == IMU_REJ_SHOCK) &&
          (count_ev(&sim, IMU_EVT_IMPACT) == 1) &&
          (sim.c.motion == IMU_MOTION_IMPACT),
          "shock mid-hump: Impact in the same period, partial hump "
          "rejected, motion = impact");
    CHECK("R-05", sim.shock_age_us + 20000U < B4_SHOCK_LIMIT_MS * 1000U,
          "worst case detect time %u us (sample age) + one period < 50 ms",
          (unsigned)sim.shock_age_us);
    sim_run_ms(&sim, 3000U);
    CHECK("R-06", (count_ev(&sim, IMU_EVT_HUMP_PEAK) == 0) &&
          (sim.c.state == IMU_ST_TRACKING) && (sim.c.max_height_mm == 0U),
          "no false peak kept after the shock; back to Motion Tracking");

    /* Stop mid-hump */
    sim_init(&sim);
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim.prof = p;
    sim.v = 150.0;
    while ((sim.c.state != IMU_ST_HUMP) && (sim.t_us < 10000000U))
    {
        sim_period(&sim);
    }
    sim_mission(&sim, B4_MISSION_STOP);
    e = last_ev(&sim, IMU_EVT_HUMP_REJECTED);
    CHECK("R-07", (sim.c.state == IMU_ST_IDLE) && (e != NULL) &&
          (e->detail == IMU_REJ_STOPPED), "stop mid-hump: rejected, Idle");

    /* Run maximum, summary, reset on a new run (not on resume) */
    sim_init(&sim);
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim.v = 150.0;
    p.kind = P_TRAP; p.start = 100.0; p.h = 30.0; p.up = 60.0; p.top = 80.0;
    sim.prof = p;
    sim_run_ms(&sim, 4000U);
    sim_mission(&sim, B4_MISSION_STOP);         /* pause            */
    sim_mission(&sim, B4_MISSION_RESUME);
    p.start = sim.s + 100.0; p.h = 15.0; p.up = 30.0; p.top = 60.0;
    sim.prof = p;
    sim_run_ms(&sim, 4000U);
    sim_mission(&sim, B4_MISSION_DONE);
    e = last_ev(&sim, IMU_EVT_RUN_SUMMARY);
    CHECK("R-08", (count_ev(&sim, IMU_EVT_HUMP_PEAK) == 2) && (e != NULL) &&
          b4_nfr02_ok((int)e->height_mm, 30) && (sim.c.humps == 2U),
          "two humps, highest kept over a pause; run summary %d mm",
          (e != NULL) ? (int)e->height_mm : -1);
    sim_mission(&sim, B4_MISSION_RESET);
    sim_mission(&sim, B4_MISSION_START);
    CHECK("R-09", (sim.c.max_height_mm == 0U) && (sim.c.humps == 0U) &&
          (sim.c.state == IMU_ST_TRACKING), "new run resets the maximum");

    /* Distance across the int16 wrap */
    sim_init(&sim);
    sim.s = 32767.0 - 500.0;
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    p.kind = P_TRAP; p.start = 32767.0 - 200.0; p.h = 20.0; p.up = 40.0;
    p.top = 80.0;
    sim.prof = p;
    sim.v = 150.0;
    sim_run_ms(&sim, 5000U);
    e = last_ev(&sim, IMU_EVT_HUMP_PEAK);
    CHECK("R-10", (e != NULL) && b4_nfr02_ok((int)e->height_mm, 20),
          "hump across the 16-bit distance wrap: %d mm",
          (e != NULL) ? (int)e->height_mm : -1);
}

static void
test_motion (void)
{
    sim_t sim;

    sim_init(&sim);
    sim.noise = 4.0;
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim_run_ms(&sim, 400U);
    CHECK("M-01", sim.c.motion == IMU_MOTION_STATIONARY, "stopped: "
          "stationary");
    sim.a = 600.0;
    sim_run_ms(&sim, 300U);
    CHECK("M-02", sim.c.motion == IMU_MOTION_ACCELERATING, "speeding up: "
          "accelerating (%s)", imu_motion_name(sim.c.motion));
    sim.a = 0.0;
    sim_run_ms(&sim, 600U);
    CHECK("M-03", sim.c.motion == IMU_MOTION_STEADY, "constant %0.f mm/s: "
          "steady (%s)", sim.v, imu_motion_name(sim.c.motion));
    sim.turn = 600.0;
    sim_run_ms(&sim, 400U);
    CHECK("M-04", sim.c.motion == IMU_MOTION_TURNING, "60 deg/s: turning");
    sim.turn = 0.0;
    sim.a = -2000.0;
    sim_run_ms(&sim, 800U);
    sim.a = 0.0;
    sim_run_ms(&sim, 600U);
    CHECK("M-05", sim.c.motion == IMU_MOTION_STATIONARY, "braked to a stop");
    CHECK("M-06", count_ev(&sim, IMU_EVT_MOTION) >= 4, "each change "
          "published once as a motion event (%d)",
          count_ev(&sim, IMU_EVT_MOTION));

    /* Climbing / descending from the IMU */
    {
        prof_t p = { P_TRAP, sim.s + 50.0, 60.0, 300.0, 300.0 };

        sim.prof = p;
        sim.v = 150.0;
        sim_run_ms(&sim, 2000U);
        CHECK("M-07", sim.c.motion == IMU_MOTION_CLIMBING, "nose up 11 deg: "
              "climbing (%s, pitch %d dd)", imu_motion_name(sim.c.motion),
              (int)sim.c.pitch_dd);
        sim_run_ms(&sim, 2800U);
        CHECK("M-08", sim.c.motion == IMU_MOTION_DESCENDING, "nose down: "
              "descending (%s, pitch %d dd)", imu_motion_name(sim.c.motion),
              (int)sim.c.pitch_dd);
    }

    /* No estimate: the IMU alone */
    sim_init(&sim);
    sim.noise = 4.0;
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim.b_motion = false;
    sim_run_ms(&sim, 400U);
    CHECK("M-09", sim.c.motion == IMU_MOTION_STATIONARY, "no estimate, IMU "
          "quiet: stationary");
    sim.noise = 80.0;
    sim_run_ms(&sim, 400U);
    CHECK("M-10", sim.c.motion == IMU_MOTION_UNKNOWN, "no estimate, "
          "vibrating: unknown (turning is never guessed without Buddy 2)");

    /* Flicker: a class shorter than the hold time is not published */
    sim_init(&sim);
    sim.noise = 4.0;
    sim_ready(&sim);
    sim_mission(&sim, B4_MISSION_START);
    sim_run_ms(&sim, 400U);
    {
        int before = count_ev(&sim, IMU_EVT_MOTION);

        sim.turn = 900.0;
        sim_run_ms(&sim, 40U);
        sim.turn = 0.0;
        sim_run_ms(&sim, 600U);
        CHECK("M-11", count_ev(&sim, IMU_EVT_MOTION) <= before + 2,
              "short blip does not flood motion events");
    }

    /* Static tilt accuracy (bench block test S4-C05) */
    sim_init(&sim);
    sim_ready(&sim);
    {
        b4_in_t in;
        int     k;
        int     i;

        for (k = 0; k < 20; k++)
        {
            memset(&in, 0, sizeof(in));
            memset(&sim.o, 0, sizeof(sim.o));
            sim.t_us += 20000U;
            in.t_us = sim.t_us; in.gap_us = 20000U; in.b_read_ok = true;
            in.n = 8U; in.b_motion = true;
            for (i = 0; i < 8; i++)
            {
                in.s[i].fwd_mg = (int16_t)lround(1000.0 * sin(10.0 * M_PI / 180.0));
                in.s[i].up_mg  = (int16_t)lround(1000.0 * cos(10.0 * M_PI / 180.0));
            }
            b4_tick(&sim.c, &in, &sim.o);
        }
        CHECK("M-12", abs(sim.c.pitch_dd - 100) <= 3, "10.0 deg tilt reads "
              "%d.%d deg", sim.c.pitch_dd / 10, abs(sim.c.pitch_dd) % 10);
    }
}

static void
test_health (void)
{
    sim_t sim;

    sim_init(&sim);
    sim_ready(&sim);
    sim.b_read_fail = true;
    sim_run_ms(&sim, 40U);
    CHECK("F-01", sim.c.state == IMU_ST_IDLE, "two failed reads: still "
          "sampling");
    sim_run_ms(&sim, 20U);
    CHECK("F-02", (sim.c.state == IMU_ST_FAULT) &&
          (sim.fault_reported == IMU_FAULT_I2C) && !sim.c.b_mission,
          "third failed read in a row (60 ms): Fault, I2C");
    sim.b_read_fail = false;
    sim_run_ms(&sim, 1000U);
    CHECK("F-03", sim.c.state == IMU_ST_FAULT, "stays in Fault although "
          "the bus works again (no self-recovery)");

    sim_init(&sim);
    sim_ready(&sim);
    sim.n_override = 255U;              /* FIFO always empty          */
    sim_run_ms(&sim, 60U);
    CHECK("F-04", (sim.c.state == IMU_ST_FAULT) &&
          (sim.fault_reported == IMU_FAULT_STALE),
          "no new sample for 3 periods: Fault, stale data");

    sim_init(&sim);
    sim_ready(&sim);
    sim.b_stuck = true;
    sim_run_ms(&sim, 980U);
    CHECK("F-05", sim.c.state != IMU_ST_FAULT, "identical data < 1 s: OK");
    sim_run_ms(&sim, 60U);
    CHECK("F-06", (sim.c.state == IMU_ST_FAULT) &&
          (sim.fault_reported == IMU_FAULT_INVALID),
          "identical data for 1 s: Fault, invalid");

    sim_init(&sim);
    sim_ready(&sim);
    sim.gap_extra_us = 40000U;          /* one period took 60 ms      */
    sim_period(&sim);
    sim.gap_extra_us = 0U;
    sim_period(&sim);
    CHECK("F-07", sim.c.missed == 2U, "a 60 ms period counts 2 missed "
          "frames (NFR-IMU-01 evidence): %u", (unsigned)sim.c.missed);

    /* Shock in Idle, then quiet: back to Idle */
    sim_init(&sim);
    sim_ready(&sim);
    sim.spike_at = 7;
    sim.spike_mg = -1500.0;
    sim_period(&sim);
    CHECK("F-08", (sim.c.state == IMU_ST_IMPACT) && sim.b_shock_seen &&
          (sim.shock_age_us == 0U), "shock in Idle (newest sample): Impact");
    sim_run_ms(&sim, 180U);
    CHECK("F-09", sim.c.state == IMU_ST_IMPACT, "latched for 200 ms");
    sim_run_ms(&sim, 40U);
    CHECK("F-10", sim.c.state == IMU_ST_IDLE, "cleared: back to Idle "
          "(no mission)");
    sim.spike_at = 0;
    sim.spike_mg = 800.0;               /* below the threshold        */
    sim_period(&sim);
    CHECK("F-11", sim.c.state == IMU_ST_IDLE, "a 0.8 g bump is not a shock");
}

int
main (void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("Buddy 4 host tests (wheelbase %d mm, shock %d mg)\n\n",
           B4_WHEELBASE_MM, B4_SHOCK_MG);
    test_helpers();
    test_pack();
    test_init_and_cal();
    test_humps();
    test_hump_sweep();
    test_false_events();
    test_hump_rules();
    test_motion();
    test_health();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
