/**
 * @file    task_imu.c
 * @brief   Buddy 4: the IMU Terrain task (Week 6 Section 3, statechart 6.4).
 *
 * Periodic, 20 ms (NFR-IMU-01), paced by a cyclic handler so the period
 * does not drift with the work done in it. Each period it
 *   1. takes the operating-state changes the Navigation Coordinator queued
 *      (imu_notify_state());
 *   2. empties the GY-511 FIFO and reads Buddy 2's motion estimate;
 *   3. runs one statechart step (imu_proc.c: pure C, host-tested);
 *   4. acts on the result: a shock is reported to Safety FIRST, in the
 *      same period it was seen (NFR-IMU-03), then faults, then telemetry.
 *
 * It never commands motion (Week 6 Section 3: "It never commands motion").
 * Priority: band 18-19, below every driving and safety task.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../system/nav_iface.h"
#include "../system/safety_iface.h"
#include "../system/sys_config.h"
#include "../system/sys_fault.h"
#include "../system/sys_hal.h"
#include "b4_config.h"
#include "drv_lsm303.h"
#include "imu_proc.h"
#include "subsystem4.h"

/* Tribunal 1: the IMU reads the shared motion estimate under the SAME
 * freshness rule as everyone else (IF-19). */
#if (B4_MOTION_FRESH_MS != NAV_LINE_STALE_MS)
#error "B4_MOTION_FRESH_MS must equal the team's NAV_LINE_STALE_MS"
#endif

#define AXIS_LAT    (3 - B4_AXIS_FWD - B4_AXIS_UP)

/* All written by the IMU task only (single writer). The getters read one
 * aligned byte, which is atomic on the RP2040. */
static b4_ctx_t              g_ctx;
static b4_in_t               g_in;
static b4_raw_t              g_raw[B4_FIFO_DEPTH];
static ID                    g_tid = 0;
static ID                    g_cyc = 0;
static ID                    g_nav_mbf = 0;
static uint8_t               g_seq = 0U;
static nav_state_t           g_prev_nav = NAV_ST_READY;
static volatile imu_state_t  g_state = IMU_ST_INIT;
static volatile imu_motion_t g_motion = IMU_MOTION_UNKNOWN;
static volatile uint8_t      g_max_mm = 0U;
static volatile uint32_t     g_periods = 0U;
static volatile uint32_t     g_missed = 0U;
static b4_out_t const        k_no_out;      /* all zero: "nothing to do" */
static imu_tlm_t const       k_no_tlm;

/* =====================================================================
 * Provided to other tasks
 * ================================================================== */

/**
 * @brief  Navigation Coordinator: operating state changed (nav_iface.h).
 *         Called from the coordinator task: queue it and return.
 */
void
imu_notify_state (nav_state_t state)
{
    uint8_t b = (uint8_t)state;

    if (g_nav_mbf > 0)
    {
        (void)tk_snd_mbf(g_nav_mbf, &b, 1, TMO_POL);
    }
}

imu_state_t
imu_get_state (void)
{
    return g_state;
}

imu_motion_t
imu_get_motion (void)
{
    return g_motion;
}

uint8_t
imu_get_highest_peak_mm (void)
{
    return g_max_mm;
}

UW
imu_periods (void)
{
    return (UW)g_periods;
}

UW
imu_frames_missed (void)
{
    return (UW)g_missed;
}

bool
imu_data_valid (void)
{
    imu_state_t s = g_state;

    return ((s == IMU_ST_IDLE) || (s == IMU_ST_TRACKING) ||
            (s == IMU_ST_HUMP) || (s == IMU_ST_IMPACT));
}

/* =====================================================================
 * Helpers
 * ================================================================== */

static void
cyclic_handler (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_tid);
}

static void
publish (imu_event_t const * p_evt)
{
    imu_tlm_t t;
    uint8_t   frame[IMU_TLM_FRAME_BYTES];

    t = k_no_tlm;
    b4_fill_tlm(&g_ctx, p_evt, &t);
    t.seq = g_seq;
    g_seq++;
    imu_tlm_pack(&t, frame);
    imu_telemetry_publish(frame, IMU_TLM_FRAME_BYTES);
}

static b4_mission_t
map_nav (nav_state_t now, nav_state_t prev)
{
    b4_mission_t ev;

    switch (now)
    {
        case NAV_ST_RUNNING:
            ev = (prev == NAV_ST_PAUSED) ? B4_MISSION_RESUME
                                         : B4_MISSION_START;
            break;

        case NAV_ST_COMPLETED:
            ev = B4_MISSION_DONE;
            break;

        case NAV_ST_READY:
            ev = B4_MISSION_RESET;      /* only reached through a reset */
            break;

        default:                        /* paused, stopped, e-stop, fault */
            ev = B4_MISSION_STOP;
            break;
    }

    return ev;
}

static void handle(b4_out_t * p_out, uint32_t t0_us);

/* Initialising: configure the GY-511 and check that it answers. */
static void
init_sensor (void)
{
    b4_chip_t chip = B4_CHIP_NONE;
    bool      b_mag = false;
    bool      b_ok = b4_drv_init(&chip, &b_mag);
    b4_out_t  out;

    if (b_ok)
    {
        tm_printf((UB *)"[imu] GY-511 found: %s, magnetometer %s\n",
                  (chip == B4_CHIP_AGR) ? "LSM303AGR" : "LSM303DLHC",
                  b_mag ? "yes" : "no");
        tm_printf((UB *)"[imu] calibrating: keep the car still for %d s\n",
                  (INT)(B4_CAL_WINDOW_MS / 1000U));
    }
    else
    {
        tm_printf((UB *)"[imu] GY-511 not answering on I2C1 "
                        "(SDA GP%d, SCL GP%d)\n",
                  (INT)B4_I2C_SDA_PIN, (INT)B4_I2C_SCL_PIN);
    }

    out = k_no_out;
    b4_init_done(&g_ctx, b_ok, sys_time_us(), &out);
    handle(&out, sys_time_us());
}

/* Act on one step's results. Shock first: it has a 50 ms budget. */
static void
handle (b4_out_t * p_out, uint32_t t0_us)
{
    uint32_t i;

    if (p_out->b_shock)
    {
        uint32_t us;

        (void)safety_report_fault(SAFETY_SRC_IMU, IMU_WARN_SHOCK, false);

        /* Detect-to-notify time: the sample's age in the FIFO when it
         * was read, plus the processing up to here (NFR-IMU-03). */
        us = p_out->shock_age_us + (sys_time_us() - t0_us);
        g_ctx.shock_ms = (uint8_t)(((us + 999U) / 1000U > 63U)
                                   ? 63U : ((us + 999U) / 1000U));
    }

    if (p_out->fault != IMU_FAULT_NONE)
    {
        /* Fault / Recovery: data flagged invalid, Safety told. */
        (void)sys_fault_set(SYS_FAULT_IMU);
        (void)safety_report_fault(SAFETY_SRC_IMU, p_out->fault,
                                  (B4_FAULT_CRITICAL != 0));
    }

    if (p_out->b_clear_fault)
    {
        (void)sys_fault_clear(SYS_FAULT_IMU);
    }

    for (i = 0U; i < (uint32_t)p_out->n_events; i++)
    {
        publish(&p_out->events[i]);
        imu_event_notify(&p_out->events[i]);
    }

    g_state  = g_ctx.state;
    g_motion = g_ctx.motion;
    g_max_mm = g_ctx.max_height_mm;

    if (p_out->b_init)
    {
        init_sensor();                  /* after an authorised reset */
    }
}

/* Sensor axes -> car axes (b4_config.h). */
static void
to_car (b4_raw_t const * p_r, b4_vec_t * p_v)
{
    int32_t a[3];

    a[0] = p_r->x;
    a[1] = p_r->y;
    a[2] = p_r->z;
    p_v->fwd_mg = (int16_t)(a[B4_AXIS_FWD] * B4_SIGN_FWD);
    p_v->up_mg  = (int16_t)(a[B4_AXIS_UP] * B4_SIGN_UP);
    p_v->lat_mg = (int16_t)a[AXIS_LAT];
}

#if SYS_CONSOLE_STATUS && SYS_CONSOLE_SENSORS
static void
print_sensors (void)
{
    static uint32_t n = 0U;

    n++;

    if ((n % (B4_STATUS_PERIOD_MS / B4_PERIOD_MS)) == 0U)
    {
        tm_printf((UB *)"[imu] a fwd %d up %d mg, pitch %d.%d deg, "
                        "missed %d\n",
                  (INT)g_in.s[0].fwd_mg, (INT)g_in.s[0].up_mg,
                  (INT)(g_ctx.pitch_dd / 10),
                  (INT)(((g_ctx.pitch_dd < 0) ? -g_ctx.pitch_dd
                                              : g_ctx.pitch_dd) % 10),
                  (INT)g_ctx.missed);
    }
}
#endif

/* =====================================================================
 * The task
 * ================================================================== */

void
imu_task (INT stacd, void * p_exinf)
{
    uint32_t    last_us;
    uint32_t    status_us;
    imu_state_t last_state = IMU_ST_COUNT;

    (void)stacd;
    (void)p_exinf;

    b4_reset(&g_ctx);
    (void)safety_watch(SAFETY_SRC_IMU, B4_WATCH_MS);
    init_sensor();
    (void)tk_sta_cyc(g_cyc);

    last_us   = sys_time_us();
    status_us = last_us;

    for (;;)
    {
        b4_out_t imu_out;
        uint8_t  b;
        uint32_t now;

        /* Woken by the cyclic handler every 20 ms. The timeout only
         * matters if the handler stopped: the period is then late and
         * counted as missed frames. */
        (void)tk_slp_tsk((TMO)(B4_PERIOD_MS * 5U));
        now = sys_time_us();
        imu_out = k_no_out;

        while (tk_rcv_mbf(g_nav_mbf, &b, TMO_POL) == 1)
        {
            nav_state_t s = (nav_state_t)b;

            b4_mission(&g_ctx, map_nav(s, g_prev_nav), now, &imu_out);
            g_prev_nav = s;
        }

        g_in.t_us   = now;
        g_in.gap_us = now - last_us;
        last_us     = now;

        if ((g_ctx.state != IMU_ST_INIT) && (g_ctx.state != IMU_ST_FAULT))
        {
            imu_motion_in_t est = { 0, 0, 0U };
            uint32_t        i;

            g_in.b_read_ok = b4_drv_read_fifo(g_raw, (uint8_t)B4_FIFO_DEPTH,
                                              &g_in.n, &g_in.b_overrun);

            for (i = 0U; i < (uint32_t)g_in.n; i++)
            {
                to_car(&g_raw[i], &g_in.s[i]);
            }

            g_in.b_motion = imu_motion_get(&est) &&
                            (est.age_ms <= B4_MOTION_FRESH_MS);
            g_in.distance_mm = est.distance_mm;
            g_in.heading_dd  = est.heading_dd;

            b4_tick(&g_ctx, &g_in, &imu_out);
        }

        handle(&imu_out, now);
        safety_kick(SAFETY_SRC_IMU);
        g_periods++;
        g_missed = g_ctx.missed;

        /* Periodic status, and at once on every state change. */
        if ((g_ctx.state != last_state) ||
            ((now - status_us) >= (B4_STATUS_PERIOD_MS * 1000U)))
        {
            publish(NULL);
            status_us  = now;
            last_state = g_ctx.state;
        }

#if SYS_CONSOLE_STATUS && SYS_CONSOLE_SENSORS
        print_sensors();
#endif
    }
}

/**
 * @brief  Phase 1: the operating-state queue and the 20 ms cyclic handler.
 */
ER
imu_task_init (void)
{
    T_CMBF cmbf = { 0 };
    T_CCYC ccyc = { 0 };

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ)(B4_NAV_QUEUE * 16U); /* 1-byte messages + header */
    cmbf.maxmsz = 1;
    g_nav_mbf   = tk_cre_mbf(&cmbf);

    if (g_nav_mbf < E_OK)
    {
        return g_nav_mbf;
    }

    ccyc.cycatr = TA_HLNG;                  /* started by the task */
    ccyc.cychdr = (FP)cyclic_handler;
    ccyc.cyctim = (RELTIM)B4_PERIOD_MS;
    ccyc.cycphs = 0;
    g_cyc       = tk_cre_cyc(&ccyc);

    return (g_cyc < E_OK) ? g_cyc : E_OK;
}

/**
 * @brief  Phase 2: the task.
 */
ER
imu_task_start (void)
{
    T_CTSK ctsk = { 0 };

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = imu_task;
    ctsk.itskpri = B4_TASK_PRIORITY;
    ctsk.stksz   = B4_TASK_STACK_BYTES;
    g_tid        = tk_cre_tsk(&ctsk);

    if (g_tid < E_OK)
    {
        return g_tid;
    }

    return tk_sta_tsk(g_tid, 0);
}
