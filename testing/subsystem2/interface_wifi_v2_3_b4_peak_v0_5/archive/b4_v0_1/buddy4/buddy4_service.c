/* Buddy4 IMU/ramp service on the v2.3 SDK firmware.
 *
 * Port of app_program/subsystem4/task_imu.c (RTOS) with only the kernel and
 * Safety/NAV couplings replaced:
 *   - 20 ms cyclic task          -> b4v23_due()/b4v23_tick() from v2.3's superloop
 *   - nav-state message queue    -> one mission edge per tick from the harness
 *   - b2_odometry snapshot       -> b4v23_odo_t from v2.3's own encoder drain
 *   - safety_report_fault/sys_fault (shock, IMU fault) -> not present in v2.3;
 *     Buddy4 stays measurement/telemetry only and never affects motor control
 *   - imu_telemetry_publish      -> v2.3 Wi-Fi send callback (frames unchanged)
 * Processing (imu_proc.c), packing (b4_pack.c, b2_pack.c), driver logic and
 * all thresholds are the validated passive v0.9 sources, unchanged. */
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "buddy4_v23.h"
#include "v23b4_rp2040_shim.h"      /* sys_time_us() = time_us_32() */
#include "subsystem4/b4_config.h"
#include "subsystem4/drv_lsm303.h"
#include "subsystem4/imu_proc.h"
#include "subsystem4/b4_diagnostics.h"
#include "system/sys_tlm.h"

#define AXIS_LAT    (3 - B4_AXIS_FWD - B4_AXIS_UP)

static b4_ctx_t         g_ctx;
static b4_in_t          g_in;
static b4_raw_t         g_raw[B4_FIFO_DEPTH];
static uint8_t          g_seq;
static uint8_t          g_motion_seq;
static b4v23_send_fn    g_send;
static bool             g_console;
static bool             g_started;
static uint32_t         g_last_us, g_status_us;
static uint64_t         g_next_us;
static imu_state_t      g_last_state = IMU_ST_COUNT;
static b4_diagnostics_t g_side_diag;
static uint32_t         g_tick_us, g_tick_max_us;
static b4_out_t const   k_no_out;
static imu_tlm_t const  k_no_tlm;
static b4v23_odo_t      g_odo;           /* latest view, for source 2 */

static void send_frame(const uint8_t *f, uint32_t n) { if (g_send) (void)g_send(f, n); }

static void publish(imu_event_t const *p_evt)            /* task_imu.c publish() */
{
    imu_tlm_t t = k_no_tlm;
    uint8_t frame[IMU_TLM_FRAME_BYTES];
    b4_fill_tlm(&g_ctx, p_evt, &t);
    t.seq = g_seq++;
    imu_tlm_pack(&t, frame);
    send_frame(frame, IMU_TLM_FRAME_BYTES);
}

static int32_t clamp_measurement(double v, int32_t lo, int32_t hi)
{
    if (!__builtin_isfinite(v)) return 0;
    if (v >= (double)hi) return hi;
    if (v <= (double)lo) return lo;
    return (int32_t)v;
}

/* task_imu.c publish_motion_snapshot(), source 2, same 37-byte payload.
 * Speeds come from v2.3 velocity[]; the odometry-valid flag is Buddy4's own
 * strict interval check, independent of motor control. */
static void publish_motion_snapshot(void)
{
    b2_tlm_t t = {0};
    uint8_t frame[B2_TLM_FRAME_BYTES];
    t.seq = g_motion_seq++;
    t.flags = 0U;                         /* motor-capable image: not passive */
#if B4_SIDE_RAMP_MODE
    t.flags |= B2_TLM_SIDE_MODE;
    if (g_ctx.b_side_valid) t.flags |= B2_TLM_SIDE_VALID;
    t.roll_dd = g_ctx.side_roll_dd;
    t.side_height_mm = g_ctx.side_height_mm;
    t.quality_reasons = g_side_diag.reasons;
    t.ramp_state = (uint8_t)((g_ctx.side_level_ready ? 1U : 0U) |
        (g_ctx.side_candidate ? 2U : 0U) | (g_ctx.side_rising ? 4U : 0U) |
        (g_ctx.side_returning ? 8U : 0U));
    t.quality_bad_samples = g_side_diag.bad;
    t.quality_total_samples = g_side_diag.total;
    t.max_spread_mg = g_side_diag.spread_mg;
    t.max_accel_mm_s2 = g_side_diag.accel_mm_s2;
    t.max_turn_dd_s = g_side_diag.turn_dd_s;
    t.rate_samples = g_side_diag.rate_samples;
    t.max_mad_mg = g_side_diag.mad_mg;
    t.max_axis_mad_mg[0] = g_side_diag.axis_mad_mg[0];
    t.max_axis_mad_mg[1] = g_side_diag.axis_mad_mg[1];
    t.max_axis_mad_mg[2] = g_side_diag.axis_mad_mg[2];
    t.min_fifo_samples = g_side_diag.min_fifo_samples;
#endif
    if (g_odo.have) {
        if (g_odo.clean && g_odo.age_ms <= B4_MOTION_FRESH_MS) t.flags |= B2_TLM_ODO_VALID;
        t.left_mm_s = (int16_t)clamp_measurement(g_odo.left_mm_s, INT16_MIN, INT16_MAX);
        t.right_mm_s = (int16_t)clamp_measurement(g_odo.right_mm_s, INT16_MIN, INT16_MAX);
        t.centre_mm_s = (int16_t)clamp_measurement(((double)g_odo.left_mm_s + g_odo.right_mm_s) * 0.5,
                                                   INT16_MIN, INT16_MAX);
        t.distance_mm = clamp_measurement(g_odo.centre_mm, INT32_MIN, INT32_MAX);
        t.invalid_left = (uint16_t)(g_odo.invalid_left > UINT16_MAX ? UINT16_MAX : g_odo.invalid_left);
        t.invalid_right = (uint16_t)(g_odo.invalid_right > UINT16_MAX ? UINT16_MAX : g_odo.invalid_right);
    }
    b2_tlm_pack(&t, frame);
    send_frame(frame, B2_TLM_FRAME_BYTES);
    g_side_diag = (b4_diagnostics_t){0};
}

static void handle(b4_out_t *p_out, uint32_t t0_us);

static void init_sensor(void)                              /* task_imu.c init_sensor() */
{
    b4_chip_t chip = B4_CHIP_NONE;
    bool b_mag = false;
    bool b_ok = b4_drv_init(&chip, &b_mag);
    b4_out_t out;
    if (g_console) {
        if (b_ok) printf("[imu] GY-511 found: %s, magnetometer %s; calibrating: keep the car still for %u s\n",
                         (chip == B4_CHIP_AGR) ? "LSM303AGR" : "LSM303DLHC", b_mag ? "yes" : "no",
                         (unsigned)(B4_CAL_WINDOW_MS / 1000U));
        else printf("[imu] GY-511 not answering on I2C1 (SDA GP%u, SCL GP%u)\n",
                    (unsigned)B4_I2C_SDA_PIN, (unsigned)B4_I2C_SCL_PIN);
    }
    out = k_no_out;
    b4_init_done(&g_ctx, b_ok, sys_time_us(), &out);
    handle(&out, sys_time_us());
}

static void handle(b4_out_t *p_out, uint32_t t0_us)       /* task_imu.c handle() */
{
    uint32_t i;
    if (p_out->b_shock) {
        /* RTOS: also safety_report_fault(IMU, SHOCK, warning). v2.3 has no
         * Safety supervisor; the shock is reported in telemetry only. */
        uint32_t us = p_out->shock_age_us + (sys_time_us() - t0_us);
        g_ctx.shock_ms = (uint8_t)(((us + 999U) / 1000U > 63U) ? 63U : ((us + 999U) / 1000U));
    }
    /* p_out->fault / b_clear_fault: RTOS sys_fault/Safety only; the fault code
     * is carried in source 4 telemetry (lastFault/state). */
    for (i = 0U; i < (uint32_t)p_out->n_events; i++) publish(&p_out->events[i]);
    if (p_out->b_init) init_sensor();                    /* after an authorised reset */
}

static void to_car(b4_raw_t const *p_r, b4_vec_t *p_v)    /* task_imu.c to_car() */
{
    int32_t a[3];
    a[0] = p_r->x; a[1] = p_r->y; a[2] = p_r->z;
    p_v->fwd_mg = (int16_t)(a[B4_AXIS_FWD] * B4_SIGN_FWD);
    p_v->up_mg  = (int16_t)(a[B4_AXIS_UP] * B4_SIGN_UP);
    p_v->lat_mg = (int16_t)a[AXIS_LAT];
}

static b4_mission_t map_mission(b4v23_mission_t m)
{
    switch (m) {
    case B4V23_START:  return B4_MISSION_START;
    case B4V23_RESUME: return B4_MISSION_RESUME;
    case B4V23_RESET:  return B4_MISSION_RESET;
    default:           return B4_MISSION_STOP;
    }
}

void b4v23_init(b4v23_send_fn send, bool console)
{
    g_send = send; g_console = console;
    b4_reset(&g_ctx);
    init_sensor();
    g_last_us = g_status_us = sys_time_us();
    g_next_us = 0U;
    g_started = true;
}

bool b4v23_due(uint64_t now_us) { return g_started && now_us >= g_next_us; }
uint32_t b4v23_last_tick_us(void) { return g_tick_us; }
uint32_t b4v23_max_tick_us(void) { return g_tick_max_us; }

/* One 20 ms period: task_imu.c loop body. */
void b4v23_tick(uint64_t now_us, b4v23_mission_t mission, const b4v23_odo_t *odo)
{
    b4_out_t imu_out = k_no_out;
    uint32_t now = (uint32_t)now_us;
    uint32_t t_begin = sys_time_us();
    g_next_us = (g_next_us == 0U || now_us - g_next_us > (uint64_t)B4_PERIOD_MS * 1000U)
                ? now_us + (uint64_t)B4_PERIOD_MS * 1000U        /* (re)start the schedule */
                : g_next_us + (uint64_t)B4_PERIOD_MS * 1000U;    /* fixed rate, no drift  */
    if (odo) g_odo = *odo;
    if (mission != B4V23_NONE) b4_mission(&g_ctx, map_mission(mission), now, &imu_out);
    g_in.t_us = now;
    g_in.gap_us = now - g_last_us;
    g_last_us = now;
    if ((g_ctx.state != IMU_ST_INIT) && (g_ctx.state != IMU_ST_FAULT)) {
        uint32_t i;
        g_in.b_read_ok = b4_drv_read_fifo(g_raw, (uint8_t)B4_FIFO_DEPTH, &g_in.n, &g_in.b_overrun);
        for (i = 0U; i < (uint32_t)g_in.n; i++) to_car(&g_raw[i], &g_in.s[i]);
        /* b4_odometry_adapter.c semantics, from v2.3's encoders. */
        g_in.b_motion = g_odo.have && g_odo.clean && (g_odo.age_ms <= B4_MOTION_FRESH_MS) &&
                        __builtin_isfinite(g_odo.centre_mm) && __builtin_isfinite(g_odo.heading_dd);
        if (g_in.b_motion) {
            int32_t mm = clamp_measurement(g_odo.centre_mm, INT32_MIN, INT32_MAX);
            int32_t dd = clamp_measurement(g_odo.heading_dd, INT32_MIN, INT32_MAX);
            double fine = g_odo.centre_mm * 1000.0;
            g_in.distance_mm = (int16_t)(uint16_t)mm;
            g_in.heading_dd = (int16_t)(uint16_t)dd;
            g_in.b_fine_motion = (fine >= (double)INT32_MIN && fine <= (double)INT32_MAX);
            g_in.distance_um = g_in.b_fine_motion ? (int32_t)fine : 0;
        } else {
            g_in.distance_mm = 0; g_in.heading_dd = 0; g_in.b_fine_motion = false; g_in.distance_um = 0;
        }
        b4_tick(&g_ctx, &g_in, &imu_out);
        b4_diag_accumulate(&g_side_diag, &g_ctx);
    }
    handle(&imu_out, now);
    if ((g_ctx.state != g_last_state) || ((now - g_status_us) >= (B4_STATUS_PERIOD_MS * 1000U))) {
        publish(NULL);
        publish_motion_snapshot();
        g_status_us = now;
        g_last_state = g_ctx.state;
    }
    g_tick_us = sys_time_us() - t_begin;
    if (g_tick_us > g_tick_max_us) g_tick_max_us = g_tick_us;
}
