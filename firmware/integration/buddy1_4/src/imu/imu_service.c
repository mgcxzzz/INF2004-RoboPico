/**
 * @file imu_service.c
 * @brief Imu service.
 * @details Sensor processing owns calibration/terrain state; RTOS readers use copies.
 * Height acceptance and motor assistance are separate from end-run safety.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
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
 * Processing began with the passive v0.9 sources. Add-on v0.2 fixes the
 * powered profile's turn-rate gate and bounded acceleration exclusions;
 * see BUDDY4_POWERED_ACCEPTANCE_V0_2.md. Packing and driver are unchanged. */
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "src/imu/imu_service.h"
#include "src/imu/pico_registers.h" /* sys_time_us() = time_us_32() */
#include "src/imu/b4_config.h"
#include "src/imu/drv_lsm303.h"
#include "src/imu/imu_proc.h"
#include "src/imu/b4_diagnostics.h"
#include "src/telemetry/sys_tlm.h"
#ifdef APP_RTOS_RELEASE
#include "src/imu/imu_lift_guard.h"
/* Implemented by the RTOS mailbox owner. Samples are copied/processed before
 * the normal hump-quality gates and outside the wrapper's cross-core lock. */
void ri_lift_guard_samples (int16_t const baseline_mg[3], int32_t gravity_mg,
                            imu_lift_vec_t const * p_samples_value,
                            uint8_t count_value, uint32_t sample_period_us,
                            bool fresh_batch);
#endif
#ifdef APP_RTOS_IMU
#include "app_program/imu_navigation/imu_acquisition.h"
#include "app_program/imu_navigation/calibration_diagnostics.h"
#include "app_program/imu_navigation/hump_diagnostics.h"
#endif

#define AXIS_LAT (3 - B4_AXIS_FWD - B4_AXIS_UP)

static b4_ctx_t g_ctx;
static b4_in_t g_in;
static b4_raw_t g_raw[B4_FIFO_DEPTH];
static uint8_t g_seq;
static uint8_t g_motion_seq;
static b4v23_send_fn g_p_g_send;
static bool g_console;
static bool g_started;
static uint32_t g_last_us;
static uint32_t g_status_us;
static uint64_t g_next_us;
static imu_state_t g_last_state = IMU_ST_COUNT;
static b4_diagnostics_t g_side_diag;
static uint32_t g_tick_us;
static uint32_t g_tick_max_us;
static b4_out_t const g_k_no_out;
static imu_tlm_t const g_k_no_tlm;
static b4v23_odo_t g_odo;   /* latest view, for source 2 */
static b4v23_tilt_t g_tilt; /* ramp-assist v0.1: latest side tilt */
#ifdef APP_RTOS_IMU
static ri_calibration_t g_calibration;
void b4v23_hump_qualification (ri_hump_qualification_t * p_out_value)
{
    p_out_value->holds = g_ctx.side_diag_hold_count;
    p_out_value->recoveries = g_ctx.side_diag_recover_count;
    p_out_value->disarms = g_ctx.side_diag_disarm_count;
    p_out_value->early_clears = g_ctx.side_diag_early_clear_count;
    p_out_value->flags =
        (g_ctx.side_level_ready ? 1U : 0U) | (g_ctx.side_candidate ? 2U : 0U) |
        (g_ctx.side_rising ? 4U : 0U) | (g_ctx.side_returning ? 8U : 0U);
    p_out_value->hold_reason = g_ctx.side_diag_last_hold_reason;
    p_out_value->clear_reason = g_ctx.side_diag_last_clear_reason;
    p_out_value->failure_reason = g_ctx.side_diag_last_failure_reason;
    uint32_t gap_ms = g_ctx.side_odo_gap_run_us / 1000U;
    p_out_value->gap_ms = gap_ms > 127U ? 127U : gap_ms;
    p_out_value->last_gap_ms = g_ctx.side_odo_gap_last_ms;
}
void b4v23_calibration (ri_calibration_t * p_out_value)
{
    *p_out_value = g_calibration;
    p_out_value->state = g_ctx.state;
    p_out_value->fault = g_ctx.fault;
}
void ri_calibration_restart (uint8_t reasons, const b4_ctx_t * p_ctx_value,
                             const int16_t mean[3])
{
    ++g_calibration.restarts;
    g_calibration.last_reason = reasons;
    g_calibration.speed_restarts += (reasons & RI_CAL_SPEED) != 0;
    if (p_ctx_value->cal_n > g_calibration.best)
    {
        g_calibration.best =
            p_ctx_value->cal_n > 100 ? 100 : p_ctx_value->cal_n;
    }
    for (unsigned index = 0; index < 3; ++index)
    {
        int32_t low = mean[index] < p_ctx_value->cal_min[index]
                          ? mean[index]
                          : p_ctx_value->cal_min[index];
        int32_t high = mean[index] > p_ctx_value->cal_max[index]
                           ? mean[index]
                           : p_ctx_value->cal_max[index];
        g_calibration.axis_restarts[index] += (reasons & (1U << index)) != 0;
        g_calibration.last_span_mg[index] = (uint16_t)(high - low);
    }
}
#endif

static void send_frame (const uint8_t * p_flags, uint32_t count)
{
    if (g_p_g_send)
    {
        (void)g_p_g_send(p_flags, count);
    }
}

static void publish (imu_event_t const * p_evt) /* task_imu.c publish() */
{
    imu_tlm_t sample = g_k_no_tlm;
    uint8_t frame[IMU_TLM_FRAME_BYTES];
    b4_fill_tlm(&g_ctx, p_evt, &sample);
    sample.seq = g_seq++;
    imu_tlm_pack(&sample, frame);
    send_frame(frame, IMU_TLM_FRAME_BYTES);
}

static int32_t clamp_measurement (double value, int32_t low, int32_t high)
{
    if (!__builtin_isfinite(value))
    {
        return 0;
    }
    if (value >= (double)high)
    {
        return high;
    }
    if (value <= (double)low)
    {
        return low;
    }
    return (int32_t)value;
}

/* task_imu.c publish_motion_snapshot(), source 2, same 37-byte payload.
 * Speeds come from v2.3 velocity[]; the odometry-valid flag is Buddy4's own
 * strict interval check, independent of motor control. */
static void publish_motion_snapshot (void)
{
    b2_tlm_t sample = {0};
    uint8_t frame[B2_TLM_FRAME_BYTES];
    sample.seq = g_motion_seq++;
    sample.flags = 0U; /* motor-capable image: not passive */
#if B4_SIDE_RAMP_MODE
    sample.flags |= B2_TLM_SIDE_MODE;
    if (g_ctx.b_side_valid)
    {
        sample.flags |= B2_TLM_SIDE_VALID;
    }
    sample.roll_dd = g_ctx.side_roll_dd;
    sample.side_height_mm = g_ctx.side_height_mm;
    sample.quality_reasons = g_side_diag.reasons;
    sample.ramp_state = (uint8_t)((g_ctx.side_level_ready ? 1U : 0U) |
                                  (g_ctx.side_candidate ? 2U : 0U) |
                                  (g_ctx.side_rising ? 4U : 0U) |
                                  (g_ctx.side_returning ? 8U : 0U));
    sample.quality_bad_samples = g_side_diag.bad;
    sample.quality_total_samples = g_side_diag.total;
    sample.max_spread_mg = g_side_diag.spread_mg;
    sample.max_accel_mm_s2 = g_side_diag.accel_mm_s2;
    sample.max_turn_dd_s = g_side_diag.turn_dd_s;
    sample.rate_samples = g_side_diag.rate_samples;
    sample.max_mad_mg = g_side_diag.mad_mg;
    sample.max_axis_mad_mg[0] = g_side_diag.axis_mad_mg[0];
    sample.max_axis_mad_mg[1] = g_side_diag.axis_mad_mg[1];
    sample.max_axis_mad_mg[2] = g_side_diag.axis_mad_mg[2];
    sample.min_fifo_samples = g_side_diag.min_fifo_samples;
#endif
    if (g_odo.have)
    {
        if (g_odo.clean && g_odo.age_ms <= B4_MOTION_FRESH_MS)
        {
            sample.flags |= B2_TLM_ODO_VALID;
        }
        sample.left_mm_s =
            (int16_t)clamp_measurement(g_odo.left_mm_s, INT16_MIN, INT16_MAX);
        sample.right_mm_s =
            (int16_t)clamp_measurement(g_odo.right_mm_s, INT16_MIN, INT16_MAX);
        sample.centre_mm_s = (int16_t)clamp_measurement(
            ((double)g_odo.left_mm_s + g_odo.right_mm_s) * 0.5, INT16_MIN,
            INT16_MAX);
        sample.distance_mm =
            clamp_measurement(g_odo.centre_mm, INT32_MIN, INT32_MAX);
        sample.invalid_left =
            (uint16_t)(g_odo.invalid_left > UINT16_MAX ? UINT16_MAX
                                                       : g_odo.invalid_left);
        sample.invalid_right =
            (uint16_t)(g_odo.invalid_right > UINT16_MAX ? UINT16_MAX
                                                        : g_odo.invalid_right);
    }
    b2_tlm_pack(&sample, frame);
    send_frame(frame, B2_TLM_FRAME_BYTES);
    g_side_diag = (b4_diagnostics_t){0};
}

static void handle (b4_out_t * p_out, uint32_t t0_us);

static void init_sensor (void) /* task_imu.c init_sensor() */
{
    b4_chip_t chip = B4_CHIP_NONE;
    bool b_mag = false;
#ifdef APP_RTOS_IMU
    ri_acquisition_init_begin();
#endif
    bool b_ok = b4_drv_init(&chip, &b_mag);
#ifdef APP_RTOS_IMU
    if (!ri_acquisition_init_end())
    {
        b_ok = false;
        chip = B4_CHIP_NONE;
        b_mag = false;
    }
#endif
    b4_out_t out;
#ifndef APP_RTOS_IMU
    if (g_console)
    {
        if (b_ok)
        {
            printf("[imu] GY-511 found: %s, magnetometer %s; calibrating: keep "
                   "the car still for %u s\n",
                   (chip == B4_CHIP_AGR) ? "LSM303AGR" : "LSM303DLHC",
                   b_mag ? "yes" : "no", (unsigned)(B4_CAL_WINDOW_MS / 1000U));
        }
        else
        {
            printf("[imu] GY-511 not answering on I2C1 (SDA GP%u, SCL GP%u)\n",
                   (unsigned)B4_I2C_SDA_PIN, (unsigned)B4_I2C_SCL_PIN);
        }
    }
#endif /* RTOS uses copied diagnostics, never SDK/newlib console output. */
    out = g_k_no_out;
    b4_init_done(&g_ctx, b_ok, sys_time_us(), &out);
    handle(&out, sys_time_us());
}

static void handle (b4_out_t * p_out, uint32_t t0_us) /* task_imu.c handle() */
{
    uint32_t index;
    if (p_out->b_shock)
    {
        /* RTOS: also safety_report_fault(IMU, SHOCK, warning). v2.3 has no
         * Safety supervisor; the shock is reported in telemetry only. */
        uint32_t microseconds = p_out->shock_age_us + (sys_time_us() - t0_us);
        g_ctx.shock_ms = (uint8_t)(((microseconds + 999U) / 1000U > 63U)
                                       ? 63U
                                       : ((microseconds + 999U) / 1000U));
    }
    /* p_out->fault / b_clear_fault: RTOS sys_fault/Safety only; the fault code
     * is carried in source 4 telemetry (lastFault/state). */
    for (index = 0U; index < (uint32_t)p_out->n_events; index++)
    {
        publish(&p_out->events[index]);
    }
    if (p_out->b_init)
    {
        init_sensor(); /* after an authorised reset */
    }
}

static void to_car (b4_raw_t const * p_r,
                    b4_vec_t * p_v) /* task_imu.c to_car() */
{
    int32_t first[3];
    first[0] = p_r->x;
    first[1] = p_r->y;
    first[2] = p_r->z;
    p_v->fwd_mg = (int16_t)(first[B4_AXIS_FWD] * B4_SIGN_FWD);
    p_v->up_mg = (int16_t)(first[B4_AXIS_UP] * B4_SIGN_UP);
    p_v->lat_mg = (int16_t)first[AXIS_LAT];
}

static b4_mission_t map_mission (b4v23_mission_t m_value)
{
    switch (m_value)
    {
    case B4V23_START:
        return B4_MISSION_START;
    case B4V23_RESUME:
        return B4_MISSION_RESUME;
    case B4V23_RESET:
        return B4_MISSION_RESET;
    default:
        return B4_MISSION_STOP;
    }
}

void b4v23_init (b4v23_send_fn p_send_value, bool console)
{
    g_p_g_send = p_send_value;
    g_console = console;
#ifdef APP_RTOS_IMU
    g_calibration = (ri_calibration_t){0};
#endif
    b4_reset(&g_ctx);
    init_sensor();
    g_last_us = g_status_us = sys_time_us();
    g_next_us = 0U;
    g_started = true;
}

bool b4v23_due (uint64_t now_us)
{
    return g_started && now_us >= g_next_us;
}
uint32_t b4v23_last_tick_us (void)
{
    return g_tick_us;
}
uint32_t b4v23_max_tick_us (void)
{
    return g_tick_max_us;
}
void b4v23_tilt (b4v23_tilt_t * p_out_value)
{
    if (p_out_value)
    {
        *p_out_value = g_tilt;
    }
}

/* One 20 ms period: task_imu.c loop body. */
void b4v23_tick (uint64_t now_us, b4v23_mission_t mission,
                 const b4v23_odo_t * p_odo_value)
{
    b4_out_t imu_out = g_k_no_out;
    uint32_t now = (uint32_t)now_us;
    uint32_t t_begin = sys_time_us();
    bool processed = false;
#ifdef APP_RTOS_RELEASE
    bool lift_fresh_batch = false;
#endif
    g_next_us =
        (g_next_us == 0U || now_us - g_next_us > (uint64_t)B4_PERIOD_MS * 1000U)
            ? now_us +
                  (uint64_t)B4_PERIOD_MS * 1000U /* (re)start the schedule */
            : g_next_us +
                  (uint64_t)B4_PERIOD_MS * 1000U; /* fixed rate, no drift  */
    if (p_odo_value)
    {
        g_odo = *p_odo_value;
    }
    if (mission != B4V23_NONE)
    {
        b4_mission(&g_ctx, map_mission(mission), now, &imu_out);
    }
#ifdef APP_RTOS_RELEASE
    bool lift_calibrated = (g_ctx.state != IMU_ST_INIT) &&
                           (g_ctx.state != IMU_ST_CALIBRATING) &&
                           (g_ctx.state != IMU_ST_FAULT);
#endif
    g_in.t_us = now;
    g_in.gap_us = now - g_last_us;
    g_last_us = now;
    if ((g_ctx.state != IMU_ST_INIT) && (g_ctx.state != IMU_ST_FAULT))
    {
        uint32_t index;
#ifdef APP_RTOS_IMU
        uint32_t processor_start_us;
        g_in.n = 0U;
        g_in.b_overrun = false;
        ri_acquisition_fifo_begin();
#endif
        g_in.b_read_ok = b4_drv_read_fifo(g_raw, (uint8_t)B4_FIFO_DEPTH,
                                          &g_in.n, &g_in.b_overrun);
#ifdef APP_RTOS_IMU
        if (!ri_acquisition_fifo_end(g_in.n))
        {
            g_in.b_read_ok = false;
        }
        ri_acquisition_record(g_in.gap_us, g_in.b_read_ok, g_in.b_overrun);
        processor_start_us = sys_time_us();
#endif
        for (index = 0U; index < (uint32_t)g_in.n; index++)
        {
            to_car(&g_raw[index], &g_in.s[index]);
        }
        /* b4_odometry_adapter.c semantics, from v2.3's encoders. */
        g_in.b_motion = g_odo.have && g_odo.clean &&
                        (g_odo.age_ms <= B4_MOTION_FRESH_MS) &&
                        __builtin_isfinite(g_odo.centre_mm) &&
                        __builtin_isfinite(g_odo.heading_dd);
        if (g_in.b_motion)
        {
            int32_t mm_value =
                clamp_measurement(g_odo.centre_mm, INT32_MIN, INT32_MAX);
            int32_t dd_value =
                clamp_measurement(g_odo.heading_dd, INT32_MIN, INT32_MAX);
            double fine = g_odo.centre_mm * 1000.0;
            g_in.distance_mm = (int16_t)(uint16_t)mm_value;
            g_in.heading_dd = (int16_t)(uint16_t)dd_value;
            g_in.b_fine_motion =
                (fine >= (double)INT32_MIN && fine <= (double)INT32_MAX);
            g_in.distance_um = g_in.b_fine_motion ? (int32_t)fine : 0;
        }
        else
        {
            g_in.distance_mm = 0;
            g_in.heading_dd = 0;
            g_in.b_fine_motion = false;
            g_in.distance_um = 0;
        }
#ifdef APP_RTOS_IMU
        bool calibrating = g_ctx.state == IMU_ST_CALIBRATING;
        if (calibrating)
        {
            g_calibration.failed_reads += !g_in.b_read_ok;
            g_calibration.empty_reads += g_in.b_read_ok && !g_in.n;
            g_calibration.late_gaps += g_in.gap_us > B4_LATE_US;
            g_calibration.fifo_overruns += g_in.b_overrun;
        }
#endif
#ifdef APP_RTOS_RELEASE
        /* Keep the guard independent of estimator validity. It sees every
         * fresh FIFO batch after car-axis conversion, including idle and
         * samples that the hump geometry/MAD filters will reject. */
        lift_fresh_batch = lift_calibrated && g_in.b_read_ok && (g_in.n > 0U) &&
                           !g_in.b_overrun && (g_in.gap_us <= B4_LATE_US);
#endif
        b4_tick(&g_ctx, &g_in, &imu_out);
#ifdef APP_RTOS_IMU
        if (calibrating)
        {
            uint32_t age_ms = (uint32_t)(now - g_ctx.cal_start_us) / 1000U;
            g_calibration.age_ms = (uint16_t)(age_ms > 65535 ? 65535 : age_ms);
            g_calibration.progress = g_ctx.cal_n > 100 ? 100 : g_ctx.cal_n;
            if (g_calibration.progress > g_calibration.best)
            {
                g_calibration.best = g_calibration.progress;
            }
        }
        g_calibration.state = g_ctx.state;
        g_calibration.fault = g_ctx.fault;
#endif
        b4_diag_accumulate(&g_side_diag, &g_ctx);
#ifdef APP_RTOS_IMU
        ri_acquisition_record_processor(sys_time_us() - processor_start_us);
#endif
        processed = g_in.b_read_ok;
    }
#ifdef APP_RTOS_RELEASE
    lift_fresh_batch = lift_fresh_batch && (g_ctx.state != IMU_ST_INIT) &&
                       (g_ctx.state != IMU_ST_CALIBRATING) &&
                       (g_ctx.state != IMU_ST_FAULT);
    ri_lift_guard_samples(g_ctx.base, g_ctx.g_mg, g_in.s, g_in.n, B4_SAMPLE_US,
                          lift_fresh_batch);
#endif
    /* ramp-assist v0.1: publish this period's tilt view (observer data). */
    g_tilt.seq++;
    g_tilt.t_us = now_us;
    g_tilt.valid =
        processed && g_ctx.b_side_valid &&
        ((g_ctx.state == IMU_ST_TRACKING) || (g_ctx.state == IMU_ST_HUMP));
    g_tilt.height_mm = g_tilt.valid ? g_ctx.side_height_mm : 0;
    g_tilt.tracking =
        processed && ((g_ctx.state == IMU_ST_TRACKING) ||
                      (g_ctx.state == IMU_ST_HUMP)); /* ramp-assist v0.3 */
    g_tilt.quality_reasons =
        processed ? g_ctx.side_quality_reason : 0U; /* ramp-assist v0.3 */
    handle(&imu_out, now);
    if ((g_ctx.state != g_last_state) ||
        ((now - g_status_us) >= (B4_STATUS_PERIOD_MS * 1000U)))
    {
        publish(NULL);
        publish_motion_snapshot();
        g_status_us = now;
        g_last_state = g_ctx.state;
    }
    g_tick_us = sys_time_us() - t_begin;
#ifdef APP_RTOS_IMU
    ri_acquisition_record_tick(g_tick_us);
#endif
    if (g_tick_us > g_tick_max_us)
    {
        g_tick_max_us = g_tick_us;
    }
}

/* End of file. */
