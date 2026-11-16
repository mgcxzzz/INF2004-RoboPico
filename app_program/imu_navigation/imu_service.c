/**
 * @file imu_service.c
 * @brief Imu service.
 * @details The IMU task owns sensor acquisition and calibration; readers use snapshots.
 * Ramp requests expire through the coordinator; lift safety may remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifdef APP_RTOS_IMU
#include "imu_service.h"
#include "../network_bridge/network_bridge.h"
#include "src/imu/imu_iface.h"
#include "imu_acquisition.h"
#include "acquisition_schedule.h"
#include "calibration_diagnostics.h"
#include "hump_diagnostics.h"
#include <string.h>
#ifdef APP_RTOS_RELEASE
#include "src/imu/imu_lift_guard.h"
#include "imu_lift_packet.h"
#include "../rtos_bringup/bringup_policy.h"
#endif

/* One I2C/estimator writer on core0, ahead of background USB/Wi-Fi.
 * Motion supplies observations;
 * all cross-task data are copied under this short lock. No I/O under lock. */
static T_SPLOCK g_copy_lock_value;
static struct
{
    b4v23_odo_t odo;
    b4v23_tilt_t tilt;
    imu_tlm_t tlm;
    ri_acquisition_t acquisition;
    ri_calibration_t calibration;
    ri_hump_qualification_t hump;
    uint64_t observed_us;
    uint32_t invalid[2];
    uint32_t run_serial;
    uint32_t ticks;
    uint32_t tick_us;
    uint32_t max_tick_us;
    uint32_t tx_drop;
    uint32_t init_epoch;
    uint32_t ready_epoch;
    uint32_t max_wake_late_us;
    unsigned have : 1;
    unsigned running : 1;
    unsigned reset : 1;
    unsigned core : 1;
#ifdef APP_RTOS_RELEASE
    imu_lift_guard_t lift;
    uint32_t lift_sample_ms;
    unsigned lift_have : 1;
    unsigned lift_reset_pending : 1;
#endif
} g_shared;
static UINT enter (void)
{
    UINT key;
    (void)ISpinLock(&g_copy_lock_value, &key);
    return key;
}
static void leave (UINT key)
{
    (void)ISpinUnlock(&g_copy_lock_value, key);
}

#ifdef APP_RTOS_RELEASE
/* The acquisition task is the only detector writer. Reset requests are copied;
 * it never enters the nav coordinator or touches motor registers under this lock. */
static bool lift_fresh_snapshot (bool have, bool reset_pending,
                                 uint32_t sample_ms, uint32_t now_ms)
{
    return have && !reset_pending && now_ms - sample_ms <= 100U;
}
static bool lift_reset_ready_snapshot (const imu_lift_guard_t * p_lift,
                                       bool have, bool reset_pending,
                                       uint32_t sample_ms, uint32_t now_ms)
{
    if (reset_pending)
    {
        return false;
    }
    bool fresh = lift_fresh_snapshot(have, reset_pending, sample_ms, now_ms);
    /* Preserve Reset recovery from a sensor/calibration fault. No qualified
     * lift may use this exception, and outputs stay blocked through reinit. */
    if (!p_lift->latched && !fresh)
    {
        return true;
    }
    return fresh && imu_lift_guard_reset_ready(p_lift);
}
void ri_lift_guard_samples (const int16_t base[3], int32_t gravity,
                            const imu_lift_vec_t * p_samples, uint8_t count,
                            uint32_t sample_period_us, bool fresh)
{
    UINT key = enter();
    imu_lift_guard_t value = g_shared.lift;
    bool pending = g_shared.lift_reset_pending;
    bool have = g_shared.lift_have;
    uint32_t previous_sample_ms = g_shared.lift_sample_ms;
    leave(key);
    uint32_t now_ms = (uint32_t)(board_time_us() / 1000U);
    bool interrupted = have && now_ms - previous_sample_ms > 30U;
    if (pending)
    {
        return;
    }
    if (interrupted)
    {
        imu_lift_guard_no_fresh_batch(&value);
    }
    /* Bounded integer work occurs outside the shared spinlock. */
    imu_lift_guard_batch(&value, base, gravity, p_samples, count,
                         sample_period_us, now_ms, fresh);
    key = enter();
    if (!g_shared.lift_reset_pending)
    {
        g_shared.lift = value;
        if (fresh && value.current_valid)
        {
            g_shared.lift_sample_ms = now_ms;
            g_shared.lift_have = true;
        }
    }
    leave(key);
}
bool ri_safety_blocked (void)
{
    UINT key = enter();
    imu_lift_guard_t lift = g_shared.lift;
    bool have = g_shared.lift_have, pending = g_shared.lift_reset_pending;
    uint32_t sample_ms = g_shared.lift_sample_ms;
    uint32_t now_ms = (uint32_t)(board_time_us() / 1000U);
    bool blocked =
        lift.latched || !lift_fresh_snapshot(have, pending, sample_ms, now_ms);
    leave(key);
    return blocked;
}
unsigned ri_safety_reason (void)
{
    UINT key = enter();
    imu_lift_guard_t lift = g_shared.lift;
    bool have = g_shared.lift_have, pending = g_shared.lift_reset_pending;
    uint32_t sample_ms = g_shared.lift_sample_ms;
    uint32_t now_ms = (uint32_t)(board_time_us() / 1000U);
    unsigned reason = pending        ? 3U
                      : lift.latched ? lift.reason
                      : lift_fresh_snapshot(have, pending, sample_ms, now_ms)
                          ? 0U
                          : 3U;
    leave(key);
    return reason;
}
bool ri_safety_can_reset (void)
{
    UINT key = enter();
    imu_lift_guard_t lift = g_shared.lift;
    bool have = g_shared.lift_have, pending = g_shared.lift_reset_pending;
    uint32_t sample_ms = g_shared.lift_sample_ms;
    uint32_t now_ms = (uint32_t)(board_time_us() / 1000U);
    bool ready =
        lift_reset_ready_snapshot(&lift, have, pending, sample_ms, now_ms);
    leave(key);
    return ready;
}
bool ri_safety_reset_latch (void)
{
    UINT key = enter();
    imu_lift_guard_t lift = g_shared.lift;
    bool have = g_shared.lift_have, pending = g_shared.lift_reset_pending;
    uint32_t sample_ms = g_shared.lift_sample_ms;
    uint32_t now_ms = (uint32_t)(board_time_us() / 1000U);
    bool ready =
        lift_reset_ready_snapshot(&lift, have, pending, sample_ms, now_ms);
    if (ready)
    {
        /* Do not race a core0 batch calculation: only its task clears detector
         * state. Pending reset blocks outputs/Start immediately, through calibration. */
        g_shared.lift_reset_pending = true;
        /* bringup_clear_latch calls ri_reset_request once after clearing its
         * own latch. Keep that as the single queue write, avoiding a duplicate
         * calibration if core0 runs between the two core1 operations. */
    }
    leave(key);
    return ready;
}
void ri_safety_packet (uint8_t seq, uint16_t run, uint8_t out[30])
{
    /* Sample age, trip and packet uptime must share the hardware timer epoch.
     * Kernel uptime has a different origin; callers cannot supply this clock. */
    bringup_status_t control;
    bringup_get_status(&control);
    bool stopped = (control.flags & BR_IMU) != 0U;
    unsigned stop_reason = control.reason == BR_REASON_IMU_LIFT       ? 1U
                           : control.reason == BR_REASON_IMU_OVERTURN ? 2U
                                                                      : 3U;
    UINT key = enter();
    imu_lift_guard_t g_value = g_shared.lift;
    bool have = g_shared.lift_have, pending = g_shared.lift_reset_pending;
    uint32_t sample_ms = g_shared.lift_sample_ms;
    uint32_t now_ms = (uint32_t)(board_time_us() / 1000U);
    bool fresh = lift_fresh_snapshot(have, pending, sample_ms, now_ms);
    bool reset_ready =
        lift_reset_ready_snapshot(&g_value, have, pending, sample_ms, now_ms);
    ri_lift_packet_t data = {.trip_ms = g_value.event_time_ms,
                             .sample_ms = g_shared.lift_sample_ms,
                             .events = g_value.event_count,
                             .up_mg = g_value.up_projection_mg,
                             .norm_mg = g_value.norm_mg,
                             .reason = g_value.latched ? g_value.reason
                                       : stopped       ? stop_reason
                                       : fresh         ? 0U
                                                       : 3U,
                             .latched = g_value.latched || stopped,
                             .armed = g_value.armed,
                             .fresh = fresh,
                             .can_reset = reset_ready,
                             .saturated = g_value.saturated,
                             .up_ms = g_value.up_samples * 5U / 2U,
                             .tilt_ms = g_value.tilt_samples * 5U / 2U,
                             .upright_ms = g_value.upright_samples * 5U / 2U};
    leave(key);
    ri_lift_packet_encode(&data, seq, now_ms, run, out);
}
void ri_safety_peak_packet (uint8_t seq, uint8_t out[RI_LIFT_PEAK_PACKET_BYTES])
{
    UINT key = enter();
    imu_lift_guard_t g_value = g_shared.lift;
    bool have = g_shared.lift_have, pending = g_shared.lift_reset_pending;
    uint32_t sample_ms = g_shared.lift_sample_ms;
    uint32_t now_ms = (uint32_t)(board_time_us() / 1000U);
    bool current_valid = g_value.current_valid &&
                         lift_fresh_snapshot(have, pending, sample_ms, now_ms);
    ri_lift_peak_packet_t data = {
        .report_ms = now_ms,
        .peak_sample_ms = g_value.min_cos_sample_ms,
        .window_start_ms = g_value.peak_window_start_ms,
        .min_cos_q15 = g_value.min_cos_q15,
        .norm_mg = g_value.min_cos_norm_mg,
        .up_mg = g_value.peak_up_projection_mg,
        .tilt_ms = g_value.peak_tilt_samples * 5U / 2U,
        .up_ms = g_value.peak_up_samples * 5U / 2U,
        .events = g_value.event_count,
        .last_candidate = g_value.last_candidate,
        .last_rejection = g_value.last_rejection,
        .armed = g_value.armed,
        .current_valid = current_valid,
        .latched = g_value.latched,
        .saturated = g_value.peak_saturated};
    leave(key);
    ri_lift_peak_packet_encode(&data, seq, out);
}
#endif

void ri_init (void)
{
    memset(&g_shared, 0, sizeof(g_shared));
#ifdef APP_RTOS_RELEASE
    imu_lift_guard_init(&g_shared.lift);
#endif
    g_shared.init_epoch = 1U;
    /* Static-zero spinlock; initialise before either task is created. */
}
void ri_reset_request (void)
{
    UINT key = enter();
#ifdef APP_RTOS_RELEASE
    if (g_shared.lift.latched && !g_shared.lift_reset_pending)
    {
        leave(key);
        return; /* Only the authoritative safety reset can clear a trip. */
    }
    g_shared.lift_reset_pending = true;
#endif
    g_shared.reset = true;
    leave(key);
}
void ri_new_run (void)
{
    ri_ramp_halt(board_time_us());
    UINT key = enter();
    ++g_shared.run_serial;
    leave(key);
}
void ri_observe (const motion_feedback_t * p_flags_value,
                 const uint32_t invalid[2], uint32_t dt_us, bool running)
{
    UINT key = enter();
    bool clean = g_shared.have && dt_us && dt_us <= 50000 &&
                 invalid[0] == g_shared.invalid[0] &&
                 invalid[1] == g_shared.invalid[1];
    g_shared.odo = (b4v23_odo_t){
        .have = true,
        .clean = clean,
        .centre_mm =
            (p_flags_value->wheel_mm[0] + p_flags_value->wheel_mm[1]) * .5,
        .heading_dd =
            (p_flags_value->wheel_mm[0] - p_flags_value->wheel_mm[1]) /
            TRACK_MM * (1800.0 / 3.141592653589793),
        .left_mm_s = p_flags_value->speed_mm_s[0],
        .right_mm_s = p_flags_value->speed_mm_s[1],
        .invalid_left = invalid[0],
        .invalid_right = invalid[1]};
    g_shared.invalid[0] = invalid[0];
    g_shared.invalid[1] = invalid[1];
    g_shared.observed_us = p_flags_value->now_us;
    g_shared.have = true;
    g_shared.running = running;
    leave(key);
}
void ri_tilt_snapshot (b4v23_tilt_t * p_out_value)
{
    UINT key = enter();
    *p_out_value = g_shared.tilt;
    leave(key);
}
static bool send_frame (const uint8_t * p_frame, uint32_t len)
{
    imu_tlm_t sample;
    if (imu_tlm_unpack(p_frame, len, &sample))
    {
        UINT key = enter();
        g_shared.tlm = sample;
        if (sample.state == IMU_ST_IDLE && sample.b_valid && !sample.fault)
        {
            g_shared.ready_epoch = g_shared.init_epoch;
        }
        leave(key);
    }
    if (!rn_drive_telemetry_enabled() || !rn_drive_online())
    {
        return false;
    }
    bool ok_value =
        len <= RN_PACKET_MAX &&
        rn_tx_push(RN_TLM, p_frame, (uint8_t)len, board_kernel_ms());
    if (!ok_value)
    {
        UINT key = enter();
        ++g_shared.tx_drop;
        leave(key);
    }
    return ok_value;
}
void ri_task (INT code, void * p_exinf_value)
{
    (void)code;
    (void)p_exinf_value;
    UINT start_key = enter();
    g_shared.core = (unsigned)(tk_get_prc() - 1);
    leave(start_key);
    b4v23_init(send_frame, false);
    bool was_running = false;
    uint32_t run_serial = 0;
    uint64_t next = board_time_us();
    for (;;)
    {
        uint64_t now = board_time_us();
        if (now >= next)
        {
            uint32_t wake_late = (uint32_t)(now - next);
            next = ri_next_release(next, now);
            UINT key = enter();
            b4v23_odo_t odo = g_shared.odo;
            uint64_t observed = g_shared.observed_us;
            bool running = g_shared.running, reset = g_shared.reset;
            uint32_t serial = g_shared.run_serial;
            g_shared.reset = false;
            leave(key);
            odo.age_ms = now >= observed ? (uint32_t)((now - observed) / 1000)
                                         : UINT32_MAX;
            b4v23_mission_t mission = B4V23_NONE;
            if (reset)
            {
                /* bringup_clear_latch accepts this request only while stopped.
                 * Reinitialise even from healthy Idle, then expose a new
                 * calibration epoch; counters in IMU_ACQ remain lifetime. */
                key = enter();
                ++g_shared.init_epoch;
#ifdef APP_RTOS_RELEASE
                uint16_t lift_events = g_shared.lift.event_count;
                imu_lift_guard_init(&g_shared.lift);
                g_shared.lift.event_count = lift_events;
                g_shared.lift_have = false;
                g_shared.lift_reset_pending = false;
#endif
                memset(&g_shared.tilt, 0, sizeof(g_shared.tilt));
                memset(&g_shared.hump, 0, sizeof(g_shared.hump));
                leave(key);
                b4v23_init(send_frame, false);
                now = board_time_us();
                next = now + RI_PERIOD_US;
                odo.age_ms = now >= observed
                                 ? (uint32_t)((now - observed) / 1000)
                                 : UINT32_MAX;
                was_running = false;
            }
            else if (running && (!was_running || run_serial != serial))
            {
                mission = run_serial == serial ? B4V23_RESUME : B4V23_START;
                run_serial = serial;
                was_running = true;
            }
            else if (!running && was_running)
            {
                mission = B4V23_STOP;
                was_running = false;
            }
            b4v23_tick(now, mission, &odo);
            b4v23_tilt_t tilt;
            b4v23_tilt(&tilt);
            ri_acquisition_t acquisition;
            ri_acquisition_snapshot(&acquisition);
            ri_calibration_t calibration;
            b4v23_calibration(&calibration);
            ri_hump_qualification_t hump;
            b4v23_hump_qualification(&hump);
            key = enter();
            g_shared.tilt = tilt;
            g_shared.acquisition = acquisition;
            g_shared.calibration = calibration;
            g_shared.hump = hump;
            if (wake_late > g_shared.max_wake_late_us)
            {
                g_shared.max_wake_late_us = wake_late;
            }
            ++g_shared.ticks;
            g_shared.tick_us = b4v23_last_tick_us();
            g_shared.max_tick_us = b4v23_max_tick_us();
            leave(key);
        }
        (void)tk_dly_tsk(ri_release_delay_ms(next, board_time_us()));
    }
}
void ri_report (void)
{
    UINT key = enter();
    imu_tlm_t sample = g_shared.tlm;
    b4v23_tilt_t tilt = g_shared.tilt;
    ri_acquisition_t acq = g_shared.acquisition;
    ri_calibration_t cal = g_shared.calibration;
    ri_hump_qualification_t hump = g_shared.hump;
    unsigned core = g_shared.core;
    uint32_t epoch = g_shared.init_epoch, ready_epoch = g_shared.ready_epoch,
             wake_max = g_shared.max_wake_late_us;
    uint32_t ticks = g_shared.ticks, cost = g_shared.tick_us,
             maxcost = g_shared.max_tick_us, drop = g_shared.tx_drop;
    leave(key);
    uint64_t now = board_time_us();
    uint32_t age =
        now >= tilt.t_us ? (uint32_t)((now - tilt.t_us) / 1000) : UINT32_MAX;
    tm_printf(
        (UB *)"IMU detector=6 state=%u motion=%u valid=%u fault=%u pitch_dd=%d "
              "humps=%u rejected=%u max_height_mm=%u i2c_errors=%u missed=%u "
              "ticks=%u tick_us=%u max_tick_us=%u tx_drop=%u\n",
        sample.state, sample.motion, sample.b_valid, sample.fault,
        sample.pitch_dd, sample.humps, sample.rejected, sample.max_height_mm,
        sample.i2c_errors, sample.missed, ticks, cost, maxcost, drop);
    tm_printf((UB *)"IMU_TILT seq=%u age_ms=%u valid=%u tracking=%u "
                    "height_mm=%d quality=%u\n",
              tilt.seq, age, tilt.valid, tilt.tracking, tilt.height_mm,
              tilt.quality_reasons);
    tm_printf((UB *)"IMU_ACQ reads=%u failed=%u late_gaps=%u fifo_overruns=%u "
                    "max_gap_us=%u wake_max_us=%u epoch=%u ready_epoch=%u\n",
              acq.reads, acq.failed_reads, acq.late_gaps, acq.fifo_overruns,
              acq.max_gap_us, wake_max, epoch, ready_epoch);
    tm_printf((UB *)"IMU_TIMING last_gap_us=%u last_late_us=%u "
                    "last_late_read=%u win_seq=%u win_reads=%u win_late=%u "
                    "win_max_us=%u prev_reads=%u prev_late=%u prev_max_us=%u\n",
              acq.last_gap_us, acq.last_late_us, acq.last_late_read,
              acq.window_seq, acq.window_reads, acq.window_late,
              acq.window_max_gap_us, acq.previous_reads, acq.previous_late,
              acq.previous_max_gap_us);
    tm_printf(
        (UB *)"IMU_PHASE units=wall_us read_us=%u process_us=%u tick_us=%u "
              "max_read_us=%u max_process_us=%u max_tick_us=%u fifo_n=%u\n",
        acq.last_fifo_read_wall_us, acq.last_processor_wall_us,
        acq.last_tick_wall_us, acq.max_fifo_read_wall_us,
        acq.max_processor_wall_us, acq.max_tick_wall_us, acq.fifo_batch_n);
    tm_printf(
        (UB *)"IMU_BUDGET fifo_limit_us=%u fifo_exhausted=%u "
              "fifo_last_overrun_us=%u fifo_max_overrun_us=%u init_limit_us=%u "
              "init_exhausted=%u init_last_overrun_us=%u "
              "init_max_overrun_us=%u reset_ready_us=%u reset_timeouts=%u "
              "full_tick_over_8ms=%u\n",
        RI_FIFO_BUDGET_US, acq.fifo_budget_overruns,
        acq.last_fifo_budget_overrun_us, acq.max_fifo_budget_overrun_us,
        RI_INIT_BUDGET_US, acq.init_budget_overruns,
        acq.last_init_budget_overrun_us, acq.max_init_budget_overrun_us,
        RI_RESET_READY_TIMEOUT_US, acq.reset_timeouts, acq.ticks_over_8ms);
    tm_printf(
        (UB *)"IMU_I2C transfers=%u aborts=%u timeout_tx=%u timeout_stop=%u "
              "timeout_rx=%u rescued=%u max_transfer_us=%u last_phase=%u "
              "address=0x%x abort_source=0x%x\n",
        acq.transfers, acq.aborts, acq.timeout_tx, acq.timeout_stop,
        acq.timeout_rx, acq.rescued, acq.max_transfer_us, acq.last_phase,
        acq.last_address, acq.last_abort_source);
    tm_printf(
        (UB *)"IMU_CAL epoch=%u core=%u priority=%u state=%u fault=%u "
              "progress=%u required=100 best=%u age_ms=%u restarts=%u "
              "axis_restarts=%u,%u,%u speed_restarts=%u last_reason=0x%x "
              "last_span_mg=%u,%u,%u failed=%u empty=%u late=%u overruns=%u\n",
        epoch, core, RI_TASK_PRIORITY, cal.state, cal.fault, cal.progress,
        cal.best, cal.age_ms, cal.restarts, cal.axis_restarts[0],
        cal.axis_restarts[1], cal.axis_restarts[2], cal.speed_restarts,
        cal.last_reason, cal.last_span_mg[0], cal.last_span_mg[1],
        cal.last_span_mg[2], cal.failed_reads, cal.empty_reads, cal.late_gaps,
        cal.fifo_overruns);
    tm_printf((UB *)"IMU_HUMP flags=%u holds=%u recoveries=%u disarms=%u "
                    "early_clears=%u hold_reason=%u clear_reason=%u "
                    "failure_reason=%u gap_ms=%u last_gap_ms=%u\n",
              hump.flags, hump.holds, hump.recoveries, hump.disarms,
              hump.early_clears, hump.hold_reason, hump.clear_reason,
              hump.failure_reason, hump.gap_ms, hump.last_gap_ms);
#ifdef APP_RTOS_RELEASE
    key = enter();
    imu_lift_guard_t lift = g_shared.lift;
    uint32_t sample_ms = g_shared.lift_sample_ms;
    leave(key);
    tm_printf((UB *)"IMU_SAFETY reason=%u latched=%u armed=%u sample_ms=%u "
                    "up_mg=%d norm_mg=%u events=%u up_samples=%u "
                    "tilt_samples=%u upright_samples=%u reset_ready=%u\n",
              ri_safety_reason(), lift.latched, lift.armed, sample_ms,
              lift.up_projection_mg, lift.norm_mg, lift.event_count,
              lift.up_samples, lift.tilt_samples, lift.upright_samples,
              ri_safety_can_reset());
#endif
    ri_ramp_report();
}
#ifdef APP_RTOS_RELEASE
void ri_release_diagnostics (uint8_t seq, uint32_t now_ms, uint8_t out[63])
{
    bringup_status_t control;
    bringup_get_status(&control);
    UINT key = enter();
    ri_acquisition_t first = g_shared.acquisition;
    ri_hump_qualification_t h_value = g_shared.hump;
    uint32_t epoch = g_shared.init_epoch;
    leave(key);
    uint32_t values[14] = {now_ms,
                           first.reads,
                           first.failed_reads,
                           first.late_gaps,
                           first.fifo_overruns,
                           first.fifo_budget_overruns,
                           first.init_budget_overruns,
                           control.control_count,
                           control.late_count,
                           h_value.holds,
                           h_value.recoveries,
                           h_value.disarms,
                           h_value.early_clears,
                           epoch};
    out[0] = 0xf2;
    out[1] = 60;
    out[2] = seq;
    out[3] = 0xc3;
    for (unsigned item_index = 0; item_index < 14; ++item_index)
    {
        for (unsigned second = 0; second < 4; ++second)
        {
            out[4 + 4 * item_index + second] =
                values[item_index] >> (8 * second);
        }
    }
    out[60] = h_value.clear_reason | (h_value.failure_reason << 4);
    out[61] = h_value.hold_reason | (h_value.flags << 2);
    out[62] = control.flags;
}
#endif
#endif

/* End of file. */
