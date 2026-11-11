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
static T_SPLOCK copy_lock;
static struct {
    b4v23_odo_t odo;
    b4v23_tilt_t tilt;
    imu_tlm_t tlm;
    RiAcquisition acquisition;
    RiCalibration calibration;
    RiHumpQualification hump;
    uint64_t observed_us;
    uint32_t invalid[2], run_serial, ticks, tick_us, max_tick_us, tx_drop;
    uint32_t init_epoch, ready_epoch, max_wake_late_us;
    unsigned have : 1, running : 1, reset : 1, core : 1;
#ifdef APP_RTOS_RELEASE
    imu_lift_guard_t lift;
    uint32_t lift_sample_ms;
    unsigned lift_have : 1, lift_reset_pending : 1;
#endif
} shared;
static UINT enter(void) {
    UINT key;
    (void)ISpinLock(&copy_lock, &key);
    return key;
}
static void leave(UINT key) {
    (void)ISpinUnlock(&copy_lock, key);
}

#ifdef APP_RTOS_RELEASE
/* The acquisition task is the only detector writer. Reset requests are copied;
 * it never enters the nav coordinator or touches motor registers under this lock. */
static bool lift_fresh_locked(uint32_t now_ms) {
    return shared.lift_have && !shared.lift_reset_pending &&
           now_ms - shared.lift_sample_ms <= 100U;
}
static bool lift_reset_ready_locked(uint32_t now_ms) {
    if (shared.lift_reset_pending) return false;
    bool fresh = lift_fresh_locked(now_ms);
    /* Preserve Reset recovery from a sensor/calibration fault. No qualified
     * lift may use this exception, and outputs stay blocked through reinit. */
    if (!shared.lift.latched && !fresh) return true;
    return fresh && imu_lift_guard_reset_ready(&shared.lift);
}
void ri_lift_guard_samples(const int16_t base[3], int32_t gravity,
                           const imu_lift_vec_t *samples, uint8_t count,
                           uint32_t sample_period_us, bool fresh) {
    uint32_t now_ms = (uint32_t)(board_time_us()/1000U);
    UINT key = enter();
    imu_lift_guard_t value = shared.lift;
    bool pending = shared.lift_reset_pending;
    bool interrupted = shared.lift_have && now_ms - shared.lift_sample_ms > 30U;
    leave(key);
    if (pending) return;
    if (interrupted) imu_lift_guard_no_fresh_batch(&value);
    /* Bounded integer work occurs outside the shared spinlock. */
    imu_lift_guard_batch(&value, base, gravity, samples, count, sample_period_us, now_ms, fresh);
    key = enter();
    if (!shared.lift_reset_pending) {
        shared.lift = value;
        if (fresh && value.current_valid) {
            shared.lift_sample_ms = now_ms;
            shared.lift_have = true;
        }
    }
    leave(key);
}
bool ri_safety_blocked(void) {
    uint32_t now_ms = (uint32_t)(board_time_us()/1000U);
    UINT key = enter();
    bool blocked = shared.lift.latched || !lift_fresh_locked(now_ms);
    leave(key);
    return blocked;
}
unsigned ri_safety_reason(void) {
    uint32_t now_ms = (uint32_t)(board_time_us()/1000U);
    UINT key = enter();
    unsigned reason = shared.lift_reset_pending ? 3U :
                      shared.lift.latched ? shared.lift.reason :
                      lift_fresh_locked(now_ms) ? 0U : 3U;
    leave(key);
    return reason;
}
bool ri_safety_can_reset(void) {
    uint32_t now_ms = (uint32_t)(board_time_us()/1000U);
    UINT key = enter();
    bool ready = lift_reset_ready_locked(now_ms);
    leave(key);
    return ready;
}
bool ri_safety_reset_latch(void) {
    uint32_t now_ms = (uint32_t)(board_time_us()/1000U);
    UINT key = enter();
    bool ready = lift_reset_ready_locked(now_ms);
    if (ready) {
        /* Do not race a core0 batch calculation: only its task clears detector
         * state. Pending reset blocks outputs/Start immediately, through calibration. */
        shared.lift_reset_pending = true;
        /* bringup_clear_latch calls ri_reset_request once after clearing its
         * own latch. Keep that as the single queue write, avoiding a duplicate
         * calibration if core0 runs between the two core1 operations. */
    }
    leave(key);
    return ready;
}
void ri_safety_packet(uint8_t seq, uint32_t now_ms, uint16_t run, uint8_t out[30]) {
    BringupStatus control;
    bringup_get_status(&control);
    bool stopped = (control.flags & BR_IMU) != 0U;
    unsigned stop_reason = control.reason == BR_REASON_IMU_LIFT ? 1U :
                           control.reason == BR_REASON_IMU_OVERTURN ? 2U : 3U;
    UINT key = enter();
    imu_lift_guard_t g = shared.lift;
    bool fresh = lift_fresh_locked(now_ms);
    bool reset_ready = lift_reset_ready_locked(now_ms);
    RiLiftPacket p = {.trip_ms=g.event_time_ms,.sample_ms=shared.lift_sample_ms,
        .events=g.event_count,.up_mg=g.up_projection_mg,.norm_mg=g.norm_mg,
        .reason=g.latched ? g.reason : stopped ? stop_reason : fresh ? 0U : 3U,
        .latched=g.latched || stopped,
        .armed=g.armed,.fresh=fresh,.can_reset=reset_ready,.saturated=g.saturated,
        .up_ms=g.up_samples*5U/2U,.tilt_ms=g.tilt_samples*5U/2U,
        .upright_ms=g.upright_samples*5U/2U};
    leave(key);
    ri_lift_packet_encode(&p,seq,now_ms,run,out);
}
#endif

void ri_init(void) {
    memset(&shared, 0, sizeof(shared));
    shared.init_epoch = 1U;
    /* Static-zero spinlock; initialise before either task is created. */
}
void ri_reset_request(void) {
    UINT key = enter();
#ifdef APP_RTOS_RELEASE
    if (shared.lift.latched && !shared.lift_reset_pending) {
        leave(key);
        return; /* Only the authoritative safety reset can clear a trip. */
    }
    shared.lift_reset_pending = true;
#endif
    shared.reset = true;
    leave(key);
}
void ri_new_run(void) {
    ri_ramp_halt(board_time_us());
    UINT key = enter();
    ++shared.run_serial;
    leave(key);
}
void ri_observe(const MotionFeedback *f, const uint32_t invalid[2], uint32_t dt_us, bool running) {
    UINT key = enter();
    bool clean = shared.have && dt_us && dt_us <= 50000 && invalid[0] == shared.invalid[0] &&
                 invalid[1] == shared.invalid[1];
    shared.odo = (b4v23_odo_t){.have = true,
                               .clean = clean,
                               .centre_mm = (f->wheel_mm[0] + f->wheel_mm[1]) * .5,
                               .heading_dd = (f->wheel_mm[0] - f->wheel_mm[1]) / TRACK_MM *
                                             (1800.0 / 3.141592653589793),
                               .left_mm_s = f->speed_mm_s[0],
                               .right_mm_s = f->speed_mm_s[1],
                               .invalid_left = invalid[0],
                               .invalid_right = invalid[1]};
    shared.invalid[0] = invalid[0];
    shared.invalid[1] = invalid[1];
    shared.observed_us = f->now_us;
    shared.have = true;
    shared.running = running;
    leave(key);
}
void ri_tilt_snapshot(b4v23_tilt_t *out) {
    UINT key = enter();
    *out = shared.tilt;
    leave(key);
}
static bool send_frame(const uint8_t *frame, uint32_t len) {
    imu_tlm_t t;
    if (imu_tlm_unpack(frame, len, &t)) {
        UINT key = enter();
        shared.tlm = t;
        if (t.state == IMU_ST_IDLE && t.b_valid && !t.fault)
            shared.ready_epoch = shared.init_epoch;
        leave(key);
    }
    if (!rn_drive_telemetry_enabled() || !rn_drive_online())
        return false;
    bool ok = len <= RN_PACKET_MAX && rn_tx_push(RN_TLM, frame, (uint8_t)len, board_kernel_ms());
    if (!ok) {
        UINT key = enter();
        ++shared.tx_drop;
        leave(key);
    }
    return ok;
}
void ri_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    UINT start_key = enter();
    shared.core = (unsigned)(tk_get_prc() - 1);
    leave(start_key);
    b4v23_init(send_frame, false);
    bool was_running = false;
    uint32_t run_serial = 0;
    uint64_t next = board_time_us();
    for (;;) {
        uint64_t now = board_time_us();
        if (now >= next) {
            uint32_t wake_late = (uint32_t)(now - next);
            next = ri_next_release(next, now);
            UINT key = enter();
            b4v23_odo_t odo = shared.odo;
            uint64_t observed = shared.observed_us;
            bool running = shared.running, reset = shared.reset;
            uint32_t serial = shared.run_serial;
            shared.reset = false;
            leave(key);
            odo.age_ms = now >= observed ? (uint32_t)((now - observed) / 1000) : UINT32_MAX;
            b4v23_mission_t mission = B4V23_NONE;
            if (reset) {
                /* bringup_clear_latch accepts this request only while stopped.
                 * Reinitialise even from healthy Idle, then expose a new
                 * calibration epoch; counters in IMU_ACQ remain lifetime. */
                key = enter();
                ++shared.init_epoch;
#ifdef APP_RTOS_RELEASE
                uint16_t lift_events = shared.lift.event_count;
                imu_lift_guard_init(&shared.lift);
                shared.lift.event_count = lift_events;
                shared.lift_have = false;
                shared.lift_reset_pending = false;
#endif
                memset(&shared.tilt, 0, sizeof(shared.tilt));
                memset(&shared.hump, 0, sizeof(shared.hump));
                leave(key);
                b4v23_init(send_frame, false);
                now = board_time_us();
                next = now + RI_PERIOD_US;
                odo.age_ms = now >= observed ? (uint32_t)((now - observed) / 1000) : UINT32_MAX;
                was_running = false;
            } else if (running && (!was_running || run_serial != serial)) {
                mission = run_serial == serial ? B4V23_RESUME : B4V23_START;
                run_serial = serial;
                was_running = true;
            } else if (!running && was_running) {
                mission = B4V23_STOP;
                was_running = false;
            }
            b4v23_tick(now, mission, &odo);
            b4v23_tilt_t tilt;
            b4v23_tilt(&tilt);
            RiAcquisition acquisition;
            ri_acquisition_snapshot(&acquisition);
            RiCalibration calibration;
            b4v23_calibration(&calibration);
            RiHumpQualification hump;
            b4v23_hump_qualification(&hump);
            key = enter();
            shared.tilt = tilt;
            shared.acquisition = acquisition;
            shared.calibration = calibration;
            shared.hump = hump;
            if (wake_late > shared.max_wake_late_us)
                shared.max_wake_late_us = wake_late;
            ++shared.ticks;
            shared.tick_us = b4v23_last_tick_us();
            shared.max_tick_us = b4v23_max_tick_us();
            leave(key);
        }
        (void)tk_dly_tsk(ri_release_delay_ms(next, board_time_us()));
    }
}
void ri_report(void) {
    UINT key = enter();
    imu_tlm_t t = shared.tlm;
    b4v23_tilt_t tilt = shared.tilt;
    RiAcquisition acq = shared.acquisition;
    RiCalibration cal = shared.calibration;
    RiHumpQualification hump = shared.hump;
    unsigned core = shared.core;
    uint32_t epoch = shared.init_epoch, ready_epoch = shared.ready_epoch,
             wake_max = shared.max_wake_late_us;
    uint32_t ticks = shared.ticks, cost = shared.tick_us, maxcost = shared.max_tick_us,
             drop = shared.tx_drop;
    leave(key);
    uint64_t now = board_time_us();
    uint32_t age = now >= tilt.t_us ? (uint32_t)((now - tilt.t_us) / 1000) : UINT32_MAX;
    tm_printf(
        (UB *)"IMU detector=6 state=%u motion=%u valid=%u fault=%u pitch_dd=%d humps=%u rejected=%u max_height_mm=%u i2c_errors=%u missed=%u ticks=%u tick_us=%u max_tick_us=%u tx_drop=%u\n",
        t.state, t.motion, t.b_valid, t.fault, t.pitch_dd, t.humps, t.rejected, t.max_height_mm,
        t.i2c_errors, t.missed, ticks, cost, maxcost, drop);
    tm_printf((UB *)"IMU_TILT seq=%u age_ms=%u valid=%u tracking=%u height_mm=%d quality=%u\n",
              tilt.seq, age, tilt.valid, tilt.tracking, tilt.height_mm, tilt.quality_reasons);
    tm_printf(
        (UB *)"IMU_ACQ reads=%u failed=%u late_gaps=%u fifo_overruns=%u max_gap_us=%u wake_max_us=%u epoch=%u ready_epoch=%u\n",
        acq.reads, acq.failed_reads, acq.late_gaps, acq.fifo_overruns, acq.max_gap_us, wake_max,
        epoch, ready_epoch);
    tm_printf(
        (UB *)"IMU_TIMING last_gap_us=%u last_late_us=%u last_late_read=%u win_seq=%u win_reads=%u win_late=%u win_max_us=%u prev_reads=%u prev_late=%u prev_max_us=%u\n",
        acq.last_gap_us, acq.last_late_us, acq.last_late_read, acq.window_seq, acq.window_reads,
        acq.window_late, acq.window_max_gap_us, acq.previous_reads, acq.previous_late,
        acq.previous_max_gap_us);
    tm_printf(
        (UB *)"IMU_PHASE units=wall_us read_us=%u process_us=%u tick_us=%u max_read_us=%u max_process_us=%u max_tick_us=%u fifo_n=%u\n",
        acq.last_fifo_read_wall_us, acq.last_processor_wall_us, acq.last_tick_wall_us,
        acq.max_fifo_read_wall_us, acq.max_processor_wall_us, acq.max_tick_wall_us,
        acq.fifo_batch_n);
    tm_printf(
        (UB *)"IMU_BUDGET fifo_limit_us=%u fifo_exhausted=%u fifo_last_overrun_us=%u fifo_max_overrun_us=%u init_limit_us=%u init_exhausted=%u init_last_overrun_us=%u init_max_overrun_us=%u reset_ready_us=%u reset_timeouts=%u full_tick_over_8ms=%u\n",
        RI_FIFO_BUDGET_US, acq.fifo_budget_overruns, acq.last_fifo_budget_overrun_us,
        acq.max_fifo_budget_overrun_us, RI_INIT_BUDGET_US, acq.init_budget_overruns,
        acq.last_init_budget_overrun_us, acq.max_init_budget_overrun_us,
        RI_RESET_READY_TIMEOUT_US, acq.reset_timeouts,
        acq.ticks_over_8ms);
    tm_printf(
        (UB *)"IMU_I2C transfers=%u aborts=%u timeout_tx=%u timeout_stop=%u timeout_rx=%u rescued=%u max_transfer_us=%u last_phase=%u address=0x%x abort_source=0x%x\n",
        acq.transfers, acq.aborts, acq.timeout_tx, acq.timeout_stop, acq.timeout_rx, acq.rescued,
        acq.max_transfer_us, acq.last_phase, acq.last_address, acq.last_abort_source);
    tm_printf(
        (UB *)"IMU_CAL epoch=%u core=%u priority=%u state=%u fault=%u progress=%u required=100 best=%u age_ms=%u restarts=%u axis_restarts=%u,%u,%u speed_restarts=%u last_reason=0x%x last_span_mg=%u,%u,%u failed=%u empty=%u late=%u overruns=%u\n",
        epoch, core, RI_TASK_PRIORITY, cal.state, cal.fault, cal.progress, cal.best, cal.age_ms,
        cal.restarts, cal.axis_restarts[0], cal.axis_restarts[1], cal.axis_restarts[2],
        cal.speed_restarts, cal.last_reason, cal.last_span_mg[0], cal.last_span_mg[1],
        cal.last_span_mg[2], cal.failed_reads, cal.empty_reads, cal.late_gaps, cal.fifo_overruns);
    tm_printf(
        (UB *)"IMU_HUMP flags=%u holds=%u recoveries=%u disarms=%u early_clears=%u hold_reason=%u clear_reason=%u failure_reason=%u gap_ms=%u last_gap_ms=%u\n",
        hump.flags, hump.holds, hump.recoveries, hump.disarms, hump.early_clears,
        hump.hold_reason, hump.clear_reason, hump.failure_reason, hump.gap_ms, hump.last_gap_ms);
#ifdef APP_RTOS_RELEASE
    key = enter();
    imu_lift_guard_t lift = shared.lift;
    uint32_t sample_ms = shared.lift_sample_ms;
    leave(key);
    tm_printf((UB *)"IMU_SAFETY reason=%u latched=%u armed=%u sample_ms=%u up_mg=%d norm_mg=%u events=%u up_samples=%u tilt_samples=%u upright_samples=%u reset_ready=%u\n",
              ri_safety_reason(), lift.latched, lift.armed, sample_ms, lift.up_projection_mg,
              lift.norm_mg, lift.event_count, lift.up_samples, lift.tilt_samples,
              lift.upright_samples, ri_safety_can_reset());
#endif
    ri_ramp_report();
}
#ifdef APP_RTOS_RELEASE
void ri_release_diagnostics(uint8_t seq, uint32_t now_ms, uint8_t out[63]) {
    BringupStatus control;
    bringup_get_status(&control);
    UINT key = enter();
    RiAcquisition a = shared.acquisition;
    RiHumpQualification h = shared.hump;
    uint32_t epoch = shared.init_epoch;
    leave(key);
    uint32_t values[14] = {now_ms, a.reads, a.failed_reads, a.late_gaps,
        a.fifo_overruns, a.fifo_budget_overruns, a.init_budget_overruns,
        control.control_count, control.late_count, h.holds, h.recoveries,
        h.disarms, h.early_clears, epoch};
    out[0] = 0xf2; out[1] = 60; out[2] = seq; out[3] = 0xc3;
    for (unsigned k = 0; k < 14; ++k)
        for (unsigned b = 0; b < 4; ++b)
            out[4 + 4*k + b] = values[k] >> (8*b);
    out[60] = h.clear_reason | (h.failure_reason << 4);
    out[61] = h.hold_reason | (h.flags << 2);
    out[62] = control.flags;
}
#endif
#endif
