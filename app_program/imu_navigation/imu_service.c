#ifdef APP_RTOS_IMU
#include "imu_service.h"
#include "../network_bridge/network_bridge.h"
#include "src/imu/imu_iface.h"
#include "imu_acquisition.h"
#include "acquisition_schedule.h"
#include "calibration_diagnostics.h"
#include <string.h>

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
    uint64_t observed_us;
    uint32_t invalid[2], run_serial, ticks, tick_us, max_tick_us, tx_drop;
    uint32_t init_epoch, ready_epoch, max_wake_late_us;
    unsigned have : 1, running : 1, reset : 1, core : 1;
} shared;
static UINT enter(void) {
    UINT key;
    (void)ISpinLock(&copy_lock, &key);
    return key;
}
static void leave(UINT key) {
    (void)ISpinUnlock(&copy_lock, key);
}

void ri_init(void) {
    memset(&shared, 0, sizeof(shared));
    shared.init_epoch = 1U;
    /* Static-zero spinlock; initialise before either task is created. */
}
void ri_reset_request(void) {
    UINT key = enter();
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
                memset(&shared.tilt, 0, sizeof(shared.tilt));
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
            key = enter();
            shared.tilt = tilt;
            shared.acquisition = acquisition;
            shared.calibration = calibration;
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
    ri_ramp_report();
}
#endif
